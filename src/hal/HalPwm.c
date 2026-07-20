#include "Hal.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/printk.h>

#include <fsl_common.h>
#include <fsl_pwm.h>

#include "MathUtil.h"

/*==============================================================================
 * HalPwm.c
 *
 * Low-level FlexPWM actuation for the three-phase inverter.
 *
 * This module owns complementary center-aligned PWM setup, deadtime, fixed
 * phase-to-submodule mapping, output floating, normalized-duty conversion, and
 * the PWM-originated ADC trigger point. Controller and commutation policy remain
 * above this layer.
 *
 * 
 * Fixed mapping:
 *   - SM1 -> Phase C
 *   - SM2 -> Phase B
 *   - SM3 -> Phase A
 *
 * Floating a phase disables both outputs of its complementary pair. Compare
 * values are still updated so the phase can resume coherently when re-enabled.
 *============================================================================*/

/*==============================================================================
 * LOCAL CONFIGURATION
 *===========================================================================*/

/* Local init-print control. Explicit debug dump functions are not gated. */
#define HAL_PWM_INIT_PRINTS          0

/* FlexPWM input clock used for all timing calculations in this module. */
#define PWM_CLOCK_HZ                 48000000u

/* Target PWM carrier frequency for the inverter. */
#define PWM_FREQUENCY_HZ             16000u

/* Deadtime in raw PWM ticks. */
#define PWM_DEADTIME_TICKS           20u

/*
 * Trigger offset used on SM1 VAL0.
 *
 * This creates the PWM-originated hardware trigger later routed to the ADC
 * path through INPUTMUX.
 */
#define PWM_TRIGGER_OFFSET_TICKS     10u

/* Half-period count for signed center-aligned PWM operation. */
#define PWM_HALF_PERIOD_TICKS        ((uint16_t)(PWM_CLOCK_HZ / (2u * PWM_FREQUENCY_HZ)))

/*
 * Fixed FlexPWM submodule mapping.
 *
 * IMPORTANT:
 *   SM1 -> Phase C
 *   SM2 -> Phase B
 *   SM3 -> Phase A
 *
 * This mapping is part of the current working hardware/software convention and
 * should not be changed casually.
 */
enum
{
    PWM_SM_PHASE_C = 1u,
    PWM_SM_PHASE_B = 2u,
    PWM_SM_PHASE_A = 3u
};

#define PWM_MOTOR_CONTROL_MODULES \
    (kPWM_Control_Module_1 |       \
     kPWM_Control_Module_2 |       \
     kPWM_Control_Module_3)

/*==============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/*
 * Clamps a normalized duty cycle into [0.0, 1.0].
 *
 * NaN is forced to 0.0 to avoid writing invalid compare values into the PWM
 * registers.
 */
static float HalPwmClampDuty(float duty)
{
    if (MathIsNaN(duty))
    {
        return 0.0f;
    }
    return MathClamp(duty, 0.0f, 1.0f);
}

/*
 * Disables both A/B outputs for all three configured motor phases.
 *
 * DESIGN INTENT:
 * - safe shutdown path
 * - used whenever gate drive should not actively switch
 */
static void HalPwmDisableAllPhaseOutputs(PWM_Type *pwm)
{
    const uint16_t clearMask =
        PWM_OUTEN_PWMA_EN(1u << PWM_SM_PHASE_A) |
        PWM_OUTEN_PWMB_EN(1u << PWM_SM_PHASE_A) |
        PWM_OUTEN_PWMA_EN(1u << PWM_SM_PHASE_B) |
        PWM_OUTEN_PWMB_EN(1u << PWM_SM_PHASE_B) |
        PWM_OUTEN_PWMA_EN(1u << PWM_SM_PHASE_C) |
        PWM_OUTEN_PWMB_EN(1u << PWM_SM_PHASE_C);

    pwm->OUTEN &= ~clearMask;
}

/*
 * Enables or disables both outputs for one FlexPWM submodule.
 *
 * When enable=false, the corresponding phase is electrically floated because
 * neither side of the complementary pair is allowed to switch.
 */
static void HalPwmSetPhaseOutputState(
    PWM_Type *pwm,
    uint8_t submodule,
    bool enable)
{
    const uint16_t maskA = PWM_OUTEN_PWMA_EN(1u << submodule);
    const uint16_t maskB = PWM_OUTEN_PWMB_EN(1u << submodule);

    if (enable)
    {
        pwm->OUTEN |= (maskA | maskB);
    }
    else
    {
        pwm->OUTEN &= ~(maskA | maskB);
    }
}

/*
 * Writes a centered duty cycle into VAL2/VAL3 for one FlexPWM submodule.
 *
 * DUTY CONVENTION:
 * - 0.0 -> zero pulse width
 * - 1.0 -> maximum pulse width
 *
 * ASSUMPTION:
 * - PWM is configured for signed center-aligned operation.
 */
static void HalPwmSetCenteredDuty(
    PWM_Type *pwm,
    uint8_t submodule,
    float duty)
{
    const float clampedDuty = HalPwmClampDuty(duty);
    const int16_t pulseHalfWidth = (int16_t)(clampedDuty * (float)PWM_HALF_PERIOD_TICKS);

    pwm->SM[submodule].VAL2 = (uint16_t)(int16_t)(-pulseHalfWidth);
    pwm->SM[submodule].VAL3 = (uint16_t)(int16_t)(pulseHalfWidth);
}


/* Configures one complementary output descriptor. */
static void HalPwmConfigureSignal(
    pwm_signal_param_t *signal,
    pwm_channels_t channel)
{
    signal->pwmChannel = channel;
    signal->dutyCyclePercent = 0U;
    signal->level = kPWM_HighTrue;
    signal->faultState = kPWM_PwmFaultState0;
    signal->pwmchannelenable = true;
    signal->deadtimeValue = PWM_DEADTIME_TICKS;
}

/* Initializes and configures one motor-phase submodule. */
static void HalPwmConfigureSubmodule(
    pwm_submodule_t submodule,
    const pwm_config_t *config,
    pwm_signal_param_t signals[2])
{
    PWM_Init(PWM1, submodule, config);

    PWM_SetupForceSignal(PWM1, submodule, kPWM_PwmA, kPWM_UsePwm);
    PWM_SetupForceSignal(PWM1, submodule, kPWM_PwmB, kPWM_UsePwm);

    PWM_SetupPwm(
        PWM1,
        submodule,
        signals,
        2U,
        kPWM_SignedCenterAligned,
        PWM_FREQUENCY_HZ,
        PWM_CLOCK_HZ);
}

/* Commits buffered compare values for all three motor phases. */
static void HalPwmCommitUpdates(void)
{
    PWM_SetPwmLdok(PWM1, PWM_MOTOR_CONTROL_MODULES, true);
}

/*==============================================================================
 * PUBLIC API
 *===========================================================================*/

/*
 * Initializes the FlexPWM hardware for 3-phase motor drive.
 *
 * FLOW:
 * - configure each submodule for complementary signed center-aligned PWM
 * - configure deadtime on both outputs
 * - enable software-controlled force behavior
 * - set SM1 VAL0 as the ADC-trigger point
 * - preload safe centered compare values
 * - start timers while keeping outputs disabled
 *
 * PHASE MAPPING:
 *   SM1 -> Phase C
 *   SM2 -> Phase B
 *   SM3 -> Phase A
 */
void HalPwm_Init(void)
{
    pwm_config_t smConfig;
    pwm_signal_param_t pwmSignals[2];

#if HAL_PWM_INIT_PRINTS
    printk("[HAL][PWM] Init begin\n");
#endif

    PWM_GetDefaultConfig(&smConfig);

    smConfig.clockSource = kPWM_BusClock;
    smConfig.prescale = kPWM_Prescale_Divide_1;
    smConfig.pairOperation = kPWM_ComplementaryPwmA;
    smConfig.initializationControl = kPWM_Initialize_LocalSync;
    smConfig.reloadLogic = kPWM_ReloadImmediate;
    smConfig.reloadSelect = kPWM_LocalReload;
    smConfig.reloadFrequency = kPWM_LoadEveryOportunity;
    smConfig.forceTrigger = kPWM_Force_Local;
    smConfig.enableDebugMode = false;

    HalPwmConfigureSignal(&pwmSignals[0], kPWM_PwmA);
    HalPwmConfigureSignal(&pwmSignals[1], kPWM_PwmB);

    HalPwmConfigureSubmodule(kPWM_Module_1, &smConfig, pwmSignals);
    HalPwmConfigureSubmodule(kPWM_Module_2, &smConfig, pwmSignals);
    HalPwmConfigureSubmodule(kPWM_Module_3, &smConfig, pwmSignals);

    /* SM1 / Phase C hosts the ADC-trigger point used by the fast sampling path. */
    PWM1->SM[PWM_SM_PHASE_C].VAL0 = PWM_TRIGGER_OFFSET_TICKS;
    PWM_OutputTriggerEnable(PWM1, kPWM_Module_1, kPWM_ValueRegister_0, true);

    /* Preload safe centered compare values while outputs remain disabled. */
    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_A, 0.5f);
    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_B, 0.5f);
    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_C, 0.5f);

    HalPwmCommitUpdates();
    PWM_StartTimer(PWM1, PWM_MOTOR_CONTROL_MODULES);

    HalPwm_DisableAllOutputs();

#if HAL_PWM_INIT_PRINTS
    printk("[HAL][PWM] Init done\n");
#endif
}

/* Disables all three phase pairs at once. */
void HalPwm_DisableAllOutputs(void)
{
    HalPwmDisableAllPhaseOutputs(PWM1);
}

/*
 * Writes one normalized three-phase PWM command to the FlexPWM hardware.
 *
 * BEHAVIOR:
 * - if cmd is NULL or gate drive is disabled, all outputs are disabled
 * - otherwise each phase output pair is enabled/disabled according to floatX
 * - centered duty values are written for all three submodules
 * - LDOK is asserted so new compare values take effect coherently
 *
 * PHASE MAPPING:
 *   cmd->dutyA / floatA -> SM3 -> Phase A
 *   cmd->dutyB / floatB -> SM2 -> Phase B
 *   cmd->dutyC / floatC -> SM1 -> Phase C
 */
void HalPwm_WriteCommand(const HalPwmCommand_t *cmd)
{
    if ((cmd == NULL) || (!cmd->enableGateDriver))
    {
        HalPwmDisableAllPhaseOutputs(PWM1);
        return;
    }

    HalPwmSetPhaseOutputState(PWM1, PWM_SM_PHASE_A, !cmd->floatA);
    HalPwmSetPhaseOutputState(PWM1, PWM_SM_PHASE_B, !cmd->floatB);
    HalPwmSetPhaseOutputState(PWM1, PWM_SM_PHASE_C, !cmd->floatC);

    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_A, cmd->dutyA);
    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_B, cmd->dutyB);
    HalPwmSetCenteredDuty(PWM1, PWM_SM_PHASE_C, cmd->dutyC);

    HalPwmCommitUpdates();
}

/*
 * Prints the key FlexPWM register state used during bring-up and validation.
 *
 * This is intentionally hardware-facing debug visibility, not runtime control
 * logic.
 */
void HalPwm_PrintRegisters(void)
{
    printk("\n--- FLEXPWM1 ---\n");
    printk("MCTRL            : 0x%08X\n", PWM1->MCTRL);
    printk("OUTEN            : 0x%08X\n", PWM1->OUTEN);

    printk("\n[SM1 - Phase C]\n");
    printk("INIT             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_C].INIT);
    printk("VAL0             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL0);
    printk("VAL1             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL1);
    printk("VAL2             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL2);
    printk("VAL3             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL3);
    printk("CTRL2            : 0x%04X\n", PWM1->SM[PWM_SM_PHASE_C].CTRL2);
    printk("TCTRL            : 0x%04X\n", PWM1->SM[PWM_SM_PHASE_C].TCTRL);

    printk("\n[SM2 - Phase B]\n");
    printk("INIT             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_B].INIT);
    printk("VAL1             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_B].VAL1);
    printk("VAL2             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_B].VAL2);
    printk("VAL3             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_B].VAL3);

    printk("\n[SM3 - Phase A]\n");
    printk("INIT             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_A].INIT);
    printk("VAL1             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_A].VAL1);
    printk("VAL2             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_A].VAL2);
    printk("VAL3             : %d\n",  (int16_t)PWM1->SM[PWM_SM_PHASE_A].VAL3);
}

/* Prints the SM1 trigger-related registers used during ADC-trigger debug. */
void HalPwm_PrintSm1TriggerRegisters(void)
{
    printk("PWM MCTRL=0x%08X\n", PWM1->MCTRL);
    printk("SM1 TCTRL=0x%04X\n", PWM1->SM[PWM_SM_PHASE_C].TCTRL);
    printk("SM1 CNT=%d\n", (int16_t)PWM1->SM[PWM_SM_PHASE_C].CNT);
    printk("SM1 INIT=%d\n", (int16_t)PWM1->SM[PWM_SM_PHASE_C].INIT);
    printk("SM1 VAL0=%d\n", (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL0);
    printk("SM1 VAL1=%d\n", (int16_t)PWM1->SM[PWM_SM_PHASE_C].VAL1);
}