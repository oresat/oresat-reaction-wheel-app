#include "Hal.h"

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <fsl_common.h>
#include <fsl_lpadc.h>

/*==============================================================================
 * HalAdc.c
 *
 * Low-level LPADC configuration and acquisition for the reaction-wheel board.
 *
 * ADC0 owns the PWM-synchronous fast bundle:
 *   Ia -> Ib -> Ic -> Vbus -> Va -> Vb -> Vc
 *
 * ADC1 owns the software-triggered thermal bundle:
 *   T_Aux -> T_PhaseA -> T_PhaseB -> T_PhaseC
 *
 * The fast trigger is gated around each acquisition so exactly one fresh
 * seven-sample bundle is consumed by the caller.
 *============================================================================*/

/*==============================================================================
 * LOCAL DEBUG PRINT CONTROL
 *============================================================================*/
#define HAL_ADC_INIT_PRINTS          0
#define HAL_ADC_CAL_PRINTS           0

/*==============================================================================
 * LPADC COMMAND / TRIGGER LAYOUT
 *
 * ADC0 (Fast Path - Hardware Triggered by PWM)
 * CMD 1-7: Ia -> Ib -> Ic -> Vbus -> Va -> Vb -> Vc
 *
 * ADC1 (Thermal Path - Software Triggered)
 * CMD 1-4: T_Aux -> T_PhaseA -> T_PhaseB -> T_PhaseC
 *============================================================================*/

#define HAL_ADC_FAST_TRIGGER_ID                 0U
#define HAL_ADC_THERMAL_TRIGGER_ID              0U

/* ADC0 Commands (Fast) */
#define HAL_ADC0_CMD_ISNS_A                     1U
#define HAL_ADC0_CMD_ISNS_B                     2U
#define HAL_ADC0_CMD_ISNS_C                     3U
#define HAL_ADC0_CMD_VBUS                       4U
#define HAL_ADC0_CMD_VSNS_A                     5U
#define HAL_ADC0_CMD_VSNS_B                     6U
#define HAL_ADC0_CMD_VSNS_C                     7U

/* ADC1 Commands (Slow) */
#define HAL_ADC1_CMD_THERM_AUX                  1U
#define HAL_ADC1_CMD_THERM_A                    2U
#define HAL_ADC1_CMD_THERM_B                    3U
#define HAL_ADC1_CMD_THERM_C                    4U

/*==============================================================================
 * ADC CHANNEL MAPPING (V2 Board)
 *
 * ADC0 (All Side A)
 * ADC1 (All Side A)
 * 
 * ADC Routing-
 * ISNS:   PHA -> ADC0_A0 
 *         PHB -> ADC0_A1
 *         PHC -> ADC0_A2
 *
 * VSNS:   VBUS -> ADC0_A3
 *         PHA -> ADC0_A4
 *         PHB -> ADC0_A5
 *         PHC -> ADC0_A6
 *
 * Therms: AUX -> ADC1_A12
 *         PHA -> ADC1_A13
 *         PHB -> ADC1_A14
 *         PHC -> ADC1_A15
 *============================================================================*/

/* ADC0 */
#define HAL_ADC0_CH_ISNS_A                      0U
#define HAL_ADC0_CH_ISNS_B                      1U
#define HAL_ADC0_CH_ISNS_C                      2U
#define HAL_ADC0_CH_VBUS                        3U
#define HAL_ADC0_CH_VSNS_A                      4U
#define HAL_ADC0_CH_VSNS_B                      5U
#define HAL_ADC0_CH_VSNS_C                      6U

/* ADC1 */
#define HAL_ADC1_CH_THERM_AUX                   12U
#define HAL_ADC1_CH_THERM_A                     13U
#define HAL_ADC1_CH_THERM_B                     14U
#define HAL_ADC1_CH_THERM_C                     15U

/*==============================================================================
 * LOCAL TIMEOUTS
 *============================================================================*/

#define HAL_ADC_FAST_ACQUIRE_TIMEOUT_LOOPS      100000U
#define HAL_ADC_THERMAL_TIMEOUT_LOOPS           1000000U

/*==============================================================================
 * PRIVATE HELPERS
 *============================================================================*/

static void HalAdcRunCalibration(ADC_Type *base, const char *name)
{
    LPADC_DoAutoCalibration(base);

#if HAL_ADC_CAL_PRINTS
    printk("[HAL][ADC] %s calibration attempted\n", name);
    printk("[HAL][ADC] %s OFSTRIM=0x%08X GCR0=0x%08X GCR1=0x%08X GCC0=0x%08X GCC1=0x%08X\n",
           name,
           (unsigned int)base->OFSTRIM,
           (unsigned int)base->GCR[0],
           (unsigned int)base->GCR[1],
           (unsigned int)base->GCC[0],
           (unsigned int)base->GCC[1]);
#endif
}

/*
 * Applies one LPADC trigger configuration.
 *
 * Used both during initialization and during the bounded one-bundle fast
 * acquisition flow when trigger gating is intentionally opened and closed.
 */
static void HalAdcConfigureTrigger(
    ADC_Type *base,
    uint32_t triggerId,
    uint32_t targetCommandId,
    bool enableHardwareTrigger,
    uint32_t priority)
{
    lpadc_conv_trigger_config_t trigConfig;
    LPADC_GetDefaultConvTriggerConfig(&trigConfig);

    trigConfig.targetCommandId       = targetCommandId;
    trigConfig.delayPower            = 0U;
    trigConfig.priority              = priority;
    trigConfig.enableHardwareTrigger = enableHardwareTrigger;

    LPADC_SetConvTriggerConfig(base, triggerId, &trigConfig);
}

/* Configures one command in an LPADC conversion chain. */
static void HalAdcConfigureCommand(
    ADC_Type *base,
    uint32_t cmdId,
    uint32_t channel,
    uint32_t nextCmdId,
    lpadc_sample_time_mode_t sampleTime)
{
    lpadc_conv_command_config_t cmdConfig;
    LPADC_GetDefaultConvCommandConfig(&cmdConfig);

    cmdConfig.channelNumber = channel;
    cmdConfig.sampleChannelMode = kLPADC_SampleChannelSingleEndSideA;
    cmdConfig.sampleTimeMode = sampleTime;
    cmdConfig.chainedNextCommandNumber = nextCmdId;

    LPADC_SetConvCommandConfig(base, cmdId, &cmdConfig);
}


/* Drains all pending results from FIFO 0. */
static void HalAdcDrainResultFifo(ADC_Type *base)
{
    while (LPADC_GetConvResultCount(base, 0U) > 0U)
    {
        (void)base->RESFIFO[0];
    }
}

/* Extracts the 12-bit conversion result from one raw LPADC FIFO word. */
static uint16_t HalAdcExtractCounts(uint32_t rawResult)
{
    return (uint16_t)((rawResult >> 3) & 0x0FFFU);
}

/*==============================================================================
 * PUBLIC API
 *============================================================================*/

void HalAdc_Init(void)
{
#if HAL_ADC_INIT_PRINTS
    printk("[HAL][ADC] Init begin\n");
#endif

    lpadc_config_t adcConfig;
    LPADC_GetDefaultConfig(&adcConfig);

    /*
     * Conservative V1-style ADC bring-up.
     * This improves early Vbus validity during calibration startup.
     */
    adcConfig.enableAnalogPreliminary = false;
    adcConfig.powerUpDelay            = 0x80U;
    adcConfig.referenceVoltageSource  = kLPADC_ReferenceVoltageAlt1;
    adcConfig.powerLevelMode          = kLPADC_PowerLevelAlt1;
    adcConfig.conversionAverageMode   = kLPADC_ConversionAverage1;

    LPADC_Init(ADC0, &adcConfig);
    LPADC_Init(ADC1, &adcConfig);

    HalAdcRunCalibration(ADC0, "ADC0");
    HalAdcRunCalibration(ADC1, "ADC1");

    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_ISNS_A, HAL_ADC0_CH_ISNS_A, HAL_ADC0_CMD_ISNS_B, kLPADC_SampleTimeADCK3);
    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_ISNS_B, HAL_ADC0_CH_ISNS_B, HAL_ADC0_CMD_ISNS_C, kLPADC_SampleTimeADCK3);
    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_ISNS_C, HAL_ADC0_CH_ISNS_C, HAL_ADC0_CMD_VBUS,   kLPADC_SampleTimeADCK3);

    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_VBUS,   HAL_ADC0_CH_VBUS,   HAL_ADC0_CMD_VSNS_A, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_VSNS_A, HAL_ADC0_CH_VSNS_A, HAL_ADC0_CMD_VSNS_B, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_VSNS_B, HAL_ADC0_CH_VSNS_B, HAL_ADC0_CMD_VSNS_C, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC0, HAL_ADC0_CMD_VSNS_C, HAL_ADC0_CH_VSNS_C, 0U,                  kLPADC_SampleTimeADCK131);

    HalAdcConfigureCommand(ADC1, HAL_ADC1_CMD_THERM_AUX, HAL_ADC1_CH_THERM_AUX, HAL_ADC1_CMD_THERM_A, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC1, HAL_ADC1_CMD_THERM_A,   HAL_ADC1_CH_THERM_A,   HAL_ADC1_CMD_THERM_B, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC1, HAL_ADC1_CMD_THERM_B,   HAL_ADC1_CH_THERM_B,   HAL_ADC1_CMD_THERM_C, kLPADC_SampleTimeADCK131);
    HalAdcConfigureCommand(ADC1, HAL_ADC1_CMD_THERM_C,   HAL_ADC1_CH_THERM_C,   0U,                   kLPADC_SampleTimeADCK131);

    HalAdcConfigureTrigger(ADC0, HAL_ADC_FAST_TRIGGER_ID,    HAL_ADC0_CMD_ISNS_A,     true,  1U);
    HalAdcConfigureTrigger(ADC1, HAL_ADC_THERMAL_TRIGGER_ID, HAL_ADC1_CMD_THERM_AUX, false, 0U);

    HalAdcDrainResultFifo(ADC0);
    HalAdcDrainResultFifo(ADC1);

    LPADC_ClearTriggerStatusFlags(ADC0, kLPADC_Trigger0CompletedFlag);
    LPADC_ClearTriggerStatusFlags(ADC1, kLPADC_Trigger0CompletedFlag);

#if HAL_ADC_INIT_PRINTS
    printk("[HAL][ADC] Init done\n");
#endif
}

bool HalAdc_AcquireHardwareTriggered(HalRawAdc_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    HalAdcConfigureTrigger(ADC0, HAL_ADC_FAST_TRIGGER_ID, HAL_ADC0_CMD_ISNS_A, false, 1U);

    HalAdcDrainResultFifo(ADC0);

    LPADC_ClearTriggerStatusFlags(ADC0, kLPADC_Trigger0CompletedFlag);

    HalAdcConfigureTrigger(ADC0, HAL_ADC_FAST_TRIGGER_ID, HAL_ADC0_CMD_ISNS_A, true, 1U);

    uint32_t timeout = HAL_ADC_FAST_ACQUIRE_TIMEOUT_LOOPS;

    while (timeout-- > 0U)
    {
        if (LPADC_GetConvResultCount(ADC0, 0U) >= 7U)
        {
            break;
        }
    }

    HalAdcConfigureTrigger(ADC0, HAL_ADC_FAST_TRIGGER_ID, HAL_ADC0_CMD_ISNS_A, false, 1U);

    if (timeout == 0U)
    {
        printk("FATAL: ADC0 Fast Loop Timed Out.\n");
        return false;
    }

    const uint32_t rawIa   = ADC0->RESFIFO[0];
    const uint32_t rawIb   = ADC0->RESFIFO[0];
    const uint32_t rawIc   = ADC0->RESFIFO[0];
    const uint32_t rawVbus = ADC0->RESFIFO[0];
    const uint32_t rawVa   = ADC0->RESFIFO[0];
    const uint32_t rawVb   = ADC0->RESFIFO[0];
    const uint32_t rawVc   = ADC0->RESFIFO[0];

    out->phaseA_counts = HalAdcExtractCounts(rawIa);
    out->phaseB_counts = HalAdcExtractCounts(rawIb);
    out->phaseC_counts = HalAdcExtractCounts(rawIc);
    out->vbus_counts = HalAdcExtractCounts(rawVbus);
    out->phaseVa_counts = HalAdcExtractCounts(rawVa);
    out->phaseVb_counts = HalAdcExtractCounts(rawVb);
    out->phaseVc_counts = HalAdcExtractCounts(rawVc);

    LPADC_ClearTriggerStatusFlags(ADC0, kLPADC_Trigger0CompletedFlag);

    return true;
}

bool HalAdc_ReadThermals(HalRawThermal_t *out)
{
    if (out == NULL)
    {
        return false;
    }

    /* Start one four-channel thermal conversion chain. */
    LPADC_DoSoftwareTrigger(ADC1, 1U << HAL_ADC_THERMAL_TRIGGER_ID);

    /* Wait for the complete four-result bundle. */
    uint32_t timeout = HAL_ADC_THERMAL_TIMEOUT_LOOPS;
    while (timeout-- > 0U)
    {
        if (LPADC_GetConvResultCount(ADC1, 0U) >= 4U)
        {
            break;
        }
    }

    if (timeout == 0U)
    {
        return false;
    }

    out->thermAux_counts = HalAdcExtractCounts(ADC1->RESFIFO[0]);
    out->thermPhaseA_counts = HalAdcExtractCounts(ADC1->RESFIFO[0]);
    out->thermPhaseB_counts = HalAdcExtractCounts(ADC1->RESFIFO[0]);
    out->thermPhaseC_counts = HalAdcExtractCounts(ADC1->RESFIFO[0]);

    LPADC_ClearTriggerStatusFlags(ADC1, kLPADC_Trigger0CompletedFlag);

    return true;
}

void HalAdc_PrintDebug(const HalRawAdc_t *raw)
{
    if (raw != NULL)
    {
        printk("ADC RAW | A:%4u  B:%4u  C:%4u  V:%4u\n",
               (unsigned int)raw->phaseA_counts,
               (unsigned int)raw->phaseB_counts,
               (unsigned int)raw->phaseC_counts,
               (unsigned int)raw->vbus_counts);
    }

    printk("[HAL][ADC] ADC0 flags=0x%08X trig=0x%08X count=%u TCTRL0=0x%08X FCTRL0=0x%08X\n",
           (unsigned int)LPADC_GetStatusFlags(ADC0),
           (unsigned int)LPADC_GetTriggerStatusFlags(ADC0),
           (unsigned int)LPADC_GetConvResultCount(ADC0, 0U),
           (unsigned int)ADC0->TCTRL[HAL_ADC_FAST_TRIGGER_ID],
           (unsigned int)ADC0->FCTRL[0]);

    printk("[HAL][ADC] ADC1 flags=0x%08X trig=0x%08X count=%u TCTRL0=0x%08X FCTRL0=0x%08X\n",
           (unsigned int)LPADC_GetStatusFlags(ADC1),
           (unsigned int)LPADC_GetTriggerStatusFlags(ADC1),
           (unsigned int)LPADC_GetConvResultCount(ADC1, 0U),
           (unsigned int)ADC1->TCTRL[HAL_ADC_FAST_TRIGGER_ID],
           (unsigned int)ADC1->FCTRL[0]);
}