#include "Hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>

#include "Config.h"
#include "Encoder.h"
#include "MathUtil.h"

/*==============================================================================
 * Hal_Zephyr.c
 *
 * Top-level hardware abstraction for the reaction-wheel electronics.
 *
 * This module coordinates low-level PWM, INPUTMUX, ADC, SPI encoder, and
 * thermistor services, then publishes coherent cached state through the HAL
 * accessors. Controller, commutation, calibration sequencing, and estimator
 * policy remain outside this file.
 *
 * Sensor-cache update order is intentionally:
 *   1. Acquire the PWM-synchronous ADC bundle.
 *   2. Convert ADC counts into engineering units.
 *   3. Acquire and update rotor state.
 *   4. Acquire and update thermal state.
 *
 * Rotor timing reflects the encoder sample cadence rather than an assumed
 * 16 kHz controller interval.
 *============================================================================*/

/*==============================================================================
 * LOCAL PRINT CONTROL
 *============================================================================*/
#define HAL_ZEPHYR_INIT_PRINTS    0
#define HAL_ZEPHYR_WARN_PRINTS    0

/*==============================================================================
 * EXTERNAL ACQUISITION HOOKS
 *============================================================================*/

extern bool HalAdc_ReadThermals(HalRawThermal_t *out);

/*==============================================================================
 * DEVICE HANDLES
 *============================================================================*/

/*
 * Zephyr device handles used by the top-level HAL.
 *
 * PWM mapping note:
 * - Phase C -> flexpwm1_pwm1
 * - Phase B -> flexpwm1_pwm2
 * - Phase A -> flexpwm1_pwm3
 */

static const struct device *g_pwmPhaseC = DEVICE_DT_GET(DT_NODELABEL(flexpwm1_pwm1));
static const struct device *g_pwmPhaseB = DEVICE_DT_GET(DT_NODELABEL(flexpwm1_pwm2));
static const struct device *g_pwmPhaseA = DEVICE_DT_GET(DT_NODELABEL(flexpwm1_pwm3));

static const struct spi_dt_spec g_ma732Spi =
    SPI_DT_SPEC_GET(DT_NODELABEL(encoder), SPI_WORD_SET(16) | SPI_TRANSFER_MSB, 0);

/*==============================================================================
 * CACHED STATE
 *============================================================================*/

static HalRawAdc_t g_rawAdc = {
    .phaseA_counts = ADC_ZERO_CURRENT_COUNTS,
    .phaseB_counts = ADC_ZERO_CURRENT_COUNTS,
    .phaseC_counts = ADC_ZERO_CURRENT_COUNTS,
    .vbus_counts   = 0u,
    .phaseVa_counts = 0u,
    .phaseVb_counts = 0u,
    .phaseVc_counts = 0u
};

static HalRawThermal_t g_rawThermal = {0};

static HalPhaseCurrents_t g_phaseCurrents = {0};
static HalPhaseVoltages_t g_phaseVoltages = {0};

static HalPower_t g_power = {
    .busVoltage_V = 0.0f,
    .busCurrent_A = 0.0f
};

static HalRotorState_t g_rotor = {
    .electricalAngle_rad = 0.0f,
    .electricalVelocity_radPerSec = 0.0f,
    .mechanicalAngle_rev = 0.0f,
    .mechanicalVelocity_revPerSec = 0.0f,
    .electricalSector = 1u,
    .isValid = false
};

static HalThermal_t g_thermal = {
    .auxTemp_C = 0.0f,
    .phaseATemp_C = 0.0f,
    .phaseBTemp_C = 0.0f,
    .phaseCTemp_C = 0.0f
};

static bool g_halInitialized = false;

/* Encoder debug and timing state (debug-only, not control state). */
static float g_rotorDtMin_s = 0.0f;
static float g_rotorDtMax_s = 0.0f;
static float g_rotorDtSum_s = 0.0f;
static uint32_t g_rotorDtSampleCount = 0u;
static uint32_t g_rotorDtOutlierCount = 0u;
static uint32_t g_rotorDtPrintDivider = 0u;
static uint32_t g_rotorRawPrintDivider = 0u;
static uint16_t g_rotorLastRawCounts = 0u;
static bool g_haveLastRawCounts = false;

static volatile uint16_t g_encDbgRaw = 0u;
static volatile int32_t g_encDbgRawDelta = 0;
static volatile uint32_t g_encDbgDtUs = 0u;

/*==============================================================================
 * PRIVATE HELPERS
 *============================================================================*/

static uint8_t HalElectricalAngleToSector(float electricalAngle_rad)
{
    float angle_rad = MathWrapPi(electricalAngle_rad);
    if (angle_rad < 0.0f)
    {
        angle_rad += MATH_TWO_PI_F;
    }

    uint8_t sector = (uint8_t)(angle_rad / (MATH_TWO_PI_F / 6.0f)) + 1u;
    if (sector < 1u)
    {
        sector = 1u;
    }
    else if (sector > 6u)
    {
        sector = 6u;
    }

    return sector;
}

static void HalResetCachedState(void)
{
    g_rawAdc.phaseA_counts = ADC_ZERO_CURRENT_COUNTS;
    g_rawAdc.phaseB_counts = ADC_ZERO_CURRENT_COUNTS;
    g_rawAdc.phaseC_counts = ADC_ZERO_CURRENT_COUNTS;
    g_rawAdc.vbus_counts   = 0u;
    g_rawAdc.phaseVa_counts = 0u;
    g_rawAdc.phaseVb_counts = 0u;
    g_rawAdc.phaseVc_counts = 0u;

    g_phaseCurrents = (HalPhaseCurrents_t){0};
    g_phaseVoltages = (HalPhaseVoltages_t){0};
    g_power = (HalPower_t){0};
    g_thermal = (HalThermal_t){0};

    g_rotor.electricalAngle_rad = 0.0f;
    g_rotor.electricalVelocity_radPerSec = 0.0f;
    g_rotor.mechanicalAngle_rev = 0.0f;
    g_rotor.mechanicalVelocity_revPerSec = 0.0f;
    g_rotor.electricalSector = 1u;
    g_rotor.isValid = false;

    g_rotorDtMin_s = 0.0f;
    g_rotorDtMax_s = 0.0f;
    g_rotorDtSum_s = 0.0f;
    g_rotorDtSampleCount = 0u;
    g_rotorDtOutlierCount = 0u;
    g_rotorDtPrintDivider = 0u;
    g_rotorRawPrintDivider = 0u;
    g_rotorLastRawCounts = 0u;
    g_haveLastRawCounts = false;
}

static bool HalRequiredDevicesReady(void)
{
    return
        device_is_ready(g_pwmPhaseA) &&
        device_is_ready(g_pwmPhaseB) &&
        device_is_ready(g_pwmPhaseC) &&
        spi_is_ready_dt(&g_ma732Spi);
}

static void HalUpdateDerivedAdcTelemetry(void)
{
    /* Convert Phase Currents (Amps) */
    g_phaseCurrents.phaseA_A = ((float)g_rawAdc.phaseA_counts - (float)ADC_ZERO_CURRENT_COUNTS) * ADC_COUNTS_TO_AMPS;
    g_phaseCurrents.phaseB_A = ((float)g_rawAdc.phaseB_counts - (float)ADC_ZERO_CURRENT_COUNTS) * ADC_COUNTS_TO_AMPS;
    g_phaseCurrents.phaseC_A = ((float)g_rawAdc.phaseC_counts - (float)ADC_ZERO_CURRENT_COUNTS) * ADC_COUNTS_TO_AMPS;

    /* Convert DC Bus (Volts) */
    g_power.busVoltage_V = (float)g_rawAdc.vbus_counts * ADC_COUNTS_TO_VBUS_VOLTS;
    g_power.busCurrent_A = 0.0f;

    /* Convert Phase Voltages (Volts) */
    g_phaseVoltages.phaseA_V = (float)g_rawAdc.phaseVa_counts * ADC_COUNTS_TO_PHASE_VOLTS;
    g_phaseVoltages.phaseB_V = (float)g_rawAdc.phaseVb_counts * ADC_COUNTS_TO_PHASE_VOLTS;
    g_phaseVoltages.phaseC_V = (float)g_rawAdc.phaseVc_counts * ADC_COUNTS_TO_PHASE_VOLTS;
}

static void HalInvalidateRotorCache(void)
{
    g_rotor.electricalAngle_rad = 0.0f;
    g_rotor.electricalVelocity_radPerSec = 0.0f;
    g_rotor.mechanicalAngle_rev = 0.0f;
    g_rotor.mechanicalVelocity_revPerSec = 0.0f;
    g_rotor.electricalSector = 1u;
    g_rotor.isValid = false;
}


/* Wakes the Zephyr FlexPWM device clocks before direct register setup. */
static void HalWakePwmFunctionalClocks(void)
{
    (void)pwm_set_cycles(g_pwmPhaseA, 0u, 100u, 1u, 0u);
    (void)pwm_set_cycles(g_pwmPhaseB, 0u, 100u, 1u, 0u);
    (void)pwm_set_cycles(g_pwmPhaseC, 0u, 100u, 1u, 0u);
}

/* Copies estimator outputs into the public HAL rotor cache. */
static void HalPublishRotorState(void)
{
    const EncoderMechanicalState_t mechanical =
        EncoderGetMechanicalState();

    const EncoderElectricalState_t electrical =
        EncoderGetElectricalState();

    g_rotor.mechanicalAngle_rev = mechanical.position_rev;
    g_rotor.mechanicalVelocity_revPerSec =
        mechanical.velocity_revPerSec;

    g_rotor.electricalAngle_rad = electrical.angle_rad;
    g_rotor.electricalVelocity_radPerSec =
        electrical.velocity_radPerSec;

    g_rotor.electricalSector =
        HalElectricalAngleToSector(electrical.angle_rad);

    g_rotor.isValid = true;
}

/*==============================================================================
 * ROTOR CACHE UPDATE
 *============================================================================*/

void HalUpdateRotorCache(void)
{
    uint16_t txData = 0x0000u;
    uint16_t rxData = 0x0000u;

    const struct spi_buf txBuffer = {
        .buf = &txData,
        .len = sizeof(txData)
    };

    const struct spi_buf_set txSet = {
        .buffers = &txBuffer,
        .count = 1
    };

    const struct spi_buf rxBuffer = {
        .buf = &rxData,
        .len = sizeof(rxData)
    };

    const struct spi_buf_set rxSet = {
        .buffers = &rxBuffer,
        .count = 1
    };

    if (spi_transceive_dt(&g_ma732Spi, &txSet, &rxSet) != 0)
    {
        HalInvalidateRotorCache();
        g_haveLastRawCounts = false;
        return;
    }

    uint16_t rawCounts = (rxData >> 2) & 0x3FFFu;

    g_rotorLastRawCounts = rawCounts;
    g_haveLastRawCounts = true;

    static uint32_t lastCycles = 0u;
    static bool haveLastCycles = false;
    static uint16_t lastRawCounts = 0u;

    float dt_s = FOC_UPDATE_PERIOD_S;

    if (haveLastCycles)
    {
        uint32_t nowCycles = k_cycle_get_32();
        uint32_t deltaCycles = nowCycles - lastCycles;

        dt_s = (float)deltaCycles /
               (float)sys_clock_hw_cycles_per_sec();

        int32_t rawDelta =
            (int32_t)rawCounts - (int32_t)lastRawCounts;

        if (rawDelta > 8192)
        {
            rawDelta -= 16384;
        }
        else if (rawDelta < -8192)
        {
            rawDelta += 16384;
        }

        /*
         * Store only. Do not printk here.
         * Print this later from the low-rate direct-ramp test loop using
         * HalPrintEncoderDebugSnapshot().
         */
        g_encDbgRaw = rawCounts;
        g_encDbgRawDelta = rawDelta;
        g_encDbgDtUs = (uint32_t)(dt_s * 1000000.0f);

        EncoderUpdate(rawCounts, dt_s);

        lastCycles = nowCycles;
        lastRawCounts = rawCounts;
    }
    else
    {
        uint32_t nowCycles = k_cycle_get_32();

        haveLastCycles = true;
        lastCycles = nowCycles;
        lastRawCounts = rawCounts;

        g_encDbgRaw = rawCounts;
        g_encDbgRawDelta = 0;
        g_encDbgDtUs = (uint32_t)(FOC_UPDATE_PERIOD_S * 1000000.0f);

        EncoderUpdate(rawCounts, FOC_UPDATE_PERIOD_S);
    }

    HalPublishRotorState();
}

/*==============================================================================
 * THERMAL ACQUISITION
 *============================================================================*/

static float HalThermistorCountsToTempC(uint16_t adcCounts)
{
    const float thermistorVoltage_V = (THERMISTOR_ADC_REFERENCE_VOLTAGE_V * (float)adcCounts) / 4095.0f;

    if (thermistorVoltage_V <= THERMISTOR_SHORT_GUARD_V)
    {
        return 150.0f;
    }
    if (thermistorVoltage_V >=
        (THERMISTOR_ADC_REFERENCE_VOLTAGE_V - THERMISTOR_OPEN_GUARD_V))
    {
        return -273.15f;
    }

    /* V2 Topology: 3.3k Pull-Up on Top, NTC on Bottom */
    const float thermistorResistance_Ohm =
        THERMISTOR_PULLUP_RESISTANCE_OHM /
        ((THERMISTOR_ADC_REFERENCE_VOLTAGE_V / thermistorVoltage_V) - 1.0f);

    const float inverseTemperature_K =
        (1.0f / THERMISTOR_T0_K) +
        (1.0f / THERMISTOR_BETA_K) *
            logf(thermistorResistance_Ohm / THERMISTOR_R0_OHM);

    return (1.0f / inverseTemperature_K) - 273.15f;
}

static void HalUpdateThermalCache(void)
{
    if (HalAdc_ReadThermals(&g_rawThermal))
    {
        g_thermal.auxTemp_C    = HalThermistorCountsToTempC(g_rawThermal.thermAux_counts);
        g_thermal.phaseATemp_C = HalThermistorCountsToTempC(g_rawThermal.thermPhaseA_counts);
        g_thermal.phaseBTemp_C = HalThermistorCountsToTempC(g_rawThermal.thermPhaseB_counts);
        g_thermal.phaseCTemp_C = HalThermistorCountsToTempC(g_rawThermal.thermPhaseC_counts);
    }
}

/*==============================================================================
 * PUBLIC HAL API
 *============================================================================*/

void HalInit(void)
{
    g_halInitialized = false;
    HalResetCachedState();

    if (!HalRequiredDevicesReady())
    {
        return;
    }

    HalWakePwmFunctionalClocks();

    HalPwm_Init();
    HalInputMux_Init();
    HalAdc_Init();

    EncoderInit();
    EncoderSetDirection(ENCODER_DIRECTION_SIGN);
    HalInvalidateRotorCache();

    HalPwm_DisableAllOutputs();
    g_halInitialized = true;
}

HalSensorUpdateStatus_t HalUpdateSensorCache(void)
{
    HalRawAdc_t raw = {0};

    if (!g_halInitialized)
    {
        return HAL_SENSOR_UPDATE_NOT_READY;
    }

    if (!HalAdc_AcquireHardwareTriggered(&raw))
    {
        return HAL_SENSOR_UPDATE_TIMEOUT;
    }

    g_rawAdc = raw;
    HalUpdateDerivedAdcTelemetry();
    HalUpdateRotorCache();
    HalUpdateThermalCache();

    return HAL_SENSOR_UPDATE_OK;
}

HalPhaseCurrents_t HalReadCurrents(void)
{
    return g_phaseCurrents;
}
HalPhaseVoltages_t HalReadPhaseVoltages(void)
{
    return g_phaseVoltages;
}
HalPower_t HalReadPower(void)
{
    return g_power;
}
HalRotorState_t HalReadRotor(void)
{
    return g_rotor;
}
HalThermal_t HalReadThermal(void)
{
    return g_thermal;
}

void HalPrintEncoderDebugSnapshot(void)
{
    printk("[ENC_DBG] raw=%u dRaw=%d dt_us=%u\n",
           (unsigned int)g_encDbgRaw,
           (int)g_encDbgRawDelta,
           (unsigned int)g_encDbgDtUs);
}

/*==============================================================================
 * PWM OUTPUT PATH
 *============================================================================*/

void HalWritePwm(const HalPwmCommand_t *cmd)
{
    if (!g_halInitialized || cmd == NULL)
    {
        HalPwm_DisableAllOutputs();
        return;
    }
    HalPwm_WriteCommand(cmd);
}

/*==============================================================================
 * HARDWARE FAULT STUBS & DEBUG
 *============================================================================*/

bool HalHasHardwareFault(void)
{
    return false;
}
void HalClearHardwareFault(void)
{
}
void HalPrintAdcDebug(void)
{
}
void HalDumpSm1TriggerRegisters(void)
{
    HalPwm_PrintSm1TriggerRegisters();
}

void HalDumpCriticalRegisters(void)
{
    printk("\n======================================================\n");
    printk("                HAL CACHED STATE DUMP                 \n");
    printk("======================================================\n\n");

    printk("Ia/Ib/Ic     : % .3f  % .3f  % .3f A\n", 
           (double)g_phaseCurrents.phaseA_A, (double)g_phaseCurrents.phaseB_A, (double)g_phaseCurrents.phaseC_A);
    printk("Va/Vb/Vc     : % .3f  % .3f  % .3f V\n", 
           (double)g_phaseVoltages.phaseA_V, (double)g_phaseVoltages.phaseB_V, (double)g_phaseVoltages.phaseC_V);
    printk("Vbus         : % .3f V\n", (double)g_power.busVoltage_V);
    
    printk("Temps (C)    : Aux:%.1f | PhA:%.1f | PhB:%.1f | PhC:%.1f\n", 
           (double)g_thermal.auxTemp_C, (double)g_thermal.phaseATemp_C, 
           (double)g_thermal.phaseBTemp_C, (double)g_thermal.phaseCTemp_C);
           
    printk("Elec Vel     : % .4f rad/s\n", (double)g_rotor.electricalVelocity_radPerSec);
    printk("Mech Vel     : % .4f rev/s\n", (double)g_rotor.mechanicalVelocity_revPerSec);
    printk("======================================================\n\n");
}

void HalPrintRegisterState(void)
{
    HalDumpCriticalRegisters();
}