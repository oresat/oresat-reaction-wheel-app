#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Calibration.h"
#include "CalibrationStore.h"
#include "Controller.h"
#include "Hal.h"
#include "MathUtil.h"
#include "Config.h"

/*=============================================================================
 * TEST: V2 CALIBRATION ENGINE VALIDATION
 *
 * PURPOSE:
 * - Run the real controller-driven calibration path
 * - Allow HAL/ADC measurements to settle before calibration begins
 * - Validate post-calibration zero-current behavior
 * - Save valid calibration data to persistent storage
 *===========================================================================*/

#define PRINT_PERIOD_TICKS               ((uint32_t)(0.2f * FOC_UPDATE_FREQ_HZ))
#define ADC_WARMUP_SAMPLES               20u
#define POST_IDLE_SETTLE_MS              300u
#define POST_VALIDATE_DURATION_TICKS     ((uint32_t)(3.0f * FOC_UPDATE_FREQ_HZ))

#define ZERO_CURRENT_WARN_ABS_A          0.35f
#define ZERO_CURRENT_WARN_SUM_AVG_A      0.30f
#define ZERO_CURRENT_WARN_SUM_MAX_A      0.60f

static const struct gpio_dt_spec g_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/*=============================================================================
 * LOCAL HELPERS
 *===========================================================================*/

static void ConfigureLed(void)
{
    ConfigureLed();
}

static void ToggleLed(void)
{
    if (gpio_is_ready_dt(&g_led))
    {
        gpio_pin_toggle_dt(&g_led);
    }
}

static void ApplyOffsets(HalPhaseCurrents_t *c, const CalibrationData_t *cal)
{
    if ((c == NULL) || (cal == NULL) || !cal->isValid)
    {
        return;
    }

    float a = cal->offsetAlpha_A;

    float b =
        -0.5f * cal->offsetAlpha_A +
        MATH_SQRT3_OVER_2_F * cal->offsetBeta_A;

    float d =
        -0.5f * cal->offsetAlpha_A -
        MATH_SQRT3_OVER_2_F * cal->offsetBeta_A;

    c->phaseA_A -= a;
    c->phaseB_A -= b;
    c->phaseC_A -= d;
}

static float WrappedDeltaRev(float current, float previous)
{
    float delta = current - previous;

    if (delta > 0.5f)
    {
        delta -= 1.0f;
    }

    if (delta < -0.5f)
    {
        delta += 1.0f;
    }

    return delta;
}

/*=============================================================================
 * VALIDATION STATS
 *===========================================================================*/

typedef struct
{
    float minA_A;
    float maxA_A;
    float minB_A;
    float maxB_A;
    float minC_A;
    float maxC_A;

    float absAccumA_A;
    float absAccumB_A;
    float absAccumC_A;

    float avgAbsA_A;
    float avgAbsB_A;
    float avgAbsC_A;

    float sumAccum_A;
    float avgSum_A;
    float maxAbsSum_A;

    float minMech_rev;
    float maxMech_rev;
    float minElec_rad;
    float maxElec_rad;

    uint32_t sampleCount;
    uint32_t invalidRotorCount;
    bool initialized;
} ValidationStats_t;

static void StatsInit(
    ValidationStats_t *stats,
    const HalPhaseCurrents_t *currents,
    const HalRotorState_t *rotor)
{
    float sum =
        currents->phaseA_A +
        currents->phaseB_A +
        currents->phaseC_A;

    stats->minA_A = currents->phaseA_A;
    stats->maxA_A = currents->phaseA_A;
    stats->minB_A = currents->phaseB_A;
    stats->maxB_A = currents->phaseB_A;
    stats->minC_A = currents->phaseC_A;
    stats->maxC_A = currents->phaseC_A;

    stats->absAccumA_A = MathAbs(currents->phaseA_A);
    stats->absAccumB_A = MathAbs(currents->phaseB_A);
    stats->absAccumC_A = MathAbs(currents->phaseC_A);

    stats->avgAbsA_A = stats->absAccumA_A;
    stats->avgAbsB_A = stats->absAccumB_A;
    stats->avgAbsC_A = stats->absAccumC_A;

    stats->sumAccum_A = sum;
    stats->avgSum_A = sum;
    stats->maxAbsSum_A = MathAbs(sum);

    stats->minMech_rev = rotor->mechanicalAngle_rev;
    stats->maxMech_rev = rotor->mechanicalAngle_rev;
    stats->minElec_rad = rotor->electricalAngle_rad;
    stats->maxElec_rad = rotor->electricalAngle_rad;

    stats->sampleCount = 1u;
    stats->invalidRotorCount = rotor->isValid ? 0u : 1u;
    stats->initialized = true;
}

static void StatsUpdate(
    ValidationStats_t *stats,
    const HalPhaseCurrents_t *currents,
    const HalRotorState_t *rotor)
{
    float sum =
        currents->phaseA_A +
        currents->phaseB_A +
        currents->phaseC_A;

    if (currents->phaseA_A < stats->minA_A) stats->minA_A = currents->phaseA_A;
    if (currents->phaseA_A > stats->maxA_A) stats->maxA_A = currents->phaseA_A;

    if (currents->phaseB_A < stats->minB_A) stats->minB_A = currents->phaseB_A;
    if (currents->phaseB_A > stats->maxB_A) stats->maxB_A = currents->phaseB_A;

    if (currents->phaseC_A < stats->minC_A) stats->minC_A = currents->phaseC_A;
    if (currents->phaseC_A > stats->maxC_A) stats->maxC_A = currents->phaseC_A;

    stats->absAccumA_A += MathAbs(currents->phaseA_A);
    stats->absAccumB_A += MathAbs(currents->phaseB_A);
    stats->absAccumC_A += MathAbs(currents->phaseC_A);

    stats->sumAccum_A += sum;

    if (MathAbs(sum) > stats->maxAbsSum_A)
    {
        stats->maxAbsSum_A = MathAbs(sum);
    }

    if (rotor->mechanicalAngle_rev < stats->minMech_rev)
    {
        stats->minMech_rev = rotor->mechanicalAngle_rev;
    }

    if (rotor->mechanicalAngle_rev > stats->maxMech_rev)
    {
        stats->maxMech_rev = rotor->mechanicalAngle_rev;
    }

    if (rotor->electricalAngle_rad < stats->minElec_rad)
    {
        stats->minElec_rad = rotor->electricalAngle_rad;
    }

    if (rotor->electricalAngle_rad > stats->maxElec_rad)
    {
        stats->maxElec_rad = rotor->electricalAngle_rad;
    }

    if (!rotor->isValid)
    {
        stats->invalidRotorCount++;
    }

    stats->sampleCount++;

    stats->avgAbsA_A =
        stats->absAccumA_A / (float)stats->sampleCount;

    stats->avgAbsB_A =
        stats->absAccumB_A / (float)stats->sampleCount;

    stats->avgAbsC_A =
        stats->absAccumC_A / (float)stats->sampleCount;

    stats->avgSum_A =
        stats->sumAccum_A / (float)stats->sampleCount;
}

/*=============================================================================
 * PRINT HELPERS
 *===========================================================================*/

static void PrintCalibrationResult(
    const CalibrationData_t *calData,
    CalibrationResult_t result,
    const HalThermal_t *thermalStart,
    const HalThermal_t *thermalEnd)
{
    printk("\n======================================================\n");
    printk("                 CALIBRATION COMPLETE                 \n");
    printk("======================================================\n");

    printk("[RESULT] Enum             : %d (%s)\n",
           (int)result,
           Calibration_ResultToString(result));

    printk("[RESULT] Passed           : %s\n",
           (result == CAL_RESULT_PASSED) ? "true" : "false");

    printk("[RESULT] Valid            : %s\n",
           ((calData != NULL) && calData->isValid) ? "true" : "false");

    printk("[RESULT] Controller Fault : %s\n",
           Controller_IsFaulted() ? "true" : "false");

    if ((thermalStart != NULL) && (thermalEnd != NULL))
    {
        printk("[THERM] Start | Aux:% .2f C | PhA:% .2f C | PhB:% .2f C | PhC:% .2f C\n",
               (double)thermalStart->auxTemp_C,
               (double)thermalStart->phaseATemp_C,
               (double)thermalStart->phaseBTemp_C,
               (double)thermalStart->phaseCTemp_C);

        printk("[THERM] End   | Aux:% .2f C | PhA:% .2f C | PhB:% .2f C | PhC:% .2f C\n",
               (double)thermalEnd->auxTemp_C,
               (double)thermalEnd->phaseATemp_C,
               (double)thermalEnd->phaseBTemp_C,
               (double)thermalEnd->phaseCTemp_C);
    }

    if (calData != NULL)
    {
        printk("[RESULT] Offset Alpha     : % .6f A\n",
               (double)calData->offsetAlpha_A);

        printk("[RESULT] Offset Beta      : % .6f A\n",
               (double)calData->offsetBeta_A);

        printk("[RESULT] Elec Offset      : % .6f rad\n",
               (double)calData->encoderElectricalOffset_rad);

        printk("[RESULT] Phase R          : % .6f Ohm\n",
               (double)calData->phaseResistance_Ohm);

        printk("[RESULT] Phase L          : % .9e H\n",
               (double)calData->phaseInductance_H);

        printk("[RESULT] Vbus             : % .3f V\n",
               (double)calData->busVoltage_V);
    }

    printk("======================================================\n");
}

static void PrintPersistedCalibration(void)
{
    const CalibrationData_t *data = Calibration_GetData();

    printk("\n======================================================\n");
    printk("              PERSISTED CALIBRATION DATA              \n");
    printk("======================================================\n");

    printk("[STORE] Valid        : %s\n",
           Calibration_HasValidData() ? "YES" : "NO");

    printk("[STORE] Offset Alpha : %.6f A\n",
           (double)Calibration_GetCurrentOffsetAlpha());

    printk("[STORE] Offset Beta  : %.6f A\n",
           (double)Calibration_GetCurrentOffsetBeta());

    if (data != NULL)
    {
        printk("[STORE] Encoder Off  : %.6f rad\n",
               (double)data->encoderElectricalOffset_rad);
    }

    printk("[STORE] Phase R      : %.6f Ohm\n",
           (double)Calibration_GetPhaseResistance());

    printk("[STORE] Phase L      : %.9e H\n",
           (double)Calibration_GetPhaseInductance());

    printk("======================================================\n");
}

/*=============================================================================
 * TEST PHASES
 *===========================================================================*/

static int RunCalibrationPhase(
    HalPhaseCurrents_t *lastCalCurrents,
    HalPower_t *lastCalPower,
    HalThermal_t *lastCalThermal)
{
    uint32_t ticks = 0u;
    uint32_t printTicks = 0u;

    while (Calibration_IsRunning())
    {
        Controller_Update16kHz();

        ticks++;
        printTicks++;

        if (Controller_IsFaulted())
        {
            printk("[FAIL] Controller faulted during calibration.\n");
            return -1;
        }

        if (lastCalCurrents != NULL)
        {
            *lastCalCurrents = HalReadCurrents();
        }

        if (lastCalPower != NULL)
        {
            *lastCalPower = HalReadPower();
        }

        if (lastCalThermal != NULL)
        {
            *lastCalThermal = HalReadThermal();
        }

        if (printTicks >= PRINT_PERIOD_TICKS)
        {
            float elapsed_s =
                (float)ticks / FOC_UPDATE_FREQ_HZ;

            HalPhaseCurrents_t currents = HalReadCurrents();
            HalPower_t power = HalReadPower();
            HalRotorState_t rotor = HalReadRotor();

            printk("[CAL] t=%.2f s | Ia:% .4f | Ib:% .4f | Ic:% .4f A | "
                   "Vbus:% .3f V | rotorValid=%s\n",
                   (double)elapsed_s,
                   (double)currents.phaseA_A,
                   (double)currents.phaseB_A,
                   (double)currents.phaseC_A,
                   (double)power.busVoltage_V,
                   rotor.isValid ? "true" : "false");

            printTicks = 0u;
            ToggleLed();
        }
    }

    return 0;
}

static int RunPostCalibrationValidation(const CalibrationData_t *calData)
{
    ValidationStats_t stats = {0};

    uint32_t ticks = 0u;
    uint32_t printTicks = 0u;

    bool haveMechRef = false;
    float mechRef_rev = 0.0f;
    float maxAbsWrappedMechDelta_rev = 0.0f;

    printk("\n======================================================\n");
    printk("             POST-CALIBRATION VALIDATION              \n");
    printk("======================================================\n");

    while (ticks < POST_VALIDATE_DURATION_TICKS)
    {
        Controller_Update16kHz();

        ticks++;
        printTicks++;

        if (Controller_IsFaulted())
        {
            printk("[FAIL] Controller faulted during post-calibration validation.\n");
            return -2;
        }

        HalPhaseCurrents_t currents = HalReadCurrents();
        HalRotorState_t rotor = HalReadRotor();

        ApplyOffsets(&currents, calData);

        if (!stats.initialized)
        {
            StatsInit(&stats, &currents, &rotor);
        }
        else
        {
            StatsUpdate(&stats, &currents, &rotor);
        }

        if (!haveMechRef)
        {
            mechRef_rev = rotor.mechanicalAngle_rev;
            haveMechRef = true;
        }
        else
        {
            float mechDelta_rev =
                WrappedDeltaRev(rotor.mechanicalAngle_rev, mechRef_rev);

            if (MathAbs(mechDelta_rev) > maxAbsWrappedMechDelta_rev)
            {
                maxAbsWrappedMechDelta_rev = MathAbs(mechDelta_rev);
            }
        }

        if (printTicks >= PRINT_PERIOD_TICKS)
        {
            float currentSum_A =
                currents.phaseA_A +
                currents.phaseB_A +
                currents.phaseC_A;

            printk("[POST] Ia:% .4f | Ib:% .4f | Ic:% .4f A | "
                   "Sum:% .4f A | Mech:% .4f rev | Elec:% .4f rad\n",
                   (double)currents.phaseA_A,
                   (double)currents.phaseB_A,
                   (double)currents.phaseC_A,
                   (double)currentSum_A,
                   (double)rotor.mechanicalAngle_rev,
                   (double)rotor.electricalAngle_rad);

            printTicks = 0u;
            ToggleLed();
        }
    }

    printk("[STATS] Avg |Ia|               : % .5f A\n",
           (double)stats.avgAbsA_A);

    printk("[STATS] Avg |Ib|               : % .5f A\n",
           (double)stats.avgAbsB_A);

    printk("[STATS] Avg |Ic|               : % .5f A\n",
           (double)stats.avgAbsC_A);

    printk("[STATS] Avg sum(Iabc)          : % .5f A\n",
           (double)stats.avgSum_A);

    printk("[STATS] Max |sum(Iabc)|        : % .5f A\n",
           (double)stats.maxAbsSum_A);

    printk("[STATS] Max wrapped mech drift : % .6f rev\n",
           (double)maxAbsWrappedMechDelta_rev);

    printk("[STATS] Invalid rotor samples  : %u\n",
           (unsigned int)stats.invalidRotorCount);

    if ((stats.avgAbsA_A <= ZERO_CURRENT_WARN_ABS_A) &&
        (stats.avgAbsB_A <= ZERO_CURRENT_WARN_ABS_A) &&
        (stats.avgAbsC_A <= ZERO_CURRENT_WARN_ABS_A) &&
        (MathAbs(stats.avgSum_A) <= ZERO_CURRENT_WARN_SUM_AVG_A) &&
        (stats.maxAbsSum_A <= ZERO_CURRENT_WARN_SUM_MAX_A))
    {
        printk("[PASS] Post-calibration zero-current checks look reasonable.\n");
    }
    else
    {
        printk("[WARN] Post-calibration zero-current checks are outside thresholds.\n");
    }

    return 0;
}

/*=============================================================================
 * MAIN
 *===========================================================================*/

int main(void)
{
    k_thread_priority_set(k_current_get(), K_PRIO_COOP(0));

    if (gpio_is_ready_dt(&g_led))
    {
        gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_ACTIVE);
    }

    printk("\n======================================================\n");
    printk("           TEST: V2 CALIBRATION ENGINE VALIDATION     \n");
    printk("======================================================\n");

    Controller_Init();

    /*
     * Start from a clean persisted state so this run proves the calibration
     * engine can generate and save a fresh V2 calibration record.
     */
    printk("[STORE] Clearing persisted calibration before test.\n");
    CalibrationStore_Clear();

    HalThermal_t thermalStart = HalReadThermal();
    HalThermal_t thermalEnd = thermalStart;

    HalPhaseCurrents_t lastCalCurrents = {0};
    HalPower_t lastCalPower = {0};

    printk("[CAL_TEST] Starting calibration.\n");

    Calibration_Begin();
    Controller_SetControlMode(CTRL_MODE_CALIBRATION);

    int calStatus =
        RunCalibrationPhase(
            &lastCalCurrents,
            &lastCalPower,
            &thermalEnd
        );

    if (calStatus != 0)
    {
        Controller_SetControlMode(CTRL_MODE_IDLE);
        return calStatus;
    }

    const CalibrationData_t *calData = Calibration_GetData();
    CalibrationResult_t calResult = Calibration_GetResult();

    PrintCalibrationResult(
        calData,
        calResult,
        &thermalStart,
        &thermalEnd
    );

    if ((calResult == CAL_RESULT_PASSED) &&
        (calData != NULL) &&
        calData->isValid &&
        Calibration_HasValidData())
    {
        if (Calibration_SavePersisted())
        {
            printk("[STORE] Calibration save: PASS\n");
        }
        else
        {
            printk("[STORE] Calibration save: FAIL\n");
        }
    }
    else
    {
        printk("[STORE] Calibration not saved because result is invalid.\n");
    }

    PrintPersistedCalibration();

    Controller_SetControlMode(CTRL_MODE_IDLE);
    k_msleep(POST_IDLE_SETTLE_MS);

    int postStatus = RunPostCalibrationValidation(calData);
    if (postStatus != 0)
    {
        return postStatus;
    }

    printk("\n======================================================\n");
    printk("              CALIBRATION TEST COMPLETE               \n");
    printk("======================================================\n");

    while (1)
    {
        ToggleLed();
        k_msleep(500);
    }

    return 0;
}