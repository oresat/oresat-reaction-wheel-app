#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Config.h"
#include "CurrentPreprocess.h"
#include "Hal.h"
#include "MathUtil.h"

/*=============================================================================
 * TEST: V2 CURRENT PREPROCESS VALIDATION
 *
 * RESPONSIBILITIES:
 * - Manual validation of CurrentPreprocess.c
 * - Raw phase-current readout
 * - Corrected current readout
 * - Common-mode removal diagnostics
 * - Best-2-of-3 reconstruction diagnostics
 * - PWM-duty-based auto-drop behavior visibility
 *
 * OUT OF SCOPE:
 * - Calibration offset correction
 * - Controller behavior
 * - Commutation strategy behavior
 * - Telemetry packet validation
 *
 * TEST FLOW:
 * 1. Initialize HAL.
 * 2. Initialize CurrentPreprocess.
 * 3. Enable centered PWM so the fast sensor path is active.
 * 4. Sweep the previous-PWM command through several duty patterns.
 * 5. Print raw/reconstructed/corrected currents and diagnostic counters.
 *
 * SAFETY / USAGE NOTES:
 * - This test validates preprocessing only.
 * - It intentionally does not apply calibration current offsets.
 * - Run testCalibration.c separately to validate offset correction.
 *===========================================================================*/

/*=============================================================================
 * CONFIG
 *===========================================================================*/

#define PRINT_PERIOD_TICKS          ((uint32_t)(0.20f * FOC_UPDATE_FREQ_HZ))
#define DUTY_PATTERN_DURATION_TICKS ((uint32_t)(3.00f * FOC_UPDATE_FREQ_HZ))

/*=============================================================================
 * TYPES
 *===========================================================================*/

typedef struct
{
    const char *name;
    HalPwmCommand_t pwm;
} DutyPattern_t;

/*=============================================================================
 * HELPERS
 *===========================================================================*/

static const struct gpio_dt_spec g_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

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

static HalPwmCommand_t MakePwmCommand(
    float dutyA,
    float dutyB,
    float dutyC,
    bool floatA,
    bool floatB,
    bool floatC,
    bool enableGateDriver)
{
    return (HalPwmCommand_t){
        .dutyA = dutyA,
        .dutyB = dutyB,
        .dutyC = dutyC,

        .floatA = floatA,
        .floatB = floatB,
        .floatC = floatC,

        .enableGateDriver = enableGateDriver,
    };
}

static const char *DroppedPhaseToString(int droppedPhase)
{
    switch (droppedPhase)
    {
        case CURRENT_PREPROCESS_DROP_NONE: return "NONE";
        case CURRENT_PREPROCESS_DROP_A:    return "A";
        case CURRENT_PREPROCESS_DROP_B:    return "B";
        case CURRENT_PREPROCESS_DROP_C:    return "C";
        default:                           return "UNK";
    }
}

static float CurrentSum(const HalPhaseCurrents_t *currents)
{
    if (currents == NULL)
    {
        return 0.0f;
    }

    return currents->phaseA_A +
           currents->phaseB_A +
           currents->phaseC_A;
}

static float MaxAbsPhaseCurrent(const HalPhaseCurrents_t *currents)
{
    if (currents == NULL)
    {
        return 0.0f;
    }

    float maxAbs_A = MathAbs(currents->phaseA_A);

    if (MathAbs(currents->phaseB_A) > maxAbs_A)
    {
        maxAbs_A = MathAbs(currents->phaseB_A);
    }

    if (MathAbs(currents->phaseC_A) > maxAbs_A)
    {
        maxAbs_A = MathAbs(currents->phaseC_A);
    }

    return maxAbs_A;
}

static void PrintCurrentTriplet(
    const char *label,
    const HalPhaseCurrents_t *currents)
{
    if ((label == NULL) || (currents == NULL))
    {
        return;
    }

    printk("%s A/B/C=% .4f/% .4f/% .4f A | sum=% .4f A | max=% .4f A\n",
           label,
           (double)currents->phaseA_A,
           (double)currents->phaseB_A,
           (double)currents->phaseC_A,
           (double)CurrentSum(currents),
           (double)MaxAbsPhaseCurrent(currents));
}

static void PrintDiagnostics(
    const char *patternName,
    const HalPwmCommand_t *previousPwmCommand,
    const CurrentPreprocessDiagnostics_t *diag)
{
    if ((patternName == NULL) ||
        (previousPwmCommand == NULL) ||
        (diag == NULL))
    {
        return;
    }

    printk("\n------------------------------------------------------\n");
    printk("[CURRENT_PREPROCESS] Pattern: %s\n", patternName);
    printk("------------------------------------------------------\n");

    printk("[PWM] duty=% .3f/% .3f/% .3f | float=%d/%d/%d | gate=%d\n",
           (double)previousPwmCommand->dutyA,
           (double)previousPwmCommand->dutyB,
           (double)previousPwmCommand->dutyC,
           previousPwmCommand->floatA ? 1 : 0,
           previousPwmCommand->floatB ? 1 : 0,
           previousPwmCommand->floatC ? 1 : 0,
           previousPwmCommand->enableGateDriver ? 1 : 0);

    PrintCurrentTriplet("[RAW ]", &diag->raw_A);
    PrintCurrentTriplet("[RECON]", &diag->reconstructed_A);
    PrintCurrentTriplet("[CORR]", &diag->corrected_A);

    printk("[SUMS] before=% .5f A | afterRecon=% .5f A | after=% .5f A | commonMode=% .5f A\n",
           (double)diag->sumBefore_A,
           (double)diag->sumAfterReconstruction_A,
           (double)diag->sumAfter_A,
           (double)diag->commonMode_A);

    printk("[DROP] phase=%s | reconstruct=%u | auto=%u | forced=%u | "
           "A=%u B=%u C=%u | bad=%u | valid=%s\n",
           DroppedPhaseToString(diag->droppedPhase),
           (unsigned int)diag->reconstructCount,
           (unsigned int)diag->autoDropCount,
           (unsigned int)diag->forcedDropCount,
           (unsigned int)diag->dropCountA,
           (unsigned int)diag->dropCountB,
           (unsigned int)diag->dropCountC,
           (unsigned int)diag->badSampleCount,
           diag->sampleValid ? "true" : "false");
}

static void PrintConfigSummary(void)
{
    printk("[CONFIG] common-mode removal: %s\n",
           CURRENT_PREPROCESS_ENABLE_COMMON_MODE_REMOVAL ? "enabled" : "disabled");

    printk("[CONFIG] best-2-of-3: %s\n",
           CURRENT_PREPROCESS_ENABLE_BEST_2_OF_3 ? "enabled" : "disabled");

    printk("[CONFIG] auto shunt selection: %s\n",
           CURRENT_PREPROCESS_ENABLE_AUTO_SHUNT_SELECTION ? "enabled" : "disabled");

    printk("[CONFIG] forced drop phase: %d\n",
           CURRENT_PREPROCESS_FORCED_DROP_PHASE);

    printk("[CONFIG] auto duty edge margin: %.3f\n",
           (double)CURRENT_PREPROCESS_AUTO_DUTY_EDGE_MARGIN);

    printk("[CONFIG] abs max current: %.2f A\n",
           (double)CURRENT_PREPROCESS_ABS_MAX_A);
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
    printk("        TEST: V2 CURRENT PREPROCESS VALIDATION        \n");
    printk("======================================================\n");

    PrintConfigSummary();

    HalInit();
    CurrentPreprocess_Init();

    static const DutyPattern_t kPatterns[] =
    {
        {
            .name = "CENTERED_50_50_50",
            .pwm = {
                .dutyA = 0.50f,
                .dutyB = 0.50f,
                .dutyC = 0.50f,
                .floatA = false,
                .floatB = false,
                .floatC = false,
                .enableGateDriver = true,
            },
        },
        {
            .name = "A_NEAR_LOW_EDGE",
            .pwm = {
                .dutyA = 0.02f,
                .dutyB = 0.50f,
                .dutyC = 0.50f,
                .floatA = false,
                .floatB = false,
                .floatC = false,
                .enableGateDriver = true,
            },
        },
        {
            .name = "B_NEAR_HIGH_EDGE",
            .pwm = {
                .dutyA = 0.50f,
                .dutyB = 0.98f,
                .dutyC = 0.50f,
                .floatA = false,
                .floatB = false,
                .floatC = false,
                .enableGateDriver = true,
            },
        },
        {
            .name = "C_NEAR_LOW_EDGE",
            .pwm = {
                .dutyA = 0.50f,
                .dutyB = 0.50f,
                .dutyC = 0.02f,
                .floatA = false,
                .floatB = false,
                .floatC = false,
                .enableGateDriver = true,
            },
        },
        {
            .name = "ALL_FLOAT_GATE_OFF",
            .pwm = {
                .dutyA = 0.50f,
                .dutyB = 0.50f,
                .dutyC = 0.50f,
                .floatA = true,
                .floatB = true,
                .floatC = true,
                .enableGateDriver = false,
            },
        },
    };

    const uint32_t patternCount =
        (uint32_t)(sizeof(kPatterns) / sizeof(kPatterns[0]));

    uint32_t patternIndex = 0u;
    uint32_t patternTicks = 0u;
    uint32_t printTicks = 0u;

    HalPwmCommand_t previousPwmCommand = kPatterns[patternIndex].pwm;
    HalWritePwm(&previousPwmCommand);

    printk("[TEST] Starting pattern: %s\n", kPatterns[patternIndex].name);

    while (1)
    {
        HalSensorUpdateStatus_t status = HalUpdateSensorCache();

        if (status != HAL_SENSOR_UPDATE_OK)
        {
            printk("[CURRENT_PREPROCESS][FAIL] HalUpdateSensorCache status=%d\n",
                   (int)status);

            HalDumpCriticalRegisters();
            k_msleep(1000);
            continue;
        }

        HalPhaseCurrents_t rawCurrents = HalReadCurrents();

        (void)CurrentPreprocess_Apply(
            &rawCurrents,
            &previousPwmCommand
        );

        patternTicks++;
        printTicks++;

        if (printTicks >= PRINT_PERIOD_TICKS)
        {
            CurrentPreprocessDiagnostics_t diag =
                CurrentPreprocess_GetDiagnostics();

            PrintDiagnostics(
                kPatterns[patternIndex].name,
                &previousPwmCommand,
                &diag
            );

            printTicks = 0u;
            ToggleLed();
        }

        if (patternTicks >= DUTY_PATTERN_DURATION_TICKS)
        {
            patternIndex++;

            if (patternIndex >= patternCount)
            {
                patternIndex = 0u;
            }

            patternTicks = 0u;

            previousPwmCommand = kPatterns[patternIndex].pwm;
            HalWritePwm(&previousPwmCommand);

            printk("\n[TEST] Switching pattern: %s\n",
                   kPatterns[patternIndex].name);
        }
    }

    return 0;
}