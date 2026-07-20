#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Calibration.h"
#include "Commutation_Foc.h"
#include "Controller.h"
#include "Hal.h"
#include "MathUtil.h"
#include "Config.h"

/*=============================================================================
 * TEST: V2 DIRECT CLOSED-LOOP RAMP
 *
 * Owns:
 * - Bring-up validation of direct closed-loop velocity control
 * - Commutation-mode comparison using the same speed profile
 * - Basic current, voltage, speed, and FOC diagnostic printing
 *
 * Does not own:
 * - Open-loop align
 * - Open-loop pull-in
 * - Stabilize / handoff behavior
 * - Thesis data capture
 * - Telemetry packet validation
 *
 * Flow:
 * 1. Initialize controller/HAL.
 * 2. Require valid persisted calibration.
 * 3. Select commutation mode.
 * 4. Enter velocity mode directly.
 * 5. Ramp command from 0 RPM to target RPM.
 * 6. Hold.
 * 7. Ramp back down.
 * 8. Idle.
 *===========================================================================*/

/*=============================================================================
 * TEST CONFIG
 *===========================================================================*/

#define TEST_COMMUTATION_MODE                  COMM_MODE_FOC

#define PRINT_PERIOD_TICKS                    ((uint32_t)(0.25f * FOC_UPDATE_FREQ_HZ))

#define RAMP_UP_DURATION_TICKS                ((uint32_t)(20.0f * FOC_UPDATE_FREQ_HZ))
#define HOLD_DURATION_TICKS                   ((uint32_t)(5.0f * FOC_UPDATE_FREQ_HZ))
#define RAMP_DOWN_DURATION_TICKS              ((uint32_t)(5.0f * FOC_UPDATE_FREQ_HZ))

#define TARGET_SPEED_RPM                      10000.0f
#define TARGET_SPEED_REV_PER_S                (TARGET_SPEED_RPM / 60.0f)

#define WARN_MAX_ABS_PHASE_CURRENT_A          8.5f
#define WARN_MAX_ABS_CURRENT_SUM_A            5.0f

#define CURRENT_WARN_PERSIST_TICKS  80u   /* 5 ms at 16 kHz */
static uint32_t highCurrentTicks = 0u;

/*=============================================================================
 * STATE
 *===========================================================================*/

typedef enum
{
    TEST_STATE_RAMP_UP = 0,
    TEST_STATE_HOLD,
    TEST_STATE_RAMP_DOWN,
    TEST_STATE_COMPLETE,
    TEST_STATE_FAULT
} TestState_t;

/*=============================================================================
 * HELPERS
 *===========================================================================*/

static float RampLinear(float start, float end, float frac)
{
    if (frac < 0.0f)
    {
        frac = 0.0f;
    }

    if (frac > 1.0f)
    {
        frac = 1.0f;
    }

    return start + ((end - start) * frac);
}

static void ApplyOffsets(
    HalPhaseCurrents_t *currents,
    const CalibrationData_t *cal)
{

    float offsetA_A = cal->offsetAlpha_A;

    float offsetB_A =
        -0.5f * cal->offsetAlpha_A +
        MATH_SQRT3_OVER_2_F * cal->offsetBeta_A;

    float offsetC_A =
        -0.5f * cal->offsetAlpha_A -
        MATH_SQRT3_OVER_2_F * cal->offsetBeta_A;

    currents->phaseA_A -= offsetA_A;
    currents->phaseB_A -= offsetB_A;
    currents->phaseC_A -= offsetC_A;
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

static void EnterFault(
    TestState_t *state,
    const struct gpio_dt_spec *led,
    const char *reason)
{

    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_IDLE);

    if (state != NULL)
    {
        *state = TEST_STATE_FAULT;
    }

    if ((led != NULL) && gpio_is_ready_dt(led))
    {
        gpio_pin_set_dt(led, 1);
    }
}

static void PrintFocDiagnosticsIfActive(void)
{
    if (TEST_COMMUTATION_MODE != COMM_MODE_FOC)
    {
        return;
    }
}

/*=============================================================================
 * MAIN
 *===========================================================================*/

int main(void)
{
    k_thread_priority_set(k_current_get(), K_PRIO_COOP(0));

    const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

    if (gpio_is_ready_dt(&led))
    {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

    Controller_Init();

    if (!Calibration_HasValidData())
    {

        Controller_SetControlMode(CTRL_MODE_IDLE);

        while (1)
        {
            if (gpio_is_ready_dt(&led))
            {
                gpio_pin_toggle_dt(&led);
            }

            k_msleep(500);
        }
    }

    Controller_ClearFaults();
    Controller_SetCommutationMode(TEST_COMMUTATION_MODE);
    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);

    TestState_t state = TEST_STATE_RAMP_UP;
    uint32_t stateTicks = 0u;
    uint32_t printTicks = 0u;
    float command_revPerSec = 0.0f;

    while (1)
    {
        Controller_Update16kHz();

        stateTicks++;
        printTicks++;

        if ((state != TEST_STATE_FAULT) &&
            (state != TEST_STATE_COMPLETE) &&
            Controller_IsFaulted())
        {
            EnterFault(&state, &led, "Controller faulted.");
        }

        switch (state)
        {
            case TEST_STATE_RAMP_UP:
            {
                float frac =
                    (float)stateTicks /
                    (float)RAMP_UP_DURATION_TICKS;

                command_revPerSec =
                    RampLinear(
                        0.0f,
                        TARGET_SPEED_REV_PER_S,
                        frac
                    );

                Controller_SetVelocity(command_revPerSec);

                if (stateTicks >= RAMP_UP_DURATION_TICKS)
                {
                    state = TEST_STATE_HOLD;
                    stateTicks = 0u;
                    command_revPerSec = TARGET_SPEED_REV_PER_S;
                    Controller_SetVelocity(command_revPerSec);
                }
            } break;

            case TEST_STATE_HOLD:
            {
                command_revPerSec = TARGET_SPEED_REV_PER_S;
                Controller_SetVelocity(command_revPerSec);

                if (stateTicks >= HOLD_DURATION_TICKS)
                {
                    state = TEST_STATE_RAMP_DOWN;
                    stateTicks = 0u;
                }
            } break;

            case TEST_STATE_RAMP_DOWN:
            {
                float frac =
                    (float)stateTicks /
                    (float)RAMP_DOWN_DURATION_TICKS;

                command_revPerSec =
                    RampLinear(
                        TARGET_SPEED_REV_PER_S,
                        0.0f,
                        frac
                    );

                Controller_SetVelocity(command_revPerSec);

                if (stateTicks >= RAMP_DOWN_DURATION_TICKS)
                {
                    state = TEST_STATE_COMPLETE;
                    stateTicks = 0u;
                    command_revPerSec = 0.0f;
                    Controller_SetVelocity(0.0f);
                    Controller_SetControlMode(CTRL_MODE_IDLE);
                }
            } break;

            case TEST_STATE_COMPLETE:
            {
                Controller_SetVelocity(0.0f);
                Controller_SetControlMode(CTRL_MODE_IDLE);
            } break;

            case TEST_STATE_FAULT:
            default:
            {
                Controller_SetVelocity(0.0f);
                Controller_SetControlMode(CTRL_MODE_IDLE);

                if ((printTicks >= PRINT_PERIOD_TICKS) &&
                    gpio_is_ready_dt(&led))
                {
                    gpio_pin_toggle_dt(&led);
                    printTicks = 0u;
                }

                continue;
            }
        }

        HalPhaseCurrents_t currents = HalReadCurrents();
        HalPower_t power = HalReadPower();
        HalRotorState_t rotor = HalReadRotor();

        ApplyOffsets(&currents, Calibration_GetData());

        float phaseCurrentMax_A = MaxAbsPhaseCurrent(&currents);
        float currentSum_A =
            currents.phaseA_A +
            currents.phaseB_A +
            currents.phaseC_A;

        bool highCurrentNow =
            (phaseCurrentMax_A > WARN_MAX_ABS_PHASE_CURRENT_A) ||
            (MathAbs(currentSum_A) > WARN_MAX_ABS_CURRENT_SUM_A);

        if ((state != TEST_STATE_COMPLETE) && highCurrentNow)
        {
            highCurrentTicks++;
        }
        else
        {
            highCurrentTicks = 0u;
        }

        if ((state != TEST_STATE_COMPLETE) &&
            (highCurrentTicks >= CURRENT_WARN_PERSIST_TICKS))
        {
            highCurrentTicks = 0u;
        }

        if (printTicks >= PRINT_PERIOD_TICKS)
        {
            PrintFocDiagnosticsIfActive();

            printTicks = 0u;

            if (gpio_is_ready_dt(&led))
            {
                gpio_pin_toggle_dt(&led);
            }
        }
    }

    return 0;
}
