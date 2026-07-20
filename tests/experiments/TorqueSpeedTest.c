#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "Calibration.h"
#include "Config.h"
#include "Controller.h"
#include "Encoder.h"
#include "Hal.h"
#include "Telemetry.h"

/*=============================================================================
 * TEST: V2 TORQUE-SPEED CHARACTERIZATION
 *
 * Owns:
 * - Firmware-timed torque-speed speed profile
 * - UART trigger handling
 * - Telemetry experiment metadata
 * - COMPLETE / FAULT reporting to host DAQ
 *
 * Does not own:
 * - Host-side DAQ capture
 * - Joulescope / LabJack acquisition
 * - Controller or commutation implementation
 *
 * Profile:
 * - Wait for UART trigger 0xAA.
 * - Enter closed-loop velocity control.
 * - Ramp commanded speed from 0 RPM to 10000 RPM at 250 RPM/s.
 * - Ramp down to zero over 10 seconds.
 * - Report COMPLETE through telemetry.
 *===========================================================================*/

#define PRINT_PERIOD_TICKS              ((uint32_t)(1.0f * FOC_UPDATE_FREQ_HZ))

/* #define TORQUE_SPEED_COMMUTATION_MODE   COMM_MODE_FOC */
/* #define TORQUE_SPEED_COMMUTATION_MODE   COMM_MODE_SINE */
/* #define TORQUE_SPEED_COMMUTATION_MODE   COMM_MODE_TRAP */

#define START_RPM                       0.0f
#define END_RPM                         10000.0f
#define RAMP_RATE_RPM_PER_S             750.0f
#define RAMP_DOWN_TIME_S                10.0f

#define RPM_TO_REV_PER_SEC(x)           ((x) / 60.0f)

#define START_REV_PER_SEC               RPM_TO_REV_PER_SEC(START_RPM)
#define END_REV_PER_SEC                 RPM_TO_REV_PER_SEC(END_RPM)
#define RAMP_RATE_REV_PER_SEC2          RPM_TO_REV_PER_SEC(RAMP_RATE_RPM_PER_S)

#define RAMP_TICKS \
    ((uint32_t)(((END_REV_PER_SEC - START_REV_PER_SEC) / RAMP_RATE_REV_PER_SEC2) * FOC_UPDATE_FREQ_HZ))

#define RAMP_DOWN_TICKS \
    ((uint32_t)(RAMP_DOWN_TIME_S * FOC_UPDATE_FREQ_HZ))

typedef enum
{
    TEST_STATE_WAIT_FOR_TRIGGER = 0,
    TEST_STATE_RAMP,
    TEST_STATE_RAMP_DOWN,
    TEST_STATE_COMPLETE,
    TEST_STATE_FAULT
} TestState_t;

static TestState_t s_state = TEST_STATE_WAIT_FOR_TRIGGER;

static uint32_t s_stateTicks = 0u;
static uint32_t s_printTicks = 0u;

static float s_command_revPerSec = 0.0f;
static float s_rampDownStart_revPerSec = 0.0f;

static float Lerp(float start, float end, float frac)
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

static const char *CommutationModeToString(CommutationMode_t mode)
{
    switch (mode)
    {
        case COMM_MODE_FOC:       return "FOC";
        case COMM_MODE_SINE:      return "SINE";
        case COMM_MODE_TRAP:      return "TRAP";
        case COMM_MODE_OPEN_LOOP: return "OPEN_LOOP";
        default:                  return "UNKNOWN";
    }
}

static TelemetryExperimentStatus_t ExperimentStatusForState(TestState_t state)
{
    switch (state)
    {
        case TEST_STATE_WAIT_FOR_TRIGGER:
            return TELEMETRY_EXPERIMENT_IDLE;

        case TEST_STATE_COMPLETE:
            return TELEMETRY_EXPERIMENT_COMPLETE;

        case TEST_STATE_FAULT:
            return TELEMETRY_EXPERIMENT_FAULT;

        case TEST_STATE_RAMP:
        case TEST_STATE_RAMP_DOWN:
        default:
            return TELEMETRY_EXPERIMENT_RUNNING;
    }
}

static void PublishTelemetryMetadata(void)
{
    Telemetry_SetSpeedCommand(s_command_revPerSec);
    Telemetry_SetTestState((uint8_t)s_state);
    Telemetry_SetCommutationMode((uint8_t)TORQUE_SPEED_COMMUTATION_MODE);
    Telemetry_SetExperimentStatus(ExperimentStatusForState(s_state));
}

static void EnterState(TestState_t nextState)
{
    s_state = nextState;
    s_stateTicks = 0u;
    PublishTelemetryMetadata();
}

static void EnterFault(
    const struct gpio_dt_spec *led,
    TelemetryFaultCode_t faultCode,
    const char *reason)
{
    s_command_revPerSec = 0.0f;

    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_IDLE);

    EnterState(TEST_STATE_FAULT);
    Telemetry_SetFaultCode(faultCode);

    if ((led != NULL) && gpio_is_ready_dt(led))
    {
        gpio_pin_set_dt(led, 1);
    }
}

static void EnterComplete(void)
{
    s_command_revPerSec = 0.0f;

    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_IDLE);

    EnterState(TEST_STATE_COMPLETE);
}

static bool RequirePersistedCalibration(void)
{
    if (!Calibration_HasValidData())
    {
        return false;
    }

    const CalibrationData_t *cal = Calibration_GetData();
    EncoderSetElectricalOffset(cal->encoderElectricalOffset_rad);

    return true;
}

static void StartTestFromTrigger(void)
{
    Controller_ClearFaults();

    s_command_revPerSec = START_REV_PER_SEC;
    s_rampDownStart_revPerSec = 0.0f;

    Controller_SetCommutationMode(TORQUE_SPEED_COMMUTATION_MODE);
    Controller_SetVelocity(s_command_revPerSec);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    Telemetry_SetExperimentStatus(TELEMETRY_EXPERIMENT_RUNNING);

    EnterState(TEST_STATE_RAMP);
}

int main(void)
{
    k_thread_priority_set(k_current_get(), K_PRIO_PREEMPT(0));

    const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

    if (gpio_is_ready_dt(&led))
    {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

    Controller_Init();

    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_IDLE);

    Telemetry_Init();
    Telemetry_Enable(false);

    if (!RequirePersistedCalibration())
    {
        EnterFault(&led, TELEMETRY_FAULT_UNKNOWN, "Missing calibration.");
    }

    s_state = TEST_STATE_WAIT_FOR_TRIGGER;
    s_stateTicks = 0u;
    s_printTicks = 0u;
    s_command_revPerSec = 0.0f;
    s_rampDownStart_revPerSec = 0.0f;

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    PublishTelemetryMetadata();

    while (1)
    {
        Controller_Update16kHz();
        Telemetry_Update16kHz();

        s_stateTicks++;
        s_printTicks++;

        if ((s_state != TEST_STATE_WAIT_FOR_TRIGGER) &&
            (s_state != TEST_STATE_FAULT) &&
            Controller_IsFaulted())
        {
            EnterFault(&led, TELEMETRY_FAULT_CONTROLLER, "Controller faulted.");
        }

        switch (s_state)
        {
            case TEST_STATE_WAIT_FOR_TRIGGER:
            {
                Controller_SetVelocity(0.0f);
                Controller_SetControlMode(CTRL_MODE_IDLE);

                if (Telemetry_ConsumeTriggerEvent())
                {
                    if (Calibration_HasValidData())
                    {
                        Telemetry_Enable(true);
                        StartTestFromTrigger();
                    }
                    else
                    {
                        Telemetry_Enable(true);
                        EnterFault(&led, TELEMETRY_FAULT_UNKNOWN, "Trigger received but calibration is invalid.");
                    }
                }
            } break;

            case TEST_STATE_RAMP:
            {
                float elapsed_s =
                    (float)s_stateTicks / FOC_UPDATE_FREQ_HZ;

                s_command_revPerSec =
                    START_REV_PER_SEC +
                    (RAMP_RATE_REV_PER_SEC2 * elapsed_s);

                if (s_command_revPerSec >= END_REV_PER_SEC)
                {
                    s_command_revPerSec = END_REV_PER_SEC;
                    s_rampDownStart_revPerSec = s_command_revPerSec;

                    Controller_SetVelocity(s_command_revPerSec);
                    PublishTelemetryMetadata();

                    EnterState(TEST_STATE_RAMP_DOWN);
                    break;
                }

                Controller_SetVelocity(s_command_revPerSec);
                PublishTelemetryMetadata();
            } break;

            case TEST_STATE_RAMP_DOWN:
            {
                float frac =
                    (float)s_stateTicks / (float)RAMP_DOWN_TICKS;

                s_command_revPerSec =
                    Lerp(s_rampDownStart_revPerSec, 0.0f, frac);

                Controller_SetVelocity(s_command_revPerSec);
                PublishTelemetryMetadata();

                if (s_stateTicks >= RAMP_DOWN_TICKS)
                {
                    EnterComplete();
                }
            } break;

            case TEST_STATE_COMPLETE:
            {
                Controller_SetVelocity(0.0f);
                Controller_SetControlMode(CTRL_MODE_IDLE);
                PublishTelemetryMetadata();
            } break;

            case TEST_STATE_FAULT:
            default:
            {
                Controller_SetVelocity(0.0f);
                Controller_SetControlMode(CTRL_MODE_IDLE);
                PublishTelemetryMetadata();
            } break;
        }

        if (s_printTicks >= PRINT_PERIOD_TICKS)
        {
            if (s_state != TEST_STATE_WAIT_FOR_TRIGGER)
            {
                HalRotorState_t rotor = HalReadRotor();
            }

            s_printTicks = 0u;

            if (gpio_is_ready_dt(&led))
            {
                gpio_pin_toggle_dt(&led);
            }
        }
    }

    return 0;
}
