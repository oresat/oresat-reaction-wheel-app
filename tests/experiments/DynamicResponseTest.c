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
 * TEST: V2 DYNAMIC RESPONSE AND CONTROL AGILITY
 *
 * Owns:
 * - Firmware-timed velocity step profile
 * - UART trigger handling
 * - Telemetry experiment metadata
 * - COMPLETE / FAULT reporting to host DAQ
 *
 * Profile:
 * - Wait for UART trigger 0xAA.
 * - Command 2000 RPM and hold.
 * - Step command to 6000 RPM and hold.
 * - Step command back to 2000 RPM and hold.
 * - Command 0 RPM and hold.
 * - Report COMPLETE through telemetry.
 *===========================================================================*/

/* #define DYNAMIC_RESPONSE_COMMUTATION_MODE   COMM_MODE_FOC */
/* #define DYNAMIC_RESPONSE_COMMUTATION_MODE   COMM_MODE_SINE */
/* #define DYNAMIC_RESPONSE_COMMUTATION_MODE   COMM_MODE_TRAP */

#define LOW_RPM                             2000.0f
#define HIGH_RPM                            6000.0f

#define LOW_HOLD_TIME_S                     8.0f
#define HIGH_HOLD_TIME_S                    8.0f
#define RETURN_HOLD_TIME_S                  8.0f
#define FINAL_STOP_TIME_S                   4.0f

#define LOW_HOLD_TICKS                      ((uint32_t)(LOW_HOLD_TIME_S * FOC_UPDATE_FREQ_HZ))
#define HIGH_HOLD_TICKS                     ((uint32_t)(HIGH_HOLD_TIME_S * FOC_UPDATE_FREQ_HZ))
#define RETURN_HOLD_TICKS                   ((uint32_t)(RETURN_HOLD_TIME_S * FOC_UPDATE_FREQ_HZ))
#define FINAL_STOP_TICKS                    ((uint32_t)(FINAL_STOP_TIME_S * FOC_UPDATE_FREQ_HZ))

#define RPM_TO_REV_PER_SEC(x)               ((x) / 60.0f)

typedef enum
{
    TEST_STATE_WAIT_FOR_TRIGGER = 0,
    TEST_STATE_LOW_HOLD,
    TEST_STATE_HIGH_HOLD,
    TEST_STATE_RETURN_HOLD,
    TEST_STATE_FINAL_STOP,
    TEST_STATE_COMPLETE,
    TEST_STATE_FAULT
} TestState_t;

static TestState_t s_state = TEST_STATE_WAIT_FOR_TRIGGER;

static uint32_t s_stateTicks = 0u;
static float s_command_revPerSec = 0.0f;

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

        case TEST_STATE_LOW_HOLD:
        case TEST_STATE_HIGH_HOLD:
        case TEST_STATE_RETURN_HOLD:
        case TEST_STATE_FINAL_STOP:
        default:
            return TELEMETRY_EXPERIMENT_RUNNING;
    }
}

static void PublishTelemetryMetadata(void)
{
    Telemetry_SetSpeedCommand(s_command_revPerSec);
    Telemetry_SetTestState((uint8_t)s_state);
    Telemetry_SetCommutationMode((uint8_t)DYNAMIC_RESPONSE_COMMUTATION_MODE);
    Telemetry_SetExperimentStatus(ExperimentStatusForState(s_state));
}

static void EnterState(TestState_t nextState)
{
    s_state = nextState;
    s_stateTicks = 0u;
    PublishTelemetryMetadata();
}

static void ApplyCommand(float command_revPerSec)
{
    s_command_revPerSec = command_revPerSec;
    Controller_SetVelocity(s_command_revPerSec);
    PublishTelemetryMetadata();
}

static void EnterFault(
    const struct gpio_dt_spec *led,
    TelemetryFaultCode_t faultCode)
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

    Controller_SetCommutationMode(DYNAMIC_RESPONSE_COMMUTATION_MODE);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    Telemetry_SetExperimentStatus(TELEMETRY_EXPERIMENT_RUNNING);

    ApplyCommand(RPM_TO_REV_PER_SEC(LOW_RPM));
    EnterState(TEST_STATE_LOW_HOLD);
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
        EnterFault(&led, TELEMETRY_FAULT_UNKNOWN);
    }

    s_state = TEST_STATE_WAIT_FOR_TRIGGER;
    s_stateTicks = 0u;
    s_command_revPerSec = 0.0f;

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    PublishTelemetryMetadata();

    while (1)
    {
        Controller_Update16kHz();
        Telemetry_Update16kHz();

        s_stateTicks++;
        if ((s_state != TEST_STATE_WAIT_FOR_TRIGGER) &&
            (s_state != TEST_STATE_FAULT) &&
            Controller_IsFaulted())
        {
            EnterFault(&led, TELEMETRY_FAULT_CONTROLLER);
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
                        EnterFault(&led, TELEMETRY_FAULT_UNKNOWN);
                    }
                }
            } break;

            case TEST_STATE_LOW_HOLD:
            {
                ApplyCommand(RPM_TO_REV_PER_SEC(LOW_RPM));

                if (s_stateTicks >= LOW_HOLD_TICKS)
                {
                    ApplyCommand(RPM_TO_REV_PER_SEC(HIGH_RPM));
                    EnterState(TEST_STATE_HIGH_HOLD);                }
            } break;

            case TEST_STATE_HIGH_HOLD:
            {
                ApplyCommand(RPM_TO_REV_PER_SEC(HIGH_RPM));

                if (s_stateTicks >= HIGH_HOLD_TICKS)
                {
                    ApplyCommand(RPM_TO_REV_PER_SEC(LOW_RPM));
                    EnterState(TEST_STATE_RETURN_HOLD);                }
            } break;

            case TEST_STATE_RETURN_HOLD:
            {
                ApplyCommand(RPM_TO_REV_PER_SEC(LOW_RPM));

                if (s_stateTicks >= RETURN_HOLD_TICKS)
                {
                    ApplyCommand(0.0f);
                    EnterState(TEST_STATE_FINAL_STOP);

                }
            } break;

            case TEST_STATE_FINAL_STOP:
            {
                ApplyCommand(0.0f);

                if (s_stateTicks >= FINAL_STOP_TICKS)
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

    }

    return 0;
}