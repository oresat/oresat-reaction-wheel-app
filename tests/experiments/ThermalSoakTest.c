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
 * TEST: V2 SUSTAINED THERMAL SOAK
 *===========================================================================*/

#define THERMAL_SOAK_COMMUTATION_MODE      COMM_MODE_FOC
/* #define THERMAL_SOAK_COMMUTATION_MODE      COMM_MODE_TRAP */
/* #define THERMAL_SOAK_COMMUTATION_MODE      COMM_MODE_SINE */

#define THERMAL_TARGET_RPM                 10000.0f
#define THERMAL_HOLD_TIME_S                600.0f
#define THERMAL_STOP_TIME_S                10.0f

#define THERMAL_HOLD_TICKS                 ((uint32_t)(THERMAL_HOLD_TIME_S * FOC_UPDATE_FREQ_HZ))
#define THERMAL_STOP_TICKS                 ((uint32_t)(THERMAL_STOP_TIME_S * FOC_UPDATE_FREQ_HZ))

#define RPM_TO_REV_PER_SEC(x)              ((x) / 60.0f)

typedef enum
{
    TEST_STATE_WAIT_FOR_TRIGGER = 0,
    TEST_STATE_SOAK,
    TEST_STATE_STOP,
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

        case TEST_STATE_SOAK:
        case TEST_STATE_STOP:
        default:
            return TELEMETRY_EXPERIMENT_RUNNING;
    }
}

static void PublishTelemetryMetadata(void)
{
    Telemetry_SetSpeedCommand(s_command_revPerSec);
    Telemetry_SetTestState((uint8_t)s_state);
    Telemetry_SetCommutationMode((uint8_t)THERMAL_SOAK_COMMUTATION_MODE);
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

    Controller_SetCommutationMode(THERMAL_SOAK_COMMUTATION_MODE);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    Telemetry_SetExperimentStatus(TELEMETRY_EXPERIMENT_RUNNING);

    ApplyCommand(RPM_TO_REV_PER_SEC(THERMAL_TARGET_RPM));
    EnterState(TEST_STATE_SOAK);
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
                    Telemetry_Enable(true);

                    if (Calibration_HasValidData())
                    {
                        StartTestFromTrigger();
                    }
                    else
                    {
                        EnterFault(&led, TELEMETRY_FAULT_UNKNOWN);
                    }
                }
            } break;

            case TEST_STATE_SOAK:
            {
                ApplyCommand(RPM_TO_REV_PER_SEC(THERMAL_TARGET_RPM));

                if (s_stateTicks >= THERMAL_HOLD_TICKS)
                {
                    ApplyCommand(0.0f);
                    EnterState(TEST_STATE_STOP);
                }
            } break;

            case TEST_STATE_STOP:
            {
                ApplyCommand(0.0f);

                if (s_stateTicks >= THERMAL_STOP_TICKS)
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