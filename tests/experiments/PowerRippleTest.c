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
 * TEST: V2 POWER + STEADY-STATE RIPPLE CHARACTERIZATION
 *
 * Owns:
 * - Firmware-timed plateau profile
 * - UART trigger handling
 * - Telemetry experiment metadata
 * - COMPLETE / FAULT reporting to host DAQ
 *
 * Profile:
 * - Wait for UART trigger 0xAA.
 * - Step through 500 RPM plateaus up to 10000 RPM.
 * - Use settle and analysis windows at each plateau.
 * - Ramp down to zero.
 * - Report COMPLETE through telemetry.
 *===========================================================================*/

#define SETTLE_TICKS                   ((uint32_t)(2.5f * FOC_UPDATE_FREQ_HZ))
#define PLATEAU_TICKS                  ((uint32_t)(5.0f * FOC_UPDATE_FREQ_HZ))
#define RAMP_DOWN_TICKS                ((uint32_t)(5.0f * FOC_UPDATE_FREQ_HZ))

#define POWER_RIPPLE_COMMUTATION_MODE  COMM_MODE_FOC
/* #define POWER_RIPPLE_COMMUTATION_MODE  COMM_MODE_SINE */
/* #define POWER_RIPPLE_COMMUTATION_MODE  COMM_MODE_TRAP */

static const float s_plateaus_revPerSec[] =
{
    8.333333f,    /*  500 RPM */
    16.666667f,   /* 1000 RPM */
    25.000000f,   /* 1500 RPM */
    33.333333f,   /* 2000 RPM */
    41.666667f,   /* 2500 RPM */
    50.000000f,   /* 3000 RPM */
    58.333333f,   /* 3500 RPM */
    66.666667f,   /* 4000 RPM */
    75.000000f,   /* 4500 RPM */
    83.333333f,   /* 5000 RPM */
    91.666667f,   /* 5500 RPM */
    100.000000f,  /* 6000 RPM */
    108.333333f,  /* 6500 RPM */
    116.666667f,  /* 7000 RPM */
    125.000000f,  /* 7500 RPM */
    133.333333f,  /* 8000 RPM */
    141.666667f,  /* 8500 RPM */
    150.000000f,  /* 9000 RPM */
    158.333333f,  /* 9500 RPM */
    166.666667f   /* 10000 RPM */
};

#define PLATEAU_COUNT \
    ((uint32_t)(sizeof(s_plateaus_revPerSec) / sizeof(s_plateaus_revPerSec[0])))

typedef enum
{
    TEST_STATE_WAIT_FOR_TRIGGER = 0,
    TEST_STATE_SETTLE,
    TEST_STATE_PLATEAU,
    TEST_STATE_RAMP_DOWN,
    TEST_STATE_COMPLETE,
    TEST_STATE_FAULT
} TestState_t;

static TestState_t s_state = TEST_STATE_WAIT_FOR_TRIGGER;

static uint32_t s_stateTicks = 0u;
static uint32_t s_plateauIndex = 0u;

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

        case TEST_STATE_SETTLE:
        case TEST_STATE_PLATEAU:
        case TEST_STATE_RAMP_DOWN:
        default:
            return TELEMETRY_EXPERIMENT_RUNNING;
    }
}

static void PublishTelemetryMetadata(void)
{
    Telemetry_SetSpeedCommand(s_command_revPerSec);
    Telemetry_SetTestState((uint8_t)s_state);
    Telemetry_SetCommutationMode((uint8_t)POWER_RIPPLE_COMMUTATION_MODE);
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

static void EnterPlateau(uint32_t plateauIndex)
{
    s_plateauIndex = plateauIndex;
    s_command_revPerSec = s_plateaus_revPerSec[s_plateauIndex];

    Controller_SetVelocity(s_command_revPerSec);
    EnterState(TEST_STATE_SETTLE);

}

static void StartTestFromTrigger(void)
{
    Controller_ClearFaults();

    s_plateauIndex = 0u;
    s_command_revPerSec = 0.0f;
    s_rampDownStart_revPerSec = 0.0f;

    Controller_SetVelocity(0.0f);
    Controller_SetCommutationMode(POWER_RIPPLE_COMMUTATION_MODE);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);

    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);
    Telemetry_SetExperimentStatus(TELEMETRY_EXPERIMENT_RUNNING);

    EnterPlateau(0u);

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
    s_plateauIndex = 0u;
    s_command_revPerSec = 0.0f;
    s_rampDownStart_revPerSec = 0.0f;

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

            case TEST_STATE_SETTLE:
            {
                Controller_SetVelocity(s_command_revPerSec);
                PublishTelemetryMetadata();

                if (s_stateTicks >= SETTLE_TICKS)
                {
                    EnterState(TEST_STATE_PLATEAU);

                }
            } break;

            case TEST_STATE_PLATEAU:
            {
                Controller_SetVelocity(s_command_revPerSec);
                PublishTelemetryMetadata();

                if (s_stateTicks >= PLATEAU_TICKS)
                {
                    uint32_t nextPlateau = s_plateauIndex + 1u;

                    if (nextPlateau < PLATEAU_COUNT)
                    {
                        EnterPlateau(nextPlateau);
                    }
                    else
                    {
                        s_rampDownStart_revPerSec = s_command_revPerSec;
                        EnterState(TEST_STATE_RAMP_DOWN);

                    }
                }
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
    }

    return 0;
}