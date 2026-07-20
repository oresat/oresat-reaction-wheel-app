#include "Fsm.h"

#include "Calibration.h"
#include "Controller.h"
#include "Hal.h"

/*=============================================================================
 * Fsm.c
 *
 * RESPONSIBILITIES:
 * - High-level system lifecycle supervision
 * - Arm/disarm request handling
 * - Calibration-to-idle transition
 * - Closed-loop entry/exit
 * - Fault-state transition
 *
 * OUT OF SCOPE:
 * - 16 kHz control execution
 * - Startup waveform generation
 * - Calibration algorithm internals
 * - Commutation strategy behavior
 * - Telemetry or CAN transport
 *
 * Role:
 * - Commands Controller at a coarse state-machine level.
 * - Keeps system sequencing separate from real-time motor control.
 *===========================================================================*/

/*=============================================================================
 * MODULE STATE
 *===========================================================================*/

static FsmState_t s_state = FSM_STATE_BOOT;
static bool s_armRequest = false;

/*=============================================================================
 * STATE ENTRY HELPERS
 *===========================================================================*/

static void EnterCalibration(void)
{
    Controller_SetControlMode(CTRL_MODE_CALIBRATION);
    Calibration_Begin();
}

static void EnterIdle(void)
{
    Controller_SetControlMode(CTRL_MODE_IDLE);
}

static void EnterArmed(void)
{
    Controller_SetControlMode(CTRL_MODE_IDLE);
}

static void EnterClosedLoop(void)
{
    Controller_SetCommutationMode(COMM_MODE_FOC);
    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_VELOCITY);
}

static void EnterFault(void)
{
    Controller_SetControlMode(CTRL_MODE_IDLE);
}

static void TransitionToState(FsmState_t state, void (*entry)(void))
{
    s_state = state;
    if (entry) entry();
}

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

void FsmInit(void)
{
    s_state = FSM_STATE_BOOT;
    s_armRequest = false;
}

void FsmArm(void)
{
    s_armRequest = true;
}

void FsmDisarm(void)
{
    s_armRequest = false;

    if ((s_state == FSM_STATE_ARMED) ||
        (s_state == FSM_STATE_CLOSED_LOOP))
    {
        TransitionToState(FSM_STATE_IDLE, EnterIdle);
    }
}

void FsmClearFault(void)
{
    if (s_state == FSM_STATE_FAULT)
    {
        if (!HalHasHardwareFault())
        {
            Controller_ClearFaults();
            s_state = FSM_STATE_IDLE;
            EnterIdle();
        }
    }
}

FsmState_t FsmGetState(void)
{
    return s_state;
}

/*=============================================================================
 * MAIN UPDATE
 *===========================================================================*/

void FsmUpdate(void)
{
    if (Controller_IsFaulted())
    {
        TransitionToState(FSM_STATE_FAULT, EnterFault);
        return;
    }

    switch (s_state)
    {
        case FSM_STATE_BOOT:
        {
            EnterCalibration();
            s_state = FSM_STATE_CALIBRATION;
        } break;

        case FSM_STATE_CALIBRATION:
        {
            if (Calibration_IsComplete())
            {
                if (Calibration_Passed())
                {
                    s_state = FSM_STATE_IDLE;
                    EnterIdle();
                }
                else
                {
                    TransitionToState(FSM_STATE_FAULT, EnterFault);
                }
            }
        } break;

        case FSM_STATE_IDLE:
        {
            if (s_armRequest)
            {
                TransitionToState(FSM_STATE_ARMED, EnterArmed);
            }
        } break;

        case FSM_STATE_ARMED:
        {
            if (!s_armRequest)
            {
                s_state = FSM_STATE_IDLE;
                EnterIdle();
            }
            else
            {
                TransitionToState(FSM_STATE_CLOSED_LOOP, EnterClosedLoop);
            }
        } break;

        case FSM_STATE_CLOSED_LOOP:
        {
            if (!s_armRequest)
            {
                s_state = FSM_STATE_IDLE;
                EnterIdle();
            }
        } break;

        case FSM_STATE_FAULT:
        default:
        {
            /* Remain in fault until cleared. */
        } break;
    }
}