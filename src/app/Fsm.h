#pragma once

#include <stdbool.h>

/*=============================================================================
 * Fsm.h
 *
 * RESPONSIBILITIES:
 * - High-level system supervision
 * - System arming/disarming
 * - Calibration sequencing
 * - Fault state transitions
 *
 * OUT OF SCOPE:
 * - Motor-control algorithms
 * - Controller operating modes
 * - Commutation strategy selection
 * - Startup waveform generation
 * - Bench-test sequencing
 *
 * Role:
 * - Supervises the overall wheel lifecycle.
 * - Commands the controller at a coarse level.
 * - Never executes the 16 kHz control algorithm.
 *===========================================================================*/

/*=============================================================================
 * SYSTEM STATES
 *===========================================================================*/

typedef enum
{
    /* Initial boot and subsystem initialization. */
    FSM_STATE_BOOT = 0,

    /* Controller-executed calibration. */
    FSM_STATE_CALIBRATION,

    /* Safe idle state with outputs disabled. */
    FSM_STATE_IDLE,

    /* Ready for closed-loop operation. */
    FSM_STATE_ARMED,

    /* Normal closed-loop operation. */
    FSM_STATE_CLOSED_LOOP,

    /* Latched fault requiring operator intervention. */
    FSM_STATE_FAULT

} FsmState_t;

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

/* Initializes the supervisor state machine. */
void FsmInit(void);

/* Executes one supervisor update. */
void FsmUpdate(void);

/* Returns the current supervisor state. */
FsmState_t FsmGetState(void);

/* Arms the system for closed-loop operation. */
void FsmArm(void);

/* Returns the system to the idle state. */
void FsmDisarm(void);

/* Clears a latched supervisor fault. */
void FsmClearFault(void);