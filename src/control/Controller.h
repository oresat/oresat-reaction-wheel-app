#pragma once

#include <stdbool.h>

/*==============================================================================
 * Controller.h
 *
 * PURPOSE:
 * Declares the public interface for the deterministic high-speed motor
 * controller.
 *
 * RESPONSIBILITIES:
 * - select controller objective and commutation modes
 * - accept torque, velocity, position, startup, and direct-voltage commands
 * - expose fault status and recovery
 * - execute the single production 16 kHz control-loop entry point
 * - expose the controller diagnostic snapshot
 *
 * OUT OF SCOPE:
 * - HAL register access and sensor-driver implementation
 * - low-rate FSM policy
 * - CAN, object-dictionary, and telemetry transport
 * - per-strategy commutation mathematics
 * - bench-test sequencing
 *
 * ARCHITECTURAL ROLE:
 * The controller converts high-level objectives into a unified
 * CommutationInputs_t request, dispatches exactly one strategy, and applies the
 * resulting inverter command through the HAL.
 *===========================================================================*/

/*=============================================================================
 * CONTROL MODES
 *===========================================================================*/

typedef enum
{
    CTRL_MODE_IDLE = 0,
    CTRL_MODE_TORQUE = 1,
    CTRL_MODE_VELOCITY = 2,
    CTRL_MODE_POSITION = 3,
    CTRL_MODE_CALIBRATION = 4,
    CTRL_MODE_STARTUP_OPEN_LOOP = 5,
    CTRL_MODE_VOLTAGE = 6
} ControlMode_t;

/*=============================================================================
 * COMMUTATION MODES
 *===========================================================================*/

typedef enum
{
    COMM_MODE_FOC = 0,
    COMM_MODE_SINE = 1,
    COMM_MODE_TRAP = 2,
    COMM_MODE_OPEN_LOOP = 3
} CommutationMode_t;

/*=============================================================================
 * TUNING PROFILE
 *===========================================================================*/

typedef struct
{
    float velocityRampRate_revPerSec2;
    float velocityKp;
    float velocityKi;
    float velocityIntegratorLimit_Nm;

    float currentLoopBandwidth_radPerSec;
} ControllerTuning_t;

/*=============================================================================
 * CONTROL LOOP
 *===========================================================================*/

/* Initializes controller state and selects the default commutation strategy. */
void Controller_Init(void);

/*
 * Executes one deterministic 16 kHz control cycle.
 *
 * This is the only production entry point for high-speed control.
 */
void Controller_Update16kHz(void);

/*=============================================================================
 * MODE SELECTION
 *===========================================================================*/

/* Selects the active controller objective mode. */
void Controller_SetControlMode(ControlMode_t mode);

/* Selects the active commutation strategy. */
void Controller_SetCommutationMode(CommutationMode_t mode);

/*=============================================================================
 * SETPOINTS
 *
 * Mechanical units:
 * - position: revolutions
 * - velocity: revolutions / second
 *
 * Electrical units:
 * - angle: radians
 * - velocity: radians / second
 *===========================================================================*/

void Controller_SetTorque(float torque_Nm);
void Controller_SetVelocity(float velocity_revPerSec);
void Controller_SetPosition(float position_rev);

/*=============================================================================
 * DIRECT VOLTAGE COMMANDS
 *
 * Used only in CTRL_MODE_VOLTAGE.
 *===========================================================================*/

void Controller_SetVoltageDQ(float vd_V, float vq_V);

/*=============================================================================
 * STARTUP COMMANDS
 *
 * Used only in CTRL_MODE_STARTUP_OPEN_LOOP.
 *===========================================================================*/

void Controller_SetStartupVoltageDQ(float vd_V, float vq_V);
void Controller_SetStartupElectricalAngle(float angle_rad);
void Controller_SetStartupElectricalVelocity(float vel_rad_per_sec);

/*=============================================================================
 * FAULTS
 *===========================================================================*/

bool Controller_IsFaulted(void);
void Controller_ClearFaults(void);

/*=============================================================================
 * DIAGNOSTICS
 *===========================================================================*/

typedef struct
{
    float targetVoltageQ_V;
    float targetCurrentQ_A;
    float speedError_revPerSec;
    float velocityTargetRamped_revPerSec;

    float dutyA;
    float dutyB;
    float dutyC;
} ControllerDiagnostics_t;

ControllerDiagnostics_t Controller_GetDiagnostics(void);