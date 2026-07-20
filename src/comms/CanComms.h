#pragma once

/*=============================================================================
 * CAN COMMUNICATIONS TRANSLATION LAYER
 *
 * PURPOSE:
 * Translates between the Object Dictionary and internal controller/FSM interfaces.
 * No control algorithms are implemented here.
 *===========================================================================*/

// Initializes the CAN communications layer and resets the OD to safe defaults.
void CanComms_Init(void);

// Copies internal telemetry into the Object Dictionary.
// Intended to run in the slow supervisory thread.
void CanComms_UpdateTelemetry(void);

// Reads new commands/configuration from the Object Dictionary and dispatches
// them to the FSM and controller.
// Intended to run in the slow supervisory thread.
void CanComms_ProcessCommands(void);

// Hook for CANopen stack reset behavior.
void CanOpenAppResetCommunication(void);