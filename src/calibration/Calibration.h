#pragma once

#include <stdbool.h>

#include "Commutation.h"
#include "Hal.h"

/*==============================================================================
 * Calibration.h
 *
 * PURPOSE:
 * Declares the public interface for the deterministic 16 kHz calibration
 * engine.
 *
 * RESPONSIBILITIES:
 * - start and query calibration execution
 * - report terminal calibration results
 * - expose the active calibration-data snapshot
 * - load and save validated persistent calibration data
 * - provide calibrated current offsets and motor parameters
 *
 * OUT OF SCOPE:
 * - flash layout, record versioning, and CRC implementation
 * - low-level ADC, PWM, and encoder drivers
 * - controller mode policy and normal closed-loop operation
 *
 * EXECUTION MODEL:
 * Calibration_Update16kHz() is called once per control-loop iteration. The
 * calibration engine updates its internal state machine and writes the drive
 * request that the controller passes to the selected commutation strategy.
 *===========================================================================*/

/*=============================================================================
 * RESULT STATUS
 *===========================================================================*/

typedef enum
{
    CAL_RESULT_NOT_STARTED = 0,
    CAL_RESULT_RUNNING,
    CAL_RESULT_PASSED,

    CAL_RESULT_FAILED_TIMEOUT,
    CAL_RESULT_FAILED_BUS_VOLTAGE,
    CAL_RESULT_FAILED_INVALID_ROTOR,
    CAL_RESULT_FAILED_OFFSET_RANGE,
    CAL_RESULT_FAILED_LOCK_CURRENT,
    CAL_RESULT_FAILED_LOCK_STABILITY,
    CAL_RESULT_FAILED_PARAM_RANGE

} CalibrationResult_t;

const char *Calibration_ResultToString(CalibrationResult_t result);

/*=============================================================================
 * CALIBRATION DATA SNAPSHOT
 *===========================================================================*/

typedef struct
{
    CalibrationResult_t result;
    bool isValid;

    float offsetAlpha_A;
    float offsetBeta_A;

    float encoderElectricalOffset_rad;

    float phaseResistance_Ohm;
    float phaseInductance_H;

    float busVoltage_V;

    /* V2 thermal snapshot captured during calibration. */
    float auxTemp_C;
    float phaseATemp_C;
    float phaseBTemp_C;
    float phaseCTemp_C;

} CalibrationData_t;

/*=============================================================================
 * CONTROL / STATUS
 *===========================================================================*/

void Calibration_Begin(void);
void Calibration_Start(void);

bool Calibration_IsRunning(void);
bool Calibration_IsComplete(void);
bool Calibration_Passed(void);

CalibrationResult_t Calibration_GetResult(void);
const CalibrationData_t *Calibration_GetData(void);

/*
 * Returns true when calibration data has been loaded or generated and is safe
 * for controller use.
 */
bool Calibration_HasValidData(void);

/*
 * Returns true when the active calibration step should execute through FOC
 * current control instead of open-loop voltage commutation.
 */
bool Calibration_RequiresFocCommutation(void);

/*=============================================================================
 * PERSISTENCE
 *===========================================================================*/

bool Calibration_LoadPersisted(void);
bool Calibration_SavePersisted(void);

/*=============================================================================
 * REAL-TIME EXECUTION
 *===========================================================================*/

/** Runs one deterministic calibration update from the 16 kHz control loop. */
void Calibration_Update16kHz(
    HalPhaseCurrents_t *currents,
    HalPower_t *power,
    HalRotorState_t *rotor,
    CommutationInputs_t *cmd);

/*=============================================================================
 * ACCESSORS
 *===========================================================================*/

float Calibration_GetCurrentOffsetAlpha(void);
float Calibration_GetCurrentOffsetBeta(void);
float Calibration_GetPhaseResistance(void);
float Calibration_GetPhaseInductance(void);