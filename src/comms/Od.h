#pragma once

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Od.h
 *
 * Object Dictionary shared between the controller, CAN interface, and telemetry.
 *
 * This module owns shared data only. It contains no control logic.
 *============================================================================*/

/*==============================================================================
 * COMMAND OBJECTS
 *============================================================================*/

typedef struct
{
    float torque_Nm;
    float velocity_revPerSec;
    float position_rev;

    bool arm;
    bool clearFault;

} OdCommand_t;

/*==============================================================================
 * CONFIGURATION OBJECTS
 *============================================================================*/

typedef struct
{
    uint8_t controlMode;
    uint8_t commutationMode;

} OdConfig_t;

/*==============================================================================
 * TELEMETRY OBJECTS
 *============================================================================*/

typedef struct
{
    float busVoltage_V;

    float phaseCurrentA_A;
    float phaseCurrentB_A;
    float phaseCurrentC_A;

    float rotorPosition_rev;
    float rotorVelocity_revPerSec;

} OdTelemetry_t;

/*==============================================================================
 * OBJECT DICTIONARY
 *============================================================================*/

typedef struct
{
    OdCommand_t command;
    OdConfig_t config;
    OdTelemetry_t telemetry;

} ObjectDictionary_t;

/*==============================================================================
 * PUBLIC API
 *============================================================================*/

void OdInit(void);

/* Returns the global object dictionary instance. */
ObjectDictionary_t *OdGet(void);