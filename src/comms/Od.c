#include "Od.h"

/*==============================================================================
 * Od.c
 *
 * Global object dictionary storage.
 *============================================================================*/

/*==============================================================================
 * PRIVATE STATE
 *============================================================================*/

static ObjectDictionary_t s_objectDictionary;

/*==============================================================================
 * PUBLIC API
 *============================================================================*/

void OdInit(void)
{
    s_objectDictionary.command.torque_Nm = 0.0f;
    s_objectDictionary.command.velocity_revPerSec = 0.0f;
    s_objectDictionary.command.position_rev = 0.0f;

    s_objectDictionary.command.arm = false;
    s_objectDictionary.command.clearFault = false;

    s_objectDictionary.config.controlMode = 0u;
    s_objectDictionary.config.commutationMode = 0u;

    s_objectDictionary.telemetry.busVoltage_V = 0.0f;

    s_objectDictionary.telemetry.phaseCurrentA_A = 0.0f;
    s_objectDictionary.telemetry.phaseCurrentB_A = 0.0f;
    s_objectDictionary.telemetry.phaseCurrentC_A = 0.0f;

    s_objectDictionary.telemetry.rotorPosition_rev = 0.0f;
    s_objectDictionary.telemetry.rotorVelocity_revPerSec = 0.0f;
}

ObjectDictionary_t *OdGet(void)
{
    return &s_objectDictionary;
}