#include "CanComms.h"

#include <stddef.h>

#include "Commutation_Foc.h"
#include "Controller.h"
#include "Encoder.h"
#include "Fsm.h"
#include "Hal.h"
#include "Od.h"

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static ControlMode_t CanCommsDecodeControlMode(uint8_t rawMode)
{
    switch (rawMode)
    {
        case CTRL_MODE_IDLE:       return CTRL_MODE_IDLE;
        case CTRL_MODE_TORQUE:     return CTRL_MODE_TORQUE;
        case CTRL_MODE_VELOCITY:   return CTRL_MODE_VELOCITY;
        case CTRL_MODE_POSITION:   return CTRL_MODE_POSITION;
        case CTRL_MODE_CALIBRATION:return CTRL_MODE_CALIBRATION;
        default:                   return CTRL_MODE_IDLE;
    }
}

static CommutationMode_t CanCommsDecodeCommutationMode(uint8_t rawMode)
{
    switch (rawMode)
    {
        case COMM_MODE_FOC:        return COMM_MODE_FOC;
        case COMM_MODE_SINE:       return COMM_MODE_SINE;
        case COMM_MODE_TRAP:       return COMM_MODE_TRAP;
        case COMM_MODE_OPEN_LOOP:  return COMM_MODE_OPEN_LOOP;
        default:                   return COMM_MODE_FOC;
    }
}

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

static ObjectDictionary_t *CanCommsGetObjectDictionary(void)
{
    return OdGet();
}


void CanComms_Init(void)
{
    OdInit();

    ObjectDictionary_t *od = CanCommsGetObjectDictionary();
    if (od == NULL)
    {
        return;
    }

    od->command.torque_Nm = 0.0f;
    od->command.velocity_revPerSec = 0.0f;
    od->command.position_rev = 0.0f;
    od->command.arm = false;
    od->command.clearFault = false;

    od->config.controlMode = (uint8_t)CTRL_MODE_IDLE;
    od->config.commutationMode = (uint8_t)COMM_MODE_FOC;

    od->telemetry.busVoltage_V = 0.0f;
    od->telemetry.phaseCurrentA_A = 0.0f;
    od->telemetry.phaseCurrentB_A = 0.0f;
    od->telemetry.phaseCurrentC_A = 0.0f;
    od->telemetry.rotorPosition_rev = 0.0f;
    od->telemetry.rotorVelocity_revPerSec = 0.0f;
}

void CanComms_UpdateTelemetry(void)
{
    ObjectDictionary_t *od = CanCommsGetObjectDictionary();
    if (od == NULL)
    {
        return;
    }

    HalPower_t power = HalReadPower();
    HalPhaseCurrents_t currents = HalReadCurrents();
    EncoderMechanicalState_t mechanical = EncoderGetMechanicalState();

    od->telemetry.busVoltage_V = power.busVoltage_V;
    od->telemetry.phaseCurrentA_A = currents.phaseA_A;
    od->telemetry.phaseCurrentB_A = currents.phaseB_A;
    od->telemetry.phaseCurrentC_A = currents.phaseC_A;
    od->telemetry.rotorPosition_rev = mechanical.position_rev;
    od->telemetry.rotorVelocity_revPerSec = mechanical.velocity_revPerSec;

    (void)FocGetDiagnostics();
}

void CanComms_ProcessCommands(void)
{
    ObjectDictionary_t *od = CanCommsGetObjectDictionary();
    if (od == NULL)
    {
        return;
    }

    if (od->command.clearFault)
    {
        FsmClearFault();
        od->command.clearFault = false;
    }

    if (od->command.arm)
    {
        FsmArm();
    }
    else
    {
        FsmDisarm();
    }

    Controller_SetCommutationMode(
        CanCommsDecodeCommutationMode(od->config.commutationMode)
    );

    if (FsmGetState() == FSM_STATE_RUNNING)
    {
        Controller_SetControlMode(
            CanCommsDecodeControlMode(od->config.controlMode)
        );
    }

    Controller_SetTorque(od->command.torque_Nm);
    Controller_SetVelocity(od->command.velocity_revPerSec);
    Controller_SetPosition(od->command.position_rev);
}

void CanOpenAppResetCommunication(void)
{
    CanComms_Init();
}