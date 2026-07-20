#include "Commutation_Trap.h"

#include <stddef.h>

#include "MathUtil.h"

/*==============================================================================
 * Commutation_Trap.c
 *
 * PURPOSE:
 * Implements stateless unipolar six-step commutation for the OreSat reaction
 * wheel inverter.
 *
 * PROCESSING PIPELINE:
 * 1. validate the input and DC-bus voltage
 * 2. reject reverse/braking requests by returning the coast command
 * 3. normalize the requested q-axis voltage into a duty amplitude
 * 4. shift and wrap the supplied electrical angle
 * 5. derive the raw six-step sector
 * 6. apply torque advance and PCB phase-routing offset
 * 7. emit the corresponding two-phase-drive / one-phase-float command
 *
 * IMPORTANT CONVENTIONS:
 * - Negative q-axis current or voltage requests do not reverse commutation.
 *   They disable the gate driver and allow the wheel to coast.
 * - A +30 electrical-degree angle shift centers 0 rad within sector 1.
 * - A +2-sector advance produces the established forward-torque relationship.
 * - The fixed sector offset aligns the logical table with physical PCB routing.
 *
 * STATE:
 * This strategy owns no persistent runtime state.
 *===========================================================================*/

/*==============================================================================
 * PRIVATE CONSTANTS
 *===========================================================================*/

/*
 * Logical-to-physical sector alignment for the current OreSat PCB routing.
 *
 * Valid values are 0 through 5. This value is part of the established hardware
 * convention and must not be changed without revalidating phase mapping.
 */
#define TRAP_SECTOR_OFFSET                 3

/* Minimum bus voltage accepted by the active commutation path. */
static const float TRAP_MINIMUM_BUS_VOLTAGE_V = 0.1f;

/* Negative q-axis current threshold that selects coasting behavior. */
static const float TRAP_REVERSE_CURRENT_THRESHOLD_A = -0.01f;

/* Centered duty used by the inactive safe/coast command. */
static const float TRAP_CENTER_DUTY = 0.5f;

/* Six electrical sectors per revolution. */
static const int TRAP_SECTOR_COUNT = 6;

/* Shift 0 rad away from a sector boundary and into the center of sector 1. */
static const float TRAP_BOUNDARY_SHIFT_RAD = MATH_PI_F / 6.0f;

/* Two-sector electrical advance used for the established forward torque. */
static const int TRAP_TORQUE_ADVANCE_SECTORS = 2;

/*==============================================================================
 * PRIVATE COMMAND CONSTRUCTION
 *===========================================================================*/

/**
 * Builds the inactive command used for invalid input, coasting, and stop.
 */
static HalPwmCommand_t TrapMakeSafeCommand(void)
{
    return (HalPwmCommand_t){
        .dutyA = TRAP_CENTER_DUTY,
        .dutyB = TRAP_CENTER_DUTY,
        .dutyC = TRAP_CENTER_DUTY,

        .floatA = true,
        .floatB = true,
        .floatC = true,

        .enableGateDriver = false
    };
}

/**
 * Builds the active command baseline before applying one commutation-table row.
 *
 * All phases begin floated with zero duty. The selected table row enables the
 * two conducting phases and applies duty to the high-side phase.
 */
static HalPwmCommand_t TrapMakeActiveCommandBase(void)
{
    return (HalPwmCommand_t){
        .dutyA = 0.0f,
        .dutyB = 0.0f,
        .dutyC = 0.0f,

        .floatA = true,
        .floatB = true,
        .floatC = true,

        .enableGateDriver = true
    };
}

/*==============================================================================
 * PRIVATE SECTOR AND DUTY HELPERS
 *===========================================================================*/

static uint8_t TrapWrapSector(int sector)
{
    while (sector < 1)
    {
        sector += TRAP_SECTOR_COUNT;
    }

    while (sector > TRAP_SECTOR_COUNT)
    {
        sector -= TRAP_SECTOR_COUNT;
    }

    return (uint8_t)sector;
}

static float TrapWrapElectricalAngle(float electricalAngle_rad)
{
    while (electricalAngle_rad < 0.0f)
    {
        electricalAngle_rad += MATH_TWO_PI_F;
    }

    while (electricalAngle_rad >= MATH_TWO_PI_F)
    {
        electricalAngle_rad -= MATH_TWO_PI_F;
    }

    return electricalAngle_rad;
}

static bool TrapShouldCoast(const CommutationInputs_t *inputs)
{
    return (inputs->targetCurrentQ_A <
            TRAP_REVERSE_CURRENT_THRESHOLD_A) ||
           (inputs->targetVoltageQ_V < 0.0f);
}

static float TrapCalculateDutyAmplitude(
    float targetVoltageQ_V,
    float busVoltage_V)
{
    const float requestedDuty = targetVoltageQ_V / busVoltage_V;

    return MathClamp(requestedDuty, 0.0f, 1.0f);
}

/*
 * Derives the physical commutation sector from the supplied electrical angle.
 *
 * The +30-degree shift centers zero angle in sector 1. The subsequent +2-sector
 * advance and fixed PCB offset preserve the existing forward-torque and wiring
 * conventions.
 */
static uint8_t TrapCalculateCommutationSector(float electricalAngle_rad)
{
    const float shiftedAngle_rad = TrapWrapElectricalAngle(
        electricalAngle_rad + TRAP_BOUNDARY_SHIFT_RAD);

    const float sectorWidth_rad =
        MATH_TWO_PI_F / (float)TRAP_SECTOR_COUNT;

    const int rawSector =
        (int)(shiftedAngle_rad / sectorWidth_rad) + 1;

    const int alignedSector =
        rawSector +
        TRAP_TORQUE_ADVANCE_SECTORS +
        TRAP_SECTOR_OFFSET;

    return TrapWrapSector(alignedSector);
}

/*
 * Applies the established unipolar six-step table.
 *
 * Each valid row:
 * - enables exactly two phase pairs
 * - leaves one phase floated
 * - applies duty to one phase while the complementary conducting phase remains
 *   at zero duty
 */
static HalPwmCommand_t TrapMakeSectorCommand(
    uint8_t sector,
    float dutyAmplitude)
{
    HalPwmCommand_t command = TrapMakeActiveCommandBase();

    switch (sector)
    {
        case 1:
            command.floatA = false;
            command.floatB = false;
            command.floatC = true;
            command.dutyA = dutyAmplitude;
            break;

        case 2:
            command.floatA = false;
            command.floatB = true;
            command.floatC = false;
            command.dutyA = dutyAmplitude;
            break;

        case 3:
            command.floatA = true;
            command.floatB = false;
            command.floatC = false;
            command.dutyB = dutyAmplitude;
            break;

        case 4:
            command.floatA = false;
            command.floatB = false;
            command.floatC = true;
            command.dutyB = dutyAmplitude;
            break;

        case 5:
            command.floatA = false;
            command.floatB = true;
            command.floatC = false;
            command.dutyC = dutyAmplitude;
            break;

        case 6:
            command.floatA = true;
            command.floatB = false;
            command.floatC = false;
            command.dutyC = dutyAmplitude;
            break;

        default:
            return TrapMakeSafeCommand();
    }

    return command;
}

/*==============================================================================
 * STRATEGY LIFECYCLE
 *===========================================================================*/

static void TrapInit(void)
{
    /* Stateless strategy: no initialization is required. */
}

static HalPwmCommand_t TrapUpdate(const CommutationInputs_t *inputs)
{
    if (inputs == NULL)
    {
        return TrapMakeSafeCommand();
    }

    if (inputs->busVoltage_V <= TRAP_MINIMUM_BUS_VOLTAGE_V)
    {
        return TrapMakeSafeCommand();
    }

    /*
     * Reverse torque requests are intentionally converted to coast behavior.
     * Immediate six-step reversal at the control-loop rate was found to create
     * severe reaction-wheel oscillation.
     */
    if (TrapShouldCoast(inputs))
    {
        return TrapMakeSafeCommand();
    }

    const float dutyAmplitude = TrapCalculateDutyAmplitude(
        inputs->targetVoltageQ_V,
        inputs->busVoltage_V);

    const uint8_t sector = TrapCalculateCommutationSector(
        inputs->electricalAngle_rad);

    return TrapMakeSectorCommand(sector, dutyAmplitude);
}

static HalPwmCommand_t TrapStop(void)
{
    return TrapMakeSafeCommand();
}

/*==============================================================================
 * PUBLIC STRATEGY EXPORT
 *===========================================================================*/

const CommutationStrategy_t g_trapezoidalStrategy =
{
    .Init = TrapInit,
    .Update = TrapUpdate,
    .Stop = TrapStop
};