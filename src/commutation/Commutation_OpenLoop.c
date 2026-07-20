#include "Commutation_OpenLoop.h"

#include <stddef.h>

#include "BldcMath.h"
#include "MathUtil.h"

/*=============================================================================
 * Commutation_OpenLoop.c
 *
 * PURPOSE:
 * Implements the stateless open-loop vector commutation strategy.
 *
 * OWNERSHIP:
 * - safe command generation for this strategy
 * - open-loop strategy lifecycle hooks
 * - d/q voltage to alpha/beta conversion
 * - DC-bus-based voltage limiting
 * - SVM duty generation for open-loop operation
 *
 * OUT OF SCOPE:
 * - forced-angle generation
 * - startup alignment / pull-in sequencing
 * - capture detection
 * - handoff logic
 * - controller policy
 * - hardware access beyond returning a unified PWM command
 *
 * ARCHITECTURAL ROLE:
 * Pure electrical synthesis block. Consumes a supplied electrical angle plus
 * supplied d/q voltage commands and converts them into a three-phase inverter
 * command.
 *
 * NOTES:
 * - This file is intentionally stateless.
 * - It does not validate or generate the electrical angle.
 *===========================================================================*/

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static HalPwmCommand_t OpenLoopMakeSafeCommand(void)
{
    return (HalPwmCommand_t){
        .dutyA = 0.5f,
        .dutyB = 0.5f,
        .dutyC = 0.5f,
        .floatA = true,
        .floatB = true,
        .floatC = true,
        .enableGateDriver = false
    };
}

static void OpenLoopInit(void)
{
}

/*=============================================================================
 * STRATEGY IMPLEMENTATION
 *===========================================================================*/

static HalPwmCommand_t OpenLoopUpdate(const CommutationInputs_t *inputs)
{
    if (inputs == NULL)
    {
        return OpenLoopMakeSafeCommand();
    }

    if (inputs->busVoltage_V <= 0.1f)
    {
        return OpenLoopMakeSafeCommand();
    }

    BldcDQ_t dqVoltage_V = {
        .d = inputs->targetVoltageD_V,
        .q = inputs->targetVoltageQ_V
    };

    /*
     * Rotate the commanded d/q vector into the stationary alpha/beta frame
     * using the controller-supplied electrical angle.
     */
    BldcAlphaBeta_t alphaBetaVoltage_V =
        BldcInversePark(dqVoltage_V, inputs->electricalAngle_rad);

    float alphaBetaMagnitude_V = MathSqrt(
        MathSquare(alphaBetaVoltage_V.alpha) +
        MathSquare(alphaBetaVoltage_V.beta)
    );

    /*
     * Stay within the linear SVM range to avoid overmodulation.
     */
    float maxLinearVoltage_V = inputs->busVoltage_V * MATH_ONE_OVER_SQRT3_F;
    if (maxLinearVoltage_V <= 0.0f)
    {
        return OpenLoopMakeSafeCommand();
    }

    if (alphaBetaMagnitude_V > maxLinearVoltage_V)
    {
        float scale = maxLinearVoltage_V / alphaBetaMagnitude_V;
        alphaBetaVoltage_V.alpha *= scale;
        alphaBetaVoltage_V.beta  *= scale;
    }

    BldcAlphaBeta_t alphaBetaNorm = {
        .alpha = alphaBetaVoltage_V.alpha / maxLinearVoltage_V,
        .beta  = alphaBetaVoltage_V.beta  / maxLinearVoltage_V
    };

    BldcPhaseABC_t duty = BldcSpaceVectorModulation(alphaBetaNorm);
    if (!BldcIsDutyCycleValid(duty))
    {
        return OpenLoopMakeSafeCommand();
    }

    return (HalPwmCommand_t){
        .dutyA = duty.a,
        .dutyB = duty.b,
        .dutyC = duty.c,
        .floatA = false,
        .floatB = false,
        .floatC = false,
        .enableGateDriver = true
    };
}

static HalPwmCommand_t OpenLoopStop(void)
{
    return OpenLoopMakeSafeCommand();
}

/*=============================================================================
 * PUBLIC STRATEGY EXPORT
 *===========================================================================*/

const CommutationStrategy_t g_openLoopStrategy =
{
    .Init = OpenLoopInit,
    .Update = OpenLoopUpdate,
    .Stop = OpenLoopStop,
};