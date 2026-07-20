#include "Commutation_Foc.h"

#include <stddef.h>

#include "BldcMath.h"
#include "Calibration.h"
#include "Config.h"
#include "MathUtil.h"

/*==============================================================================
 * Commutation_Foc.c
 *
 * PURPOSE:
 * Implements closed-loop d/q current regulation and converts the resulting
 * voltage vector into a three-phase SVM inverter command.
 *
 * CONTROL PIPELINE:
 * 1. validate required inputs
 * 2. transform measured phase currents into d/q
 * 3. calculate d/q current error
 * 4. update and clamp the PI current controller
 * 5. calculate enabled feedforward and decoupling terms
 * 6. apply circular d/q voltage limiting and anti-windup back-calculation
 * 7. inverse-Park the limited voltage and run SVM
 * 8. publish diagnostics and return the active PWM command
 *
 * OWNED STATE:
 * - loaded motor resistance and inductance
 * - current-loop bandwidth and derived PI gains
 * - d/q integrator voltage
 * - most recent diagnostic snapshot
 *
 * OUT OF SCOPE:
 * - outer-loop velocity, torque, and position control
 * - startup and calibration sequencing
 * - rotor estimation and sensor acquisition
 * - direct hardware access
 *===========================================================================*/

/*==============================================================================
 * PRIVATE TYPES
 *===========================================================================*/

/**
 * Individual model-based voltage terms retained separately for diagnostics.
 */
typedef struct
{
    BldcDQ_t resistive_V;
    BldcDQ_t backEmf_V;
    BldcDQ_t omegaL_V;
    BldcDQ_t decoupling_V;
    BldcDQ_t total_V;
} FocFeedforwardTerms_t;

/**
 * Values needed to publish one complete diagnostic snapshot.
 */
typedef struct
{
    BldcDQ_t measuredCurrent_A;
    BldcDQ_t targetCurrent_A;
    BldcDQ_t currentError_A;

    BldcDQ_t proportionalVoltage_V;
    BldcDQ_t piVoltage_V;
    FocFeedforwardTerms_t feedforward;

    BldcDQ_t requestedVoltage_V;
    BldcDQ_t commandedVoltage_V;

    float voltageLimit_V;
    bool voltageSaturated;

    BldcPhaseABC_t duty;
} FocDiagnosticFrame_t;

/*==============================================================================
 * PRIVATE CONSTANTS
 *===========================================================================*/

static const float FOC_DEFAULT_CURRENT_LOOP_BANDWIDTH_RAD_PER_SEC = 4000.0f;
static const float FOC_MINIMUM_BUS_VOLTAGE_V = 0.1f;
static const float FOC_CENTER_DUTY = 0.5f;

/*==============================================================================
 * MODULE STATE
 *===========================================================================*/

static float s_currentKp = 0.0f;
static float s_currentKi = 0.0f;
static float s_currentLoopBandwidth_radPerSec =
    FOC_DEFAULT_CURRENT_LOOP_BANDWIDTH_RAD_PER_SEC;

static float s_phaseResistance_Ohm = 0.0f;
static float s_phaseInductance_H = 0.0f;

static BldcDQ_t s_integrator_V =
{
    .d = 0.0f,
    .q = 0.0f
};

static volatile FocDiagnostics_t s_diagnostics = {0};

/*==============================================================================
 * PRIVATE COMMAND HELPERS
 *===========================================================================*/

static HalPwmCommand_t FocMakeSafeCommand(void)
{
    return (HalPwmCommand_t){
        .dutyA = FOC_CENTER_DUTY,
        .dutyB = FOC_CENTER_DUTY,
        .dutyC = FOC_CENTER_DUTY,

        .floatA = true,
        .floatB = true,
        .floatC = true,

        .enableGateDriver = false
    };
}

static HalPwmCommand_t FocMakeActiveCommand(BldcPhaseABC_t duty)
{
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

/*==============================================================================
 * PRIVATE PARAMETER AND PI HELPERS
 *===========================================================================*/

/**
 * Loads calibrated motor parameters, falling back to configured defaults when
 * calibration does not provide a positive value.
 */
static void FocLoadMotorParameters(void)
{
    float phaseResistance_Ohm = Calibration_GetPhaseResistance();
    float phaseInductance_H = MOTOR_PHASE_INDUCTANCE_H;

    if (phaseResistance_Ohm <= 0.0f)
    {
        phaseResistance_Ohm = MOTOR_PHASE_RESISTANCE_OHM;
    }

    if (phaseInductance_H <= 0.0f)
    {
        phaseInductance_H = MOTOR_PHASE_INDUCTANCE_H;
    }

    s_phaseResistance_Ohm = phaseResistance_Ohm;
    s_phaseInductance_H = phaseInductance_H;
}

/**
 * Computes current-loop PI gains from the configured bandwidth:
 *
 *   Kp = bandwidth * L
 *   Ki = bandwidth * R
 */
static void FocComputeCurrentLoopGains(void)
{
    s_currentKp =
        s_currentLoopBandwidth_radPerSec * s_phaseInductance_H;

    s_currentKi =
        s_currentLoopBandwidth_radPerSec * s_phaseResistance_Ohm;
}

static void FocResetIntegrator(void)
{
    s_integrator_V.d = 0.0f;
    s_integrator_V.q = 0.0f;
}

static float FocClampIntegratorVoltage(float value_V)
{
    return MathClamp(
        value_V,
        -FOC_INTEGRATOR_LIMIT_V,
        FOC_INTEGRATOR_LIMIT_V);
}

static void FocUpdateIntegrator(
    BldcDQ_t currentError_A,
    float dt_s)
{
    s_integrator_V.d +=
        s_currentKi * dt_s * currentError_A.d;

    s_integrator_V.q +=
        s_currentKi * dt_s * currentError_A.q;

    s_integrator_V.d =
        FocClampIntegratorVoltage(s_integrator_V.d);

    s_integrator_V.q =
        FocClampIntegratorVoltage(s_integrator_V.q);
}

static BldcDQ_t FocCalculateProportionalVoltage(
    BldcDQ_t currentError_A)
{
    return (BldcDQ_t){
        .d = s_currentKp * currentError_A.d,
        .q = s_currentKp * currentError_A.q
    };
}

static BldcDQ_t FocCalculatePiVoltage(
    BldcDQ_t proportionalVoltage_V)
{
    return (BldcDQ_t){
        .d = proportionalVoltage_V.d + s_integrator_V.d,
        .q = proportionalVoltage_V.q + s_integrator_V.q
    };
}

/*==============================================================================
 * PRIVATE TRANSFORM HELPERS
 *===========================================================================*/

static BldcDQ_t FocMeasureCurrentDq(
    const CommutationInputs_t *inputs)
{
    const BldcPhaseABC_t phaseCurrents_A = {
        .a = inputs->phaseCurrents_A.phaseA_A,
        .b = inputs->phaseCurrents_A.phaseB_A,
        .c = inputs->phaseCurrents_A.phaseC_A
    };

    const BldcAlphaBeta_t alphaBetaCurrents_A =
        BldcClarke(phaseCurrents_A);

    return BldcPark(
        alphaBetaCurrents_A,
        inputs->electricalAngle_rad);
}

/*==============================================================================
 * PRIVATE FEEDFORWARD HELPERS
 *===========================================================================*/

static BldcDQ_t FocCalculateResistiveFeedforward(
    const CommutationInputs_t *inputs)
{
    BldcDQ_t voltage_V = {0.0f, 0.0f};

    if (FOC_ENABLE_RESISTIVE_FF)
    {
        voltage_V.d =
            FOC_RESISTIVE_FF_GAIN *
            s_phaseResistance_Ohm *
            inputs->targetCurrentD_A;

        voltage_V.q =
            FOC_RESISTIVE_FF_GAIN *
            s_phaseResistance_Ohm *
            inputs->targetCurrentQ_A;
    }

    return voltage_V;
}

static BldcDQ_t FocCalculateBackEmfFeedforward(
    float electricalVelocity_radPerSec)
{
    BldcDQ_t voltage_V = {0.0f, 0.0f};

    if (FOC_ENABLE_BACKEMF_FEEDFORWARD)
    {
        voltage_V.q =
            FOC_BACKEMF_FF_GAIN *
            electricalVelocity_radPerSec *
            (2.0f / 3.0f) *
            (MOTOR_TORQUE_CONSTANT / (float)MOTOR_POLE_PAIRS);
    }

    return voltage_V;
}

static BldcDQ_t FocCalculateOmegaLFeedforward(
    const CommutationInputs_t *inputs)
{
    BldcDQ_t voltage_V = {0.0f, 0.0f};

    if (FOC_ENABLE_WL_FEEDFORWARD)
    {
        voltage_V.d =
            -inputs->electricalVelocity_radPerSec *
            s_phaseInductance_H *
            inputs->targetCurrentQ_A;

        voltage_V.q =
            inputs->electricalVelocity_radPerSec *
            s_phaseInductance_H *
            inputs->targetCurrentD_A;
    }

    return voltage_V;
}

static BldcDQ_t FocCalculateDecouplingVoltage(
    const CommutationInputs_t *inputs,
    BldcDQ_t measuredCurrent_A)
{
    BldcDQ_t voltage_V = {0.0f, 0.0f};

    if (FOC_ENABLE_DECOUPLING)
    {
        voltage_V.d =
            FOC_DECOUPLING_GAIN *
            (-inputs->electricalVelocity_radPerSec *
             s_phaseInductance_H *
             measuredCurrent_A.q);

        voltage_V.q =
            FOC_DECOUPLING_GAIN *
            (inputs->electricalVelocity_radPerSec *
             s_phaseInductance_H *
             measuredCurrent_A.d);
    }

    return voltage_V;
}

static FocFeedforwardTerms_t FocCalculateFeedforwardTerms(
    const CommutationInputs_t *inputs,
    BldcDQ_t measuredCurrent_A)
{
    FocFeedforwardTerms_t terms = {
        .resistive_V = FocCalculateResistiveFeedforward(inputs),
        .backEmf_V = FocCalculateBackEmfFeedforward(
            inputs->electricalVelocity_radPerSec),
        .omegaL_V = FocCalculateOmegaLFeedforward(inputs),
        .decoupling_V = FocCalculateDecouplingVoltage(
            inputs,
            measuredCurrent_A),
        .total_V = {0.0f, 0.0f}
    };

    terms.total_V.d =
        terms.resistive_V.d +
        terms.backEmf_V.d +
        terms.omegaL_V.d +
        terms.decoupling_V.d;

    terms.total_V.q =
        terms.resistive_V.q +
        terms.backEmf_V.q +
        terms.omegaL_V.q +
        terms.decoupling_V.q;

    return terms;
}

/*==============================================================================
 * PRIVATE VOLTAGE-LIMITING HELPER
 *===========================================================================*/

/**
 * Applies circular limiting in d/q and reconstructs the integrator only when
 * the requested vector exceeds the available linear-SVM magnitude.
 *
 * The anti-windup decomposition is preserved exactly:
 *
 *   Vcmd = Vp + Vi + Vff
 *   Vi   = Vcmd - Vp - Vff
 */
static BldcDQ_t FocApplyVoltageLimit(
    BldcDQ_t requestedVoltage_V,
    BldcDQ_t proportionalVoltage_V,
    BldcDQ_t feedforwardVoltage_V,
    float maxVoltage_V,
    bool *saturated_out)
{
    if (saturated_out != NULL)
    {
        *saturated_out = false;
    }

    const float requestedMagnitude_V = MathSqrt(
        MathSquare(requestedVoltage_V.d) +
        MathSquare(requestedVoltage_V.q));

    if ((requestedMagnitude_V > maxVoltage_V) &&
        (requestedMagnitude_V > 0.0f))
    {
        const float scale =
            maxVoltage_V / requestedMagnitude_V;

        requestedVoltage_V.d *= scale;
        requestedVoltage_V.q *= scale;

        s_integrator_V.d =
            requestedVoltage_V.d -
            proportionalVoltage_V.d -
            feedforwardVoltage_V.d;

        s_integrator_V.q =
            requestedVoltage_V.q -
            proportionalVoltage_V.q -
            feedforwardVoltage_V.q;

        s_integrator_V.d =
            FocClampIntegratorVoltage(s_integrator_V.d);

        s_integrator_V.q =
            FocClampIntegratorVoltage(s_integrator_V.q);

        if (saturated_out != NULL)
        {
            *saturated_out = true;
        }
    }

    return requestedVoltage_V;
}

/*==============================================================================
 * PRIVATE DIAGNOSTIC HELPER
 *===========================================================================*/

static void FocPublishDiagnostics(
    const FocDiagnosticFrame_t *frame)
{
    s_diagnostics.measuredId_A = frame->measuredCurrent_A.d;
    s_diagnostics.measuredIq_A = frame->measuredCurrent_A.q;

    s_diagnostics.targetId_A = frame->targetCurrent_A.d;
    s_diagnostics.targetIq_A = frame->targetCurrent_A.q;

    s_diagnostics.errorId_A = frame->currentError_A.d;
    s_diagnostics.errorIq_A = frame->currentError_A.q;

    s_diagnostics.proportionalVd_V =
        frame->proportionalVoltage_V.d;
    s_diagnostics.proportionalVq_V =
        frame->proportionalVoltage_V.q;

    s_diagnostics.integratorVd_V = s_integrator_V.d;
    s_diagnostics.integratorVq_V = s_integrator_V.q;

    s_diagnostics.piVd_V = frame->piVoltage_V.d;
    s_diagnostics.piVq_V = frame->piVoltage_V.q;

    s_diagnostics.resistiveFfVd_V =
        frame->feedforward.resistive_V.d;
    s_diagnostics.resistiveFfVq_V =
        frame->feedforward.resistive_V.q;

    s_diagnostics.backEmfFfVd_V =
        frame->feedforward.backEmf_V.d;
    s_diagnostics.backEmfFfVq_V =
        frame->feedforward.backEmf_V.q;

    s_diagnostics.wlFfVd_V =
        frame->feedforward.omegaL_V.d;
    s_diagnostics.wlFfVq_V =
        frame->feedforward.omegaL_V.q;

    s_diagnostics.decouplingVd_V =
        frame->feedforward.decoupling_V.d;
    s_diagnostics.decouplingVq_V =
        frame->feedforward.decoupling_V.q;

    s_diagnostics.totalFeedforwardVd_V =
        frame->feedforward.total_V.d;
    s_diagnostics.totalFeedforwardVq_V =
        frame->feedforward.total_V.q;

    s_diagnostics.requestedVd_V =
        frame->requestedVoltage_V.d;
    s_diagnostics.requestedVq_V =
        frame->requestedVoltage_V.q;

    s_diagnostics.commandedVd_V =
        frame->commandedVoltage_V.d;
    s_diagnostics.commandedVq_V =
        frame->commandedVoltage_V.q;

    s_diagnostics.voltageLimit_V = frame->voltageLimit_V;
    s_diagnostics.voltageSaturated = frame->voltageSaturated;

    s_diagnostics.dutyA = frame->duty.a;
    s_diagnostics.dutyB = frame->duty.b;
    s_diagnostics.dutyC = frame->duty.c;

    s_diagnostics.phaseResistance_Ohm = s_phaseResistance_Ohm;
    s_diagnostics.phaseInductance_H = s_phaseInductance_H;

    s_diagnostics.currentKp = s_currentKp;
    s_diagnostics.currentKi = s_currentKi;
}

/*==============================================================================
 * STRATEGY LIFECYCLE
 *===========================================================================*/

static void FocInit(void)
{
    FocResetIntegrator();
    FocLoadMotorParameters();
    FocComputeCurrentLoopGains();

    s_diagnostics = (FocDiagnostics_t){0};
}

static HalPwmCommand_t FocUpdate(
    const CommutationInputs_t *inputs)
{
    if (inputs == NULL)
    {
        return FocMakeSafeCommand();
    }

    if ((inputs->busVoltage_V <= FOC_MINIMUM_BUS_VOLTAGE_V) ||
        (inputs->dt_s <= 0.0f))
    {
        return FocMakeSafeCommand();
    }

    const BldcDQ_t measuredCurrent_A =
        FocMeasureCurrentDq(inputs);

    const BldcDQ_t targetCurrent_A = {
        .d = inputs->targetCurrentD_A,
        .q = inputs->targetCurrentQ_A
    };

    const BldcDQ_t currentError_A = {
        .d = targetCurrent_A.d - measuredCurrent_A.d,
        .q = targetCurrent_A.q - measuredCurrent_A.q
    };

    FocUpdateIntegrator(
        currentError_A,
        inputs->dt_s);

    const BldcDQ_t proportionalVoltage_V =
        FocCalculateProportionalVoltage(currentError_A);

    const BldcDQ_t piVoltage_V =
        FocCalculatePiVoltage(proportionalVoltage_V);

    const FocFeedforwardTerms_t feedforward =
        FocCalculateFeedforwardTerms(
            inputs,
            measuredCurrent_A);

    const BldcDQ_t requestedVoltage_V = {
        .d = piVoltage_V.d + feedforward.total_V.d,
        .q = piVoltage_V.q + feedforward.total_V.q
    };

    const float maxLinearVoltage_V =
        inputs->busVoltage_V * MATH_ONE_OVER_SQRT3_F;

    if (maxLinearVoltage_V <= 0.0f)
    {
        return FocMakeSafeCommand();
    }

    bool voltageSaturated = false;

    const BldcDQ_t commandedVoltage_V =
        FocApplyVoltageLimit(
            requestedVoltage_V,
            proportionalVoltage_V,
            feedforward.total_V,
            maxLinearVoltage_V,
            &voltageSaturated);

    const BldcAlphaBeta_t commandedAlphaBeta_V =
        BldcInversePark(
            commandedVoltage_V,
            inputs->electricalAngle_rad);

    const BldcAlphaBeta_t normalizedAlphaBeta = {
        .alpha =
            commandedAlphaBeta_V.alpha / maxLinearVoltage_V,
        .beta =
            commandedAlphaBeta_V.beta / maxLinearVoltage_V
    };

    const BldcPhaseABC_t duty =
        BldcSpaceVectorModulation(normalizedAlphaBeta);

    if (!BldcIsDutyCycleValid(duty))
    {
        return FocMakeSafeCommand();
    }

    const FocDiagnosticFrame_t diagnosticFrame = {
        .measuredCurrent_A = measuredCurrent_A,
        .targetCurrent_A = targetCurrent_A,
        .currentError_A = currentError_A,

        .proportionalVoltage_V = proportionalVoltage_V,
        .piVoltage_V = piVoltage_V,
        .feedforward = feedforward,

        .requestedVoltage_V = requestedVoltage_V,
        .commandedVoltage_V = commandedVoltage_V,

        .voltageLimit_V = maxLinearVoltage_V,
        .voltageSaturated = voltageSaturated,

        .duty = duty
    };

    FocPublishDiagnostics(&diagnosticFrame);

    return FocMakeActiveCommand(duty);
}

static HalPwmCommand_t FocStop(void)
{
    FocResetIntegrator();

    return FocMakeSafeCommand();
}

/*==============================================================================
 * PUBLIC DIAGNOSTIC AND CONFIGURATION API
 *===========================================================================*/

FocDiagnostics_t FocGetDiagnostics(void)
{
    return s_diagnostics;
}

void FocSetCurrentLoopBandwidth(
    float bandwidth_radPerSec)
{
    if (bandwidth_radPerSec <= 0.0f)
    {
        return;
    }

    s_currentLoopBandwidth_radPerSec =
        bandwidth_radPerSec;

    FocComputeCurrentLoopGains();
}

/*==============================================================================
 * PUBLIC STRATEGY EXPORT
 *===========================================================================*/

const CommutationStrategy_t g_focStrategy =
{
    .Init = FocInit,
    .Update = FocUpdate,
    .Stop = FocStop
};