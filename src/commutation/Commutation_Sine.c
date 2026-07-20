#include "Commutation_Sine.h"

#include <stddef.h>

#include "MathUtil.h"

/*==============================================================================
 * Commutation_Sine.c
 *
 * PURPOSE:
 * Implements direct three-phase sinusoidal voltage commutation.
 *
 * PROCESSING PIPELINE:
 * 1. validate the commutation input and DC-bus voltage
 * 2. advance the supplied rotor angle by 90 electrical degrees
 * 3. normalize and limit the requested q-axis voltage
 * 4. generate three phase references separated by 120 electrical degrees
 * 5. convert the references into centered PWM duty cycles
 *
 * VOLTAGE AND SIGN CONVENTION:
 * The existing implementation applies a negative q-axis modulation sign and
 * then advances the supplied electrical angle by +pi/2. These conventions are
 * intentionally preserved because together they define the established motor
 * direction and phase relationship.
 *
 * SAFETY BEHAVIOR:
 * Invalid inputs, an insufficient DC bus, or an explicit strategy stop return
 * a centered, fully floated command with the gate driver disabled.
 *
 * STATE:
 * This strategy owns no persistent runtime state.
 *===========================================================================*/

/*==============================================================================
 * PRIVATE CONSTANTS
 *===========================================================================*/

/* Minimum bus voltage accepted by the active modulation path. */
static const float SINE_MINIMUM_BUS_VOLTAGE_V = 0.1f;

/*
 * Maximum absolute direct-SPWM modulation amplitude.
 *
 * A value below 0.5 preserves margin from the 0% and 100% duty rails.
 */
static const float SINE_MAX_MODULATION = 0.45f;

/* Center duty representing zero average phase voltage. */
static const float SINE_CENTER_DUTY = 0.5f;

/* Electrical separation between adjacent three-phase references. */
static const float SINE_PHASE_SEPARATION_RAD = MATH_TWO_PI_F / 3.0f;

/* q-axis torque-producing angle advance. */
static const float SINE_Q_AXIS_ADVANCE_RAD = MATH_PI_F / 2.0f;

/*==============================================================================
 * PRIVATE COMMAND CONSTRUCTION
 *===========================================================================*/

/**
 * Builds the inactive command used for invalid input and stop handling.
 */
static HalPwmCommand_t SineMakeSafeCommand(void)
{
    return (HalPwmCommand_t){
        .dutyA = SINE_CENTER_DUTY,
        .dutyB = SINE_CENTER_DUTY,
        .dutyC = SINE_CENTER_DUTY,

        .floatA = true,
        .floatB = true,
        .floatC = true,

        .enableGateDriver = false
    };
}

/**
 * Builds an active three-phase command from normalized phase references.
 *
 * Each phase reference is expected to lie near [-1, +1]. The final duty clamp
 * remains in place as a defensive boundary even though the modulation command
 * is independently limited.
 */
static HalPwmCommand_t SineMakeActiveCommand(
    float modulation,
    float phaseReferenceA,
    float phaseReferenceB,
    float phaseReferenceC)
{
    return (HalPwmCommand_t){
        .dutyA = MathClamp(
            SINE_CENTER_DUTY + (modulation * phaseReferenceA),
            0.0f,
            1.0f),
        .dutyB = MathClamp(
            SINE_CENTER_DUTY + (modulation * phaseReferenceB),
            0.0f,
            1.0f),
        .dutyC = MathClamp(
            SINE_CENTER_DUTY + (modulation * phaseReferenceC),
            0.0f,
            1.0f),

        .floatA = false,
        .floatB = false,
        .floatC = false,

        .enableGateDriver = true
    };
}

/*==============================================================================
 * PRIVATE MODULATION HELPERS
 *===========================================================================*/

 /**
 * Converts the requested q-axis voltage into the established SPWM modulation
 * convention.
 *
 * The negative sign is intentional and preserves the existing phase/direction
 * convention. The returned value is bounded to the configured linear margin.
 */
static float SineCalculateModulation(
    float targetVoltageQ_V,
    float busVoltage_V)
{
    const float requestedModulation = -targetVoltageQ_V / busVoltage_V;

    return MathClamp(
        requestedModulation,
        -SINE_MAX_MODULATION,
        SINE_MAX_MODULATION);
}

/**
 * Generates the three cosine phase projections for one stator-vector angle.
 */
static void SineCalculatePhaseReferences(
    float statorAngle_rad,
    float *phaseReferenceA,
    float *phaseReferenceB,
    float *phaseReferenceC)
{
    *phaseReferenceA = MathCos(statorAngle_rad);
    *phaseReferenceB = MathCos(
        statorAngle_rad - SINE_PHASE_SEPARATION_RAD);
    *phaseReferenceC = MathCos(
        statorAngle_rad + SINE_PHASE_SEPARATION_RAD);
}

/*==============================================================================
 * STRATEGY LIFECYCLE
 *===========================================================================*/

static void SineInit(void)
{
    /* Stateless strategy: no initialization is required. */
}

static HalPwmCommand_t SineUpdate(const CommutationInputs_t *inputs)
{
    if (inputs == NULL)
    {
        return SineMakeSafeCommand();
    }

    if (inputs->busVoltage_V <= SINE_MINIMUM_BUS_VOLTAGE_V)
    {
        return SineMakeSafeCommand();
    }

    /*
     * The supplied angle represents rotor electrical position. Advancing it by
     * 90 electrical degrees produces the torque-oriented stator-vector angle
     * used by this strategy.
     */
    const float statorAngle_rad =
        inputs->electricalAngle_rad + SINE_Q_AXIS_ADVANCE_RAD;

    const float modulation = SineCalculateModulation(
        inputs->targetVoltageQ_V,
        inputs->busVoltage_V);

    float phaseReferenceA;
    float phaseReferenceB;
    float phaseReferenceC;

    SineCalculatePhaseReferences(
        statorAngle_rad,
        &phaseReferenceA,
        &phaseReferenceB,
        &phaseReferenceC);

    return SineMakeActiveCommand(
        modulation,
        phaseReferenceA,
        phaseReferenceB,
        phaseReferenceC);
}

static HalPwmCommand_t SineStop(void)
{
    return SineMakeSafeCommand();
}

/*==============================================================================
 * PUBLIC STRATEGY EXPORT
 *===========================================================================*/

const CommutationStrategy_t g_sinusoidalStrategy =
{
    .Init = SineInit,
    .Update = SineUpdate,
    .Stop = SineStop
};