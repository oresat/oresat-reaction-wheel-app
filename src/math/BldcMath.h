#pragma once

#include <stdbool.h>

#include "MathUtil.h"

/*==============================================================================
 * BldcMath.h
 *
 * BLDC-specific coordinate transforms, vector types, space vector modulation,
 * and duty-cycle validation used by the motor-control stack.
 *============================================================================*/

/* Three-phase A/B/C frame for currents, voltages, or duty cycles. */
typedef struct
{
    float a;
    float b;
    float c;
} BldcPhaseABC_t;

/* Stationary alpha/beta reference frame. */
typedef struct
{
    float alpha;
    float beta;
} BldcAlphaBeta_t;

/* Rotor-aligned d/q reference frame. */
typedef struct
{
    float d;
    float q;
} BldcDQ_t;

/*==============================================================================
 * FORWARD TRANSFORMS
 *============================================================================*/

/*
 * Converts balanced three-phase quantities into the stationary alpha/beta frame.
 *
 * Assumes:
 *     phase.a + phase.b + phase.c ≈ 0
 */
BldcAlphaBeta_t BldcClarke(BldcPhaseABC_t phase);

/* Rotates alpha/beta quantities into the rotor-aligned d/q frame. */
BldcDQ_t BldcPark(
    BldcAlphaBeta_t alphaBeta,
    float electricalAngleRad);

/*==============================================================================
 * INVERSE TRANSFORMS
 *============================================================================*/

/* Rotates d/q quantities back into the stationary alpha/beta frame. */
BldcAlphaBeta_t BldcInversePark(
    BldcDQ_t dq,
    float electricalAngleRad);

/*==============================================================================
 * SPACE VECTOR MODULATION
 *============================================================================*/

/*
 * Converts normalized alpha/beta voltage commands into phase duty cycles.
 *
 * The input must already be scaled to the linear SVM range. Returned duty cycles
 * are clamped to [0, 1].
 */
BldcPhaseABC_t BldcSpaceVectorModulation(
    BldcAlphaBeta_t alphaBetaNorm);


/*==============================================================================
 * VALIDATION
 *============================================================================*/

/* Returns true when all three duty cycles are within [0, 1]. */
bool BldcIsDutyCycleValid(BldcPhaseABC_t duty);