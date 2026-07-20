#include "BldcMath.h"

/*==============================================================================
 * BldcMath.c
 *
 * Implements BLDC coordinate transforms, space vector modulation, and output
 * validation. All transforms use the firmware's shared frame conventions.
 *============================================================================*/

/*==============================================================================
 * FORWARD TRANSFORMS
 *============================================================================*/

BldcAlphaBeta_t BldcClarke(BldcPhaseABC_t phase)
{
    return (BldcAlphaBeta_t)
    {
        .alpha = phase.a,
        .beta = MATH_ONE_OVER_SQRT3_F * (phase.b - phase.c),
    };
}

BldcDQ_t BldcPark(
    BldcAlphaBeta_t alphaBeta,
    float electricalAngleRad)
{
    const MathSinCos_t sinCos = MathSinCos(electricalAngleRad);

    return (BldcDQ_t)
    {
        .d = alphaBeta.alpha * sinCos.cos
             + alphaBeta.beta * sinCos.sin,
        .q = -alphaBeta.alpha * sinCos.sin
             + alphaBeta.beta * sinCos.cos,
    };
}

/*==============================================================================
 * INVERSE TRANSFORMS
 *============================================================================*/

BldcAlphaBeta_t BldcInversePark(
    BldcDQ_t dq,
    float electricalAngleRad)
{
    const MathSinCos_t sinCos = MathSinCos(electricalAngleRad);

    return (BldcAlphaBeta_t)
    {
        .alpha = dq.d * sinCos.cos - dq.q * sinCos.sin,
        .beta = dq.d * sinCos.sin + dq.q * sinCos.cos,
    };
}

/*==============================================================================
 * SPACE VECTOR MODULATION
 *============================================================================*/

BldcPhaseABC_t BldcSpaceVectorModulation(
    BldcAlphaBeta_t alphaBetaNorm)
{
    const float alpha = alphaBetaNorm.alpha;
    const float beta = alphaBetaNorm.beta;

    float dutyA = 0.5f;
    float dutyB = 0.5f;
    float dutyC = 0.5f;
    int sextant;

    /* Determine the active SVM sextant. */
    if (beta >= 0.0f)
    {
        if (alpha >= 0.0f)
        {
            sextant =
                (MATH_ONE_OVER_SQRT3_F * beta > alpha) ? 2 : 1;
        }
        else
        {
            sextant =
                (-MATH_ONE_OVER_SQRT3_F * beta > alpha) ? 3 : 2;
        }
    }
    else
    {
        if (alpha >= 0.0f)
        {
            sextant =
                (-MATH_ONE_OVER_SQRT3_F * beta > alpha) ? 5 : 6;
        }
        else
        {
            sextant =
                (MATH_ONE_OVER_SQRT3_F * beta > alpha) ? 4 : 5;
        }
    }

    /* Compute active-vector dwell times for the selected sextant. */
    switch (sextant)
    {
        case 1:
        {
            const float t1 =
                alpha - MATH_ONE_OVER_SQRT3_F * beta;
            const float t2 =
                MATH_TWO_OVER_SQRT3_F * beta;

            dutyA = 0.5f * (1.0f - t1 - t2);
            dutyB = dutyA + t1;
            dutyC = dutyB + t2;
            break;
        }

        case 2:
        {
            const float t2 =
                alpha + MATH_ONE_OVER_SQRT3_F * beta;
            const float t3 =
                -alpha + MATH_ONE_OVER_SQRT3_F * beta;

            dutyB = 0.5f * (1.0f - t2 - t3);
            dutyA = dutyB + t3;
            dutyC = dutyA + t2;
            break;
        }

        case 3:
        {
            const float t3 =
                MATH_TWO_OVER_SQRT3_F * beta;
            const float t4 =
                -alpha - MATH_ONE_OVER_SQRT3_F * beta;

            dutyB = 0.5f * (1.0f - t3 - t4);
            dutyC = dutyB + t3;
            dutyA = dutyC + t4;
            break;
        }

        case 4:
        {
            const float t4 =
                -alpha + MATH_ONE_OVER_SQRT3_F * beta;
            const float t5 =
                -MATH_TWO_OVER_SQRT3_F * beta;

            dutyC = 0.5f * (1.0f - t4 - t5);
            dutyB = dutyC + t5;
            dutyA = dutyB + t4;
            break;
        }

        case 5:
        {
            const float t5 =
                -alpha - MATH_ONE_OVER_SQRT3_F * beta;
            const float t6 =
                alpha - MATH_ONE_OVER_SQRT3_F * beta;

            dutyC = 0.5f * (1.0f - t5 - t6);
            dutyA = dutyC + t5;
            dutyB = dutyA + t6;
            break;
        }

        case 6:
        {
            const float t6 =
                -MATH_TWO_OVER_SQRT3_F * beta;
            const float t1 =
                alpha + MATH_ONE_OVER_SQRT3_F * beta;

            dutyA = 0.5f * (1.0f - t6 - t1);
            dutyC = dutyA + t1;
            dutyB = dutyC + t6;
            break;
        }

        default:
        {
            dutyA = 0.5f;
            dutyB = 0.5f;
            dutyC = 0.5f;
            break;
        }
    }

    return (BldcPhaseABC_t)
    {
        .a = MathClamp(dutyA, 0.0f, 1.0f),
        .b = MathClamp(dutyB, 0.0f, 1.0f),
        .c = MathClamp(dutyC, 0.0f, 1.0f),
    };
}

/*==============================================================================
 * VALIDATION
 *============================================================================*/

bool BldcIsDutyCycleValid(BldcPhaseABC_t duty)
{
    return duty.a >= 0.0f && duty.a <= 1.0f
        && duty.b >= 0.0f && duty.b <= 1.0f
        && duty.c >= 0.0f && duty.c <= 1.0f;
}