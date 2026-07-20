#include "MathUtil.h"

#include <float.h>
#include <math.h>

/*=============================================================================
 * FILE: MathUtil.c
 *
 * PURPOSE:
 * Implements small, generic math helpers used throughout the motor-control
 * stack.
 *
 * OWNERSHIP:
 * - generic scalar math helpers
 * - generic trigonometric helper wrappers
 * - fast approximation helpers
 * - deterministic angle wrapping
 *
 * DOES NOT OWN:
 * - BLDC-specific transforms
 * - controller policy
 * - hardware assumptions
 * - motor-specific modeling
 *
 * ARCHITECTURAL ROLE:
 * Generic math foundation beneath higher-level modules such as BldcMath,
 * Encoder, Controller, and Calibration.
 *===========================================================================*/

/*=============================================================================
 * BASIC MATH HELPERS
 *===========================================================================*/

float MathAbs(float x)                      { return fabsf(x); }
float MathSqrt(float x)                     { return sqrtf(x); }
float MathSquare(float x)                   { return x * x; }
float MathMin(float a, float b)             { return fminf(a, b); }
float MathMax(float a, float b)             { return fmaxf(a, b); }
float MathClamp(float value, float minValue, float maxValue)
{
    return fminf(fmaxf(value, minValue), maxValue);
}

/*=============================================================================
 * TRIGONOMETRY
 *===========================================================================*/

float MathSin(float x)                      { return sinf(x); }
float MathCos(float x)                      { return cosf(x); }

MathSinCos_t MathSinCos(float x)
{
    return (MathSinCos_t){
        .sin = sinf(x),
        .cos = cosf(x)
    };
}

/*=============================================================================
 * FAST APPROXIMATIONS
 *===========================================================================*/

/*
 * Fast atan2 approximation.
 *
 * Intended for places where a lightweight angle estimate is sufficient and a
 * small approximation error is acceptable.
 */
float MathFastAtan2(float y, float x)
{
    float absY = MathAbs(y);
    float absX = MathAbs(x);

    /* Safe even when x = y = 0 */
    float a = MathMin(absX, absY) / (MathMax(absX, absY) + (float)FLT_MIN);
    float s = a * a;

    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;

    if (absY > absX) r = MATH_HALF_PI_F - r;
    if (x < 0.0f)    r = MATH_PI_F - r;
    if (y < 0.0f)    r = -r;

    return r;
}

/*=============================================================================
 * ANGLE UTILITIES
 *===========================================================================*/

/*
 * Wraps x into [-range/2, +range/2).
 *
 * Uses explicit floor-based wrapping to keep behavior deterministic and guards
 * against tiny floating-point edge excursions near the interval boundary.
 */
float MathWrapSymmetric(float x, float range)
{
    float wrapped = x - range * floorf((x + 0.5f * range) / range);

    if (wrapped >=  0.5f * range) wrapped -= range;
    if (wrapped <  -0.5f * range) wrapped += range;

    return wrapped;
}

float MathWrapPi(float angleRad)
{
    return MathWrapSymmetric(angleRad, MATH_TWO_PI_F);
}