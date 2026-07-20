#pragma once

#include <stdbool.h>

/*==============================================================================
 * MathUtil.h
 *
 * Generic scalar, trigonometric, and angle utilities shared across the firmware.
 * Motor-specific transforms and control logic belong in higher-level modules.
 *============================================================================*/

/*==============================================================================
 * CONSTANTS
 *============================================================================*/

static const float MATH_PI_F             = 3.14159265358979323846f;
static const float MATH_TWO_PI_F         = 6.28318530717958647692f;
static const float MATH_HALF_PI_F        = 1.57079632679489661923f;
static const float MATH_RAD_TO_DEG_F     = 57.2957795130823208768f;
static const float MATH_DEG_TO_RAD_F     = 0.01745329251994329577f;

static const float MATH_ONE_OVER_SQRT3_F = 0.57735026919f;
static const float MATH_TWO_OVER_SQRT3_F = 1.15470053838f;
static const float MATH_SQRT3_OVER_2_F   = 0.86602540378f;

/*==============================================================================
 * TYPES
 *============================================================================*/

/* Sine and cosine evaluated for the same input angle. */
typedef struct
{
    float sin;
    float cos;
} MathSinCos_t;

/*==============================================================================
 * SCALAR UTILITIES
 *============================================================================*/

float MathAbs(float x);
float MathSqrt(float x);
float MathSquare(float x);
float MathMin(float a, float b);
float MathMax(float a, float b);
float MathClamp(float value, float minValue, float maxValue);

/*==============================================================================
 * TRIGONOMETRY
 *============================================================================*/

float MathSin(float x);
float MathCos(float x);
MathSinCos_t MathSinCos(float x);

/*==============================================================================
 * ANGLE UTILITIES
 *============================================================================*/

/* Wraps x into the interval [-range / 2, range / 2). */
float MathWrapSymmetric(float x, float range);

/* Wraps an angle into the interval [-pi, pi). */
float MathWrapPi(float angleRad);

/*==============================================================================
 * APPROXIMATIONS
 *============================================================================*/

/* Returns a fast approximation of atan2(y, x). */
float MathFastAtan2(float y, float x);

/*==============================================================================
 * INLINE UTILITIES
 *============================================================================*/

static inline bool MathIsNaN(float x)
{
    return __builtin_isnan(x);
}