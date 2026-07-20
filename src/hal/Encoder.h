#pragma once

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Encoder.h
 *
 * Public interface for the MA732 rotor-state estimator.
 *
 * The HAL supplies raw encoder counts and the elapsed time between samples.
 * This module owns the resulting wrapped mechanical state and derived electrical
 * state used by the control and commutation layers.
 *
 * Units:
 *   - Mechanical position: revolutions (rev)
 *   - Mechanical velocity: revolutions per second (rev/s)
 *   - Electrical angle: radians (rad)
 *   - Electrical velocity: radians per second (rad/s)
 *
 * The mechanical position remains wrapped to a single revolution. Electrical
 * state includes the configured direction convention and electrical offset.
 *============================================================================*/

/*==============================================================================
 * STATE TYPES
 *============================================================================*/

typedef struct
{
    float position_rev;       /* Wrapped mechanical position. */
    float velocity_revPerSec; /* Mechanical velocity. */
} EncoderMechanicalState_t;

typedef struct
{
    float angle_rad;          /* Wrapped electrical angle. */
    float velocity_radPerSec; /* Electrical angular velocity. */
} EncoderElectricalState_t;

/*==============================================================================
 * INITIALIZATION AND CONFIGURATION
 *============================================================================*/

/* Resets estimator and debug state. */
void EncoderInit(void);

/* Sets the electrical direction convention to either +1 or -1. */
void EncoderSetDirection(float direction);

/* Sets and wraps the electrical-angle offset, in radians. */
void EncoderSetElectricalOffset(float offset_rad);

/*==============================================================================
 * ESTIMATOR UPDATE
 *============================================================================*/

/*
 * Incorporates one raw encoder sample into the estimator.
 *
 * rawCounts:
 *   Encoder position in the range [0, ENCODER_CPR - 1].
 *
 * dt_s:
 *   Time elapsed since the previous encoder update. This is the encoder sample
 *   interval, not necessarily the controller-loop interval.
 */
void EncoderUpdate(uint16_t rawCounts, float dt_s);

/*==============================================================================
 * STATE ACCESS
 *============================================================================*/

EncoderMechanicalState_t EncoderGetMechanicalState(void);
EncoderElectricalState_t EncoderGetElectricalState(void);