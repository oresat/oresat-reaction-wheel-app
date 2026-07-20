#pragma once

#include <stdbool.h>

#include "Commutation.h"

/*==============================================================================
 * Commutation_Foc.h
 *
 * PURPOSE:
 * Declares the closed-loop field-oriented current-control strategy and its
 * diagnostic/configuration interface.
 *
 * RESPONSIBILITIES:
 * - expose FOC through the common commutation strategy interface
 * - define the most recent FOC diagnostic snapshot
 * - provide current-loop bandwidth configuration
 *
 * OUT OF SCOPE:
 * - velocity, torque, and position control
 * - controller mode and state decisions
 * - startup, alignment, and calibration sequencing
 * - rotor estimation and sensor fusion
 * - HAL, PWM, ADC, or register access
 *
 * ARCHITECTURAL ROLE:
 * FOC consumes measured phase currents, electrical angle/velocity, d/q current
 * targets, DC-bus voltage, and loop period. It regulates d/q current, adds the
 * configured feedforward/decoupling terms, limits the voltage vector to the
 * linear SVM region, and returns a unified inverter command.
 *===========================================================================*/

/*==============================================================================
 * DIAGNOSTICS
 *===========================================================================*/

/**
 * Snapshot of the most recent successful FOC update.
 *
 * Voltage terminology:
 * - proportional: P-only contribution
 * - integrator: accumulated I contribution
 * - PI: proportional + integrator
 * - feedforward: sum of all enabled model-based voltage terms
 * - requested: PI + feedforward before circular limiting
 * - commanded: post-limit voltage passed to inverse Park and SVM
 */
typedef struct
{
    float measuredId_A;
    float measuredIq_A;

    float targetId_A;
    float targetIq_A;

    float errorId_A;
    float errorIq_A;

    float proportionalVd_V;
    float proportionalVq_V;

    float integratorVd_V;
    float integratorVq_V;

    float piVd_V;
    float piVq_V;

    float resistiveFfVd_V;
    float resistiveFfVq_V;

    float backEmfFfVd_V;
    float backEmfFfVq_V;

    float wlFfVd_V;
    float wlFfVq_V;

    float decouplingVd_V;
    float decouplingVq_V;

    float totalFeedforwardVd_V;
    float totalFeedforwardVq_V;

    float requestedVd_V;
    float requestedVq_V;

    float commandedVd_V;
    float commandedVq_V;

    float voltageLimit_V;
    bool voltageSaturated;

    float dutyA;
    float dutyB;
    float dutyC;

    float phaseResistance_Ohm;
    float phaseInductance_H;

    float currentKp;
    float currentKi;
} FocDiagnostics_t;

/*==============================================================================
 * PUBLIC API
 *===========================================================================*/

/** Public FOC strategy consumed through the generic commutation interface. */
extern const CommutationStrategy_t g_focStrategy;

/** Returns the most recent FOC diagnostic snapshot. */
FocDiagnostics_t FocGetDiagnostics(void);

/**
 * Sets the current-loop bandwidth used to derive PI gains.
 *
 * Non-positive values are ignored. Positive values immediately recompute:
 *   Kp = bandwidth * L
 *   Ki = bandwidth * R
 */
void FocSetCurrentLoopBandwidth(float bandwidth_radPerSec);