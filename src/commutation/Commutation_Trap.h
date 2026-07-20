#pragma once

#include "Commutation.h"

/*==============================================================================
 * Commutation_Trap.h
 *
 * PURPOSE:
 * Declares the stateless trapezoidal six-step commutation strategy.
 *
 * RESPONSIBILITIES:
 * - expose trapezoidal commutation through the common strategy interface
 * - document the supplied-angle and six-step switching conventions
 *
 * OUT OF SCOPE:
 * - controller mode selection
 * - startup, alignment, and capture sequencing
 * - electrical-angle estimation
 * - sector scheduling outside the supplied angle
 * - rotor synchronization validation
 * - direct hardware access
 *
 * ARCHITECTURAL ROLE:
 * The strategy converts a controller-supplied electrical angle and positive
 * q-axis voltage request into a unipolar six-step inverter command. During
 * active commutation, two phases are enabled and one phase is floated.
 *
 * IMPLEMENTATION NOTES:
 * - The strategy is intentionally stateless.
 * - Negative torque requests are handled by coasting rather than reversing.
 * - Sector alignment includes both a 30-degree boundary shift and a fixed PCB
 *   routing offset implemented in the corresponding source file.
 * - Startup and handoff policy remain the responsibility of higher layers.
 *===========================================================================*/

/**
 * Public trapezoidal commutation strategy.
 *
 * The instance is consumed through CommutationStrategy_t and provides the
 * standard Init, Update, and Stop lifecycle callbacks.
 */
extern const CommutationStrategy_t g_trapezoidalStrategy;