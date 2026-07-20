#pragma once

#include "Commutation.h"

/*==============================================================================
 * Commutation_Sine.h
 *
 * PURPOSE:
 * Declares the stateless sinusoidal-voltage commutation strategy.
 *
 * RESPONSIBILITIES:
 * - expose the sinusoidal strategy through the common commutation interface
 * - document the inputs and waveform convention used by the implementation
 *
 * OUT OF SCOPE:
 * - controller mode selection
 * - startup, alignment, and capture sequencing
 * - electrical-angle estimation
 * - rotor synchronization validation
 * - hardware access
 *
 * ARCHITECTURAL ROLE:
 * The strategy consumes a controller-provided electrical angle and q-axis
 * voltage command, generates three sinusoidal phase references separated by
 * 120 electrical degrees, and maps those references directly to centered PWM
 * duty cycles.
 *
 * IMPLEMENTATION NOTES:
 * - The strategy is intentionally stateless.
 * - All three inverter phases remain actively driven during normal operation.
 * - The implementation uses direct sinusoidal PWM rather than inverse Park
 *   transformation or space-vector modulation.
 * - Startup and handoff policy remain the responsibility of higher layers.
 *===========================================================================*/

/**
 * Public sinusoidal commutation strategy.
 *
 * The instance is consumed through CommutationStrategy_t and provides the
 * standard Init, Update, and Stop lifecycle callbacks.
 */
extern const CommutationStrategy_t g_sinusoidalStrategy;