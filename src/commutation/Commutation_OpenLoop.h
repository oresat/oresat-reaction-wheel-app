#pragma once

#include "Commutation.h"

/*=============================================================================
 * Commutation_OpenLoop.h
 *
 * PURPOSE:
 * Declares the open-loop vector commutation strategy.
 *
 * OWNERSHIP:
 * - public declaration of the open-loop strategy instance
 *
 * OUT OF SCOPE:
 * - controller mode selection
 * - startup sequencing
 * - forced-angle generation
 * - handoff logic
 * - hardware access
 *
 * ARCHITECTURAL ROLE:
 * This strategy consumes a controller-supplied electrical angle and controller-
 * supplied d/q voltage commands, then synthesizes a three-phase inverter
 * command from them.
 *
 * NOTES:
 * - This strategy is intentionally stateless.
 * - It does not decide what electrical angle to use.
 * - Startup / forced-angle sequencing belongs above this strategy.
 *===========================================================================*/

/* Public strategy instance consumed through the generic commutation interface. */
extern const CommutationStrategy_t g_openLoopStrategy;