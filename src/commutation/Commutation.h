#pragma once

#include <stdint.h>

#include "Hal.h"

/*==============================================================================
 * Commutation.h
 *
 * Common interface implemented by every commutation strategy.
 *
 * This header defines the controller-to-commutation boundary. The controller
 * constructs a single physical input bundle each control cycle, and every
 * commutation strategy consumes that bundle to produce a unified PWM command.
 *
 * Strategy implementations remain interchangeable because they all implement
 * this interface.
 *============================================================================*/

/*==============================================================================
 * STRATEGY INPUTS
 *============================================================================*/

/*
 * Physical quantities supplied by the controller once per control cycle.
 *
 * Ownership:
 *  - Constructed and populated by the controller.
 *  - Read-only from the perspective of the commutation strategy.
 *
 * Individual strategies should ignore fields that are not relevant to their
 * implementation.
 */
typedef struct
{
    /* Control-loop period. */
    float dt_s;

    /* Commanded d/q voltages. */
    float targetVoltageD_V;
    float targetVoltageQ_V;

    /* Commanded d/q currents. */
    float targetCurrentD_A;
    float targetCurrentQ_A;

    /* Measured motor currents. */
    HalPhaseCurrents_t phaseCurrents_A;

    /* Measured DC bus voltage. */
    float busVoltage_V;

    /* Rotor electrical state. */
    float electricalAngle_rad;
    float electricalVelocity_radPerSec;
    uint8_t electricalSector;

} CommutationInputs_t;


/*==============================================================================
 * STRATEGY INTERFACE
 *============================================================================*/

/*
 * Common strategy interface.
 *
 * Init()
 *     Initialize any strategy-local state.
 *
 * Update()
 *     Consume one controller-owned input bundle and return the PWM command for
 *     the current control cycle.
 *
 * Stop()
 *     Return the strategy's preferred safe or idle PWM command.
 */
typedef struct
{
    void (*Init)(void);

    HalPwmCommand_t (*Update)(
        const CommutationInputs_t *inputs);

    HalPwmCommand_t (*Stop)(void);

} CommutationStrategy_t;