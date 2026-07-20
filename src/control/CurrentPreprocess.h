#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "Hal.h"

/*==============================================================================
 * CurrentPreprocess.h
 *
 * PURPOSE:
 * Defines the current-measurement preprocessing interface used between the HAL
 * current cache and the controller / commutation layers.
 *
 * RESPONSIBILITIES:
 * - optional best-two-of-three phase-current reconstruction
 * - optional three-phase common-mode removal
 * - dropped-phase selection and accounting
 * - current-sense validation diagnostics
 *
 * OUT OF SCOPE:
 * - ADC acquisition and counts-to-amperes conversion
 * - PWM generation
 * - controller policy
 * - commutation mathematics
 *
 * DATA FLOW:
 *   HAL currents -> optional reconstruction -> optional common-mode removal
 *                -> corrected currents + diagnostics
 *
 * UNITS:
 * - all current values are expressed in amperes
 *===========================================================================*/

/*==============================================================================
 * DROPPED-PHASE SELECTION
 *===========================================================================*/

typedef enum
{
    CURRENT_PREPROCESS_DROP_NONE = -1,
    CURRENT_PREPROCESS_DROP_A    = 0,
    CURRENT_PREPROCESS_DROP_B    = 1,
    CURRENT_PREPROCESS_DROP_C    = 2
} CurrentPreprocessDropPhase_t;

/*==============================================================================
 * DIAGNOSTICS
 *===========================================================================*/

typedef struct
{
    /* Current snapshots from each preprocessing stage. */
    HalPhaseCurrents_t raw_A;
    HalPhaseCurrents_t reconstructed_A;
    HalPhaseCurrents_t corrected_A;

    /* Three-phase current sums and removed common-mode component. */
    float commonMode_A;
    float sumBefore_A;
    float sumAfterReconstruction_A;
    float sumAfter_A;

    /* Drop decision used for the most recent valid input sample. */
    int droppedPhase;

    /* Lifetime reconstruction and selection counters. */
    uint32_t reconstructCount;
    uint32_t autoDropCount;
    uint32_t forcedDropCount;

    /* Lifetime per-phase reconstruction counters. */
    uint32_t dropCountA;
    uint32_t dropCountB;
    uint32_t dropCountC;

    /* Lifetime count of rejected or missing input samples. */
    uint32_t badSampleCount;

    /* Validity of the most recently processed sample. */
    bool sampleValid;
} CurrentPreprocessDiagnostics_t;

/*==============================================================================
 * PUBLIC API
 *===========================================================================*/

/**
 * Resets preprocessing diagnostics and restores the default no-drop state.
 */
void CurrentPreprocess_Init(void);

/**
 * Applies the configured current preprocessing pipeline.
 *
 * rawCurrents_A
 *     Raw phase currents from the HAL, in amperes. A null pointer is treated as
 *     an invalid sample and returns zero currents.
 *
 * previousPwmCommand
 *     Previous PWM command used only by automatic shunt selection. May be null.
 *
 * return
 *     Corrected phase currents in amperes.
 */
HalPhaseCurrents_t CurrentPreprocess_Apply(
    const HalPhaseCurrents_t *rawCurrents_A,
    const HalPwmCommand_t *previousPwmCommand);

/**
 * Returns a snapshot of the current preprocessing diagnostics.
 */
CurrentPreprocessDiagnostics_t CurrentPreprocess_GetDiagnostics(void);