#include "CurrentPreprocess.h"

#include <stddef.h>

#include "Config.h"
#include "MathUtil.h"

/*=============================================================================
 * CurrentPreprocess.c
 *
 * Owns:
 * - Phase-current common-mode removal
 * - Best-2-of-3 current reconstruction
 * - PWM-duty-based automatic dropped-phase selection
 * - Current preprocessing diagnostics
 *
 * Does not own:
 * - ADC conversion
 * - PWM generation
 * - Controller policy
 * - Commutation math
 *
 * Role:
 * - Converts raw HAL phase currents into corrected currents before controller
 *   and commutation use.
 *===========================================================================*/

/*=============================================================================
 * MODULE STATE
 *===========================================================================*/

static CurrentPreprocessDiagnostics_t s_diag = {0};

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static bool CurrentPreprocess_IsDropPhaseValid(int phase)
{
    return (phase == CURRENT_PREPROCESS_DROP_A) ||
           (phase == CURRENT_PREPROCESS_DROP_B) ||
           (phase == CURRENT_PREPROCESS_DROP_C);
}

static float CurrentPreprocess_DutyEdgeDistance(float duty)
{
    float lowDistance = MathAbs(duty);
    float highDistance = MathAbs(1.0f - duty);

    return (lowDistance < highDistance) ? lowDistance : highDistance;
}

static int CurrentPreprocess_SelectAutoDropPhase(
    const HalPwmCommand_t *previousPwmCommand)
{
    if (previousPwmCommand == NULL)
    {
        return CURRENT_PREPROCESS_DROP_NONE;
    }

    if (!previousPwmCommand->enableGateDriver)
    {
        return CURRENT_PREPROCESS_DROP_NONE;
    }

    float edgeA = CurrentPreprocess_DutyEdgeDistance(
        previousPwmCommand->dutyA);

    float edgeB = CurrentPreprocess_DutyEdgeDistance(
        previousPwmCommand->dutyB);

    float edgeC = CurrentPreprocess_DutyEdgeDistance(
        previousPwmCommand->dutyC);

    float minEdge = edgeA;
    int dropPhase = CURRENT_PREPROCESS_DROP_A;

    if (edgeB < minEdge)
    {
        minEdge = edgeB;
        dropPhase = CURRENT_PREPROCESS_DROP_B;
    }

    if (edgeC < minEdge)
    {
        minEdge = edgeC;
        dropPhase = CURRENT_PREPROCESS_DROP_C;
    }

    if (minEdge > CURRENT_PREPROCESS_AUTO_DUTY_EDGE_MARGIN)
    {
        return CURRENT_PREPROCESS_DROP_NONE;
    }

    return dropPhase;
}

static HalPhaseCurrents_t CurrentPreprocess_ReconstructBest2Of3(
    HalPhaseCurrents_t currents_A,
    int dropPhase)
{
    switch (dropPhase)
    {
        case CURRENT_PREPROCESS_DROP_A:
        {
            currents_A.phaseA_A =
                -(currents_A.phaseB_A + currents_A.phaseC_A);
        } break;

        case CURRENT_PREPROCESS_DROP_B:
        {
            currents_A.phaseB_A =
                -(currents_A.phaseA_A + currents_A.phaseC_A);
        } break;

        case CURRENT_PREPROCESS_DROP_C:
        {
            currents_A.phaseC_A =
                -(currents_A.phaseA_A + currents_A.phaseB_A);
        } break;

        default:
        {
            /* No reconstruction. */
        } break;
    }

    return currents_A;
}

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

void CurrentPreprocess_Init(void)
{
    s_diag = (CurrentPreprocessDiagnostics_t){0};
    s_diag.droppedPhase = CURRENT_PREPROCESS_DROP_NONE;
    s_diag.sampleValid = true;
}

HalPhaseCurrents_t CurrentPreprocess_Apply(
    const HalPhaseCurrents_t *rawCurrents_A,
    const HalPwmCommand_t *previousPwmCommand)
{
    if (rawCurrents_A == NULL)
    {
        s_diag.badSampleCount++;
        s_diag.sampleValid = false;
        s_diag.droppedPhase = CURRENT_PREPROCESS_DROP_NONE;

        return (HalPhaseCurrents_t){
            .phaseA_A = 0.0f,
            .phaseB_A = 0.0f,
            .phaseC_A = 0.0f
        };
    }

    HalPhaseCurrents_t reconstructed_A = *rawCurrents_A;
    HalPhaseCurrents_t corrected_A = *rawCurrents_A;

    float sumBefore_A =
        rawCurrents_A->phaseA_A +
        rawCurrents_A->phaseB_A +
        rawCurrents_A->phaseC_A;

    int droppedPhase = CURRENT_PREPROCESS_DROP_NONE;

    if (CURRENT_PREPROCESS_ENABLE_BEST_2_OF_3)
    {
        if (CurrentPreprocess_IsDropPhaseValid(
                CURRENT_PREPROCESS_FORCED_DROP_PHASE))
        {
            droppedPhase = CURRENT_PREPROCESS_FORCED_DROP_PHASE;
            s_diag.forcedDropCount++;
        }
        else if (CURRENT_PREPROCESS_ENABLE_AUTO_SHUNT_SELECTION)
        {
            droppedPhase =
                CurrentPreprocess_SelectAutoDropPhase(previousPwmCommand);

            if (CurrentPreprocess_IsDropPhaseValid(droppedPhase))
            {
                s_diag.autoDropCount++;
            }
        }

        if (CurrentPreprocess_IsDropPhaseValid(droppedPhase))
        {
            reconstructed_A =
                CurrentPreprocess_ReconstructBest2Of3(
                    *rawCurrents_A,
                    droppedPhase
                );

            corrected_A = reconstructed_A;
            s_diag.reconstructCount++;

            if (droppedPhase == CURRENT_PREPROCESS_DROP_A)
            {
                s_diag.dropCountA++;
            }

            if (droppedPhase == CURRENT_PREPROCESS_DROP_B)
            {
                s_diag.dropCountB++;
            }

            if (droppedPhase == CURRENT_PREPROCESS_DROP_C)
            {
                s_diag.dropCountC++;
            }
        }
    }

    float sumAfterReconstruction_A =
        reconstructed_A.phaseA_A +
        reconstructed_A.phaseB_A +
        reconstructed_A.phaseC_A;

    float commonMode_A = sumAfterReconstruction_A / 3.0f;

    if (CURRENT_PREPROCESS_ENABLE_COMMON_MODE_REMOVAL)
    {
        corrected_A.phaseA_A -= commonMode_A;
        corrected_A.phaseB_A -= commonMode_A;
        corrected_A.phaseC_A -= commonMode_A;
    }

    float sumAfter_A =
        corrected_A.phaseA_A +
        corrected_A.phaseB_A +
        corrected_A.phaseC_A;

    bool valid = true;

    if ((MathAbs(rawCurrents_A->phaseA_A) > CURRENT_PREPROCESS_ABS_MAX_A) ||
        (MathAbs(rawCurrents_A->phaseB_A) > CURRENT_PREPROCESS_ABS_MAX_A) ||
        (MathAbs(rawCurrents_A->phaseC_A) > CURRENT_PREPROCESS_ABS_MAX_A))
    {
        valid = false;
    }

    if (!valid)
    {
        s_diag.badSampleCount++;
    }

    s_diag.raw_A = *rawCurrents_A;
    s_diag.reconstructed_A = reconstructed_A;
    s_diag.corrected_A = corrected_A;

    s_diag.commonMode_A = commonMode_A;
    s_diag.sumBefore_A = sumBefore_A;
    s_diag.sumAfterReconstruction_A = sumAfterReconstruction_A;
    s_diag.sumAfter_A = sumAfter_A;

    s_diag.droppedPhase = droppedPhase;
    s_diag.sampleValid = valid;

    return corrected_A;
}

CurrentPreprocessDiagnostics_t CurrentPreprocess_GetDiagnostics(void)
{
    return s_diag;
}