#include "Encoder.h"

#include "Config.h"
#include "MathUtil.h"

/*==============================================================================
 * Encoder.c
 *
 * Converts raw MA732 position counts into wrapped mechanical position,
 * mechanical velocity, electrical angle, and electrical velocity.
 *
 * The HAL owns SPI acquisition and supplies the measured sample interval.
 * This module owns estimator state, direction convention, electrical offset,
 * invalid-dt recovery, and encoder-glitch recovery.
 *============================================================================*/

/*==============================================================================
 * ESTIMATOR CONFIGURATION
 *============================================================================*/

/*
 * Second-order PLL observer gains:
 *
 *   Kp = 2 * bandwidth
 *   Ki = bandwidth^2
 */
static const float ENCODER_PLL_KP =
    2.0f * ENCODER_ESTIMATOR_BANDWIDTH;

static const float ENCODER_PLL_KI =
    ENCODER_ESTIMATOR_BANDWIDTH * ENCODER_ESTIMATOR_BANDWIDTH;

/*==============================================================================
 * ESTIMATOR STATE
 *============================================================================*/

static float g_mechanicalPosition_estRev = 0.0f;
static float g_mechanicalVelocity_estRevPerSec = 0.0f;
static float g_lastMechanicalMeasurement_rev = 0.0f;

static float g_directionSign = 1.0f;
static float g_electricalOffset_rad = 0.0f;

static bool g_isInitialized = false;

/*==============================================================================
 * DEBUG STATE
 *============================================================================*/

static uint32_t g_glitchCount = 0u;
static uint32_t g_dtResetCount = 0u;
static uint32_t g_glitchResetCount = 0u;
static uint32_t g_encoderDebugPrintDivider = 0u;

/*==============================================================================
 * PRIVATE HELPERS
 *============================================================================*/

static float EncoderCountsToMechanicalRevolutions(uint16_t rawCounts)
{
    return (float)rawCounts / (float)ENCODER_CPR;
}

/*
 * Re-seeds the estimator from a trusted mechanical measurement.
 *
 * This common path is used for first-sample initialization, invalid-dt
 * recovery, and glitch recovery. It intentionally clears velocity while
 * preserving the configured direction and electrical offset.
 */
static void EncoderResetEstimate(float mechanicalMeasurement_rev)
{
    g_mechanicalPosition_estRev = mechanicalMeasurement_rev;
    g_mechanicalVelocity_estRevPerSec = 0.0f;
    g_lastMechanicalMeasurement_rev = mechanicalMeasurement_rev;
    g_isInitialized = true;
}

/*
 * Converts wrapped mechanical revolutions into wrapped electrical angle.
 *
 * Direction and offset are applied only to the electrical representation; the
 * mechanical estimator remains in the raw encoder convention.
 */
static float EncoderMechanicalToElectricalAngle(
    float mechanicalPosition_rev)
{
    float electricalAngle_rad =
        mechanicalPosition_rev *
        (float)MOTOR_POLE_PAIRS *
        MATH_TWO_PI_F;

    electricalAngle_rad *= g_directionSign;
    electricalAngle_rad += g_electricalOffset_rad;

    return MathWrapPi(electricalAngle_rad);
}

#if ENCODER_RAW_DEBUG_PRINT
static bool EncoderDebugPrintDue(void)
{
    g_encoderDebugPrintDivider++;

    if (g_encoderDebugPrintDivider < ENCODER_DEBUG_PRINT_DIVIDER)
    {
        return false;
    }

    g_encoderDebugPrintDivider = 0u;
    return true;
}

static void EncoderPrintDtReset(
    uint16_t rawCounts,
    float dt_s)
{
    if (!EncoderDebugPrintDue())
    {
        return;
    }

    printk(
        "[ENC][DT RESET] dt=%0.6f ms | raw=%u | "
        "dtResets=%u | glitchResets=%u | glitches=%u\n",
        (double)(dt_s * 1000.0f),
        (unsigned int)rawCounts,
        (unsigned int)g_dtResetCount,
        (unsigned int)g_glitchResetCount,
        (unsigned int)g_glitchCount);
}

static void EncoderPrintGlitchReset(
    uint16_t rawCounts,
    float dt_s,
    float positionError_rev)
{
    if (!EncoderDebugPrintDue())
    {
        return;
    }

    printk(
        "[ENC][GLITCH RESET] err=%0.6f rev | raw=%u | dt=%0.6f ms | "
        "dtResets=%u | glitchResets=%u | glitches=%u\n",
        (double)positionError_rev,
        (unsigned int)rawCounts,
        (double)(dt_s * 1000.0f),
        (unsigned int)g_dtResetCount,
        (unsigned int)g_glitchResetCount,
        (unsigned int)g_glitchCount);
}
#endif

/*==============================================================================
 * PUBLIC API
 *============================================================================*/

void EncoderInit(void)
{
    g_mechanicalPosition_estRev = 0.0f;
    g_mechanicalVelocity_estRevPerSec = 0.0f;
    g_lastMechanicalMeasurement_rev = 0.0f;

    g_isInitialized = false;

    g_glitchCount = 0u;
    g_dtResetCount = 0u;
    g_glitchResetCount = 0u;
    g_encoderDebugPrintDivider = 0u;
}

void EncoderUpdate(uint16_t rawCounts, float dt_s)
{
    const float mechanicalMeasurement_rev =
        EncoderCountsToMechanicalRevolutions(rawCounts);

    /*
     * Invalid or implausibly large sample intervals invalidate the observer
     * integration step. Re-seed from the current measurement instead.
     */
    if ((dt_s <= 0.0f) || (dt_s > ENCODER_MAX_UPDATE_DT_S))
    {
        g_dtResetCount++;
        EncoderResetEstimate(mechanicalMeasurement_rev);

#if ENCODER_RAW_DEBUG_PRINT
        EncoderPrintDtReset(rawCounts, dt_s);
#endif
        return;
    }

    if (!g_isInitialized)
    {
        EncoderResetEstimate(mechanicalMeasurement_rev);
        return;
    }

    const float positionError_rev =
        MathWrapSymmetric(
            mechanicalMeasurement_rev - g_mechanicalPosition_estRev,
            1.0f);

    /*
     * A position innovation larger than the configured threshold is treated as
     * a corrupted sample or discontinuity. Re-seeding avoids integrating the
     * discontinuity into the velocity estimate.
     */
    if (MathAbs(positionError_rev) > ENCODER_GLITCH_THRESHOLD_REV)
    {
        g_glitchCount++;
        g_glitchResetCount++;

        EncoderResetEstimate(mechanicalMeasurement_rev);

#if ENCODER_RAW_DEBUG_PRINT
        EncoderPrintGlitchReset(rawCounts, dt_s, positionError_rev);
#endif
        return;
    }

    /*
     * Second-order PLL observer:
     *
     *   position += (velocity + Kp * error) * dt
     *   velocity += Ki * error * dt
     */
    g_mechanicalPosition_estRev +=
        (g_mechanicalVelocity_estRevPerSec +
         (ENCODER_PLL_KP * positionError_rev)) *
        dt_s;

    g_mechanicalVelocity_estRevPerSec +=
        ENCODER_PLL_KI * positionError_rev * dt_s;

    g_mechanicalPosition_estRev =
        MathWrapSymmetric(g_mechanicalPosition_estRev, 1.0f);

    g_lastMechanicalMeasurement_rev = mechanicalMeasurement_rev;
}

void EncoderSetDirection(float directionSign)
{
    g_directionSign = (directionSign < 0.0f) ? -1.0f : 1.0f;
}

void EncoderSetElectricalOffset(float electricalOffset_rad)
{
    g_electricalOffset_rad = MathWrapPi(electricalOffset_rad);
}

EncoderMechanicalState_t EncoderGetMechanicalState(void)
{
    return (EncoderMechanicalState_t){
        .position_rev = g_mechanicalPosition_estRev,
        .velocity_revPerSec = g_mechanicalVelocity_estRevPerSec
    };
}

EncoderElectricalState_t EncoderGetElectricalState(void)
{
    return (EncoderElectricalState_t){
        .angle_rad =
            EncoderMechanicalToElectricalAngle(
                g_mechanicalPosition_estRev),

        .velocity_radPerSec =
            g_directionSign *
            g_mechanicalVelocity_estRevPerSec *
            (float)MOTOR_POLE_PAIRS *
            MATH_TWO_PI_F
    };
}