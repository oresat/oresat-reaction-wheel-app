#include "Calibration.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Config.h"
#include "MathUtil.h"
#include "Encoder.h"
#include "Controller.h"
#include "BldcMath.h"
#include "CalibrationStore.h"

/*==============================================================================
 * Calibration.c
 *
 * PURPOSE:
 * Implements the deterministic 16 kHz calibration state machine.
 *
 * RESPONSIBILITIES:
 * - current-sensor alpha/beta offset calibration
 * - static multi-point encoder electrical-offset calibration
 * - optional forward/reverse moving phase-sweep diagnostics
 * - phase resistance and inductance identification
 * - runtime calibration snapshot management
 * - validated persistence handoff
 *
 * OUT OF SCOPE:
 * - controller mode policy
 * - HAL register access
 * - commutation-strategy implementation
 * - experiment/test sequencing
 *
 * EXECUTION CONTRACT:
 * - Calibration_Update16kHz() is called from the normal controller path.
 * - Sensor snapshots are supplied by Controller.c.
 * - The module writes calibration drive requests through CommutationInputs_t.
 * - All state transitions, counters, and command updates are synchronous with
 *   the 16 kHz loop.
 *===========================================================================*/

/*=============================================================================
 * PRIVATE TIMING CONSTANTS
 *===========================================================================*/

static const uint32_t TICKS_LOCK_TOTAL =
    (uint32_t)(1.0f * FOC_UPDATE_FREQ_HZ);

static const uint32_t TICKS_DECAY =
    800u;      /* 50 ms current decay before R/L pulse */

static const uint32_t TICKS_L_STEP =
    1u;        /* first available sample after step */

static const uint32_t TICKS_R_SETTLE =
    8000u;     /* 500 ms settle for resistance estimate */

static const uint32_t TICKS_PULSE_END =
    16000u;    /* 1.0 s total pulse window */

static const uint32_t BUS_QUALIFICATION_TICKS =
    (uint32_t)(0.25f * FOC_UPDATE_FREQ_HZ);

/*=============================================================================
 * STATE MACHINE
 *===========================================================================*/

typedef enum
{
    CAL_STATE_IDLE = 0,
    CAL_STATE_WAIT_FOR_SETTLE,
    CAL_STATE_OFFSETS,
    CAL_STATE_LOCK,
    CAL_STATE_ENCODER_OFFSET,
    CAL_STATE_PHASE_SWEEP_LOCK,
    CAL_STATE_PHASE_SWEEP_FORWARD,
    CAL_STATE_PHASE_SWEEP_REVERSE,
    CAL_STATE_PHASE_SWEEP_FINISH,
    CAL_STATE_RL_DECAY,
    CAL_STATE_RL_PULSE,
    CAL_STATE_COMPLETE
} CalibrationState_t;

static CalibrationState_t s_state = CAL_STATE_IDLE;
static CalibrationResult_t s_result = CAL_RESULT_NOT_STARTED;
static CalibrationData_t s_data = {0};

/*=============================================================================
 * CALIBRATION WORKING STATE
 *===========================================================================*/

static uint32_t s_totalTicks = 0u;
static uint32_t s_busQualificationTicks = 0u;

/* Current-offset calibration */
static float s_offsetAlpha_A = 0.0f;
static float s_offsetBeta_A = 0.0f;
static float s_offsetAccumAlpha_A = 0.0f;
static float s_offsetAccumBeta_A = 0.0f;
static uint32_t s_offsetSamples = 0u;

/* Rotor lock validation */
static uint32_t s_lockTicks = 0u;
static float s_lockAlphaAccum_A = 0.0f;
static float s_lockBetaAccum_A = 0.0f;
static uint32_t s_lockSamples = 0u;
static float s_lockAvgAlpha_A = 0.0f;
static float s_lockAvgBeta_A = 0.0f;
static float s_lockCurrentAbs_A = 0.0f;

/* Encoder electrical offset calibration */
static uint32_t s_encoderPointIndex = 0u;
static uint32_t s_encoderPointTicks = 0u;
static uint32_t s_encoderPointSamples = 0u;

static float s_encoderPointSinAccum = 0.0f;
static float s_encoderPointCosAccum = 0.0f;

static float s_encoderOffsetSinAccum = 0.0f;
static float s_encoderOffsetCosAccum = 0.0f;

static float s_encoderPointOffsets_rad[CALIBRATION_ENCODER_MULTI_POINT_COUNT] = {0.0f};
static float s_encoderMaxSpread_rad = 0.0f;

/* Moving phase-sweep diagnostic */
static uint32_t s_phaseSweepTicks = 0u;
static float s_phaseSweepAngle_rad = 0.0f;
static float s_phaseSweepVelocity_radPerSec = 0.0f;

static float s_phaseSweepForwardSin = 0.0f;
static float s_phaseSweepForwardCos = 0.0f;
static uint32_t s_phaseSweepForwardSamples = 0u;
static float s_phaseSweepForwardOffset_rad = 0.0f;

static float s_phaseSweepReverseSin = 0.0f;
static float s_phaseSweepReverseCos = 0.0f;
static uint32_t s_phaseSweepReverseSamples = 0u;
static float s_phaseSweepReverseOffset_rad = 0.0f;

/* R/L calibration */
static uint32_t s_rlTicks = 0u;
static float s_lStartCurrent_A = 0.0f;
static float s_rAccum_A = 0.0f;

/* Final calibrated values */
static float s_encoderOffset_rad = 0.0f;
static float s_phaseResistance_Ohm = 0.0f;
static float s_phaseInductance_H = 0.0f;

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/**
 * Selects the inactive open-loop calibration request.
 *
 * Current targets are intentionally not modified here because the original
 * state-machine paths only cleared voltage and angle in these states.
 */
static void Calibration_SetZeroVoltageCommand(CommutationInputs_t *cmd)
{
    cmd->targetVoltageD_V = 0.0f;
    cmd->targetVoltageQ_V = 0.0f;
    cmd->electricalAngle_rad = 0.0f;
}

/**
 * Builds the phase-sweep lock/rotation drive request.
 *
 * This preserves the existing configuration switch between FOC d-axis current
 * control and open-loop d-axis voltage drive.
 */
static void Calibration_SetPhaseSweepDriveCommand(
    CommutationInputs_t *cmd,
    float electricalAngle_rad)
{
    if (CALIBRATION_PHASE_SWEEP_USE_CURRENT_CONTROL)
    {
        cmd->targetVoltageD_V = 0.0f;
        cmd->targetVoltageQ_V = 0.0f;
        cmd->targetCurrentD_A = CALIBRATION_PHASE_SWEEP_ID_A;
        cmd->targetCurrentQ_A = 0.0f;
    }
    else
    {
        cmd->targetVoltageD_V = CALIBRATION_PHASE_SWEEP_VD_V;
        cmd->targetVoltageQ_V = 0.0f;
        cmd->targetCurrentD_A = 0.0f;
        cmd->targetCurrentQ_A = 0.0f;
    }

    cmd->electricalAngle_rad = electricalAngle_rad;
}

static void Calibration_ResetOffsetState(void)
{
    s_offsetAlpha_A = 0.0f;
    s_offsetBeta_A = 0.0f;
    s_offsetAccumAlpha_A = 0.0f;
    s_offsetAccumBeta_A = 0.0f;
    s_offsetSamples = 0u;
}

static void Calibration_ResetLockState(void)
{
    s_lockTicks = 0u;
    s_lockAlphaAccum_A = 0.0f;
    s_lockBetaAccum_A = 0.0f;
    s_lockSamples = 0u;
    s_lockAvgAlpha_A = 0.0f;
    s_lockAvgBeta_A = 0.0f;
    s_lockCurrentAbs_A = 0.0f;
}

static void Calibration_ResetEncoderState(void)
{
    s_encoderPointIndex = 0u;
    s_encoderPointTicks = 0u;
    s_encoderPointSamples = 0u;

    s_encoderPointSinAccum = 0.0f;
    s_encoderPointCosAccum = 0.0f;
    s_encoderOffsetSinAccum = 0.0f;
    s_encoderOffsetCosAccum = 0.0f;

    s_encoderMaxSpread_rad = 0.0f;
    s_encoderOffset_rad = 0.0f;
}

static void Calibration_ResetPhaseSweepState(void)
{
    s_phaseSweepTicks = 0u;
    s_phaseSweepAngle_rad = 0.0f;
    s_phaseSweepVelocity_radPerSec = 0.0f;

    s_phaseSweepForwardSin = 0.0f;
    s_phaseSweepForwardCos = 0.0f;
    s_phaseSweepForwardSamples = 0u;
    s_phaseSweepForwardOffset_rad = 0.0f;

    s_phaseSweepReverseSin = 0.0f;
    s_phaseSweepReverseCos = 0.0f;
    s_phaseSweepReverseSamples = 0u;
    s_phaseSweepReverseOffset_rad = 0.0f;
}

static void Calibration_ResetRlState(void)
{
    s_rlTicks = 0u;
    s_lStartCurrent_A = 0.0f;
    s_rAccum_A = 0.0f;
}

static void Calibration_ResetIdentifiedParameters(void)
{
    s_phaseResistance_Ohm = 0.0f;
    s_phaseInductance_H = 0.0f;
}

static CalibrationData_t Calibration_MakeEmptyRunningData(void)
{
    return (CalibrationData_t){
        .result = CAL_RESULT_RUNNING,
        .isValid = false,
        .offsetAlpha_A = 0.0f,
        .offsetBeta_A = 0.0f,
        .encoderElectricalOffset_rad = 0.0f,
        .phaseResistance_Ohm = 0.0f,
        .phaseInductance_H = 0.0f,
        .busVoltage_V = 0.0f,
        .auxTemp_C = 0.0f,
        .phaseATemp_C = 0.0f,
        .phaseBTemp_C = 0.0f,
        .phaseCTemp_C = 0.0f
    };
}

static CalibrationData_t Calibration_MakePassedData(
    const HalPower_t *power,
    HalThermal_t thermal)
{
    return (CalibrationData_t){
        .result = CAL_RESULT_PASSED,
        .isValid = true,
        .offsetAlpha_A = s_offsetAlpha_A,
        .offsetBeta_A = s_offsetBeta_A,
        .encoderElectricalOffset_rad = s_encoderOffset_rad,
        .phaseResistance_Ohm = s_phaseResistance_Ohm,
        .phaseInductance_H = s_phaseInductance_H,
        .busVoltage_V = power->busVoltage_V,
        .auxTemp_C = thermal.auxTemp_C,
        .phaseATemp_C = thermal.phaseATemp_C,
        .phaseBTemp_C = thermal.phaseBTemp_C,
        .phaseCTemp_C = thermal.phaseCTemp_C
    };
}

static float Calibration_GetEncoderCommandAngle(uint32_t pointIndex)
{
    float frac =
        (float)pointIndex /
        (float)CALIBRATION_ENCODER_MULTI_POINT_COUNT;

    return MathWrapPi(frac * MATH_TWO_PI_F);
}

static float Calibration_ComputeEncoderOffset(
    float commandedElectricalAngle_rad,
    const HalRotorState_t *rotor)
{
    float mechanicalElectrical_rad =
        rotor->mechanicalAngle_rev *
        (float)MOTOR_POLE_PAIRS *
        MATH_TWO_PI_F;

    return MathWrapPi(commandedElectricalAngle_rad - mechanicalElectrical_rad);
}

static float Calibration_ComputePhaseSweepVelocity(uint32_t ticks, float sign)
{
    if (ticks < CALIBRATION_PHASE_SWEEP_RAMP_TICKS)
    {
        float frac =
            (float)ticks /
            (float)CALIBRATION_PHASE_SWEEP_RAMP_TICKS;

        return sign *
               CALIBRATION_PHASE_SWEEP_SPEED_RAD_PER_SEC *
               frac;
    }

    return sign * CALIBRATION_PHASE_SWEEP_SPEED_RAD_PER_SEC;
}

static bool Calibration_PhaseSweepVelocityAccepted(
    const HalRotorState_t *rotor,
    float commandedVelocity_radPerSec)
{
    if (!CALIBRATION_PHASE_SWEEP_ENABLE_VELOCITY_GATE)
    {
        return true;
    }

    if (rotor == NULL || !rotor->isValid)
    {
        return false;
    }

    float commandedAbs = MathAbs(commandedVelocity_radPerSec);
    if (commandedAbs <= 0.001f)
    {
        return false;
    }

    float measuredAbs = MathAbs(rotor->electricalVelocity_radPerSec);
    float errorAbs = MathAbs(measuredAbs - commandedAbs);

    return errorAbs <=
           (CALIBRATION_PHASE_SWEEP_VELOCITY_GATE_FRACTION * commandedAbs);
}

static void Calibration_ResetEncoderPointAccum(void)
{
    s_encoderPointTicks = 0u;
    s_encoderPointSamples = 0u;
    s_encoderPointSinAccum = 0.0f;
    s_encoderPointCosAccum = 0.0f;
}

static void Calibration_Fail(CalibrationResult_t result)
{
    printk("[CAL][FAIL] %s (%d)\n",
       Calibration_ResultToString(result),
       (int)result);

    s_state = CAL_STATE_IDLE;
    s_result = result;

    s_data.result = result;
    s_data.isValid = false;
}

static bool Calibration_IsLoadedDataReasonable(const CalibrationData_t *data)
{
    if (data == NULL)
    {
        return false;
    }

    if ((data->phaseResistance_Ohm < 0.05f) ||
        (data->phaseResistance_Ohm > 0.50f))
    {
        return false;
    }

    if ((data->phaseInductance_H < 1.0e-6f) ||
        (data->phaseInductance_H > 100.0e-6f))
    {
        return false;
    }

    if ((MathAbs(data->offsetAlpha_A) > 0.25f) ||
        (MathAbs(data->offsetBeta_A) > 0.25f))
    {
        return false;
    }

    return true;
}

/*=============================================================================
 * PUBLIC QUERY API
 *===========================================================================*/

bool Calibration_IsRunning(void)
{
    return (s_state != CAL_STATE_IDLE) &&
           (s_state != CAL_STATE_COMPLETE);
}

bool Calibration_IsComplete(void)
{
    return (s_result == CAL_RESULT_PASSED) ||
           (s_result >= CAL_RESULT_FAILED_TIMEOUT);
}

bool Calibration_Passed(void)
{
    return s_result == CAL_RESULT_PASSED;
}

CalibrationResult_t Calibration_GetResult(void)
{
    return s_result;
}

const char *Calibration_ResultToString(CalibrationResult_t result)
{
    switch (result)
    {
        case CAL_RESULT_NOT_STARTED:
            return "NOT_STARTED";

        case CAL_RESULT_RUNNING:
            return "RUNNING";

        case CAL_RESULT_PASSED:
            return "PASSED";

        case CAL_RESULT_FAILED_TIMEOUT:
            return "FAILED_TIMEOUT";

        case CAL_RESULT_FAILED_BUS_VOLTAGE:
            return "FAILED_BUS_VOLTAGE";

        case CAL_RESULT_FAILED_INVALID_ROTOR:
            return "FAILED_INVALID_ROTOR";

        case CAL_RESULT_FAILED_OFFSET_RANGE:
            return "FAILED_OFFSET_RANGE";

        case CAL_RESULT_FAILED_LOCK_CURRENT:
            return "FAILED_LOCK_CURRENT";

        case CAL_RESULT_FAILED_LOCK_STABILITY:
            return "FAILED_LOCK_STABILITY";

        case CAL_RESULT_FAILED_PARAM_RANGE:
            return "FAILED_PARAM_RANGE";

        default:
            return "UNKNOWN";
    }
}

const CalibrationData_t *Calibration_GetData(void)
{
    return &s_data;
}

float Calibration_GetCurrentOffsetAlpha(void)
{
    return s_offsetAlpha_A;
}

float Calibration_GetCurrentOffsetBeta(void)
{
    return s_offsetBeta_A;
}

float Calibration_GetPhaseResistance(void)
{
    return s_phaseResistance_Ohm;
}

float Calibration_GetPhaseInductance(void)
{
    return s_phaseInductance_H;
}

bool Calibration_HasValidData(void)
{
    return s_data.isValid && (s_result == CAL_RESULT_PASSED);
}

bool Calibration_RequiresFocCommutation(void)
{
    if (!CALIBRATION_PHASE_SWEEP_USE_CURRENT_CONTROL)
    {
        return false;
    }

    return (s_state == CAL_STATE_PHASE_SWEEP_LOCK) ||
           (s_state == CAL_STATE_PHASE_SWEEP_FORWARD) ||
           (s_state == CAL_STATE_PHASE_SWEEP_REVERSE);
}

/*=============================================================================
 * PERSISTENCE API
 *===========================================================================*/

bool Calibration_LoadPersisted(void)
{
    CalibrationData_t loaded;

    if (!CalibrationStore_Load(&loaded))
    {
        return false;
    }

    if (!Calibration_IsLoadedDataReasonable(&loaded))
    {
        printk("[CAL] Persisted calibration rejected: unreasonable values\n");
        return false;
    }

    s_data = loaded;
    s_result = loaded.result;

    s_offsetAlpha_A = loaded.offsetAlpha_A;
    s_offsetBeta_A = loaded.offsetBeta_A;
    s_encoderOffset_rad = loaded.encoderElectricalOffset_rad;
    s_phaseResistance_Ohm = loaded.phaseResistance_Ohm;
    s_phaseInductance_H = loaded.phaseInductance_H;

    EncoderSetElectricalOffset(s_encoderOffset_rad);

    printk("[CAL] Loaded persisted calibration: R=%f Ohm L=%e H offset=%f rad\n",
           (double)s_phaseResistance_Ohm,
           (double)s_phaseInductance_H,
           (double)s_encoderOffset_rad);

    return true;
}

bool Calibration_SavePersisted(void)
{
    return CalibrationStore_Save(&s_data);
}

/*=============================================================================
 * START / RESET
 *===========================================================================*/

void Calibration_Begin(void)
{
    Controller_ClearFaults();
    EncoderSetElectricalOffset(0.0f);
    Calibration_Start();
}

void Calibration_Start(void)
{
    s_state = CAL_STATE_WAIT_FOR_SETTLE;
    s_result = CAL_RESULT_RUNNING;

    s_totalTicks = 0u;
    s_busQualificationTicks = 0u;

    Calibration_ResetOffsetState();
    Calibration_ResetLockState();
    Calibration_ResetEncoderState();
    Calibration_ResetPhaseSweepState();
    Calibration_ResetRlState();
    Calibration_ResetIdentifiedParameters();

    EncoderSetElectricalOffset(0.0f);

    s_data = Calibration_MakeEmptyRunningData();
}

/*=============================================================================
 * MAIN 16 kHz UPDATE
 *===========================================================================*/

void Calibration_Update16kHz(
    HalPhaseCurrents_t *currents,
    HalPower_t *power,
    HalRotorState_t *rotor,
    CommutationInputs_t *cmd)
{
    const uint32_t calibrationTimeout_ticks =
        (uint32_t)(CALIBRATION_TIMEOUT_S * FOC_UPDATE_FREQ_HZ);

    const uint32_t lockSettleTicks =
        (uint32_t)(CALIBRATION_LOCK_SETTLE_TIME_S * FOC_UPDATE_FREQ_HZ);

    if ((currents == NULL) || (power == NULL) ||
        (rotor == NULL) || (cmd == NULL))
    {
        Calibration_Fail(CAL_RESULT_FAILED_TIMEOUT);
        return;
    }

    if (s_result != CAL_RESULT_RUNNING)
    {
        return;
    }

    s_totalTicks++;

    if (s_totalTicks > calibrationTimeout_ticks)
    {
        Calibration_Fail(CAL_RESULT_FAILED_TIMEOUT);
        return;
    }

    /*
     * Vbus qualification is delayed slightly because ADC channels can need
     * a short warm-up period after boot before the cached bus reading is valid.
     */
    if (s_busQualificationTicks < BUS_QUALIFICATION_TICKS)
    {
        s_busQualificationTicks++;
    }
    else if ((power->busVoltage_V < CALIBRATION_BUS_VOLTAGE_MIN_V) ||
             (power->busVoltage_V > CALIBRATION_BUS_VOLTAGE_MAX_V))
    {

        Calibration_Fail(CAL_RESULT_FAILED_BUS_VOLTAGE);
        return;
    }

    switch (s_state)
    {
        case CAL_STATE_WAIT_FOR_SETTLE:
        {
            Calibration_SetZeroVoltageCommand(cmd);

            /*
             * Pure time delay. This avoids using noisy near-zero velocity
             * estimates as a settle criterion.
             */
            s_rlTicks++;

            if (s_rlTicks >= (uint32_t)(0.5f * FOC_UPDATE_FREQ_HZ))
            {
                s_rlTicks = 0u;
                s_state = CAL_STATE_OFFSETS;
            }
        } break;

        case CAL_STATE_OFFSETS:
        {
            Calibration_SetZeroVoltageCommand(cmd);

            BldcAlphaBeta_t ab = BldcClarke((BldcPhaseABC_t){
                .a = currents->phaseA_A,
                .b = currents->phaseB_A,
                .c = currents->phaseC_A
            });

            s_offsetAccumAlpha_A += ab.alpha;
            s_offsetAccumBeta_A += ab.beta;
            s_offsetSamples++;

            if (s_offsetSamples >= CURRENT_OFFSET_SAMPLE_COUNT)
            {
                s_offsetAlpha_A =
                    s_offsetAccumAlpha_A / (float)s_offsetSamples;

                s_offsetBeta_A =
                    s_offsetAccumBeta_A / (float)s_offsetSamples;

                if ((MathAbs(s_offsetAlpha_A) > CALIBRATION_OFFSET_ABS_MAX_A) ||
                    (MathAbs(s_offsetBeta_A) > CALIBRATION_OFFSET_ABS_MAX_A))
                {
                    Calibration_Fail(CAL_RESULT_FAILED_OFFSET_RANGE);
                    return;
                }

                s_state = CAL_STATE_LOCK;
            }
        } break;

        case CAL_STATE_LOCK:
        {
            if (!rotor->isValid)
            {
                Calibration_Fail(CAL_RESULT_FAILED_INVALID_ROTOR);
                return;
            }

            uint32_t rampTicks = lockSettleTicks / 2u;
            float rampFrac = 1.0f;

            if ((rampTicks > 0u) && (s_lockTicks < rampTicks))
            {
                rampFrac = (float)s_lockTicks / (float)rampTicks;
            }

            cmd->targetVoltageD_V = CALIBRATION_LOCK_VOLTAGE_V * rampFrac;
            cmd->targetVoltageQ_V = 0.0f;
            cmd->electricalAngle_rad = 0.0f;

            BldcAlphaBeta_t ab = BldcClarke((BldcPhaseABC_t){
                .a = currents->phaseA_A,
                .b = currents->phaseB_A,
                .c = currents->phaseC_A
            });

            float alpha_A = ab.alpha - s_offsetAlpha_A;
            float beta_A = ab.beta - s_offsetBeta_A;

            if (s_lockTicks >= lockSettleTicks)
            {
                s_lockAlphaAccum_A += alpha_A;
                s_lockBetaAccum_A += beta_A;
                s_lockSamples++;
            }

            s_lockTicks++;

            if (s_lockTicks >= TICKS_LOCK_TOTAL)
            {
                if (s_lockSamples == 0u)
                {
                    Calibration_Fail(CAL_RESULT_FAILED_LOCK_STABILITY);
                    return;
                }

                s_lockAvgAlpha_A =
                    s_lockAlphaAccum_A / (float)s_lockSamples;

                s_lockAvgBeta_A =
                    s_lockBetaAccum_A / (float)s_lockSamples;

                s_lockCurrentAbs_A = MathSqrt(
                    (s_lockAvgAlpha_A * s_lockAvgAlpha_A) +
                    (s_lockAvgBeta_A * s_lockAvgBeta_A)
                );

                if ((s_lockCurrentAbs_A < CALIBRATION_LOCK_CURRENT_MIN_A) ||
                    (s_lockCurrentAbs_A > CALIBRATION_LOCK_CURRENT_MAX_A))
                {
                    Calibration_Fail(CAL_RESULT_FAILED_LOCK_CURRENT);
                    return;
                }

                if (MathAbs(s_lockAvgBeta_A) >
                    CALIBRATION_LOCK_BETA_ABS_MAX_A)
                {
                    Calibration_Fail(CAL_RESULT_FAILED_LOCK_STABILITY);
                    return;
                }

                s_state = CAL_STATE_ENCODER_OFFSET;
            }
        } break;

        case CAL_STATE_ENCODER_OFFSET:
        {
            if (!rotor->isValid)
            {
                Calibration_Fail(CAL_RESULT_FAILED_INVALID_ROTOR);
                return;
            }

            float commandedAngle_rad =
                Calibration_GetEncoderCommandAngle(s_encoderPointIndex);

            cmd->targetVoltageD_V = CALIBRATION_LOCK_VOLTAGE_V;
            cmd->targetVoltageQ_V = 0.0f;
            cmd->electricalAngle_rad = commandedAngle_rad;

            if (s_encoderPointTicks < CALIBRATION_ENCODER_LOCK_SETTLE_TICKS)
            {
                s_encoderPointTicks++;
                break;
            }

            float offset_rad =
                Calibration_ComputeEncoderOffset(commandedAngle_rad, rotor);

            s_encoderPointSinAccum += MathSin(offset_rad);
            s_encoderPointCosAccum += MathCos(offset_rad);
            s_encoderPointSamples++;

            if (s_encoderPointSamples >=
                CALIBRATION_ENCODER_OFFSET_SAMPLES_PER_POINT)
            {
                float pointOffset_rad = MathFastAtan2(
                    s_encoderPointSinAccum,
                    s_encoderPointCosAccum
                );

                s_encoderPointOffsets_rad[s_encoderPointIndex] =
                    pointOffset_rad;

                printk("[CAL] point=%u offset (mrad)=%d\n",
                    (unsigned int)s_encoderPointIndex,
                    (int)(pointOffset_rad * 1000.0f));

                s_encoderOffsetSinAccum += MathSin(pointOffset_rad);
                s_encoderOffsetCosAccum += MathCos(pointOffset_rad);

                s_encoderPointIndex++;

                if (s_encoderPointIndex >=
                    CALIBRATION_ENCODER_MULTI_POINT_COUNT)
                {
                    s_encoderOffset_rad = MathFastAtan2(
                        s_encoderOffsetSinAccum,
                        s_encoderOffsetCosAccum
                    );

                    s_encoderMaxSpread_rad = 0.0f;

                    for (uint32_t i = 0u;
                         i < CALIBRATION_ENCODER_MULTI_POINT_COUNT;
                         i++)
                    {
                        float deviation_rad = MathAbs(MathWrapPi(
                            s_encoderPointOffsets_rad[i] -
                            s_encoderOffset_rad
                        ));

                        printk("[CAL] point=%u deviation (mrad)=%d\n",
                        (unsigned int)i,
                        (int)(deviation_rad * 1000.0f));

                        if (deviation_rad > s_encoderMaxSpread_rad)
                        {
                            s_encoderMaxSpread_rad = deviation_rad;
                        }
                    }

                    printk("[CAL] mean offset (mrad)=%d max deviation (mrad)=%d limit (mrad)=%d\n",
                        (int)(s_encoderOffset_rad * 1000.0f),
                        (int)(s_encoderMaxSpread_rad * 1000.0f),
                        (int)(CALIBRATION_ENCODER_OFFSET_MAX_SPREAD_RAD * 1000.0f));

                    if (s_encoderMaxSpread_rad >
                        CALIBRATION_ENCODER_OFFSET_MAX_SPREAD_RAD)
                    {
                        Calibration_Fail(CAL_RESULT_FAILED_LOCK_STABILITY);
                        return;
                    }

                    EncoderSetElectricalOffset(s_encoderOffset_rad);

                    if (CALIBRATION_ENABLE_PHASE_SWEEP_DIAGNOSTIC)
                    {
                        printk("[CAL][PHASE_SWEEP] start | static offset (mrad)=%d\n",
                        (int)(s_encoderOffset_rad * 1000.0f));

                        s_phaseSweepTicks = 0u;
                        s_phaseSweepAngle_rad = 0.0f;
                        s_phaseSweepVelocity_radPerSec = 0.0f;

                        s_state = CAL_STATE_PHASE_SWEEP_LOCK;
                    }
                    else
                    {
                        s_rlTicks = 0u;
                        s_state = CAL_STATE_RL_DECAY;
                    }
                }
                else
                {
                    Calibration_ResetEncoderPointAccum();
                }
            }
        } break;

        case CAL_STATE_PHASE_SWEEP_LOCK:
        {
            if (!rotor->isValid)
            {
                Calibration_Fail(CAL_RESULT_FAILED_INVALID_ROTOR);
                return;
            }

            Calibration_SetPhaseSweepDriveCommand(cmd, 0.0f);

            s_phaseSweepTicks++;

            if (s_phaseSweepTicks >= CALIBRATION_PHASE_SWEEP_LOCK_TICKS)
            {
                s_phaseSweepTicks = 0u;
                s_phaseSweepAngle_rad = 0.0f;
                s_state = CAL_STATE_PHASE_SWEEP_FORWARD;
            }
        } break;

        case CAL_STATE_PHASE_SWEEP_FORWARD:
        {
            if (!rotor->isValid)
            {
                Calibration_Fail(CAL_RESULT_FAILED_INVALID_ROTOR);
                return;
            }

            s_phaseSweepVelocity_radPerSec =
                Calibration_ComputePhaseSweepVelocity(
                    s_phaseSweepTicks,
                    +1.0f
                );

            s_phaseSweepAngle_rad = MathWrapPi(
                s_phaseSweepAngle_rad +
                s_phaseSweepVelocity_radPerSec * FOC_UPDATE_PERIOD_S
            );

            Calibration_SetPhaseSweepDriveCommand(
                cmd,
                s_phaseSweepAngle_rad);

            if ((s_phaseSweepTicks >= CALIBRATION_PHASE_SWEEP_RAMP_TICKS) &&
                Calibration_PhaseSweepVelocityAccepted(
                    rotor,
                    s_phaseSweepVelocity_radPerSec))
            {
                float offset_rad =
                    Calibration_ComputeEncoderOffset(
                        s_phaseSweepAngle_rad,
                        rotor
                    );

                s_phaseSweepForwardSin += MathSin(offset_rad);
                s_phaseSweepForwardCos += MathCos(offset_rad);
                s_phaseSweepForwardSamples++;
            }

            s_phaseSweepTicks++;

            if (s_phaseSweepTicks >=
                (CALIBRATION_PHASE_SWEEP_RAMP_TICKS +
                 CALIBRATION_PHASE_SWEEP_CRUISE_TICKS))
            {
                if (s_phaseSweepForwardSamples <
                    CALIBRATION_PHASE_SWEEP_MIN_ACCEPTED_SAMPLES)
                {
                    Calibration_Fail(CAL_RESULT_FAILED_LOCK_STABILITY);
                    return;
                }

                s_phaseSweepForwardOffset_rad = MathFastAtan2(
                    s_phaseSweepForwardSin,
                    s_phaseSweepForwardCos
                );

                printk("[CAL][PHASE_SWEEP] forward offset=% .6f samples=%u\n",
                       (double)s_phaseSweepForwardOffset_rad,
                       (unsigned int)s_phaseSweepForwardSamples);

                s_phaseSweepTicks = 0u;
                s_state = CAL_STATE_PHASE_SWEEP_REVERSE;
            }
        } break;

        case CAL_STATE_PHASE_SWEEP_REVERSE:
        {
            if (!rotor->isValid)
            {
                Calibration_Fail(CAL_RESULT_FAILED_INVALID_ROTOR);
                return;
            }

            s_phaseSweepVelocity_radPerSec =
                Calibration_ComputePhaseSweepVelocity(
                    s_phaseSweepTicks,
                    -1.0f
                );

            s_phaseSweepAngle_rad = MathWrapPi(
                s_phaseSweepAngle_rad +
                s_phaseSweepVelocity_radPerSec * FOC_UPDATE_PERIOD_S
            );

            Calibration_SetPhaseSweepDriveCommand(
                cmd,
                s_phaseSweepAngle_rad);

            if ((s_phaseSweepTicks >= CALIBRATION_PHASE_SWEEP_RAMP_TICKS) &&
                Calibration_PhaseSweepVelocityAccepted(
                    rotor,
                    s_phaseSweepVelocity_radPerSec))
            {
                float offset_rad =
                    Calibration_ComputeEncoderOffset(
                        s_phaseSweepAngle_rad,
                        rotor
                    );

                s_phaseSweepReverseSin += MathSin(offset_rad);
                s_phaseSweepReverseCos += MathCos(offset_rad);
                s_phaseSweepReverseSamples++;
            }

            s_phaseSweepTicks++;

            if (s_phaseSweepTicks >=
                (CALIBRATION_PHASE_SWEEP_RAMP_TICKS +
                 CALIBRATION_PHASE_SWEEP_CRUISE_TICKS))
            {
                if (s_phaseSweepReverseSamples <
                    CALIBRATION_PHASE_SWEEP_MIN_ACCEPTED_SAMPLES)
                {
                    Calibration_Fail(CAL_RESULT_FAILED_LOCK_STABILITY);
                    return;
                }

                s_phaseSweepReverseOffset_rad = MathFastAtan2(
                    s_phaseSweepReverseSin,
                    s_phaseSweepReverseCos
                );

                printk("[CAL][PHASE_SWEEP] reverse offset=% .6f samples=%u\n",
                       (double)s_phaseSweepReverseOffset_rad,
                       (unsigned int)s_phaseSweepReverseSamples);

                s_state = CAL_STATE_PHASE_SWEEP_FINISH;
            }
        } break;

        case CAL_STATE_PHASE_SWEEP_FINISH:
        {
            float meanSin =
                MathSin(s_phaseSweepForwardOffset_rad) +
                MathSin(s_phaseSweepReverseOffset_rad);

            float meanCos =
                MathCos(s_phaseSweepForwardOffset_rad) +
                MathCos(s_phaseSweepReverseOffset_rad);

            float movingOffset_rad =
                MathFastAtan2(meanSin, meanCos);

            float frSpread_rad = MathAbs(MathWrapPi(
                s_phaseSweepForwardOffset_rad -
                s_phaseSweepReverseOffset_rad
            ));

            float staticDelta_rad = MathAbs(MathWrapPi(
                s_encoderOffset_rad -
                movingOffset_rad
            ));

            printk("[CAL][PHASE_SWEEP] mean=% .6f fr_spread=% .6f static_delta=% .6f\n",
                   (double)movingOffset_rad,
                   (double)frSpread_rad,
                   (double)staticDelta_rad);

            /*
             * Diagnostic only:
             * Keep using the static multi-point offset for saved calibration.
             */
            s_rlTicks = 0u;
            s_state = CAL_STATE_RL_DECAY;
        } break;

        case CAL_STATE_RL_DECAY:
        {
            Calibration_SetZeroVoltageCommand(cmd);

            s_rlTicks++;

            if (s_rlTicks >= TICKS_DECAY)
            {
                s_rlTicks = 0u;
                s_state = CAL_STATE_RL_PULSE;
            }
        } break;

        case CAL_STATE_RL_PULSE:
        {
            cmd->targetVoltageD_V = CALIBRATION_LOCK_VOLTAGE_V;
            cmd->targetVoltageQ_V = 0.0f;
            cmd->electricalAngle_rad = 0.0f;

            /*
             * Existing R/L estimate path.
             * Note: this uses phase A as a best-effort proxy for the locked
             * D-axis current. It is adequate for current testing, but V2 should
             * replace this with a cleaner identified d-axis current estimate.
             */
            float phaseCurrent_A = currents->phaseA_A - s_offsetAlpha_A;

            if (s_rlTicks == 0u)
            {
                s_lStartCurrent_A = phaseCurrent_A;
            }
            else if (s_rlTicks == TICKS_L_STEP)
            {
                float dI_A = phaseCurrent_A - s_lStartCurrent_A;
                float dt_s = (float)TICKS_L_STEP / FOC_UPDATE_FREQ_HZ;

                if (MathAbs(dI_A) > 0.02f)
                {
                    s_phaseInductance_H =
                        MathAbs(CALIBRATION_LOCK_VOLTAGE_V) *
                        dt_s /
                        MathAbs(dI_A);
                }
                else
                {
                    s_phaseInductance_H = 0.0f;
                }
            }
            else if (s_rlTicks >= TICKS_R_SETTLE)
            {
                s_rAccum_A += phaseCurrent_A;
            }

            s_rlTicks++;

            if (s_rlTicks >= TICKS_PULSE_END)
            {
                float avgCurrent_A =
                    s_rAccum_A /
                    (float)(TICKS_PULSE_END - TICKS_R_SETTLE);

                if (MathAbs(avgCurrent_A) > 0.05f)
                {
                    s_phaseResistance_Ohm =
                        MathAbs(CALIBRATION_LOCK_VOLTAGE_V) /
                        MathAbs(avgCurrent_A);
                }

                if ((s_phaseResistance_Ohm <
                     CALIBRATION_PHASE_RESISTANCE_MIN_OHM) ||
                    (s_phaseResistance_Ohm >
                     CALIBRATION_PHASE_RESISTANCE_MAX_OHM))
                {
                    Calibration_Fail(CAL_RESULT_FAILED_PARAM_RANGE);
                    return;
                }

                /*
                 * L is best-effort at the current loop rate. If the estimate is implausible, keep
                 * calibration valid and fall back to the configured nominal L.
                 */
                if ((s_phaseInductance_H <=
                     CALIBRATION_PHASE_INDUCTANCE_MIN_H) ||
                    (s_phaseInductance_H >
                     CALIBRATION_PHASE_INDUCTANCE_MAX_H))
                {
                    s_phaseInductance_H = MOTOR_PHASE_INDUCTANCE_H;
                }

                HalThermal_t thermal = HalReadThermal();

                s_data = Calibration_MakePassedData(power, thermal);

                (void)Calibration_SavePersisted();

                s_result = CAL_RESULT_PASSED;
                s_state = CAL_STATE_COMPLETE;
            }
        } break;

        case CAL_STATE_IDLE:
        case CAL_STATE_COMPLETE:
        default:
        {
            Calibration_SetZeroVoltageCommand(cmd);
        } break;
    }
}