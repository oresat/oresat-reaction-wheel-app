#include "Controller.h"

#include <stddef.h>

#include "Calibration.h"
#include "Commutation.h"
#include "Commutation_Foc.h"
#include "Commutation_OpenLoop.h"
#include "Commutation_Sine.h"
#include "Commutation_Trap.h"
#include "Config.h"
#include "Hal.h"
#include "MathUtil.h"
#include "CalibrationStore.h"
#include "CurrentPreprocess.h"

/*==============================================================================
 * Controller.c
 *
 * PURPOSE:
 * Implements the deterministic 16 kHz motor-control orchestration path.
 *
 * RESPONSIBILITIES:
 * - own controller and commutation mode state
 * - own public setpoints and startup commands
 * - run velocity ramping and PI integration
 * - latch controller-side faults and apply safe output
 * - construct the unified commutation input snapshot
 * - apply calibrated current-offset correction and preprocessing
 * - dispatch exactly one commutation strategy per cycle
 * - publish controller diagnostics
 *
 * OUT OF SCOPE:
 * - HAL register access and rotor-estimator implementation
 * - calibration algorithm internals
 * - per-strategy commutation mathematics
 * - telemetry, CAN, and experiment sequencing
 *
 * EXECUTION CONTRACT:
 * Controller_Update16kHz() preserves a fixed order:
 * sensor refresh, safety validation, current conditioning, command generation,
 * strategy execution, diagnostics publication, then PWM write.
 *===========================================================================*/

/*=============================================================================
 * PRIVATE CONSTANTS
 *===========================================================================*/

static const float CONTROLLER_CENTER_DUTY = 0.5f;
static const float CONTROLLER_VELOCITY_FILTER_ALPHA = 0.02f;
static const float CONTROLLER_SPEED_DEADBAND_REV_PER_SEC = 25.0f / 60.0f;
static const float CONTROLLER_TORQUE_SLEW_RATE_NM_PER_SEC = 40.0f;

/*=============================================================================
 * INTERNAL STATE
 *===========================================================================*/

/* Current controller objective mode. */
static ControlMode_t s_controlMode = CTRL_MODE_IDLE;

/* Current electrical commutation mode. */
static CommutationMode_t s_commutationMode = COMM_MODE_FOC;

/* Currently bound commutation strategy implementation. */
static const CommutationStrategy_t *s_activeStrategy = &g_focStrategy;

/* Currently bound controller tuning profile. */
static const ControllerTuning_t *s_activeTuning = &CONTROLLER_TUNING_FOC;

/* Commanded outer-loop targets. */
static float s_targetTorque_Nm = 0.0f;
static float s_targetVelocity_revPerSec = 0.0f;
static float s_targetPosition_rev = 0.0f;

/* Direct voltage-mode targets. */
static float s_targetVoltageD_V = 0.0f;
static float s_targetVoltageQ_V = 0.0f;

/* Velocity-loop dynamic state. */
static float s_velocityTargetRamped_revPerSec = 0.0f;
static float s_velocityIntegrator_Nm = 0.0f;

/*=============================================================================
 * STARTUP STATE
 *
 * Used only in CTRL_MODE_STARTUP_OPEN_LOOP.
 * Electrical angle advances internally at the fixed 16 kHz timestep.
 *===========================================================================*/
static float s_startupVoltageD_V = 0.0f;
static float s_startupVoltageQ_V = 0.0f;
static float s_startupElectricalAngle_rad = 0.0f;
static float s_startupElectricalVelocity_radPerSec = 0.0f;

/* Latched controller-side fault flag. */
static bool s_isFaulted = false;

static volatile ControllerDiagnostics_t s_controllerDiagnostics =
{
    .targetVoltageQ_V = 0.0f,
    .targetCurrentQ_A = 0.0f,
    .speedError_revPerSec = 0.0f,
    .velocityTargetRamped_revPerSec = 0.0f,
    .dutyA = CONTROLLER_CENTER_DUTY,
    .dutyB = CONTROLLER_CENTER_DUTY,
    .dutyC = CONTROLLER_CENTER_DUTY,
};

static HalPwmCommand_t s_lastPwmCommand =
{
    .dutyA = CONTROLLER_CENTER_DUTY,
    .dutyB = CONTROLLER_CENTER_DUTY,
    .dutyC = CONTROLLER_CENTER_DUTY,
    .floatA = true,
    .floatB = true,
    .floatC = true,
    .enableGateDriver = false
};

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static bool ControllerIsTorqueProducingMode(ControlMode_t mode)
{
    return (mode == CTRL_MODE_TORQUE) ||
           (mode == CTRL_MODE_VELOCITY) ||
           (mode == CTRL_MODE_POSITION);
}

static CommutationInputs_t ControllerMakeCommutationInputs(
    HalPhaseCurrents_t currents_A,
    HalPower_t power,
    HalRotorState_t rotor)
{
    return (CommutationInputs_t){
        .dt_s = FOC_UPDATE_PERIOD_S,

        .targetVoltageD_V = 0.0f,
        .targetVoltageQ_V = 0.0f,

        .targetCurrentD_A = 0.0f,
        .targetCurrentQ_A = 0.0f,

        .phaseCurrents_A = currents_A,
        .busVoltage_V = power.busVoltage_V,

        .electricalAngle_rad = rotor.electricalAngle_rad,
        .electricalVelocity_radPerSec =
            rotor.electricalVelocity_radPerSec,
        .electricalSector = rotor.electricalSector,
    };
}

static void ControllerPublishDiagnostics(
    const CommutationInputs_t *inputs,
    float speedError_revPerSec,
    HalPwmCommand_t pwmCommand)
{
    s_controllerDiagnostics.targetVoltageQ_V =
        inputs->targetVoltageQ_V;

    s_controllerDiagnostics.targetCurrentQ_A =
        inputs->targetCurrentQ_A;

    s_controllerDiagnostics.speedError_revPerSec =
        speedError_revPerSec;

    s_controllerDiagnostics.velocityTargetRamped_revPerSec =
        s_velocityTargetRamped_revPerSec;

    s_controllerDiagnostics.dutyA = pwmCommand.dutyA;
    s_controllerDiagnostics.dutyB = pwmCommand.dutyB;
    s_controllerDiagnostics.dutyC = pwmCommand.dutyC;
}

static const CommutationStrategy_t *ControllerSelectExecutionStrategy(void)
{
    if (s_controlMode == CTRL_MODE_CALIBRATION)
    {
        return Calibration_RequiresFocCommutation()
            ? &g_focStrategy
            : &g_openLoopStrategy;
    }

    return s_activeStrategy;
}


/*
 * Selects the active commutation strategy based on controller mode.
 *
 * RULES:
 * - Calibration mode forces open-loop commutation.
 * - Startup open-loop mode also forces open-loop commutation.
 * - Closed-loop mode uses the selected commutation mode.
 *
 * IMPORTANT:
 * - Startup must always use open-loop commutation to ensure deterministic
 *   rotor excitation independent of estimator validity.
 */
static const CommutationStrategy_t *ControllerSelectStrategy(
    ControlMode_t controlMode,
    CommutationMode_t commutationMode)
{
    if (controlMode == CTRL_MODE_CALIBRATION)
    {
        if (Calibration_RequiresFocCommutation())
        {
            return &g_focStrategy;
        }

        return &g_openLoopStrategy;
    }

    if (controlMode == CTRL_MODE_STARTUP_OPEN_LOOP)
    {
        return &g_openLoopStrategy;
    }

    switch (commutationMode)
    {
        case COMM_MODE_FOC:       return &g_focStrategy;
        case COMM_MODE_SINE:      return &g_sinusoidalStrategy;
        case COMM_MODE_TRAP:      return &g_trapezoidalStrategy;
        case COMM_MODE_OPEN_LOOP: return &g_openLoopStrategy;
        default:                  return &g_openLoopStrategy;
    }
}

/*
 * Selects the active controller tuning profile.
 *
 * RULES:
 * - Calibration and startup-open-loop both force the open-loop tuning profile.
 * - All other modes use the profile associated with the selected commutation
 *   mode.
 */
static const ControllerTuning_t *ControllerSelectTuning(
    ControlMode_t controlMode,
    CommutationMode_t commutationMode)
{
    if ((controlMode == CTRL_MODE_CALIBRATION) ||
        (controlMode == CTRL_MODE_STARTUP_OPEN_LOOP))
    {
        return &CONTROLLER_TUNING_OPEN_LOOP;
    }

    switch (commutationMode)
    {
        case COMM_MODE_FOC:       return &CONTROLLER_TUNING_FOC;
        case COMM_MODE_SINE:      return &CONTROLLER_TUNING_SINE;
        case COMM_MODE_TRAP:      return &CONTROLLER_TUNING_TRAP;
        case COMM_MODE_OPEN_LOOP: return &CONTROLLER_TUNING_OPEN_LOOP;
        default:                  return &CONTROLLER_TUNING_OPEN_LOOP;
    }
}

/*
 * Returns a fully safe inverter command:
 * - all duties centered
 * - all phases floated
 * - gate driver disabled
 */
static HalPwmCommand_t ControllerMakeSafeCommand(void)
{
    return (HalPwmCommand_t){
        .dutyA = CONTROLLER_CENTER_DUTY,
        .dutyB = CONTROLLER_CENTER_DUTY,
        .dutyC = CONTROLLER_CENTER_DUTY,
        .floatA = true,
        .floatB = true,
        .floatC = true,
        .enableGateDriver = false
    };
}

/*
 * Latches a controller fault and immediately applies a safe inverter output.
 * This is the controller's final protective action when a control-path error
 * is detected.
 */
static void ControllerApplySafeOutputAndLatchFault(void)
{
    s_isFaulted = true;

    HalPwmCommand_t safeCmd = ControllerMakeSafeCommand();
    HalWritePwm(&safeCmd);
    s_lastPwmCommand = safeCmd;
}

/*
 * Determines whether invalid rotor state should force a controller fault.
 *
 * POLICY:
 * - Calibration mode does not require valid rotor state.
 * - Startup open-loop mode does not require valid rotor state.
 * - Direct voltage bench mode also tolerates invalid rotor state so non-FOC
 *   bring-up can proceed without estimator validity.
 * - All other runtime modes require a valid rotor estimate.
 */
static bool ControllerShouldFaultOnInvalidRotor(void)
{
    return (s_controlMode != CTRL_MODE_CALIBRATION) &&
           (s_controlMode != CTRL_MODE_STARTUP_OPEN_LOOP) &&
           (s_controlMode != CTRL_MODE_VOLTAGE);
}

/*
 * Applies previously identified current-sensor offsets to the latest phase
 * current measurement bundle.
 *
 * The calibration stores offsets in alpha/beta form, so they are reconstructed
 * back into phase A/B/C using the same reduced Clarke convention used
 * elsewhere in the control stack:
 *
 *   alpha = phaseA
 *   beta  = (phaseB - phaseC) / sqrt(3)
 *
 * IMPORTANT:
 * - This helper must stay consistent with the system Clarke convention.
 * - If the Clarke convention changes, this reconstruction must change too.
 */
static void ControllerApplyCurrentOffsetCorrection(HalPhaseCurrents_t *currents)
{
    if (currents == NULL)
    {
        return;
    }

    float offsetAlpha_A = Calibration_GetCurrentOffsetAlpha();
    float offsetBeta_A  = Calibration_GetCurrentOffsetBeta();

    float offsetA_A = offsetAlpha_A;
    float offsetB_A = -0.5f * offsetAlpha_A + MATH_SQRT3_OVER_2_F * offsetBeta_A;
    float offsetC_A = -0.5f * offsetAlpha_A - MATH_SQRT3_OVER_2_F * offsetBeta_A;

    currents->phaseA_A -= offsetA_A;
    currents->phaseB_A -= offsetB_A;
    currents->phaseC_A -= offsetC_A;
}

/*
 * Converts a requested torque into a q-axis current command using the motor
 * torque constant, then clamps it to the configured phase-current limit.
 */
static float ControllerTorqueToCurrentQ_A(float torque_Nm)
{
    if (MOTOR_TORQUE_CONSTANT <= 0.0f)
    {
        return 0.0f;
    }

    float iq_A = torque_Nm / MOTOR_TORQUE_CONSTANT;
    return MathClamp(iq_A, -PHASE_CURRENT_COMMAND_LIMIT_A, PHASE_CURRENT_COMMAND_LIMIT_A);
}

/*
 * Converts a requested torque into an approximate q-axis voltage command for
 * non-FOC strategies.
 *
 * MODEL:
 * - torque -> q-axis current using the motor torque constant
 * - q-axis current -> q-axis voltage using only phase resistance
 *
 * IMPORTANT:
 * - This is a simple resistive approximation intended for bench bring-up and
 *   coarse non-FOC command generation.
 * - It does NOT include back-EMF, inductive dynamics, or speed-dependent
 *   voltage requirements.
 * - It should not be interpreted as an exact torque-equivalent model.
 */
static float ControllerTorqueToVoltageQ_V(float torque_Nm, float mechVelocity_revPerSec)
{
    float iq_A = ControllerTorqueToCurrentQ_A(torque_Nm);

    /* 1. Resistive drop needed to produce torque-producing current */
    float resistiveVq_V = iq_A * MOTOR_PHASE_RESISTANCE_OHM;

    /* 2. True Mechanical Back-EMF Feedforward:
     * Ke [V/(rad/s mech)] ~= Kt [N*m/A] in SI units.
     * Convert rev/s to mechanical rad/s to match the Kt unit!
     */
    float mechVel_radPerSec = mechVelocity_revPerSec * MATH_TWO_PI_F;
    float backEmfVq_V = mechVel_radPerSec * MOTOR_TORQUE_CONSTANT;

    float vq_V = resistiveVq_V + backEmfVq_V;

    float maxAbsVq_V = 0.5f * HalReadPower().busVoltage_V;
    return MathClamp(vq_V, -maxAbsVq_V, +maxAbsVq_V);
}

/*
 * Resets controller dynamic runtime state.
 *
 * - This does NOT reset startup command parameters.
 * - Startup parameters persist across mode transitions unless explicitly
 *   overwritten by the caller.
 *
 * DESIGN CHOICE:
 * - Allows external logic to configure startup once and reuse it.
 * - Requires careful handling to avoid stale startup commands.
 */
static void ControllerResetDynamicState(void)
{
    s_velocityTargetRamped_revPerSec = 0.0f;
    s_velocityIntegrator_Nm = 0.0f;
}

/*
 * Re-selects the active commutation strategy and re-runs that strategy's
 * initialization hook, if present.
 *
 * This is the main binding point that keeps all strategies executing through a
 * common controller-owned path.
 */
static void ControllerRebindStrategy(void)
{
    s_activeStrategy = ControllerSelectStrategy(s_controlMode, s_commutationMode);
    s_activeTuning = ControllerSelectTuning(s_controlMode, s_commutationMode);

    if (s_activeTuning == NULL)
    {
        s_activeTuning = &CONTROLLER_TUNING_OPEN_LOOP;
    }

    if ((s_commutationMode == COMM_MODE_FOC) &&
        (s_activeTuning->currentLoopBandwidth_radPerSec > 0.0f))
    {
        FocSetCurrentLoopBandwidth(s_activeTuning->currentLoopBandwidth_radPerSec);
    }

    if (s_activeStrategy != NULL && s_activeStrategy->Init != NULL)
    {
        s_activeStrategy->Init();
    }
}

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

/*
 * Initializes HAL and resets all controller-owned state to a safe default.
 */
void Controller_Init(void)
{
    HalInit();

    CurrentPreprocess_Init();

    CalibrationStore_Init();
    (void)Calibration_LoadPersisted();

    s_controlMode = CTRL_MODE_IDLE;
    s_commutationMode = COMM_MODE_FOC;
    s_activeTuning = &CONTROLLER_TUNING_FOC;

    s_targetTorque_Nm = 0.0f;
    s_targetVelocity_revPerSec = 0.0f;
    s_targetPosition_rev = 0.0f;

    s_targetVoltageD_V = 0.0f;
    s_targetVoltageQ_V = 0.0f;

    s_startupVoltageD_V = 0.0f;
    s_startupVoltageQ_V = 0.0f;
    s_startupElectricalAngle_rad = 0.0f;
    s_startupElectricalVelocity_radPerSec = 0.0f;
    
    s_lastPwmCommand = ControllerMakeSafeCommand();

    ControllerResetDynamicState();

    s_isFaulted = false;
    ControllerRebindStrategy();
}

/*
 * Sets the active controller objective mode.
 *
 * If the controller is faulted, only a transition back to IDLE is allowed.
 */
void Controller_SetControlMode(ControlMode_t mode)
{
    if (s_isFaulted && mode != CTRL_MODE_IDLE)
    {
        return;
    }

    if (mode != s_controlMode)
    {
        if (mode == CTRL_MODE_VELOCITY)
        {
            HalRotorState_t rotor = HalReadRotor();

            if (rotor.isValid)
            {
                s_velocityTargetRamped_revPerSec =
                    rotor.mechanicalVelocity_revPerSec;
            }
            else
            {
                s_velocityTargetRamped_revPerSec =
                    s_targetVelocity_revPerSec;
            }

            if (s_controlMode == CTRL_MODE_STARTUP_OPEN_LOOP)
            {
                /* 1. Calculate the true physical Back-EMF based on mechanical speed */
                float mechVel_radPerSec = rotor.mechanicalVelocity_revPerSec * MATH_TWO_PI_F;
                float backEmf_V = mechVel_radPerSec * MOTOR_TORQUE_CONSTANT;
                
                /* 2. Subtract Back-EMF from the total open-loop Vq to isolate the pure resistive effort */
                float trueResistiveVq_V = s_startupVoltageQ_V - backEmf_V;
                
                /* 3. Convert that pure resistive voltage into the equivalent integrator seed (Nm) */
                float iq_A = trueResistiveVq_V / MOTOR_PHASE_RESISTANCE_OHM;
                s_velocityIntegrator_Nm = iq_A * MOTOR_TORQUE_CONSTANT;
            }
            else if (s_controlMode == CTRL_MODE_VOLTAGE)
            {
                float iq_A = s_targetVoltageQ_V / MOTOR_PHASE_RESISTANCE_OHM;
                s_velocityIntegrator_Nm = iq_A * MOTOR_TORQUE_CONSTANT;
            }
            else if (s_controlMode == CTRL_MODE_TORQUE)
            {
                s_velocityIntegrator_Nm = s_targetTorque_Nm;
            }
            else
            {
                s_velocityIntegrator_Nm = 0.0f;
            }

            float integratorLimit_Nm =
                (s_activeTuning != NULL)
                    ? s_activeTuning->velocityIntegratorLimit_Nm
                    : CONTROLLER_TUNING_FOC.velocityIntegratorLimit_Nm;

            s_velocityIntegrator_Nm = MathClamp(
                s_velocityIntegrator_Nm,
                -integratorLimit_Nm,
                +integratorLimit_Nm
            );
        }
        else if (mode == CTRL_MODE_IDLE)
        {
            /*
            * Preserve velocity-loop state so a subsequent re-entry into
            * velocity control can be seeded from the current operating point.
            *
            * Dynamic state is still cleared explicitly by
            * Controller_ClearFaults().
            */
        }
        else
        {
            ControllerResetDynamicState();
        }

        s_controlMode = mode;
    }

    ControllerRebindStrategy();
}

/*
 * Sets the active commutation mode.
 *
 * NOTE:
 * - The selected commutation mode is not always the strategy that will execute.
 * - Calibration mode and startup open-loop mode both force the open-loop
 *   commutation strategy regardless of the requested commutation mode.
 * - In all other modes, the requested commutation mode is rebound immediately.
 */
void Controller_SetCommutationMode(CommutationMode_t mode)
{
    s_commutationMode = mode;

    if ((s_controlMode != CTRL_MODE_CALIBRATION) &&
        (s_controlMode != CTRL_MODE_STARTUP_OPEN_LOOP))
    {
        ControllerRebindStrategy();
    }
}

/* Stores the torque command for later use in the next control cycle. */
void Controller_SetTorque(float torque_Nm)
{
    s_targetTorque_Nm = torque_Nm;
}

/* Stores the velocity command for later use in the next control cycle. */
void Controller_SetVelocity(float velocity_revPerSec)
{
    s_targetVelocity_revPerSec = velocity_revPerSec;
}

/* Stores the position command for later use in the next control cycle. */
void Controller_SetPosition(float position_rev)
{
    s_targetPosition_rev = position_rev;
}

void Controller_SetVoltageDQ(float vd_V, float vq_V)
{
    s_targetVoltageD_V = vd_V;
    s_targetVoltageQ_V = vq_V;
}

void Controller_SetStartupVoltageDQ(float vd_V, float vq_V)
{
    s_startupVoltageD_V = vd_V;
    s_startupVoltageQ_V = vq_V;
}

void Controller_SetStartupElectricalAngle(float angle_rad)
{
    s_startupElectricalAngle_rad = MathWrapPi(angle_rad);
}

void Controller_SetStartupElectricalVelocity(float vel_rad_per_sec)
{
    s_startupElectricalVelocity_radPerSec = vel_rad_per_sec;
}

/* Returns the current controller fault-latched state. */
bool Controller_IsFaulted(void)
{
    return s_isFaulted;
}

/*
 * Clears the controller fault latch, resets controller-side dynamic state, and
 * reinitializes the currently selected strategy binding.
 *
 * IMPORTANT:
 * - This does not clear hardware faults by itself.
 * - Rebinding here ensures strategy-local state is refreshed before control is
 *   resumed in the current mode.
 */
void Controller_ClearFaults(void)
{
    s_isFaulted = false;
    ControllerResetDynamicState();
    ControllerRebindStrategy();
}

/*=============================================================================
 * HIGH-SPEED CONTROL LOOP
 *===========================================================================*/

/*
 * Executes exactly one controller-owned high-speed control cycle.
 *
 * Execution order:
 * 1. Refresh sensor cache through HAL
 * 2. Validate control-path safety conditions
 * 3. Apply current offset correction when appropriate
 * 4. Build a unified commutation input structure
 * 5. Run outer-loop control or calibration logic
 * 6. Dispatch the active commutation strategy
 * 7. Write the resulting PWM command
 */
void Controller_Update16kHz(void)
{
    HalSensorUpdateStatus_t sensorStatus = HalUpdateSensorCache();

    HalPhaseCurrents_t currents_A = HalReadCurrents();
    HalPower_t power = HalReadPower();
    HalRotorState_t rotor = HalReadRotor();

    if (sensorStatus != HAL_SENSOR_UPDATE_OK)
    {
        ControllerApplySafeOutputAndLatchFault();
        return;
    }

    if (HalHasHardwareFault())
    {
        ControllerApplySafeOutputAndLatchFault();
        return;
    }

    if (!rotor.isValid && ControllerShouldFaultOnInvalidRotor())
    {
        ControllerApplySafeOutputAndLatchFault();
        return;
    }

    if (s_controlMode != CTRL_MODE_CALIBRATION)
    {
        ControllerApplyCurrentOffsetCorrection(&currents_A);
        currents_A = CurrentPreprocess_Apply(&currents_A, &s_lastPwmCommand);
    }
    else if (Calibration_RequiresFocCommutation())
    {
        /*
        * Current-controlled calibration needs zero-centered currents because FOC
        * consumes measured phase currents directly.
        */
        ControllerApplyCurrentOffsetCorrection(&currents_A);
    }

    CommutationInputs_t commutationInputs =
        ControllerMakeCommutationInputs(
            currents_A,
            power,
            rotor);

    float targetTorque_Nm = 0.0f;
    float speedError_revPerSec = 0.0f;

    switch (s_controlMode)
    {
        case CTRL_MODE_IDLE:
        {
            targetTorque_Nm = 0.0f;
            break;
        }

        case CTRL_MODE_TORQUE:
        {
            targetTorque_Nm = s_targetTorque_Nm;
            break;
        }

        case CTRL_MODE_VELOCITY:
        {
            float velocityError_revPerSec =
                s_targetVelocity_revPerSec - s_velocityTargetRamped_revPerSec;

            float velocityRampRate_revPerSec2 =
                (s_activeTuning != NULL)
                    ? s_activeTuning->velocityRampRate_revPerSec2
                    : CONTROLLER_TUNING_FOC.velocityRampRate_revPerSec2;

            float maxStep =
                velocityRampRate_revPerSec2 * FOC_UPDATE_PERIOD_S;

            s_velocityTargetRamped_revPerSec += MathClamp(
                velocityError_revPerSec,
                -maxStep,
                +maxStep
            );

            static float s_velocityFiltered_revPerSec = 0.0f;
            static bool s_velocityFilterInitialized = false;

            if (!s_velocityFilterInitialized)
            {
                s_velocityFiltered_revPerSec = rotor.mechanicalVelocity_revPerSec;
                s_velocityFilterInitialized = true;
            }

            s_velocityFiltered_revPerSec +=
                CONTROLLER_VELOCITY_FILTER_ALPHA *
                (rotor.mechanicalVelocity_revPerSec - s_velocityFiltered_revPerSec);

            speedError_revPerSec =
                s_velocityTargetRamped_revPerSec -
                s_velocityFiltered_revPerSec;

            if (MathAbs(speedError_revPerSec) <
                CONTROLLER_SPEED_DEADBAND_REV_PER_SEC)
            {
                speedError_revPerSec = 0.0f;
            }

            float velocityKi =
                (s_activeTuning != NULL)
                    ? s_activeTuning->velocityKi
                    : CONTROLLER_TUNING_FOC.velocityKi;

            float integratorLimit_Nm =
                (s_activeTuning != NULL)
                    ? s_activeTuning->velocityIntegratorLimit_Nm
                    : CONTROLLER_TUNING_FOC.velocityIntegratorLimit_Nm;

            float velocityKp =
                (s_activeTuning != NULL)
                    ? s_activeTuning->velocityKp
                    : CONTROLLER_TUNING_FOC.velocityKp;

            s_velocityIntegrator_Nm +=
                velocityKi * speedError_revPerSec * FOC_UPDATE_PERIOD_S;

            s_velocityIntegrator_Nm = MathClamp(
                s_velocityIntegrator_Nm,
                -integratorLimit_Nm,
                +integratorLimit_Nm
            );

            targetTorque_Nm =
                velocityKp * speedError_revPerSec +
                s_velocityIntegrator_Nm;

            break;
        }

        case CTRL_MODE_POSITION:
        {
            (void)s_targetPosition_rev;
            targetTorque_Nm = 0.0f;
            break;
        }

        case CTRL_MODE_CALIBRATION:
        {
            Calibration_Update16kHz(
                &currents_A,
                &power,
                &rotor,
                &commutationInputs
            );
            break;
        }

        case CTRL_MODE_STARTUP_OPEN_LOOP:
        {
            s_startupElectricalAngle_rad +=
                s_startupElectricalVelocity_radPerSec *
                FOC_UPDATE_PERIOD_S;

            s_startupElectricalAngle_rad =
                MathWrapPi(s_startupElectricalAngle_rad);

            commutationInputs.targetVoltageD_V = s_startupVoltageD_V;
            commutationInputs.targetVoltageQ_V = s_startupVoltageQ_V;
            commutationInputs.targetCurrentD_A = 0.0f;
            commutationInputs.targetCurrentQ_A = 0.0f;

            commutationInputs.electricalAngle_rad =
                s_startupElectricalAngle_rad;

            commutationInputs.electricalVelocity_radPerSec =
                s_startupElectricalVelocity_radPerSec;

            break;
        }

        case CTRL_MODE_VOLTAGE:
        {
            commutationInputs.targetVoltageD_V = s_targetVoltageD_V;
            commutationInputs.targetVoltageQ_V = s_targetVoltageQ_V;
            commutationInputs.targetCurrentD_A = 0.0f;
            commutationInputs.targetCurrentQ_A = 0.0f;
            break;
        }

        default:
        {
            targetTorque_Nm = 0.0f;
            break;
        }
    }

    /*--------------------------------------------------------------
     * Slew-limit torque command to reduce audible high-speed jerk.
     *--------------------------------------------------------------*/
    static float s_targetTorqueSlewed_Nm = 0.0f;

    float maxTorqueStep_Nm =
        CONTROLLER_TORQUE_SLEW_RATE_NM_PER_SEC *
        FOC_UPDATE_PERIOD_S;

    s_targetTorqueSlewed_Nm += MathClamp(
        targetTorque_Nm - s_targetTorqueSlewed_Nm,
        -maxTorqueStep_Nm,
        +maxTorqueStep_Nm);

    targetTorque_Nm = s_targetTorqueSlewed_Nm;


    bool isTorqueMode =
        ControllerIsTorqueProducingMode(s_controlMode);

    if (isTorqueMode)
    {
        if (s_commutationMode == COMM_MODE_FOC)
        {
            commutationInputs.targetCurrentD_A = 0.0f;
            commutationInputs.targetCurrentQ_A =
                ControllerTorqueToCurrentQ_A(targetTorque_Nm);
        }
        else
        {
            commutationInputs.targetVoltageD_V = 0.0f;
            commutationInputs.targetVoltageQ_V =
                ControllerTorqueToVoltageQ_V(
                    targetTorque_Nm,
                    rotor.mechanicalVelocity_revPerSec
                );
        }
    }

    HalPwmCommand_t pwmCommand = ControllerMakeSafeCommand();

    const CommutationStrategy_t *executionStrategy =
        ControllerSelectExecutionStrategy();

    if (s_controlMode == CTRL_MODE_IDLE)
    {
        if ((executionStrategy != NULL) &&
            (executionStrategy->Stop != NULL))
        {
            pwmCommand = executionStrategy->Stop();
        }
    }
    else
    {
        if ((executionStrategy != NULL) &&
            (executionStrategy->Update != NULL))
        {
            pwmCommand = executionStrategy->Update(&commutationInputs);
        }
    }

    ControllerPublishDiagnostics(
        &commutationInputs,
        speedError_revPerSec,
        pwmCommand);

    HalWritePwm(&pwmCommand);
    s_lastPwmCommand = pwmCommand;
}

ControllerDiagnostics_t Controller_GetDiagnostics(void)
{
    return s_controllerDiagnostics;
}