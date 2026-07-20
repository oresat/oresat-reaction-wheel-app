#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "Controller.h"

/*==============================================================================
 * Config.h
 *
 * Central compile-time configuration for the reaction-wheel firmware.
 *
 * This file contains:
 * - Hardware characteristics and conversion constants
 * - Motor and encoder parameters
 * - Safety and calibration limits
 * - Controller tuning profiles
 * - Optional control and measurement-processing features
 * - CAN node identifiers
 *
 * Runtime state, control logic, peripheral initialization, and calibration
 * sequencing do not belong in this file.
 *============================================================================*/

/*==============================================================================
 * SYSTEM TIMING
 *============================================================================*/

#define FOC_UPDATE_FREQ_HZ                       16000.0f
#define FOC_UPDATE_PERIOD_S                      (1.0f / FOC_UPDATE_FREQ_HZ)

#define PWM_RELOAD_CYCLES                        1

/*==============================================================================
 * ADC CONVERSION
 *============================================================================*/

/* Measured hardware constants. */
static const float ADC_REFERENCE_VOLTAGE_V       = 3.0f;
static const uint16_t ADC_FULL_SCALE_COUNTS      = 4096u;

/* Derived conversion from raw ADC counts to volts. */
static const float ADC_COUNTS_TO_VOLTS =
    ADC_REFERENCE_VOLTAGE_V / (float)ADC_FULL_SCALE_COUNTS;

/*==============================================================================
 * CURRENT SENSE
 *============================================================================*/

/* Nominal zero-current operating point. */
static const float ADC_ZERO_CURRENT_BIAS_V       = 1.50f;
static const uint16_t ADC_ZERO_CURRENT_COUNTS    = 2048u;

/* Current-sense hardware parameters. */
static const float SHUNT_RESISTANCE_OHM          = 0.002f;

/*
 * Positive gain matches the V2 analog front-end polarity convention.
 */
static const float CURRENT_SENSE_GAIN            = 50.0f;

/* Derived current conversion factors. */
static const float CURRENT_SENSE_CONDUCTANCE =
    1.0f / (CURRENT_SENSE_GAIN * SHUNT_RESISTANCE_OHM);

static const float ADC_COUNTS_TO_AMPS =
    ADC_COUNTS_TO_VOLTS * CURRENT_SENSE_CONDUCTANCE;

/*==============================================================================
 * DC BUS VOLTAGE SENSE
 *============================================================================*/

/* Bus-voltage divider hardware. */
static const float VBUS_R_TOP_OHM                = 5100.0f;
static const float VBUS_R_BOTTOM_OHM             = 3300.0f;

/* Derived bus-voltage conversion factors. */
static const float VBUS_DIVIDER_RATIO =
    VBUS_R_BOTTOM_OHM / (VBUS_R_TOP_OHM + VBUS_R_BOTTOM_OHM);

static const float ADC_COUNTS_TO_VBUS_VOLTS =
    ADC_COUNTS_TO_VOLTS / VBUS_DIVIDER_RATIO;

/*==============================================================================
 * PHASE VOLTAGE SENSE
 *============================================================================*/

/* V2 phase-voltage divider hardware. */
static const float PHASE_V_R_TOP_OHM             = 5100.0f;
static const float PHASE_V_R_BOTTOM_OHM          = 3300.0f;

/* Derived phase-voltage conversion factors. */
static const float PHASE_V_DIVIDER_RATIO =
    PHASE_V_R_BOTTOM_OHM
    / (PHASE_V_R_TOP_OHM + PHASE_V_R_BOTTOM_OHM);

static const float ADC_COUNTS_TO_PHASE_VOLTS =
    ADC_COUNTS_TO_VOLTS / PHASE_V_DIVIDER_RATIO;

/*==============================================================================
 * ENCODER
 *============================================================================*/

/* Mechanical orientation relative to the firmware's positive rotation. */
#define ENCODER_DIRECTION_SIGN                   (1.0f)

/* Encoder hardware and estimator parameters. */
static const uint16_t ENCODER_CPR                = 16384u;
static const float ENCODER_ESTIMATOR_BANDWIDTH   = 600.0f;
static const float ENCODER_MAX_UPDATE_DT_S       = 0.010f;

/* Optional encoder diagnostics. */
#define ENCODER_DT_DEBUG_PRINT                   0
#define ENCODER_RAW_DEBUG_PRINT                  0
#define ENCODER_DEBUG_PRINT_DIVIDER              200u

/* Optional fixed-period estimator operation. */
#define ENCODER_FORCE_FIXED_DT                   0
#define ENCODER_FIXED_DT_S                       FOC_UPDATE_PERIOD_S

/* Reject implausible single-sample mechanical-angle changes. */
#define ENCODER_GLITCH_THRESHOLD_REV             0.10f

/* Expected timing range around the nominal 16 kHz update period. */
static const float ENCODER_DT_OUTLIER_MIN_S =
    0.8f * FOC_UPDATE_PERIOD_S;

static const float ENCODER_DT_OUTLIER_MAX_S =
    1.2f * FOC_UPDATE_PERIOD_S;

/*==============================================================================
 * MOTOR PARAMETERS
 *
 * Used as fallback values when no valid persisted calibration is available.
 *============================================================================*/

#define MOTOR_POLE_PAIRS                         6

static const float MOTOR_KV_RPM_PER_V            = 4500.0f;

static const float MOTOR_TORQUE_CONSTANT =
    8.27f / MOTOR_KV_RPM_PER_V;

static const float MOTOR_PHASE_RESISTANCE_OHM    = 0.35f;
static const float MOTOR_PHASE_INDUCTANCE_H      = 6.39e-6f / 2.0f;

/*==============================================================================
 * THERMISTOR MODEL
 *============================================================================*/

/* Thermistor measurement hardware. */
static const float THERMISTOR_ADC_REFERENCE_VOLTAGE_V = 3.0f;
static const float THERMISTOR_PULLUP_RESISTANCE_OHM   = 3300.0f;

/* Beta-model parameters. */
static const float THERMISTOR_BETA_K                  = 3380.0f;
static const float THERMISTOR_R0_OHM                  = 10000.0f;
static const float THERMISTOR_T0_K                    = 298.15f;

/* Voltage guards used to detect open and shorted sensors. */
static const float THERMISTOR_OPEN_GUARD_V            = 0.05f;
static const float THERMISTOR_SHORT_GUARD_V           = 0.05f;

/*==============================================================================
 * SYSTEM SAFETY LIMITS
 *============================================================================*/

/* DC bus limits. */
static const float BUS_OVERVOLTAGE_LIMIT_V       = 8.6f;
static const float BUS_UNDERVOLTAGE_LIMIT_V      = 5.4f;
static const float BUS_OVERCURRENT_LIMIT_A       = 5.0f;
static const float BUS_REGEN_LIMIT_A             = 5.0f;

/* Phase-current limits. */
static const float PHASE_CURRENT_COMMAND_LIMIT_A = 6.0f;
static const float PHASE_CURRENT_TRIP_LIMIT_A    = 10.0f;

/* Inverter thermal limits. */
static const float INVERTER_TEMP_WARNING_C       = 90.0f;
static const float INVERTER_TEMP_CRITICAL_C      = 110.0f;

/* Mechanical operating limit. */
static const float MAX_MOTOR_SPEED_RPM           = 12500.0f;

/* Current-controller integrator clamp. */
static const float FOC_INTEGRATOR_LIMIT_V        = 4.0f;

/*==============================================================================
 * ACTIVE CALIBRATION
 *============================================================================*/

/*------------------------------------------------------------------------------
 * Current-offset calibration
 *----------------------------------------------------------------------------*/

static const uint32_t CURRENT_OFFSET_SAMPLE_COUNT = 2000u;

static const float CALIBRATION_OFFSET_ABS_MAX_A = 0.15f;

/*------------------------------------------------------------------------------
 * Static encoder-offset calibration
 *----------------------------------------------------------------------------*/

#define CALIBRATION_ENCODER_MULTI_POINT_COUNT    6u

static const float CALIBRATION_LOCK_VOLTAGE_V = 0.35f;

static const uint32_t CALIBRATION_ENCODER_OFFSET_SAMPLES_PER_POINT =
    200u;

static const uint32_t CALIBRATION_ENCODER_LOCK_SETTLE_TICKS =
    (uint32_t)(0.75f * FOC_UPDATE_FREQ_HZ);

static const float CALIBRATION_ENCODER_OFFSET_MAX_SPREAD_RAD =
    0.35f;

/*------------------------------------------------------------------------------
 * Calibration operating limits
 *----------------------------------------------------------------------------*/

static const float CALIBRATION_BUS_VOLTAGE_MIN_V = 6.3f;
static const float CALIBRATION_BUS_VOLTAGE_MAX_V = 8.5f;

static const float CALIBRATION_LOCK_CURRENT_MIN_A = 0.10f;
static const float CALIBRATION_LOCK_CURRENT_MAX_A = 3.00f;
static const float CALIBRATION_LOCK_BETA_ABS_MAX_A = 3.00f;

static const float CALIBRATION_LOCK_SETTLE_TIME_S = 0.25f;

/*------------------------------------------------------------------------------
 * Motor-parameter validation
 *----------------------------------------------------------------------------*/

static const float CALIBRATION_PHASE_RESISTANCE_MIN_OHM = 0.01f;
static const float CALIBRATION_PHASE_RESISTANCE_MAX_OHM = 1.00f;

static const float CALIBRATION_PHASE_INDUCTANCE_MIN_H = 1.0e-9f;
static const float CALIBRATION_PHASE_INDUCTANCE_MAX_H = 0.003f;

/* Global calibration timeout. */
static const float CALIBRATION_TIMEOUT_S = 30.0f;

/*==============================================================================
 * MOVING ENCODER PHASE CALIBRATION
 *
 * Diagnostic sweep used to compare static lock calibration with a moving
 * electrical-phase estimate. The sweep may run open-loop or under FOC current
 * control.
 *============================================================================*/

/* Enable the optional moving-phase diagnostic. */
static const bool CALIBRATION_ENABLE_PHASE_SWEEP_DIAGNOSTIC = false;

/* Open-loop sweep command. */
static const float CALIBRATION_PHASE_SWEEP_VD_V = 0.35f;
static const float CALIBRATION_PHASE_SWEEP_SPEED_RAD_PER_SEC = 8.0f;

/* Sweep timing. */
static const uint32_t CALIBRATION_PHASE_SWEEP_LOCK_TICKS =
    (uint32_t)(0.50f * FOC_UPDATE_FREQ_HZ);

static const uint32_t CALIBRATION_PHASE_SWEEP_RAMP_TICKS =
    (uint32_t)(0.75f * FOC_UPDATE_FREQ_HZ);

static const uint32_t CALIBRATION_PHASE_SWEEP_CRUISE_TICKS =
    (uint32_t)(1.50f * FOC_UPDATE_FREQ_HZ);

/* Minimum data requirements. */
static const uint32_t CALIBRATION_PHASE_SWEEP_MIN_SAMPLES = 1000u;

static const bool CALIBRATION_PHASE_SWEEP_ENABLE_VELOCITY_GATE = true;

static const float CALIBRATION_PHASE_SWEEP_VELOCITY_GATE_FRACTION =
    0.50f;

static const uint32_t CALIBRATION_PHASE_SWEEP_MIN_ACCEPTED_SAMPLES =
    2000u;

/* Optional current-controlled sweep. */
static const bool CALIBRATION_PHASE_SWEEP_USE_CURRENT_CONTROL = true;
static const float CALIBRATION_PHASE_SWEEP_ID_A = 3.0f;

/* Maximum permitted disagreement with the static calibration result. */
static const float CALIBRATION_PHASE_SWEEP_STATIC_DELTA_MAX_RAD =
    0.10f;

/*==============================================================================
 * CONTROLLER TUNING PROFILES
 *============================================================================*/

static const ControllerTuning_t CONTROLLER_TUNING_FOC =
{
    .velocityRampRate_revPerSec2    = 40.0f,
    .velocityKp                     = 0.0022f,
    .velocityKi                     = 0.000020f,
    .velocityIntegratorLimit_Nm     = 0.8f * MOTOR_TORQUE_CONSTANT,
    .currentLoopBandwidth_radPerSec = 800.0f,
};

static const ControllerTuning_t CONTROLLER_TUNING_SINE =
{
    .velocityRampRate_revPerSec2    = 40.0f,
    .velocityKp                     = 0.0022f,
    .velocityKi                     = 0.000020f,
    .velocityIntegratorLimit_Nm     = 0.8f * MOTOR_TORQUE_CONSTANT,
    .currentLoopBandwidth_radPerSec = 800.0f,
};

static const ControllerTuning_t CONTROLLER_TUNING_TRAP =
{
    .velocityRampRate_revPerSec2    = 40.0f,
    .velocityKp                     = 0.0022f,
    .velocityKi                     = 0.000020f,
    .velocityIntegratorLimit_Nm     = 0.8f * MOTOR_TORQUE_CONSTANT,
    .currentLoopBandwidth_radPerSec = 800.0f,
};

static const ControllerTuning_t CONTROLLER_TUNING_OPEN_LOOP =
{
    .velocityRampRate_revPerSec2    = 20.0f,
    .velocityKp                     = 0.0025f,
    .velocityKi                     = 0.0010f,
    .velocityIntegratorLimit_Nm     = 5.0f * MOTOR_TORQUE_CONSTANT,
    .currentLoopBandwidth_radPerSec = 0.0f,
};

/*==============================================================================
 * OPTIONAL FOC COMPENSATION
 *
 * Each feature can be enabled independently. Gains scale the corresponding
 * compensation voltage before it is added to the PI-controller output.
 *============================================================================*/

static const bool FOC_ENABLE_BACKEMF_FEEDFORWARD = true;
static const bool FOC_ENABLE_WL_FEEDFORWARD      = false;
static const bool FOC_ENABLE_RESISTIVE_FF        = true;
static const bool FOC_ENABLE_DECOUPLING          = true;

static const float FOC_RESISTIVE_FF_GAIN         = 1.0f;
static const float FOC_DECOUPLING_GAIN           = 1.0f;
static const float FOC_BACKEMF_FF_GAIN           = 1.0f;

/*==============================================================================
 * CURRENT PREPROCESSING
 *
 * Applied once after HAL conversion and before controller or commutation use.
 *============================================================================*/

static const bool CURRENT_PREPROCESS_ENABLE_COMMON_MODE_REMOVAL =
    false;

static const bool CURRENT_PREPROCESS_ENABLE_BEST_2_OF_3 =
    false;

static const bool CURRENT_PREPROCESS_ENABLE_AUTO_SHUNT_SELECTION =
    false;

/*
 * Manual phase-reconstruction override:
 * -1: automatic selection or no forced phase drop
 *  0: reconstruct phase A
 *  1: reconstruct phase B
 *  2: reconstruct phase C
 */
static const int CURRENT_PREPROCESS_FORCED_DROP_PHASE = -1;

/* Avoid measurements taken too close to a PWM switching edge. */
static const float CURRENT_PREPROCESS_AUTO_DUTY_EDGE_MARGIN = 0.08f;

/* Reject physically implausible preprocessed phase-current values. */
static const float CURRENT_PREPROCESS_ABS_MAX_A = 20.0f;

/*==============================================================================
 * CAN NODE IDENTIFIERS
 *============================================================================*/

static const uint8_t CAN_NODE_WHEEL1 = 0x3C;
static const uint8_t CAN_NODE_WHEEL2 = 0x40;
static const uint8_t CAN_NODE_WHEEL3 = 0x44;
static const uint8_t CAN_NODE_WHEEL4 = 0x48;