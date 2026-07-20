#pragma once

#include <stdbool.h>
#include <stdint.h>

/*==============================================================================
 * Hal.h
 *
 * Defines the hardware boundary used by the controller, calibration, telemetry,
 * and test layers. Hardware-independent code should use this interface instead
 * of accessing device-specific drivers directly.
 *============================================================================*/

/*==============================================================================
 * SENSOR TYPES
 *============================================================================*/

/* Measured motor phase currents in amperes. */
typedef struct
{
    float phaseA_A;
    float phaseB_A;
    float phaseC_A;
} HalPhaseCurrents_t;

/* Measured DC bus voltage and current. */
typedef struct
{
    float busVoltage_V;
    float busCurrent_A;
} HalPower_t;

/*
 * Measured motor phase voltages in volts.
 *
 * These V2 measurements represent sensed phase voltages, not commanded values.
 */
typedef struct
{
    float phaseA_V;
    float phaseB_V;
    float phaseC_V;
} HalPhaseVoltages_t;

/*
 * Rotor position and velocity produced by the HAL/encoder path.
 *
 * Mechanical position and velocity use revolutions and revolutions per second.
 * Electrical position and velocity use radians and radians per second.
 */
typedef struct
{
    float electricalAngle_rad;
    float electricalVelocity_radPerSec;

    float mechanicalAngle_rev;
    float mechanicalVelocity_revPerSec;

    uint8_t electricalSector;
    bool isValid;
} HalRotorState_t;

/*
 * Measured V2 temperatures in degrees Celsius.
 *
 * auxTemp_C is the board- or inverter-side auxiliary thermal channel.
 */
typedef struct
{
    float auxTemp_C;
    float phaseATemp_C;
    float phaseBTemp_C;
    float phaseCTemp_C;
} HalThermal_t;

/*
 * Raw fast-path ADC samples used for bring-up and conversion validation.
 * Runtime control should normally consume physical-unit measurements instead.
 */
typedef struct
{
    uint16_t phaseA_counts;
    uint16_t phaseB_counts;
    uint16_t phaseC_counts;

    uint16_t vbus_counts;

    uint16_t phaseVa_counts;
    uint16_t phaseVb_counts;
    uint16_t phaseVc_counts;
} HalRawAdc_t;

/* Raw low-rate thermal ADC samples. */
typedef struct
{
    uint16_t thermAux_counts;
    uint16_t thermPhaseA_counts;
    uint16_t thermPhaseB_counts;
    uint16_t thermPhaseC_counts;
} HalRawThermal_t;

/*==============================================================================
 * PWM COMMAND
 *============================================================================*/

/*
 * Unified inverter command shared by all commutation strategies.
 *
 * dutyA, dutyB, and dutyC are normalized to [0, 1]. A phase is actively driven
 * only when its float flag is false. Clearing enableGateDriver disables the
 * inverter output stage.
 */
typedef struct
{
    float dutyA;
    float dutyB;
    float dutyC;

    bool floatA;
    bool floatB;
    bool floatC;

    bool enableGateDriver;
} HalPwmCommand_t;

/*==============================================================================
 * SENSOR UPDATE STATUS
 *============================================================================*/

typedef enum
{
    HAL_SENSOR_UPDATE_OK = 0,
    HAL_SENSOR_UPDATE_TIMEOUT,
    HAL_SENSOR_UPDATE_NOT_READY,
    HAL_SENSOR_UPDATE_INVALID_STATE,
} HalSensorUpdateStatus_t;

/*==============================================================================
 * TOP-LEVEL HAL API
 *============================================================================*/

/* Initializes the complete hardware abstraction layer. */
void HalInit(void);

/* Refreshes the cached sensor measurements consumed by the control loop. */
HalSensorUpdateStatus_t HalUpdateSensorCache(void);

/* Returns the most recently cached physical-unit measurements. */
HalPhaseCurrents_t HalReadCurrents(void);
HalPower_t HalReadPower(void);
HalRotorState_t HalReadRotor(void);
HalThermal_t HalReadThermal(void);
HalPhaseVoltages_t HalReadPhaseVoltages(void);

/* Applies one unified inverter command to the PWM hardware. */
void HalWritePwm(const HalPwmCommand_t *command);

/* Reports and clears latched hardware fault state. */
bool HalHasHardwareFault(void);
void HalClearHardwareFault(void);

/*==============================================================================
 * DEBUG AND BRING-UP API
 *============================================================================*/

void HalPrintEncoderDebugSnapshot(void);
void HalPrintAdcDebug(void);
void HalPrintRegisterState(void);
void HalDumpCriticalRegisters(void);
void HalDumpSm1TriggerRegisters(void);

/*==============================================================================
 * PWM SUBMODULE API
 *============================================================================*/

void HalPwm_Init(void);
void HalPwm_DisableAllOutputs(void);
void HalPwm_WriteCommand(const HalPwmCommand_t *command);
void HalPwm_PrintRegisters(void);
void HalPwm_PrintSm1TriggerRegisters(void);

/*==============================================================================
 * ADC SUBMODULE API
 *============================================================================*/

void HalAdc_Init(void);
void HalAdc_PrepareForAcquisition(void);
bool HalAdc_AcquireSoftwareTriggered(HalRawAdc_t *output);
bool HalAdc_AcquireHardwareTriggered(HalRawAdc_t *output);
void HalAdc_PrintDebug(const HalRawAdc_t *rawSamples);

/*==============================================================================
 * INPUTMUX SUBMODULE API
 *============================================================================*/

void HalInputMux_Init(void);
void HalInputMux_PrintDebug(void);