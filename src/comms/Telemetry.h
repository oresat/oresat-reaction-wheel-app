#pragma once

#include <stdbool.h>
#include <stdint.h>

/*=============================================================================
 * Telemetry.h
 *
 * PURPOSE:
 * Defines the stable UART telemetry contract used by firmware tests and host
 * acquisition tools.
 *
 * RESPONSIBILITIES:
 * - fixed packed packet layout and versioning
 * - experiment lifecycle and fault metadata
 * - host-trigger byte definition
 * - V1/V2 measurement validity flags
 * - public metadata and 16 kHz update API
 *
 * OUT OF SCOPE:
 * - HAL sensor conversion
 * - controller behavior
 * - test sequencing
 * - host-side CSV and plot processing
 *
 * EXECUTION MODEL:
 * The 16 kHz producer only builds and queues packets. The asynchronous UART
 * callback owns queue draining so transport work never blocks the control loop.
 *===========================================================================*/

/*=============================================================================
 * UART TELEMETRY FORMAT
 *===========================================================================*/

#define TELEMETRY_MAGIC                 0xBEEFu
#define TELEMETRY_PACKET_VERSION        2u
#define TELEMETRY_TRIGGER_BYTE          0xAAu

/*
 * 16 kHz / 160 = 100 packets/s.
 *
 * This rate preserves high-resolution experiment telemetry while keeping UART
 * utilization comfortably below saturation for the 77-byte thesis packet.
 */
#define TELEMETRY_DECIMATION_TICKS      160u

/*
 * Packet layout, packed little-endian:
 *
 * uint16_t magic
 * uint16_t packet_version
 * uint32_t timestamp_us
 *
 * float vbus_V
 * float ia_A
 * float ib_A
 * float ic_A
 * float omega_rev_s
 * float speed_command_rev_s
 *
 * float va_V                  V2 phase voltage
 * float vb_V                  V2 phase voltage
 * float vc_V                  V2 phase voltage
 *
 * float temp_inverter_C        legacy field; V2 maps auxTemp_C here
 * float temp_motor_C           legacy field; NaN on V2 unless populated
 *
 * float temp_phase_a_C         V2 phase-A temperature
 * float temp_phase_b_C         V2 phase-B temperature
 * float temp_phase_c_C         V2 phase-C temperature
 * float temp_mcu_C             V2 MCU temperature if provided
 *
 * uint8_t test_state
 * uint8_t commutation_mode
 * uint8_t experiment_status
 * uint8_t fault_code
 *
 * uint32_t valid_flags
 *
 * uint8_t checksum
 *
 * Total = 77 bytes.
 */
#define TELEMETRY_PACKET_SIZE_BYTES     77u

/*=============================================================================
 * EXPERIMENT LIFECYCLE STATUS
 *===========================================================================*/

typedef enum
{
    TELEMETRY_EXPERIMENT_IDLE = 0,
    TELEMETRY_EXPERIMENT_RUNNING = 1,
    TELEMETRY_EXPERIMENT_COMPLETE = 2,
    TELEMETRY_EXPERIMENT_FAULT = 3
} TelemetryExperimentStatus_t;

/*=============================================================================
 * FAULT CODES
 *
 * Keep this small and generic. Individual tests may map local causes into these
 * common values.
 *===========================================================================*/

typedef enum
{
    TELEMETRY_FAULT_NONE = 0,
    TELEMETRY_FAULT_CONTROLLER = 1,
    TELEMETRY_FAULT_HARDWARE = 2,
    TELEMETRY_FAULT_OVERCURRENT = 3,
    TELEMETRY_FAULT_UNDERVOLTAGE = 4,
    TELEMETRY_FAULT_OVERVOLTAGE = 5,
    TELEMETRY_FAULT_TIMEOUT = 6,
    TELEMETRY_FAULT_TEST_ABORTED = 7,
    TELEMETRY_FAULT_UNKNOWN = 255
} TelemetryFaultCode_t;

/*=============================================================================
 * VALIDITY FLAGS
 *
 * These tell the host which fields are real measurements for the current
 * hardware revision.
 *===========================================================================*/

#define TELEMETRY_VALID_VBUS                 (1u << 0)
#define TELEMETRY_VALID_PHASE_CURRENTS       (1u << 1)
#define TELEMETRY_VALID_OMEGA                (1u << 2)
#define TELEMETRY_VALID_SPEED_COMMAND        (1u << 3)
#define TELEMETRY_VALID_V1_TEMPERATURES      (1u << 4)

#define TELEMETRY_VALID_PHASE_VOLTAGES       (1u << 8)   /* V2 */
#define TELEMETRY_VALID_PHASE_TEMPERATURES   (1u << 9)   /* V2 */
#define TELEMETRY_VALID_MCU_TEMPERATURE      (1u << 10)  /* V2 */

/*=============================================================================
 * PACKET STRUCTURE
 *===========================================================================*/

#pragma pack(push, 1)
typedef struct
{
    uint16_t magic;
    uint16_t packet_version;
    uint32_t timestamp_us;

    float vbus_V;
    float ia_A;
    float ib_A;
    float ic_A;
    float omega_rev_s;
    float speed_command_rev_s;

    float va_V;
    float vb_V;
    float vc_V;

    float temp_inverter_C;
    float temp_motor_C;

    float temp_phase_a_C;
    float temp_phase_b_C;
    float temp_phase_c_C;
    float temp_mcu_C;

    uint8_t test_state;
    uint8_t commutation_mode;
    uint8_t experiment_status;
    uint8_t fault_code;

    uint32_t valid_flags;

    uint8_t checksum;
} Oresat_Telemetry_t;
#pragma pack(pop)

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

void Telemetry_Init(void);
void Telemetry_Enable(bool enable);
bool Telemetry_IsEnabled(void);

void Telemetry_SetSpeedCommand(float speedCommand_revPerSec);
void Telemetry_SetTestState(uint8_t testState);
void Telemetry_SetCommutationMode(uint8_t commutationMode);

void Telemetry_SetExperimentStatus(TelemetryExperimentStatus_t status);
void Telemetry_SetFaultCode(TelemetryFaultCode_t faultCode);

void Telemetry_SetPhaseVoltagesV2(float va_V, float vb_V, float vc_V, bool valid);

void Telemetry_SetExpandedThermalsV2(
    float phaseA_C,
    float phaseB_C,
    float phaseC_C,
    float mcu_C,
    bool phaseTempsValid,
    bool mcuTempValid);

bool Telemetry_ConsumeTriggerEvent(void);

uint32_t Telemetry_GetDroppedPacketCount(void);

void Telemetry_Update16kHz(void);