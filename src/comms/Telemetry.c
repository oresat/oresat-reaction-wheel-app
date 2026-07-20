#include "Telemetry.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/util.h>

#include "Hal.h"

/*=============================================================================
 * Telemetry.c
 *
 * PURPOSE:
 * Implements non-blocking telemetry transport alongside the deterministic
 * 16 kHz control loop.
 *
 * RESPONSIBILITIES:
 * - asynchronous UART initialization
 * - fixed packet construction and checksum
 * - trigger-byte polling
 * - experiment metadata storage
 * - single-producer/single-consumer packet queue
 *
 * OUT OF SCOPE:
 * - HAL sensor conversion
 * - controller state
 * - test sequencing
 * - host-side parsing
 *
 * EXECUTION CONTRACT:
 * The fast path only polls RX, decimates, builds, and queues. UART transmission
 * and queue draining are completed by the asynchronous driver callback.
 *===========================================================================*/

#ifndef TELEMETRY_UART_NODE
#define TELEMETRY_UART_NODE DT_NODELABEL(flexcomm4_lpuart4)
#endif

BUILD_ASSERT(sizeof(Oresat_Telemetry_t) == TELEMETRY_PACKET_SIZE_BYTES,
             "Oresat_Telemetry_t size mismatch");

#define TELEMETRY_PACKET_QUEUE_DEPTH 64u
#define TELEMETRY_UART_TX_TIMEOUT_US 1000000

static const struct device *s_uart = DEVICE_DT_GET(TELEMETRY_UART_NODE);

static bool s_initialized = false;
static bool s_enabled = false;
static volatile bool s_trigger_event = false;

static uint32_t s_loop_counter = 0u;
static uint32_t s_decimation_counter = 0u;
static int64_t s_start_time_us = 0;

static volatile uint32_t s_droppedPackets = 0u;
static volatile uint32_t s_txErrors = 0u;

static float s_speedCommand_revPerSec = 0.0f;
static uint8_t s_testState = 0u;
static uint8_t s_commutationMode = 0u;

static TelemetryExperimentStatus_t s_experimentStatus =
    TELEMETRY_EXPERIMENT_IDLE;

static TelemetryFaultCode_t s_faultCode =
    TELEMETRY_FAULT_NONE;

static float s_va_V = NAN;
static float s_vb_V = NAN;
static float s_vc_V = NAN;

static float s_tempPhaseA_C = NAN;
static float s_tempPhaseB_C = NAN;
static float s_tempPhaseC_C = NAN;
static float s_tempMcu_C = NAN;

static uint32_t s_v2_valid_flags = 0u;

/*
 * Packet ring.
 *
 * Producer: 16 kHz loop.
 * Consumer: UART async callback.
 *
 * Packets must remain valid until UART_TX_DONE, so we cannot transmit from a
 * local stack packet.
 */
static Oresat_Telemetry_t s_txQueue[TELEMETRY_PACKET_QUEUE_DEPTH];
static uint16_t s_txHead = 0u;
static uint16_t s_txTail = 0u;
static uint16_t s_txCount = 0u;
static bool s_txBusy = false;

static uint16_t Telemetry_NextQueueIndex(uint16_t index)
{
    return (uint16_t)((index + 1u) % TELEMETRY_PACKET_QUEUE_DEPTH);
}

static void Telemetry_CompleteActiveTxLocked(bool countError)
{
    if (s_txCount > 0u)
    {
        s_txTail = Telemetry_NextQueueIndex(s_txTail);
        s_txCount--;
    }

    s_txBusy = false;

    if (countError)
    {
        s_txErrors++;
    }
}

static void Telemetry_SetV2Flag(uint32_t flag, bool valid)
{
    if (valid)
    {
        s_v2_valid_flags |= flag;
    }
    else
    {
        s_v2_valid_flags &= ~flag;
    }
}

static uint8_t Telemetry_Checksum(const uint8_t *data, size_t len)
{
    uint32_t sum = 0u;

    if (data == NULL)
    {
        return 0u;
    }

    for (size_t i = 0u; i < len; i++)
    {
        sum += data[i];
    }

    return (uint8_t)(sum & 0xFFu);
}

static uint32_t Telemetry_GetTimestampUs(void)
{
    int64_t now_us = k_uptime_get() * 1000LL;
    int64_t delta_us = now_us - s_start_time_us;

    if (delta_us < 0)
    {
        delta_us = 0;
    }

    if (delta_us > UINT32_MAX)
    {
        delta_us = UINT32_MAX;
    }

    return (uint32_t)delta_us;
}

static uint32_t Telemetry_BuildValidFlags(void)
{
    uint32_t flags =
        TELEMETRY_VALID_VBUS |
        TELEMETRY_VALID_PHASE_CURRENTS |
        TELEMETRY_VALID_OMEGA |
        TELEMETRY_VALID_SPEED_COMMAND |
        TELEMETRY_VALID_V1_TEMPERATURES |
        TELEMETRY_VALID_PHASE_VOLTAGES |
        TELEMETRY_VALID_PHASE_TEMPERATURES;

    flags |= s_v2_valid_flags;

    return flags;
}

static void Telemetry_CheckForTriggerByteFast(void)
{
    if (!s_initialized || (s_uart == NULL))
    {
        return;
    }

    uint8_t byte;

    while (uart_poll_in(s_uart, &byte) == 0)
    {
        if (byte == TELEMETRY_TRIGGER_BYTE)
        {
            s_trigger_event = true;
        }
    }
}

static void Telemetry_ResetTxQueue(void)
{
    unsigned int key = irq_lock();

    s_txHead = 0u;
    s_txTail = 0u;
    s_txCount = 0u;
    s_txBusy = false;

    irq_unlock(key);
}

static void Telemetry_ResetRuntimeState(void)
{
    s_loop_counter = 0u;
    s_decimation_counter = 0u;
    s_start_time_us = k_uptime_get() * 1000LL;
    s_droppedPackets = 0u;
    s_txErrors = 0u;

    Telemetry_ResetTxQueue();
}

static void Telemetry_ResetExperimentMetadata(void)
{
    s_speedCommand_revPerSec = 0.0f;
    s_testState = 0u;
    s_commutationMode = 0u;

    s_experimentStatus = TELEMETRY_EXPERIMENT_IDLE;
    s_faultCode = TELEMETRY_FAULT_NONE;

    s_va_V = NAN;
    s_vb_V = NAN;
    s_vc_V = NAN;

    s_tempPhaseA_C = NAN;
    s_tempPhaseB_C = NAN;
    s_tempPhaseC_C = NAN;
    s_tempMcu_C = NAN;

    s_v2_valid_flags = 0u;
}

static void Telemetry_BuildPacket(Oresat_Telemetry_t *pkt)
{
    if (pkt == NULL)
    {
        return;
    }

    HalPower_t power = HalReadPower();
    HalPhaseCurrents_t currents = HalReadCurrents();
    HalRotorState_t rotor = HalReadRotor();
    HalThermal_t thermal = HalReadThermal();
    HalPhaseVoltages_t phaseVoltages = HalReadPhaseVoltages();

    memset(pkt, 0, sizeof(*pkt));

    pkt->magic = TELEMETRY_MAGIC;
    pkt->packet_version = TELEMETRY_PACKET_VERSION;
    pkt->timestamp_us = Telemetry_GetTimestampUs();

    pkt->vbus_V = power.busVoltage_V;
    pkt->ia_A = currents.phaseA_A;
    pkt->ib_A = currents.phaseB_A;
    pkt->ic_A = currents.phaseC_A;
    pkt->omega_rev_s = rotor.mechanicalVelocity_revPerSec;
    pkt->speed_command_rev_s = s_speedCommand_revPerSec;

    /*
     * V2 has real phase-voltage and per-phase thermal channels.
     * The legacy inverter/motor temperature fields are kept for packet
     * compatibility with existing DAQ scripts.
     */
    pkt->va_V = phaseVoltages.phaseA_V;
    pkt->vb_V = phaseVoltages.phaseB_V;
    pkt->vc_V = phaseVoltages.phaseC_V;

    pkt->temp_inverter_C = thermal.auxTemp_C;
    pkt->temp_motor_C = NAN;

    pkt->temp_phase_a_C = thermal.phaseATemp_C;
    pkt->temp_phase_b_C = thermal.phaseBTemp_C;
    pkt->temp_phase_c_C = thermal.phaseCTemp_C;
    pkt->temp_mcu_C = s_tempMcu_C;

    pkt->test_state = s_testState;
    pkt->commutation_mode = s_commutationMode;
    pkt->experiment_status = (uint8_t)s_experimentStatus;
    pkt->fault_code = (uint8_t)s_faultCode;

    pkt->valid_flags = Telemetry_BuildValidFlags();

    pkt->checksum = 0u;
    pkt->checksum = Telemetry_Checksum(
        (const uint8_t *)pkt,
        TELEMETRY_PACKET_SIZE_BYTES - 1u
    );
}

static void Telemetry_StartNextTx(void)
{
    if (!s_initialized || (s_uart == NULL))
    {
        return;
    }

    Oresat_Telemetry_t *pkt = NULL;

    unsigned int key = irq_lock();

    if (!s_txBusy && (s_txCount > 0u))
    {
        pkt = &s_txQueue[s_txTail];
        s_txBusy = true;
    }

    irq_unlock(key);

    if (pkt != NULL)
    {
        int err = uart_tx(
            s_uart,
            (const uint8_t *)pkt,
            TELEMETRY_PACKET_SIZE_BYTES,
            TELEMETRY_UART_TX_TIMEOUT_US
        );

        if (err != 0)
        {
            key = irq_lock();

            Telemetry_CompleteActiveTxLocked(true);

            irq_unlock(key);

            Telemetry_StartNextTx();
        }
    }
}

static void Telemetry_QueuePacketNonBlocking(void)
{
    Oresat_Telemetry_t pkt;

    Telemetry_BuildPacket(&pkt);

    unsigned int key = irq_lock();

    if (s_txCount >= TELEMETRY_PACKET_QUEUE_DEPTH)
    {
        s_droppedPackets++;
        irq_unlock(key);
        return;
    }

    memcpy(&s_txQueue[s_txHead], &pkt, sizeof(pkt));

    s_txHead = Telemetry_NextQueueIndex(s_txHead);
    s_txCount++;

    irq_unlock(key);

    Telemetry_StartNextTx();
}

static void Telemetry_UartCallback(
    const struct device *dev,
    struct uart_event *evt,
    void *user_data)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(user_data);

    if (evt == NULL)
    {
        return;
    }

    switch (evt->type)
    {
        case UART_TX_DONE:
        {
            unsigned int key = irq_lock();

            Telemetry_CompleteActiveTxLocked(false);

            irq_unlock(key);

            Telemetry_StartNextTx();
        } break;

        case UART_TX_ABORTED:
        {
            unsigned int key = irq_lock();

            Telemetry_CompleteActiveTxLocked(true);

            irq_unlock(key);

            Telemetry_StartNextTx();
        } break;

        default:
            break;
    }
}

/*=============================================================================
 * PUBLIC METADATA API
 *===========================================================================*/

void Telemetry_SetSpeedCommand(float speedCommand_revPerSec)
{
    s_speedCommand_revPerSec = speedCommand_revPerSec;
}

void Telemetry_SetTestState(uint8_t testState)
{
    s_testState = testState;
}

void Telemetry_SetCommutationMode(uint8_t commutationMode)
{
    s_commutationMode = commutationMode;
}

void Telemetry_SetExperimentStatus(TelemetryExperimentStatus_t status)
{
    s_experimentStatus = status;

    if (status != TELEMETRY_EXPERIMENT_FAULT)
    {
        s_faultCode = TELEMETRY_FAULT_NONE;
    }
}

void Telemetry_SetFaultCode(TelemetryFaultCode_t faultCode)
{
    s_faultCode = faultCode;

    if (faultCode != TELEMETRY_FAULT_NONE)
    {
        s_experimentStatus = TELEMETRY_EXPERIMENT_FAULT;
    }
}

void Telemetry_SetPhaseVoltagesV2(float va_V, float vb_V, float vc_V, bool valid)
{
    if (valid)
    {
        s_va_V = va_V;
        s_vb_V = vb_V;
        s_vc_V = vc_V;
        Telemetry_SetV2Flag(TELEMETRY_VALID_PHASE_VOLTAGES, true);
    }
    else
    {
        s_va_V = NAN;
        s_vb_V = NAN;
        s_vc_V = NAN;
        Telemetry_SetV2Flag(TELEMETRY_VALID_PHASE_VOLTAGES, false);
    }
}

void Telemetry_SetExpandedThermalsV2(
    float phaseA_C,
    float phaseB_C,
    float phaseC_C,
    float mcu_C,
    bool phaseTempsValid,
    bool mcuTempValid)
{
    if (phaseTempsValid)
    {
        s_tempPhaseA_C = phaseA_C;
        s_tempPhaseB_C = phaseB_C;
        s_tempPhaseC_C = phaseC_C;
        Telemetry_SetV2Flag(TELEMETRY_VALID_PHASE_TEMPERATURES, true);
    }
    else
    {
        s_tempPhaseA_C = NAN;
        s_tempPhaseB_C = NAN;
        s_tempPhaseC_C = NAN;
        Telemetry_SetV2Flag(TELEMETRY_VALID_PHASE_TEMPERATURES, false);
    }

    if (mcuTempValid)
    {
        s_tempMcu_C = mcu_C;
        Telemetry_SetV2Flag(TELEMETRY_VALID_MCU_TEMPERATURE, true);
    }
    else
    {
        s_tempMcu_C = NAN;
        Telemetry_SetV2Flag(TELEMETRY_VALID_MCU_TEMPERATURE, false);
    }
}

/*=============================================================================
 * PUBLIC CONTROL API
 *===========================================================================*/

void Telemetry_Init(void)
{
    s_initialized = false;
    s_enabled = false;
    s_trigger_event = false;

    Telemetry_ResetRuntimeState();
    Telemetry_ResetExperimentMetadata();

    if (!device_is_ready(s_uart))
    {
        printk("[TEL] UART device not ready.\n");
        return;
    }

    int err = uart_callback_set(s_uart, Telemetry_UartCallback, NULL);

    if (err != 0)
    {
        printk("[TEL] uart_callback_set failed: %d\n", err);
        return;
    }

    s_initialized = true;

    printk("[TEL] Async UART telemetry initialized.\n");
}

void Telemetry_Enable(bool enable)
{
    if (!s_initialized)
    {
        return;
    }

    s_enabled = enable;
    s_decimation_counter = 0u;
    s_loop_counter = 0u;

    if (enable)
    {
        s_start_time_us = k_uptime_get() * 1000LL;

        if (s_experimentStatus == TELEMETRY_EXPERIMENT_IDLE)
        {
            s_experimentStatus = TELEMETRY_EXPERIMENT_RUNNING;
        }
    }
}

bool Telemetry_IsEnabled(void)
{
    return s_enabled;
}

bool Telemetry_ConsumeTriggerEvent(void)
{
    if (!s_initialized)
    {
        return false;
    }

    if (!s_trigger_event)
    {
        return false;
    }

    s_trigger_event = false;
    return true;
}

uint32_t Telemetry_GetDroppedPacketCount(void)
{
    return s_droppedPackets;
}

/*=============================================================================
 * REAL-TIME UPDATE
 *===========================================================================*/

void Telemetry_Update16kHz(void)
{
    if (!s_initialized)
    {
        return;
    }

    /*
     * RX trigger polling only. Non-blocking.
     */
    Telemetry_CheckForTriggerByteFast();

    if (!s_enabled)
    {
        return;
    }

    s_loop_counter++;
    s_decimation_counter++;

    if (s_decimation_counter >= TELEMETRY_DECIMATION_TICKS)
    {
        s_decimation_counter = 0u;

        /*
         * Queue only. UART bytes are transmitted by the async driver callback.
         */
        Telemetry_QueuePacketNonBlocking();
    }
}