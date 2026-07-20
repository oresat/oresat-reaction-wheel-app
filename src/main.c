/*
 * OreSat Reaction Wheel Firmware
 *
 * Final runtime entry point for:
 * - high-speed motor control loop
 * - low-speed supervisory loop
 * - CAN / OD integration
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "CanComms.h"
#include "Controller.h"
#include "Fsm.h"

/*=============================================================================
 * THREAD CONFIGURATION
 *===========================================================================*/

#define CONTROL_STACK_SIZE             2048
#define CONTROL_THREAD_PRIORITY        -2

#define SUPERVISOR_STACK_SIZE          2048
#define SUPERVISOR_THREAD_PRIORITY     7

#define CONTROL_LOOP_PERIOD_US         62
#define SUPERVISOR_LOOP_PERIOD_MS      1

/*=============================================================================
 * THREAD STORAGE
 *===========================================================================*/

K_THREAD_STACK_DEFINE(g_controlStack, CONTROL_STACK_SIZE);
static struct k_thread g_controlThreadData;
static k_tid_t g_controlThreadId;

K_THREAD_STACK_DEFINE(g_supervisorStack, SUPERVISOR_STACK_SIZE);
static struct k_thread g_supervisorThreadData;
static k_tid_t g_supervisorThreadId;

/*=============================================================================
 * HIGH-SPEED CONTROL THREAD
 *===========================================================================*/

static void ControlThreadEntry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("[SYSTEM] High-speed control thread started.\n");

    while (1)
    {
        Controller_Update16kHz();

        // Keep the fast loop deterministic and independent of the RTOS tick.
        k_busy_wait(CONTROL_LOOP_PERIOD_US);
    }
}

/*=============================================================================
 * LOW-SPEED SUPERVISORY THREAD
 *===========================================================================*/

static void SupervisorThreadEntry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    printk("[SYSTEM] Supervisory thread started.\n");

    while (1)
    {
        // Apply new OD commands first so the supervisor reacts immediately.
        CanComms_ProcessCommands();

        // Run the system supervisor state machine.
        FsmUpdate();

        // Publish the latest internal telemetry back into the OD.
        CanComms_UpdateTelemetry();

        k_msleep(SUPERVISOR_LOOP_PERIOD_MS);
    }
}

/*=============================================================================
 * MAIN ENTRY POINT
 *===========================================================================*/

int main(void)
{
    printk("\n--- OreSat Reaction Wheel Firmware ---\n");

    // ------------------------------------------------------------------------
    // Initialize core subsystems
    // ------------------------------------------------------------------------
    Controller_Init();
    FsmInit();
    CanComms_Init();

    // ------------------------------------------------------------------------
    // Launch the deterministic fast control loop
    // ------------------------------------------------------------------------
    g_controlThreadId = k_thread_create(
        &g_controlThreadData,
        g_controlStack,
        K_THREAD_STACK_SIZEOF(g_controlStack),
        ControlThreadEntry,
        NULL, NULL, NULL,
        CONTROL_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
    k_thread_name_set(g_controlThreadId, "rw_control");

    // ------------------------------------------------------------------------
    // Launch the lower-rate supervisory loop
    // ------------------------------------------------------------------------
    g_supervisorThreadId = k_thread_create(
        &g_supervisorThreadData,
        g_supervisorStack,
        K_THREAD_STACK_SIZEOF(g_supervisorStack),
        SupervisorThreadEntry,
        NULL, NULL, NULL,
        SUPERVISOR_THREAD_PRIORITY,
        0,
        K_NO_WAIT
    );
    k_thread_name_set(g_supervisorThreadId, "rw_supervisor");

    // ------------------------------------------------------------------------
    // Main thread idles forever
    // ------------------------------------------------------------------------
    while (1)
    {
        k_msleep(1000);
    }

    return 0;
}