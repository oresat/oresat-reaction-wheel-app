/*
 * 2026 Portland State Aerospace Society
 * SPDX-License-Identifier: Apache-2.0
 *
 * OreSat Reaction Wheel Controller
 * Production firmware entry point for the V2 controller board.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "CanComms.h"
#include "Controller.h"
#include "Fsm.h"
#include "Telemetry.h"

/*==============================================================================
 * APPLICATION CONFIGURATION
 *============================================================================*/

#define CONTROL_THREAD_STACK_SIZE            3072
#define CONTROL_THREAD_PRIORITY              K_PRIO_PREEMPT(0)

#define SUPERVISOR_THREAD_STACK_SIZE         2048
#define SUPERVISOR_THREAD_PRIORITY           K_PRIO_PREEMPT(1)

/*
 * The supervisor owns only coarse lifecycle and command translation. A 100 Hz
 * rate is sufficient for arming, disarming, setpoint updates, and fault
 * reporting while keeping all non-control work out of the 16 kHz path.
 */
#define SUPERVISOR_PERIOD                    K_MSEC(10)

/*==============================================================================
 * THREAD STORAGE
 *============================================================================*/

K_THREAD_STACK_DEFINE(
    s_controlThreadStack,
    CONTROL_THREAD_STACK_SIZE);

static struct k_thread s_controlThread;

K_THREAD_STACK_DEFINE(
    s_supervisorThreadStack,
    SUPERVISOR_THREAD_STACK_SIZE);

static struct k_thread s_supervisorThread;

/*==============================================================================
 * HIGH-SPEED CONTROL EXECUTION
 *============================================================================*/

/*
 * Controller_Update16kHz() owns the complete deterministic motor-control cycle.
 * Its HAL acquisition path is synchronized to the 16 kHz PWM/ADC hardware
 * trigger; main.c must not add sleeps, UART transmission, CAN processing,
 * control mathematics, or application policy to this loop.
 *
 * Telemetry_Update16kHz() performs only bounded trigger polling and decimated
 * packet enqueueing. UART transmission remains owned by the asynchronous UART
 * callback and therefore cannot block the control path.
 */
static void ControlThreadEntry(
    void *arg1,
    void *arg2,
    void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true)
    {
        Controller_Update16kHz();
        Telemetry_Update16kHz();
    }
}

/*==============================================================================
 * LOW-RATE SUPERVISION
 *============================================================================*/

static void SupervisorThreadEntry(
    void *arg1,
    void *arg2,
    void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    while (true)
    {
        /*
         * NOTE:
         * CAN/CANopen is the intended production flight command and telemetry
         * interface, but it is currently non-functional on the V2 hardware and
         * has not yet been validated. These translation calls are intentionally
         * retained so the production architecture remains correct. They do not
         * make the CAN transport operational; CAN hardware/stack bring-up and
         * end-to-end validation are still required before flight use.
         */
        CanComms_ProcessCommands();

        /*
         * The FSM alone owns arming, disarming, calibration sequencing, normal
         * closed-loop entry, and latched-fault lifecycle policy.
         */
        FsmUpdate();

        /*
         * NOTE:
         * This updates the Object Dictionary only. CAN transmission is currently
         * non-functional and requires separate hardware/stack validation.
         */
        CanComms_UpdateTelemetry();

        k_sleep(SUPERVISOR_PERIOD);
    }
}

/*==============================================================================
 * THREAD CREATION
 *============================================================================*/

static int StartProductionThreads(void)
{
    k_tid_t controlThreadId = k_thread_create(
        &s_controlThread,
        s_controlThreadStack,
        K_THREAD_STACK_SIZEOF(s_controlThreadStack),
        ControlThreadEntry,
        NULL,
        NULL,
        NULL,
        CONTROL_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    if (controlThreadId == NULL)
    {
        return -1;
    }

    (void)k_thread_name_set(controlThreadId, "rw_control_16khz");

    k_tid_t supervisorThreadId = k_thread_create(
        &s_supervisorThread,
        s_supervisorThreadStack,
        K_THREAD_STACK_SIZEOF(s_supervisorThreadStack),
        SupervisorThreadEntry,
        NULL,
        NULL,
        NULL,
        SUPERVISOR_THREAD_PRIORITY,
        0,
        K_NO_WAIT);

    if (supervisorThreadId == NULL)
    {
        /*
         * The controller was initialized in idle and all PWM outputs remain
         * disabled. Abort application startup rather than operating without
         * lifecycle supervision.
         */
        Controller_SetControlMode(CTRL_MODE_IDLE);
        return -2;
    }

    (void)k_thread_name_set(supervisorThreadId, "rw_supervisor");

    return 0;
}

/*==============================================================================
 * MAIN
 *============================================================================*/

int main(void)
{
    printk("\n[RW] OreSat V2 production firmware\n");

    /*
     * Controller_Init() initializes the complete V2 HAL, disables inverter
     * outputs, initializes current preprocessing, loads validated persisted
     * calibration when available, resets controller state, and selects FOC as
     * the default commutation strategy. It must complete before either runtime
     * execution context begins.
     */
    Controller_Init();

    /*
     * Restate the production-safe initial command explicitly at the application
     * boundary. No motor actuation is permitted merely because firmware booted.
     */
    Controller_SetCommutationMode(COMM_MODE_FOC);
    Controller_SetVelocity(0.0f);
    Controller_SetControlMode(CTRL_MODE_IDLE);

    /*
     * The current telemetry implementation reads the V2 HAL phase-voltage and
     * phase-temperature caches directly. V2 measurements are therefore the
     * authoritative runtime values; legacy packet fields remain only for host
     * compatibility.
     *
     * UART telemetry starts disabled. It may be enabled by an approved ground
     * interface without placing UART transmission in the fast loop.
     */
    Telemetry_Init();
    Telemetry_Enable(false);
    Telemetry_SetCommutationMode((uint8_t)COMM_MODE_FOC);
    Telemetry_SetExperimentStatus(TELEMETRY_EXPERIMENT_IDLE);
    Telemetry_SetFaultCode(TELEMETRY_FAULT_NONE);

    /*
     * Initialize supervision before commands can be translated. The FSM starts
     * in BOOT and remains responsible for all subsequent lifecycle transitions.
     */
    FsmInit();

    /*
     * NOTE:
     * CAN/CANopen is intentionally initialized because it is the production
     * flight interface and establishes safe Object Dictionary defaults.
     * CAN is currently non-functional on the V2 hardware and must not be treated
     * as a validated command or telemetry path until hardware/stack bring-up and
     * end-to-end testing are complete.
     */
    CanComms_Init();

    int result = StartProductionThreads();
    if (result != 0)
    {
        Controller_SetControlMode(CTRL_MODE_IDLE);
        printk("[RW][FAULT] Runtime thread startup failed: %d\n", result);
        return result;
    }

    printk("[RW] Initialized in safe state; awaiting supervisor commands\n");
    printk("[RW][WARN] CAN/CANopen is present but not operational\n");

    /*
     * All production work is owned by the control and supervisor contexts.
     * Keeping main blocked avoids accidental application-level control policy.
     */
    while (true)
    {
        k_sleep(K_FOREVER);
    }

    return 0;

}