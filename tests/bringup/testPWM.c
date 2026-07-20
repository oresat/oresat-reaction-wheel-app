#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Hal.h"

/*=============================================================================
 * TEST: V2 PWM / DEADTIME / OUTPUT-STATE VALIDATION
 *
 * Owns:
 * - Low-level PWM bring-up validation
 * - Safe disabled-state validation
 * - Centered complementary PWM validation
 * - Per-phase duty-command validation
 * - Per-phase float-state validation
 * - Register dump checkpoints
 *
 * Does not own:
 * - Controller behavior
 * - Calibration
 * - Commutation strategy validation
 * - Closed-loop motor operation
 *
 * Notes:
 * - Bench-only hardware validation test.
 * - Probe outputs with motor disconnected or supply current-limited.
 *===========================================================================*/

#define STEP_WAIT_MS             4000u
#define STARTUP_SAFE_WAIT_MS     2000u
#define CENTERED_PWM_WAIT_MS     5000u
#define FINAL_DISABLED_WAIT_MS   3000u
#define LED_TOGGLE_PERIOD_MS     100u

static void ToggleLedIfReady(const struct gpio_dt_spec *led)
{
    if ((led != NULL) && gpio_is_ready_dt(led))
    {
        gpio_pin_toggle_dt(led);
    }
}

static void WaitWithLed(const struct gpio_dt_spec *led, uint32_t wait_ms)
{
    uint32_t elapsed_ms = 0u;

    while (elapsed_ms < wait_ms)
    {
        ToggleLedIfReady(led);
        k_msleep(LED_TOGGLE_PERIOD_MS);
        elapsed_ms += LED_TOGGLE_PERIOD_MS;
    }
}

static void PrintStepHeader(uint32_t step, const char *title)
{
    printk("\n------------------------------------------------------\n");
    printk("[STEP %u] %s\n", (unsigned int)step, title);
    printk("------------------------------------------------------\n");
}

static HalPwmCommand_t MakePwmCommand(
    float dutyA,
    float dutyB,
    float dutyC,
    bool floatA,
    bool floatB,
    bool floatC,
    bool enableGateDriver)
{
    return (HalPwmCommand_t){
        .dutyA = dutyA,
        .dutyB = dutyB,
        .dutyC = dutyC,

        .floatA = floatA,
        .floatB = floatB,
        .floatC = floatC,

        .enableGateDriver = enableGateDriver,
    };
}

static void ApplyPwmAndDump(const HalPwmCommand_t *cmd)
{
    if (cmd != NULL)
    {
        HalWritePwm(cmd);
    }

    HalDumpCriticalRegisters();
}

static void BlinkStartup(const struct gpio_dt_spec *led)
{
    for (int i = 0; i < 5; i++)
    {
        ToggleLedIfReady(led);
        k_msleep(250);
    }
}

int main(void)
{
    const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

    if (gpio_is_ready_dt(&led))
    {
        gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    }

    BlinkStartup(&led);

    printk("\n======================================================\n");
    printk("          TEST: V2 PWM / DEADTIME VALIDATION          \n");
    printk("======================================================\n");

    printk("[TEST] Initializing HAL...\n");
    HalInit();
    printk("[TEST] HAL initialized.\n");

    PrintStepHeader(1u, "Verify safe startup state");
    printk("[TEST] Expect all PWM outputs disabled after HalInit().\n");
    HalDumpCriticalRegisters();
    WaitWithLed(&led, STARTUP_SAFE_WAIT_MS);

    PrintStepHeader(2u, "Enable 50% centered PWM on all phases");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.5f,
                false, false, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Probe PWM outputs now.\n");
    printk("[TEST] Expect complementary switching with deadtime on each phase pair.\n");
    printk("[TEST] Confirm V2 phase mapping against schematic / harness.\n");
    WaitWithLed(&led, CENTERED_PWM_WAIT_MS);

    PrintStepHeader(10u, "Continuous 50% PWM scope hold");
{
    HalPwmCommand_t cmd =
        MakePwmCommand(
            0.5f, 0.5f, 0.5f,
            false, false, false,
            true
        );

    ApplyPwmAndDump(&cmd);
}

    printk("[TEST] Holding 50%% PWM forever for oscilloscope probing.\n");

    while (1)
    {
        ToggleLedIfReady(&led);
        k_msleep(500);
    }

    PrintStepHeader(3u, "Change only Phase A duty");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.25f, 0.5f, 0.5f,
                false, false, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect only Phase A duty to change.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(4u, "Change only Phase B duty");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.75f, 0.5f,
                false, false, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect only Phase B duty to change.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(5u, "Change only Phase C duty");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.75f,
                false, false, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect only Phase C duty to change.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(6u, "Float Phase A only");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.5f,
                true, false, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect Phase A disabled/floating, B and C still switching.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(7u, "Float Phase B only");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.5f,
                false, true, false,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect Phase B disabled/floating, A and C still switching.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(8u, "Float Phase C only");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.5f,
                false, false, true,
                true
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect Phase C disabled/floating, A and B still switching.\n");
    WaitWithLed(&led, STEP_WAIT_MS);

    PrintStepHeader(9u, "Disable all outputs");
    {
        HalPwmCommand_t cmd =
            MakePwmCommand(
                0.5f, 0.5f, 0.5f,
                true, true, true,
                false
            );

        ApplyPwmAndDump(&cmd);
    }

    printk("[TEST] Expect all phases disabled and gate driver disabled.\n");
    WaitWithLed(&led, FINAL_DISABLED_WAIT_MS);

    printk("\n======================================================\n");
    printk("               PWM VALIDATION COMPLETE                \n");
    printk("======================================================\n");
    printk("[TEST] Outputs left disabled. Press reset or reflash to rerun.\n");

    while (1)
    {
        ToggleLedIfReady(&led);
        k_msleep(500);
    }

    return 0;
}