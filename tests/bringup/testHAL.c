#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "Hal.h"
#include "Config.h"

#define PRINT_PERIOD_TICKS ((uint32_t)(0.2f * FOC_UPDATE_FREQ_HZ))

int main(void)
{
    k_thread_priority_set(k_current_get(), K_PRIO_COOP(0));

    printk("\n========================================\n");
    printk("       V2 HAL SYSTEM INTEGRATION TEST   \n");
    printk("========================================\n\n");

    HalInit();
    k_msleep(500);

    HalPwmCommand_t pwmCmd = {
        .dutyA = 0.5f,
        .dutyB = 0.5f,
        .dutyC = 0.5f,
        .floatA = false,
        .floatB = false,
        .floatC = false,
        .enableGateDriver = true
    };

    HalWritePwm(&pwmCmd);

    printk("[TEST] PWM armed at 50%% duty cycle.\n");
    printk("[TEST] Hardware-triggered ADC path should now be active.\n\n");

    k_msleep(500);

    uint32_t loopCounter = 0u;
    uint32_t ticks = 0u;
    uint32_t failCount = 0u;

    while (1)
    {
        HalSensorUpdateStatus_t status = HalUpdateSensorCache();
        ticks++;

        if (status == HAL_SENSOR_UPDATE_OK)
        {
            if (ticks >= PRINT_PERIOD_TICKS)
            {
                HalPhaseCurrents_t c = HalReadCurrents();
                HalPhaseVoltages_t v = HalReadPhaseVoltages();
                HalPower_t p = HalReadPower();
                HalRotorState_t r = HalReadRotor();
                HalThermal_t t = HalReadThermal();

                printk("\033[2J\033[H");

                printk("=== V2 HAL TELEMETRY DASHBOARD ===\n\n");

                printk("--- ELECTRICAL ---\n");
                printk("Phase A : % .4f A | % .3f V\n",
                       (double)c.phaseA_A,
                       (double)v.phaseA_V);
                printk("Phase B : % .4f A | % .3f V\n",
                       (double)c.phaseB_A,
                       (double)v.phaseB_V);
                printk("Phase C : % .4f A | % .3f V\n",
                       (double)c.phaseC_A,
                       (double)v.phaseC_V);
                printk("Vbus    : % .3f V\n", (double)p.busVoltage_V);
                printk("Ibus    : % .4f A\n\n", (double)p.busCurrent_A);

                printk("--- ENCODER / ROTOR ---\n");
                printk("Valid      : %s\n", r.isValid ? "true" : "false");
                printk("Sector     : %u\n", (unsigned int)r.electricalSector);
                printk("Mech Angle : % .6f rev\n", (double)r.mechanicalAngle_rev);
                printk("Mech Vel   : % .6f rev/s\n", (double)r.mechanicalVelocity_revPerSec);
                printk("Elec Angle : % .6f rad\n", (double)r.electricalAngle_rad);
                printk("Elec Vel   : % .6f rad/s\n\n", (double)r.electricalVelocity_radPerSec);

                printk("--- THERMALS ---\n");
                printk("Aux     : % .2f C\n", (double)t.auxTemp_C);
                printk("Phase A : % .2f C\n", (double)t.phaseATemp_C);
                printk("Phase B : % .2f C\n", (double)t.phaseBTemp_C);
                printk("Phase C : % .2f C\n\n", (double)t.phaseCTemp_C);

                printk("--- STATUS ---\n");
                printk("Loop Count : %u\n", (unsigned int)loopCounter++);
                printk("Failures   : %u\n", (unsigned int)failCount);

                ticks = 0u;
            }
        }
        else
        {
            failCount++;

            printk("\n[HAL_TEST][FAIL] HalUpdateSensorCache status=%d | failures=%u\n",
                   (int)status,
                   (unsigned int)failCount);

            HalPrintAdcDebug();
            k_msleep(2000);
        }
    }

    return 0;
}