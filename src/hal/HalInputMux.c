#include "Hal.h"

#include <stdint.h>

#include <zephyr/sys/printk.h>

#include <fsl_common.h>

/*==============================================================================
 * HalInputMux.c
 *
 * Owns the hardware trigger route from FlexPWM1 to ADC0.
 *
 * ADC0 uses the PWM-synchronous trigger for the fast current/voltage sample
 * bundle. ADC1 is intentionally excluded from this route because thermal
 * acquisition is software-triggered at a relaxed rate.
 *============================================================================*/

/*==============================================================================
 * LOCAL CONFIGURATION
 *============================================================================*/

#define HAL_INPUTMUX_DEBUG_PRINTS              0
#define INPUTMUX_ADC_TRIG_PWM1_A1_TRIG0        0x22u

/*==============================================================================
 * PUBLIC API
 *============================================================================*/

void HalInputMux_Init(void)
{
    CLOCK_EnableClock(kCLOCK_InputMux0);

    /*
     * Route PWM1_A1_TRIG0 exclusively to ADC0 trigger 0. ADC1 remains
     * disconnected from the 16 kHz PWM carrier.
     */
    INPUTMUX0->ADC0_TRIG[0] = INPUTMUX_ADC_TRIG_PWM1_A1_TRIG0;
}

void HalInputMux_PrintDebug(void)
{
#if HAL_INPUTMUX_DEBUG_PRINTS
    printk(
        "[HAL][MUX] ADC0_TRIG0=0x%08X "
        "(ADC1 hardware trigger disabled for V2)\n",
        (unsigned int)INPUTMUX0->ADC0_TRIG[0]);
#endif
}