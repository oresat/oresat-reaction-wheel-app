#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "Config.h"
#include "Encoder.h"
#include "MathUtil.h"

/*=============================================================================
 * TEST: V2 ENCODER BRING-UP VALIDATION
 *
 * Owns:
 * - Raw MA732 SPI readout validation
 * - Encoder.c mechanical estimator validation
 * - Encoder.c electrical angle and sector validation
 *
 * Does not own:
 * - HAL rotor cache validation
 * - Controller behavior
 * - Calibration
 * - Commutation behavior
 *
 * Test modes:
 * - RAW_SPI:
 *     Validate SPI wiring, raw MA732 word format, 14-bit count extraction,
 *     wrapped count delta, and mechanical position conversion.
 *
 * - MECHANICAL:
 *     Feed raw MA732 counts into EncoderUpdate() and validate wrapped
 *     mechanical position and velocity behavior.
 *
 * - ELECTRICAL:
 *     Feed raw MA732 counts into EncoderUpdate() and validate pole-pair
 *     scaling, electrical angle wrapping, and sector progression.
 *===========================================================================*/

/*=============================================================================
 * TEST SELECTION
 *===========================================================================*/

typedef enum
{
    ENCODER_TEST_MODE_RAW_SPI = 0,
    ENCODER_TEST_MODE_MECHANICAL,
    ENCODER_TEST_MODE_ELECTRICAL
} EncoderTestMode_t;

/*
 * Bring-up order:
 * 1. RAW_SPI
 * 2. MECHANICAL
 * 3. ELECTRICAL
 */
#define ENCODER_TEST_MODE        ENCODER_TEST_MODE_ELECTRICAL

#define UPDATE_PERIOD_MS         2u
#define PRINT_PERIOD_MS          200u

#define ENCODER_DIR_SIGN         (1.0f)

/*=============================================================================
 * DEVICE HANDLES
 *===========================================================================*/

static const struct gpio_dt_spec g_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

static const struct spi_dt_spec g_ma732Spi =
    SPI_DT_SPEC_GET(
        DT_NODELABEL(encoder),
        SPI_WORD_SET(16) | SPI_TRANSFER_MSB,
        0
    );

/*=============================================================================
 * COMMON HELPERS
 *===========================================================================*/

static void ToggleLed(void)
{
    if (gpio_is_ready_dt(&g_led))
    {
        gpio_pin_toggle_dt(&g_led);
    }
}

static bool DevicesReady(void)
{
    if (!spi_is_ready_dt(&g_ma732Spi))
    {
        printk("[ENC][FAIL] MA732 SPI device not ready.\n");
        return false;
    }

    return true;
}

static void PrintBinary(uint16_t value, uint8_t bits)
{
    for (int i = (int)bits - 1; i >= 0; i--)
    {
        printk("%c", (value & (1u << i)) ? '1' : '0');
    }
}

static bool ReadMa732RawWord(uint16_t *rawWord_out)
{
    if (rawWord_out == NULL)
    {
        return false;
    }

    uint16_t txData = 0x0000u;
    uint16_t rxData = 0x0000u;

    struct spi_buf txBuf =
    {
        .buf = &txData,
        .len = sizeof(txData),
    };

    struct spi_buf_set txBufs =
    {
        .buffers = &txBuf,
        .count = 1,
    };

    struct spi_buf rxBuf =
    {
        .buf = &rxData,
        .len = sizeof(rxData),
    };

    struct spi_buf_set rxBufs =
    {
        .buffers = &rxBuf,
        .count = 1,
    };

    int err = spi_transceive_dt(&g_ma732Spi, &txBufs, &rxBufs);
    if (err != 0)
    {
        return false;
    }

    *rawWord_out = rxData;
    return true;
}

static bool ReadMa732Counts(uint16_t *counts_out)
{
    if (counts_out == NULL)
    {
        return false;
    }

    uint16_t rawWord = 0u;

    if (!ReadMa732RawWord(&rawWord))
    {
        return false;
    }

    /*
     * MA732 convention used by the HAL path:
     * - Raw SPI transaction returns a 16-bit word.
     * - 14-bit angle is contained in bits [15:2].
     */
    *counts_out = (rawWord >> 2) & 0x3FFFu;
    return true;
}

static int16_t WrappedDeltaCounts(uint16_t now, uint16_t previous)
{
    int32_t delta = (int32_t)now - (int32_t)previous;
    int32_t halfCounts = (int32_t)ENCODER_CPR / 2;

    if (delta > halfCounts)
    {
        delta -= (int32_t)ENCODER_CPR;
    }
    else if (delta < -halfCounts)
    {
        delta += (int32_t)ENCODER_CPR;
    }

    return (int16_t)delta;
}

static float WrappedDeltaRev(float now, float previous)
{
    float delta = now - previous;

    if (delta > 0.5f)
    {
        delta -= 1.0f;
    }
    else if (delta < -0.5f)
    {
        delta += 1.0f;
    }

    return delta;
}

static float WrappedDeltaRad(float now, float previous)
{
    float delta = now - previous;

    while (delta > MATH_PI_F)
    {
        delta -= MATH_TWO_PI_F;
    }

    while (delta < -MATH_PI_F)
    {
        delta += MATH_TWO_PI_F;
    }

    return delta;
}

static uint8_t ElectricalAngleToSector(float electricalAngle_rad)
{
    float angle = electricalAngle_rad;

    while (angle < 0.0f)
    {
        angle += MATH_TWO_PI_F;
    }

    while (angle >= MATH_TWO_PI_F)
    {
        angle -= MATH_TWO_PI_F;
    }

    uint8_t sector =
        (uint8_t)(angle / (MATH_TWO_PI_F / 6.0f)) + 1u;

    if (sector < 1u)
    {
        sector = 1u;
    }
    else if (sector > 6u)
    {
        sector = 6u;
    }

    return sector;
}

static int8_t WrappedDeltaSector(uint8_t now, uint8_t previous)
{
    int delta = (int)now - (int)previous;

    if (delta > 3)
    {
        delta -= 6;
    }
    else if (delta < -3)
    {
        delta += 6;
    }

    return (int8_t)delta;
}

static float ComputeDtFromCycles(uint32_t now, uint32_t previous)
{
    uint32_t cycles = now - previous;
    return (float)k_cyc_to_ns_floor32(cycles) / 1000000000.0f;
}

/*=============================================================================
 * MODE 0: RAW SPI TEST
 *===========================================================================*/

static int RunRawSpiTest(void)
{
    printk("\n======================================================\n");
    printk("             TEST: V2 MA732 RAW SPI READOUT           \n");
    printk("======================================================\n");

    if (!DevicesReady())
    {
        return -1;
    }

    printk("[RAW] Rotate shaft slowly by hand.\n");
    printk("[RAW] Expect smooth count changes, correct wrap, and few/no failures.\n\n");

    uint16_t previousCounts = 0u;
    bool havePrevious = false;
    uint32_t failCount = 0u;

    while (1)
    {
        uint16_t rawWord = 0u;

        if (!ReadMa732RawWord(&rawWord))
        {
            failCount++;

            printk("[RAW][FAIL] SPI transfer failed | failures=%u\n",
                   (unsigned int)failCount);

            k_msleep(PRINT_PERIOD_MS);
            continue;
        }

        uint16_t counts =
            (rawWord >> 2) & 0x3FFFu;

        int16_t deltaCounts = 0;

        if (havePrevious)
        {
            deltaCounts =
                WrappedDeltaCounts(counts, previousCounts);
        }

        float mechanicalPosition_rev =
            (float)counts / (float)ENCODER_CPR;

        printk("[RAW] raw16=");
        PrintBinary(rawWord, 16u);

        printk(" | 0x%04X", rawWord);

        printk(" | counts14=");
        PrintBinary(counts, 14u);

        printk(" | %5u", (unsigned int)counts);
        printk(" | dCounts=%6d", (int)deltaCounts);
        printk(" | mech=% .6f rev", (double)mechanicalPosition_rev);
        printk(" | fails=%u\n", (unsigned int)failCount);

        previousCounts = counts;
        havePrevious = true;

        ToggleLed();
        k_msleep(PRINT_PERIOD_MS);
    }

    return 0;
}

/*=============================================================================
 * MODE 1: MECHANICAL ESTIMATOR TEST
 *===========================================================================*/

static int RunMechanicalEstimatorTest(void)
{
    printk("\n======================================================\n");
    printk("        TEST: V2 MA732 MECHANICAL ESTIMATOR CHECK     \n");
    printk("======================================================\n");

    if (!DevicesReady())
    {
        return -1;
    }

    EncoderInit();
    EncoderSetDirection(ENCODER_DIR_SIGN);

    printk("[MECH] Convention: CCW from board view = positive.\n");
    printk("[MECH] Rotate slowly, reverse direction, and cross wrap.\n\n");

    uint32_t failCount = 0u;

    uint32_t lastCycle = k_cycle_get_32();
    bool haveDt = false;

    uint16_t windowStartCounts = 0u;
    uint16_t latestCounts = 0u;
    bool windowInitialized = false;

    float windowElapsed_s = 0.0f;
    uint32_t printElapsed_ms = 0u;

    float velMin_revPerSec = 0.0f;
    float velMax_revPerSec = 0.0f;
    float velSum_revPerSec = 0.0f;
    uint32_t velSampleCount = 0u;
    bool velWindowInitialized = false;

    while (1)
    {
        uint16_t rawCounts = 0u;

        if (!ReadMa732Counts(&rawCounts))
        {
            failCount++;

            printk("[MECH][FAIL] SPI transfer failed | failures=%u\n",
                   (unsigned int)failCount);

            k_msleep(UPDATE_PERIOD_MS);
            continue;
        }

        uint32_t now = k_cycle_get_32();

        if (haveDt)
        {
            float dt_s = ComputeDtFromCycles(now, lastCycle);

            if (dt_s > 0.0f)
            {
                EncoderUpdate(rawCounts, dt_s);
                windowElapsed_s += dt_s;
            }
        }

        lastCycle = now;
        haveDt = true;

        EncoderMechanicalState_t mech = EncoderGetMechanicalState();

        if (!windowInitialized)
        {
            windowStartCounts = rawCounts;
            latestCounts = rawCounts;
            windowInitialized = true;
        }
        else
        {
            latestCounts = rawCounts;
        }

        if (!velWindowInitialized)
        {
            velMin_revPerSec = mech.velocity_revPerSec;
            velMax_revPerSec = mech.velocity_revPerSec;
            velSum_revPerSec = mech.velocity_revPerSec;
            velSampleCount = 1u;
            velWindowInitialized = true;
        }
        else
        {
            if (mech.velocity_revPerSec < velMin_revPerSec)
            {
                velMin_revPerSec = mech.velocity_revPerSec;
            }

            if (mech.velocity_revPerSec > velMax_revPerSec)
            {
                velMax_revPerSec = mech.velocity_revPerSec;
            }

            velSum_revPerSec += mech.velocity_revPerSec;
            velSampleCount++;
        }

        if (printElapsed_ms >= PRINT_PERIOD_MS)
        {
            int16_t windowDeltaCounts =
                WrappedDeltaCounts(latestCounts, windowStartCounts);

            float rawVelocity_revPerSec = 0.0f;

            if (windowElapsed_s > 0.0f)
            {
                rawVelocity_revPerSec =
                    ((float)windowDeltaCounts / (float)ENCODER_CPR) /
                    windowElapsed_s;
            }

            float velAvg_revPerSec = 0.0f;

            if (velSampleCount > 0u)
            {
                velAvg_revPerSec =
                    velSum_revPerSec / (float)velSampleCount;
            }

            printk("[MECH] cnt:%5u->%5u | dCnt:%6d | dt:% .3f ms | "
                   "rawVel:% .6f rev/s | estVel:% .6f rev/s | "
                   "avg:% .6f | min:% .6f | max:% .6f | pos:% .6f rev | fails:%u\n",
                   (unsigned int)windowStartCounts,
                   (unsigned int)latestCounts,
                   (int)windowDeltaCounts,
                   (double)(windowElapsed_s * 1000.0f),
                   (double)rawVelocity_revPerSec,
                   (double)mech.velocity_revPerSec,
                   (double)velAvg_revPerSec,
                   (double)velMin_revPerSec,
                   (double)velMax_revPerSec,
                   (double)mech.position_rev,
                   (unsigned int)failCount);

            windowStartCounts = latestCounts;
            windowElapsed_s = 0.0f;
            printElapsed_ms = 0u;

            velMin_revPerSec = mech.velocity_revPerSec;
            velMax_revPerSec = mech.velocity_revPerSec;
            velSum_revPerSec = mech.velocity_revPerSec;
            velSampleCount = 1u;
            velWindowInitialized = true;

            ToggleLed();
        }

        k_msleep(UPDATE_PERIOD_MS);
        printElapsed_ms += UPDATE_PERIOD_MS;
    }

    return 0;
}

/*=============================================================================
 * MODE 2: ELECTRICAL ANGLE / SECTOR TEST
 *===========================================================================*/

static int RunElectricalSectorTest(void)
{
    printk("\n======================================================\n");
    printk("       TEST: V2 ELECTRICAL ANGLE / SECTOR CHECK       \n");
    printk("======================================================\n");

    if (!DevicesReady())
    {
        return -1;
    }

    EncoderInit();
    EncoderSetDirection(ENCODER_DIR_SIGN);
    EncoderSetElectricalOffset(0.0f);

    printk("[ELEC] Electrical offset forced to 0 for this structural check.\n");
    printk("[ELEC] Rotate slowly and verify pole-pair scaling and sector order.\n\n");

    uint32_t failCount = 0u;

    uint32_t lastCycle = k_cycle_get_32();
    bool haveDt = false;

    uint16_t windowStartCounts = 0u;
    uint16_t latestCounts = 0u;
    bool countWindowInitialized = false;

    float mechStart_rev = 0.0f;
    float elecStart_rad = 0.0f;
    uint8_t sectorStart = 1u;
    bool stateWindowInitialized = false;

    uint8_t prevSector = 1u;
    bool havePrevSector = false;

    int sectorSteps = 0;
    int sectorSkips = 0;

    uint32_t printElapsed_ms = 0u;

    while (1)
    {
        uint16_t rawCounts = 0u;

        if (!ReadMa732Counts(&rawCounts))
        {
            failCount++;

            printk("[ELEC][FAIL] SPI transfer failed | failures=%u\n",
                   (unsigned int)failCount);

            k_msleep(UPDATE_PERIOD_MS);
            continue;
        }

        uint32_t now = k_cycle_get_32();

        if (haveDt)
        {
            float dt_s = ComputeDtFromCycles(now, lastCycle);

            if (dt_s > 0.0f)
            {
                EncoderUpdate(rawCounts, dt_s);
            }
        }

        lastCycle = now;
        haveDt = true;

        EncoderMechanicalState_t mech = EncoderGetMechanicalState();
        EncoderElectricalState_t elec = EncoderGetElectricalState();

        uint8_t sector = ElectricalAngleToSector(elec.angle_rad);

        if (!countWindowInitialized)
        {
            windowStartCounts = rawCounts;
            latestCounts = rawCounts;
            countWindowInitialized = true;
        }
        else
        {
            latestCounts = rawCounts;
        }

        if (!stateWindowInitialized)
        {
            mechStart_rev = mech.position_rev;
            elecStart_rad = elec.angle_rad;
            sectorStart = sector;
            stateWindowInitialized = true;
        }

        if (!havePrevSector)
        {
            prevSector = sector;
            havePrevSector = true;
        }
        else
        {
            int8_t dSector = WrappedDeltaSector(sector, prevSector);

            if (dSector != 0)
            {
                sectorSteps++;

                if ((dSector > 1) || (dSector < -1))
                {
                    sectorSkips++;
                }
            }

            prevSector = sector;
        }

        if (printElapsed_ms >= PRINT_PERIOD_MS)
        {
            int16_t windowDeltaCounts =
                WrappedDeltaCounts(latestCounts, windowStartCounts);

            float mechDelta_rev =
                WrappedDeltaRev(mech.position_rev, mechStart_rev);

            float elecDelta_rad =
                WrappedDeltaRad(elec.angle_rad, elecStart_rad);

            float expectedElecDelta_rad =
                mechDelta_rev *
                (float)MOTOR_POLE_PAIRS *
                MATH_TWO_PI_F;

            expectedElecDelta_rad =
                WrappedDeltaRad(expectedElecDelta_rad, 0.0f);

            float elecError_rad =
                WrappedDeltaRad(elecDelta_rad, expectedElecDelta_rad);

            int8_t sectorDelta =
                WrappedDeltaSector(sector, sectorStart);

            printk("[ELEC] cnt:%5u->%5u | dCnt:%6d | "
                   "mech:% .6f->% .6f rev | "
                   "elec:% .6f->% .6f rad | "
                   "dElec:% .6f exp:% .6f err:% .6f | "
                   "sec:%u->%u d=%d | steps:%d skips:%d | fails:%u\n",
                   (unsigned int)windowStartCounts,
                   (unsigned int)latestCounts,
                   (int)windowDeltaCounts,
                   (double)mechStart_rev,
                   (double)mech.position_rev,
                   (double)elecStart_rad,
                   (double)elec.angle_rad,
                   (double)elecDelta_rad,
                   (double)expectedElecDelta_rad,
                   (double)elecError_rad,
                   (unsigned int)sectorStart,
                   (unsigned int)sector,
                   (int)sectorDelta,
                   sectorSteps,
                   sectorSkips,
                   (unsigned int)failCount);

            windowStartCounts = latestCounts;
            mechStart_rev = mech.position_rev;
            elecStart_rad = elec.angle_rad;
            sectorStart = sector;

            sectorSteps = 0;
            sectorSkips = 0;
            printElapsed_ms = 0u;

            ToggleLed();
        }

        k_msleep(UPDATE_PERIOD_MS);
        printElapsed_ms += UPDATE_PERIOD_MS;
    }

    return 0;
}

/*=============================================================================
 * MAIN
 *===========================================================================*/

int main(void)
{
    if (gpio_is_ready_dt(&g_led))
    {
        gpio_pin_configure_dt(&g_led, GPIO_OUTPUT_ACTIVE);
    }

    printk("\n======================================================\n");
    printk("           TEST: V2 ENCODER BRING-UP VALIDATION       \n");
    printk("======================================================\n");
    printk("[TEST] ENCODER_TEST_MODE=%d\n", (int)ENCODER_TEST_MODE);
    printk("[TEST] Direction convention: CCW from board view = positive.\n\n");

#if ENCODER_TEST_MODE == ENCODER_TEST_MODE_RAW_SPI
    return RunRawSpiTest();

#elif ENCODER_TEST_MODE == ENCODER_TEST_MODE_MECHANICAL
    return RunMechanicalEstimatorTest();

#elif ENCODER_TEST_MODE == ENCODER_TEST_MODE_ELECTRICAL
    return RunElectricalSectorTest();

#else
    printk("[TEST][FAIL] Invalid ENCODER_TEST_MODE.\n");
    return -1;
#endif
}