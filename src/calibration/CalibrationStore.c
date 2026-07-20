#include "CalibrationStore.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/printk.h>

/*=============================================================================
 * CalibrationStore.c
 *
 * RESPONSIBILITIES:
 * - Zephyr NVS initialization
 * - Calibration record serialization
 * - Magic/version validation
 * - CRC integrity checking
 * - Calibration load/save/clear operations
 *
 * OUT OF SCOPE:
 * - Calibration algorithm
 * - Calibration physical sanity checks
 * - Controller state
 * - FOC or commutation behavior
 *
 * Role:
 * - Persists the most recent passing CalibrationData_t snapshot.
 * - Verifies storage integrity only.
 * - Leaves physical reasonableness checks to Calibration.c.
 *===========================================================================*/

/*=============================================================================
 * STORAGE FORMAT
 *===========================================================================*/

#define CAL_STORE_ID              1u
#define CAL_STORE_MAGIC           0x43414C31u  /* "CAL1" */
#define CAL_STORE_VERSION         1u

typedef struct
{
    uint32_t magic;
    uint32_t version;
    CalibrationData_t data;
    uint32_t crc32;
} CalibrationStoreRecord_t;

/*=============================================================================
 * MODULE STATE
 *===========================================================================*/

static struct nvs_fs s_nvs;
static bool s_ready = false;

/*=============================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

static uint32_t CalibrationStore_ComputeCrc(
    const CalibrationStoreRecord_t *record)
{
    if (record == NULL)
    {
        return 0u;
    }

    return crc32_ieee(
        (const uint8_t *)record,
        offsetof(CalibrationStoreRecord_t, crc32)
    );
}

static bool CalibrationStore_IsRecordStructurallyValid(
    const CalibrationStoreRecord_t *record)
{
    if (record == NULL)
    {
        return false;
    }

    if (record->magic != CAL_STORE_MAGIC)
    {
        printk("[CAL_STORE] rejected: bad magic\n");
        return false;
    }

    if (record->version != CAL_STORE_VERSION)
    {
        printk("[CAL_STORE] rejected: version mismatch\n");
        return false;
    }

    uint32_t expectedCrc = CalibrationStore_ComputeCrc(record);
    if (record->crc32 != expectedCrc)
    {
        printk("[CAL_STORE] rejected: CRC mismatch\n");
        return false;
    }

    if (!record->data.isValid ||
        record->data.result != CAL_RESULT_PASSED)
    {
        printk("[CAL_STORE] rejected: calibration not valid/passed\n");
        return false;
    }

    return true;
}

/*=============================================================================
 * PUBLIC API
 *===========================================================================*/

void CalibrationStore_Init(void)
{
    const struct flash_area *fa = NULL;

    s_ready = false;
    memset(&s_nvs, 0, sizeof(s_nvs));

    int err = flash_area_open(FIXED_PARTITION_ID(storage_partition), &fa);
    if (err != 0)
    {
        printk("[CAL_STORE] flash_area_open failed: %d\n", err);
        return;
    }

    s_nvs.flash_device = fa->fa_dev;
    s_nvs.offset = fa->fa_off;

    /*
     * MCXN947 internal flash sector size is 8 KB for this partition layout.
     * The storage partition should be an integer multiple of this value.
     */
    s_nvs.sector_size = 8192u;
    s_nvs.sector_count = fa->fa_size / s_nvs.sector_size;

    flash_area_close(fa);

    if ((s_nvs.flash_device == NULL) ||
        (s_nvs.sector_size == 0u) ||
        (s_nvs.sector_count < 2u))
    {
        printk("[CAL_STORE] invalid NVS geometry\n");
        return;
    }

    err = nvs_mount(&s_nvs);
    if (err != 0)
    {
        printk("[CAL_STORE] nvs_mount failed: %d\n", err);
        return;
    }

    s_ready = true;

    printk("[CAL_STORE] ready | sectors=%u sector_size=%u\n",
           (unsigned int)s_nvs.sector_count,
           (unsigned int)s_nvs.sector_size);
}

bool CalibrationStore_Load(CalibrationData_t *out)
{
    if (!s_ready || out == NULL)
    {
        return false;
    }

    CalibrationStoreRecord_t record;
    memset(&record, 0, sizeof(record));

    int rc = nvs_read(&s_nvs, CAL_STORE_ID, &record, sizeof(record));

    if (rc == -ENOENT)
    {
        printk("[CAL_STORE] no stored calibration\n");
        return false;
    }

    if (rc != (int)sizeof(record))
    {
        printk("[CAL_STORE] read failed or wrong size: %d\n", rc);
        return false;
    }

    if (!CalibrationStore_IsRecordStructurallyValid(&record))
    {
        return false;
    }

    *out = record.data;
    return true;
}

bool CalibrationStore_Save(const CalibrationData_t *data)
{
    if (!s_ready || data == NULL)
    {
        return false;
    }

    if (!data->isValid || data->result != CAL_RESULT_PASSED)
    {
        printk("[CAL_STORE] refusing to save invalid calibration\n");
        return false;
    }

    CalibrationStoreRecord_t record;
    memset(&record, 0, sizeof(record));

    record.magic = CAL_STORE_MAGIC;
    record.version = CAL_STORE_VERSION;
    record.data = *data;
    record.crc32 = CalibrationStore_ComputeCrc(&record);

    int rc = nvs_write(&s_nvs, CAL_STORE_ID, &record, sizeof(record));

    if (rc < 0)
    {
        printk("[CAL_STORE] save failed: %d\n", rc);
        return false;
    }

    printk("[CAL_STORE] saved calibration\n");
    return true;
}

bool CalibrationStore_Clear(void)
{
    if (!s_ready)
    {
        return false;
    }

    int rc = nvs_delete(&s_nvs, CAL_STORE_ID);

    if ((rc == 0) || (rc == -ENOENT))
    {
        printk("[CAL_STORE] cleared calibration\n");
        return true;
    }

    printk("[CAL_STORE] clear failed: %d\n", rc);
    return false;
}