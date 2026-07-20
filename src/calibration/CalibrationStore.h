#pragma once

#include <stdbool.h>

#include "Calibration.h"

/*=============================================================================
 * CalibrationStore.h
 *
 * RESPONSIBILITIES:
 * - Persistent storage of calibration data
 * - Flash initialization
 * - Calibration load/save/erase operations
 *
 * OUT OF SCOPE:
 * - Calibration algorithm
 * - Flash driver implementation details
 * - Runtime controller state
 *
 * Notes:
 * - Stores CalibrationData_t as a versioned persistent record.
 * - Integrity checking (CRC/version) is handled internally.
 *===========================================================================*/

/*=============================================================================
 * INITIALIZATION
 *===========================================================================*/

/*
 * Initializes the persistent calibration storage subsystem.
 * Safe to call once during startup before controller initialization.
 */
void CalibrationStore_Init(void);

/*=============================================================================
 * PERSISTENCE
 *===========================================================================*/

/*
 * Loads the most recent valid calibration record.
 *
 * Returns:
 *     true  - valid calibration successfully loaded.
 *     false - no valid calibration present.
 */
bool CalibrationStore_Load(
    CalibrationData_t *out);

/*
 * Saves the supplied calibration snapshot.
 *
 * Returns:
 *     true  - save completed successfully.
 *     false - save failed.
 */
bool CalibrationStore_Save(
    const CalibrationData_t *data);

/*
 * Erases all stored calibration records.
 *
 * Returns:
 *     true  - storage successfully cleared.
 *     false - erase failed.
 */
bool CalibrationStore_Clear(void);