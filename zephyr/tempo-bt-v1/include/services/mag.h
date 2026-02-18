/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - Magnetometer Service Interface
 *
 * Service layer for MMC5983MA magnetometer with calibration support
 * and AHRS-compatible output format.
 */

#ifndef SERVICES_MAG_H
#define SERVICES_MAG_H

#include <stdint.h>
#include <stdbool.h>
#include "Fusion.h"

/**
 * @brief Magnetometer sample data
 */
typedef struct {
    uint64_t timestamp_us;      /* Microseconds since boot */

    /* Calibrated field values in Gauss */
    float mag_x;
    float mag_y;
    float mag_z;

    /* Normalized unit vector for AHRS (magnitude ~1.0) */
    float norm_x;
    float norm_y;
    float norm_z;

    /* Field magnitude in Gauss */
    float magnitude;

    /* Temperature from sensor (Celsius) */
    float temperature;

    /* Calibration applied flag */
    bool calibrated;
} mag_sample_t;

/**
 * @brief Magnetometer configuration
 */
typedef struct {
    uint16_t odr_hz;            /* Output data rate (0=single-shot) */
    bool auto_set_reset;        /* Enable automatic SET/RESET */
    uint8_t bandwidth;          /* Bandwidth setting (0-3) */
} mag_config_t;

/**
 * @brief Calibration data
 */
typedef struct {
    /* Hard-iron offsets (bias) in raw counts */
    int32_t offset_x;
    int32_t offset_y;
    int32_t offset_z;

    /* Soft-iron scale factors (Q1.15, 32768 = 1.0) */
    uint16_t scale_x;
    uint16_t scale_y;
    uint16_t scale_z;

    /* Validity flag */
    bool valid;
} mag_calibration_t;

/**
 * @brief Initialize the magnetometer service
 *
 * Initializes the MMC5983MA sensor and loads any stored calibration.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_init(void);

/**
 * @brief Configure magnetometer parameters
 *
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int mag_configure(const mag_config_t *config);

/**
 * @brief Get current configuration
 *
 * @param config Output configuration structure
 * @return 0 on success, negative error code on failure
 */
int mag_get_config(mag_config_t *config);

/**
 * @brief Take a single magnetometer reading
 *
 * Triggers a measurement and returns the calibrated result.
 * The sample includes normalized values suitable for AHRS input.
 *
 * @param sample Output sample structure
 * @return 0 on success, negative error code on failure
 */
int mag_read(mag_sample_t *sample);

/**
 * @brief Get magnetometer reading as FusionVector
 *
 * Convenience function that returns the magnetometer reading
 * in the format expected by Fusion AHRS algorithms.
 * The vector is normalized (unit length).
 *
 * @param vec Output FusionVector (normalized)
 * @return 0 on success, negative error code on failure
 */
int mag_read_fusion(FusionVector *vec);

/**
 * @brief Perform SET operation
 *
 * Applies the SET magnetization to the sensor bridge.
 * Useful for manual offset compensation.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_set(void);

/**
 * @brief Perform RESET operation
 *
 * Applies the RESET magnetization to the sensor bridge.
 * Useful for manual offset compensation.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_reset(void);

/**
 * @brief Start calibration sample collection
 *
 * Begins collecting samples for hard-iron calibration.
 * Rotate the device through all orientations while collecting.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_start(void);

/**
 * @brief Add current reading to calibration dataset
 *
 * Call this repeatedly while rotating the device.
 * The function tracks min/max values for each axis.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_add_sample(void);

/**
 * @brief Get number of calibration samples collected
 *
 * @return Sample count, or negative error code on failure
 */
int mag_cal_get_sample_count(void);

/**
 * @brief Compute and apply calibration
 *
 * Calculates hard-iron offsets from collected samples
 * and applies the calibration.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_compute(void);

/**
 * @brief Get current calibration data
 *
 * @param cal Output calibration structure
 * @return 0 on success, negative error code on failure
 */
int mag_cal_get(mag_calibration_t *cal);

/**
 * @brief Set calibration data
 *
 * @param cal Calibration data to apply
 * @return 0 on success, negative error code on failure
 */
int mag_cal_set(const mag_calibration_t *cal);

/**
 * @brief Save calibration to persistent storage
 *
 * Stores the current calibration to NVS settings.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_save(void);

/**
 * @brief Load calibration from persistent storage
 *
 * Loads calibration from NVS settings if available.
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_load(void);

/**
 * @brief Clear calibration
 *
 * Resets calibration to defaults (no correction).
 *
 * @return 0 on success, negative error code on failure
 */
int mag_cal_clear(void);

/**
 * @brief Check if magnetometer is initialized and ready
 *
 * @return true if ready, false otherwise
 */
bool mag_is_ready(void);

/**
 * @brief Check if calibration is valid
 *
 * @return true if valid calibration is applied
 */
bool mag_is_calibrated(void);

#endif /* SERVICES_MAG_H */
