/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 *
 * Extended public API for MEMSIC MMC5983MA magnetometer
 *
 * This header provides calibration and conversion functions beyond
 * the standard Zephyr sensor API.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Calibration method identifiers
 */
#define MMC5983MA_CAL_NONE           0x00  /* No calibration */
#define MMC5983MA_CAL_FACTORY        0x01  /* Factory/devicetree */
#define MMC5983MA_CAL_MINMAX         0x02  /* Simple min/max */
#define MMC5983MA_CAL_ELLIPSOID      0x03  /* Ellipsoid fit */

/*
 * Extended sensor attributes for MMC5983MA
 */
enum sensor_attribute_mmc5983ma {
	/** Perform SET operation (write-only, val ignored) */
	SENSOR_ATTR_MMC5983MA_SET = SENSOR_ATTR_PRIV_START,

	/** Perform RESET operation (write-only, val ignored) */
	SENSOR_ATTR_MMC5983MA_RESET,

	/** Enable/disable automatic SET/RESET */
	SENSOR_ATTR_MMC5983MA_AUTO_SR,

	/** Start calibration collection (write-only) */
	SENSOR_ATTR_MMC5983MA_CAL_START,

	/** Compute calibration from collected data (write-only) */
	SENSOR_ATTR_MMC5983MA_CAL_COMPUTE,
};

/*
 * Calibration data structure
 */
struct mmc5983ma_cal_data {
	/* Hard-iron offsets in raw 18-bit counts */
	int32_t offset_x;
	int32_t offset_y;
	int32_t offset_z;

	/* Scale factors (Q1.15 fixed-point: 32768 = 1.0) */
	uint16_t scale_x;
	uint16_t scale_y;
	uint16_t scale_z;

	/* Calibration validity and method */
	uint8_t valid;
	uint8_t method;
};

/*
 * Magnetometer sample in formats suitable for AHRS algorithms
 */
struct mmc5983ma_sample {
	/* Calibrated values in Gauss */
	float x_gauss;
	float y_gauss;
	float z_gauss;

	/* Normalized vector (magnitude = 1.0, for AHRS input) */
	float x_norm;
	float y_norm;
	float z_norm;

	/* Raw calibrated counts (18-bit, signed) */
	int32_t x_raw;
	int32_t y_raw;
	int32_t z_raw;

	/* Temperature in Celsius */
	float temperature;
};

/**
 * @brief Get magnetometer sample with multiple format conversions
 *
 * Fetches the latest sample and converts to multiple useful formats:
 * - Gauss (for display/logging)
 * - Normalized unit vector (for AHRS magnetometer input)
 * - Raw calibrated counts (for custom processing)
 *
 * The normalized values can be passed directly to Fusion AHRS:
 *   FusionVector mag = {sample.x_norm, sample.y_norm, sample.z_norm};
 *
 * @param dev MMC5983MA device
 * @param sample Output sample structure
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_get_sample(const struct device *dev, struct mmc5983ma_sample *sample);

/**
 * @brief Get current calibration data
 *
 * @param dev MMC5983MA device
 * @param cal Output calibration structure
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_get_calibration(const struct device *dev, struct mmc5983ma_cal_data *cal);

/**
 * @brief Set calibration data
 *
 * @param dev MMC5983MA device
 * @param cal Calibration data to apply
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_set_calibration(const struct device *dev, const struct mmc5983ma_cal_data *cal);

/**
 * @brief Start calibration sample collection
 *
 * Begins collecting min/max samples for calibration.
 * Call mmc5983ma_cal_add_sample() after each sensor read.
 *
 * @param dev MMC5983MA device
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_cal_start(const struct device *dev);

/**
 * @brief Add current reading to calibration collection
 *
 * Call this after each sensor_sample_fetch() during calibration.
 * The device should be rotated through all orientations.
 *
 * @param dev MMC5983MA device
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_cal_add_sample(const struct device *dev);

/**
 * @brief Compute calibration from collected samples
 *
 * Calculates hard-iron offsets using the min/max method.
 * The calculated calibration is automatically applied.
 *
 * @param dev MMC5983MA device
 * @param method Calibration method (MMC5983MA_CAL_MINMAX recommended)
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_cal_compute(const struct device *dev, uint8_t method);

/**
 * @brief Get number of calibration samples collected
 *
 * @param dev MMC5983MA device
 * @return Number of samples collected, or negative errno on failure
 */
int mmc5983ma_cal_get_sample_count(const struct device *dev);

/**
 * @brief Perform SET operation
 *
 * Magnetizes the sensor bridge in the positive direction.
 * Used for offset compensation.
 *
 * @param dev MMC5983MA device
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_do_set(const struct device *dev);

/**
 * @brief Perform RESET operation
 *
 * Magnetizes the sensor bridge in the negative direction.
 * Used for offset compensation.
 *
 * @param dev MMC5983MA device
 * @return 0 on success, negative errno on failure
 */
int mmc5983ma_do_reset(const struct device *dev);

/**
 * @brief Convert raw counts to Gauss
 *
 * @param raw_counts Raw 18-bit signed count value
 * @return Field strength in Gauss
 */
static inline float mmc5983ma_counts_to_gauss(int32_t raw_counts)
{
	/* 16384 counts per Gauss at 18-bit resolution */
	return (float)raw_counts / 16384.0f;
}

/**
 * @brief Convert Gauss to raw counts
 *
 * @param gauss Field strength in Gauss
 * @return Raw 18-bit count value
 */
static inline int32_t mmc5983ma_gauss_to_counts(float gauss)
{
	return (int32_t)(gauss * 16384.0f);
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_MMC5983MA_H_ */
