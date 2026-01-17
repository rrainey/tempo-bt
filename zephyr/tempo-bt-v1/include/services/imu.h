/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - IMU Service Interface
 *
 * Simple FIFO-based interface for ICM42688 IMU.
 * The orientation service polls the FIFO directly.
 */

#ifndef SERVICES_IMU_H
#define SERVICES_IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint64_t timestamp_us;
    float accel_x;          /* m/s^2 */
    float accel_y;          /* m/s^2 */
    float accel_z;          /* m/s^2 */
    float gyro_x;           /* rad/s */
    float gyro_y;           /* rad/s */
    float gyro_z;           /* rad/s */
    float temperature;      /* Celsius */
} imu_sample_t;

typedef struct {
    uint16_t odr_hz;        /* Output data rate (both accel and gyro) */
    uint8_t accel_range_g;  /* Accelerometer full-scale range in g */
    uint16_t gyro_range_dps; /* Gyroscope full-scale range in deg/s */
} imu_config_t;

/**
 * @brief Initialize the IMU device
 *
 * Resets the device and verifies communication.
 * Does not start sampling - call imu_fifo_start() for that.
 *
 * @return 0 on success, negative error code on failure
 */
int imu_init(void);

/**
 * @brief Configure IMU parameters
 *
 * Sets ODR, accelerometer range, and gyroscope range.
 * Can be called before or after imu_fifo_start().
 *
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int imu_configure(const imu_config_t *config);

/**
 * @brief Start FIFO-based sampling
 *
 * Enables the sensor FIFO in streaming mode.
 * Samples accumulate in hardware FIFO until read.
 *
 * @return 0 on success, negative error code on failure
 */
int imu_fifo_start(void);

/**
 * @brief Stop FIFO-based sampling
 *
 * Disables the sensor FIFO and flushes any pending data.
 *
 * @return 0 on success, negative error code on failure
 */
int imu_fifo_stop(void);

/**
 * @brief Read available samples from FIFO
 *
 * Drains all available samples from the hardware FIFO up to max_count.
 * Non-blocking - returns immediately with available samples.
 *
 * @param samples Array to store samples
 * @param max_count Maximum number of samples to read
 * @return Number of samples read (0 if none available), negative on error
 */
int imu_fifo_read(imu_sample_t *samples, size_t max_count);

/**
 * @brief Get current configuration
 *
 * @param config Pointer to store current configuration
 * @return 0 on success, negative error code on failure
 */
int imu_get_config(imu_config_t *config);

/**
 * @brief Get current time in microseconds
 *
 * @return Current time in microseconds since boot
 */
uint64_t time_now_us(void);

#endif /* SERVICES_IMU_H */
