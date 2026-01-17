/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - Orientation Tracking Service
 *
 * Uses the Fusion library by xioTechnologies (Sebastian Madgwick)
 * to maintain orientation quaternion from IMU data.
 *
 * The orientation service owns a thread that polls the IMU FIFO
 * and feeds samples to the Fusion AHRS algorithm.
 */

#ifndef SERVICES_ORIENTATION_H
#define SERVICES_ORIENTATION_H

#include <stdint.h>
#include <stdbool.h>

/* Orientation quaternion structure */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} orientation_quaternion_t;

/* Euler angles structure (for debugging/visualization) */
typedef struct {
    float roll;   /* degrees */
    float pitch;  /* degrees */
    float yaw;    /* degrees */
} orientation_euler_t;

/* Orientation service configuration */
typedef struct {
    /* Algorithm gain (typical: 0.5 for MARG, 2.5 for IMU-only) */
    float gain;

    /* Sample period in seconds (e.g., 0.005 for 200Hz) */
    float sample_period;

    /* Enable magnetometer integration (false for V1) */
    bool use_magnetometer;

    /* Acceleration rejection threshold (m/s²) */
    float acceleration_rejection;

    /* Magnetic rejection threshold (uT) */
    float magnetic_rejection;

    /* Recovery trigger period (seconds) */
    float recovery_trigger_period;
} orientation_config_t;

/**
 * @brief Initialize the orientation tracking service
 *
 * Sets up the Fusion AHRS algorithm but does not start processing.
 * Call orientation_start() to begin IMU polling.
 *
 * @param config Configuration parameters (NULL for defaults)
 * @return 0 on success, negative error code on failure
 */
int orientation_init(const orientation_config_t *config);

/**
 * @brief Start the orientation tracking thread
 *
 * Starts IMU FIFO and begins polling for samples.
 *
 * @return 0 on success, negative error code on failure
 */
int orientation_start(void);

/**
 * @brief Stop the orientation tracking thread
 *
 * Stops IMU FIFO and terminates the polling thread.
 *
 * @return 0 on success, negative error code on failure
 */
int orientation_stop(void);

/**
 * @brief Get current orientation quaternion
 *
 * @param quat Output quaternion structure
 * @return 0 on success, negative error code on failure
 */
int orientation_get_quaternion(orientation_quaternion_t *quat);

/**
 * @brief Get current orientation as Euler angles
 *
 * @param euler Output Euler angles structure
 * @return 0 on success, negative error code on failure
 */
int orientation_get_euler(orientation_euler_t *euler);

/**
 * @brief Reset orientation to identity
 *
 * Sets quaternion to [1, 0, 0, 0]
 *
 * @return 0 on success, negative error code on failure
 */
int orientation_reset(void);

/**
 * @brief Set initial heading reference
 *
 * Aligns current heading to specified angle
 *
 * @param heading_deg Heading in degrees (0 = North)
 * @return 0 on success, negative error code on failure
 */
int orientation_set_heading(float heading_deg);

/**
 * @brief Get algorithm settings
 *
 * @param config Output configuration structure
 * @return 0 on success, negative error code on failure
 */
int orientation_get_config(orientation_config_t *config);

/**
 * @brief Update algorithm settings
 *
 * @param config New configuration parameters
 * @return 0 on success, negative error code on failure
 */
int orientation_set_config(const orientation_config_t *config);

/**
 * @brief Check if orientation is ready (AHRS initialized)
 *
 * @return true if AHRS has completed initialization
 */
bool orientation_is_ready(void);

/**
 * @brief Get total number of samples processed
 *
 * @return Sample count
 */
uint32_t orientation_get_sample_count(void);

#endif /* SERVICES_ORIENTATION_H */
