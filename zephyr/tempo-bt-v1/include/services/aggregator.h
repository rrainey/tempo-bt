/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Aggregator Service
 * 
 * Merges sensor data streams and formats them into log sentences
 */

#ifndef SERVICES_AGGREGATOR_H
#define SERVICES_AGGREGATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "app/log_format.h"

/* Aggregator configuration */
typedef struct {
    /* Output rates in Hz */
    uint16_t imu_output_rate;      /* Target rate for $PIMU sentences (50 Hz) */
    uint16_t env_output_rate;      /* Target rate for $PENV sentences (4 Hz) */
    uint16_t gnss_output_rate;     /* Target rate for GNSS (1 Hz) */
    uint16_t mag_output_rate;      /* Target rate for $PMAG sentences (0 = disabled) */
    
    /* Options */
    bool enable_quaternion;         /* Output $PIM2 after each $PIMU */
    bool enable_magnetometer;       /* Enable magnetometer sentences */
    
    /* Session info */
    uint32_t session_id;           /* Unique session identifier */
    uint64_t session_start_us;     /* Session start time in microseconds */
} aggregator_config_t;

/* Callback for formatted log lines */
typedef void (*aggregator_output_callback_t)(const char *line, size_t len);

/**
 * @brief Initialize the aggregator service
 * 
 * @return 0 on success, negative error code on failure
 */
int aggregator_init(void);

/**
 * @brief Configure the aggregator
 * 
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int aggregator_configure(const aggregator_config_t *config);

/**
 * @brief Register callback for formatted output
 * 
 * @param callback Function to call with formatted log lines
 */
void aggregator_register_output_callback(aggregator_output_callback_t callback);

/**
 * @brief Start aggregation
 * 
 * Begins pulling data from sensors and formatting output
 * 
 * @return 0 on success, negative error code on failure
 */
int aggregator_start(void);

/**
 * @brief Stop aggregation
 * 
 * @return 0 on success, negative error code on failure
 */
int aggregator_stop(void);

/**
 * @brief Write version sentence
 * 
 * Outputs a $PVER sentence with version information
 */
void aggregator_write_version(void);

/**
 * @brief Write session file config sentence
 * 
 * Outputs a $PSFC sentence with session configuration
 */
void aggregator_write_session_config(void);

/**
 * @brief Write state change sentence
 * 
 * @param old_state Previous state name
 * @param new_state New state name
 * @param trigger What triggered the change
 */
void aggregator_write_state_change(const char *old_state, const char *new_state, 
                                   const char *trigger);

/**
 * @brief Update GNSS output rate
 * 
 * Call this when switching between logging modes to adjust GNSS rate
 * 
 * @param rate_hz New GNSS rate (1 Hz or 10 Hz)
 * @return 0 on success, negative error code on failure
 */
int aggregator_set_gnss_rate(uint16_t rate_hz);

/**
 * @brief Get current aggregator statistics
 * 
 * @param imu_count Output: Number of IMU samples processed
 * @param env_count Output: Number of environmental samples processed
 * @param gnss_count Output: Number of GNSS fixes processed
 * @param dropped_count Output: Number of samples dropped
 */
void aggregator_get_stats(uint32_t *imu_count, uint32_t *env_count,
                          uint32_t *gnss_count, uint32_t *dropped_count);

/**
 * @brief Write header sentences for new session
 * 
 * Writes $PVER and $PSFC sentences at the start of a log file
 */
void aggregator_write_session_header(void);

/**
 * @brief Set session start time
 * 
 * @param start_us Session start time in microseconds
 */
void aggregator_set_session_start_time(uint64_t start_us);

/**
 * @brief Set session ID
 * 
 * @param id Unique session identifier
 */
void aggregator_set_session_id(uint32_t id);

/* Add this declaration to aggregator.h, replacing the old aggregator_write_session_config */

/**
 * @brief Write surface altitude sentence
 * 
 * Outputs a $PSFC sentence with the estimated MSL altitude of the
 * departure airfield. This value is obtained from barometer readings
 * taken 10-15 minutes before logging starts.
 */
void aggregator_write_surface_altitude(void);

/* Optional: If you need session info elsewhere, add this */
/**
 * @brief Write session information sentence (custom)
 * 
 * Outputs a custom sentence with session configuration details.
 * Note: This is not part of the standard log format.
 */
void aggregator_write_session_info(void);

#endif /* SERVICES_AGGREGATOR_H */