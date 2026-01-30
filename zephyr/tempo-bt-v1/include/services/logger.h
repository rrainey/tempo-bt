/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Logger Service
 * 
 * Manages logging sessions and coordinates aggregator with file writer
 */

#ifndef SERVICES_LOGGER_H
#define SERVICES_LOGGER_H

#include <stdint.h>
#include <stdbool.h>

/* Logger states */
typedef enum {
    LOGGER_STATE_IDLE,
    LOGGER_STATE_ARMED,
    LOGGER_STATE_LOGGING,
    LOGGER_STATE_JUMPED,      /* Active jump in progress (freefall/canopy) */
    LOGGER_STATE_POSTFLIGHT,
    LOGGER_STATE_ERROR
} logger_state_t;

/* Logger configuration */
typedef struct {
    /* Base path for log files */
    const char *base_path;
    
    /* Session naming */
    bool use_date_folders;      /* /logs/YYYYMMDD/... */
    bool use_uuid_names;        /* Use UUID for session folder */
    
    /* Rates and options */
    uint16_t imu_rate_hz;       /* IMU output rate */
    uint16_t env_rate_hz;       /* Environmental output rate */
    uint16_t gnss_rate_hz;      /* GNSS output rate */
    bool enable_quaternion;     /* Enable $PIM2 output */
    bool enable_magnetometer;   /* Enable $PMAG output */
    
    /* Auto-start/stop */
    bool auto_start_on_takeoff;
    bool auto_stop_on_landing;
} logger_config_t;

/**
 * @brief Initialize logger service
 * 
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int logger_init(const logger_config_t *config);

/**
 * @brief Start a new logging session, enter LOGGING state
 * 
 * @return 0 on success, negative error code on failure
 */
int logger_start(void);

/**
 * @brief Switch from LOGGING to JUMPED state
 * 
 * @return 0 on success, negative error code on failure
 */
int logger_jumped(void);


/**
 * @brief Stop current logging session, return to ARMED state
 * 
 * @return 0 on success, negative error code on failure
 */
int logger_stop(void);

/**
 * @brief Get current logger state
 * 
 * @return Current state
 */
logger_state_t logger_get_state(void);

/**
 * @brief Get current session info
 * 
 * @param session_id Output: Session ID (can be NULL)
 * @param start_time Output: Start time in microseconds (can be NULL)
 * @param path Output buffer for session path (can be NULL)
 * @param path_size Size of path buffer
 * @return 0 on success, -EINVAL if no active session
 */
int logger_get_session_info(uint32_t *session_id, uint64_t *start_time,
                            char *path, size_t path_size);

/**
 * @brief Transition to armed state
 * 
 * Prepares for logging but doesn't start yet
 * 
 * @return 0 on success, negative error code on failure
 */
int logger_arm(void);

/**
 * @brief Transition to idle state
 *
 * @return 0 on success, negative error code on failure
 */
int logger_disarm(void);

/**
 * @brief Logger executive function - manages automatic state transitions
 *
 * Call this function every 1 second from the main loop.
 * Handles automatic transitions based on barometric rate of climb:
 * - ARMED -> LOGGING on takeoff detection (climb > 200 ft/min)
 * - LOGGING -> JUMPED on freefall detection (descent > 1000 ft/min)
 * - JUMPED -> ARMED on landing detection (low activity for 60s)
 * - LOGGING -> ARMED on abort (low activity for 360s, no jump)
 */
void logger_executive(void);

#endif /* SERVICES_LOGGER_H */