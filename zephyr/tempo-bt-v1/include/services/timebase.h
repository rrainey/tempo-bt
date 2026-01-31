/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Time Base Service
 */

#ifndef SERVICES_TIMEBASE_H
#define SERVICES_TIMEBASE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Time correlation structure
 * 
 * Maps monotonic time to UTC time when GNSS fix is available
 */
typedef struct {
    uint64_t mono_us;      /* Monotonic timestamp in microseconds */
    uint64_t utc_ms;       /* UTC timestamp in milliseconds since epoch */
    bool valid;            /* True if correlation is valid */
    uint32_t accuracy_ms;  /* Time accuracy estimate in milliseconds */
} time_correlation_t;

/**
 * @brief Initialize the timebase service
 * 
 * Sets up the high-resolution timer for monotonic timestamps
 * 
 * @return 0 on success, negative error code on failure
 */
int timebase_init(void);

/**
 * @brief Get current monotonic time in microseconds
 * 
 * This is the primary timestamp source for all sensor data.
 * Time is monotonic since boot and not affected by time adjustments.
 * 
 * @return Current time in microseconds since boot
 */
uint64_t time_now_us(void);

/**
 * @brief Get current monotonic time in milliseconds
 * 
 * Convenience function for millisecond timestamps
 * 
 * @return Current time in milliseconds since boot
 */
static inline uint32_t time_now_ms(void)
{
    return (uint32_t)(time_now_us() / 1000ULL);
}

/**
 * @brief Update time correlation from GNSS
 * 
 * Called when GNSS provides accurate time information
 * 
 * @param utc_ms UTC time in milliseconds since epoch
 * @param accuracy_ms Estimated accuracy in milliseconds
 */
void timebase_update_correlation(uint64_t utc_ms, uint32_t accuracy_ms);

/**
 * @brief Convert monotonic time to UTC
 * 
 * @param mono_us Monotonic timestamp in microseconds
 * @param utc_ms Output: UTC time in milliseconds (if available)
 * @return true if conversion successful, false if no valid correlation
 */
bool timebase_mono_to_utc(uint64_t mono_us, uint64_t *utc_ms);

/**
 * @brief Get current time correlation
 * 
 * @param corr Output: Current correlation data
 * @return true if correlation is valid, false otherwise
 */
bool timebase_get_correlation(time_correlation_t *corr);

/**
 * @brief PPS pulse data structure
 *
 * Contains the current PPS pulse count and the system timestamp
 * at the time of the query.
 */
typedef struct {
    uint32_t pulse_count;      /* Number of PPS pulses received since init */
    uint64_t timestamp_us;     /* System time when this data was captured */
} timebase_pps_data_t;

/**
 * @brief Initialize PPS interrupt handling
 *
 * Configures GPIO P0.27 for rising edge interrupt from GNSS time pulse.
 * Also configures the red LED (led1) for visual feedback.
 *
 * @return 0 on success, negative error code on failure
 */
int timebase_pps_init(void);

/**
 * @brief Get current PPS data
 *
 * Returns the current pulse count and system timestamp atomically.
 * The pulse count increments on each rising edge of the GNSS time pulse.
 *
 * @param data Output: PPS pulse count and timestamp
 */
void timebase_get_pps_data(timebase_pps_data_t *data);

/**
 * @brief Check if PPS pulses are being received
 *
 * @return true if PPS pulses have been received, false otherwise
 */
bool timebase_pps_active(void);

/**
 * @brief Get UTC time string placeholder
 *
 * Returns "unknown" until GNSS provides valid time
 * Used by aggregator for $PTH sentences when UTC is not available
 *
 * @return "unknown" string
 */
const char *timebase_utc_string_placeholder(void);

/**
 * @brief Get current UTC time as ISO 8601 string
 *
 * Formats current UTC time as "YYYY-MM-DDTHH:MM:SS.dZ" with 0.1s resolution.
 * Requires valid GNSS time correlation.
 *
 * @param buf Output buffer for the ISO 8601 string (minimum 25 bytes)
 * @param buf_size Size of the output buffer
 * @return 0 on success, -EAGAIN if no valid UTC correlation
 */
int timebase_get_utc_iso8601(char *buf, size_t buf_size);

/*
 * Test Alarm Interface
 *
 * Used by mcumgr TEST_LOGGING command to synchronize logging start
 * across multiple Tempo-BT devices using the GNSS PPS signal.
 * State and parameters are stored in mcumgr_custom.c
 */

/* Test alarm states - must match enum in mcumgr_custom.c */
#define TEST_ALARM_IDLE           0  /* No test alarm active */
#define TEST_ALARM_WAITING_START  1  /* Waiting for start time to trigger LOGGING */
#define TEST_ALARM_WAITING_JUMP   2  /* In LOGGING, counting down to JUMPED */

/* Accessor functions implemented in mcumgr_custom.c */
int test_alarm_get_state(void);
uint64_t test_alarm_get_target_utc_ms(void);
uint32_t test_alarm_get_jump_delay(void);
uint32_t test_alarm_get_jump_countdown(void);
void test_alarm_set_state(int state);
void test_alarm_start_jump_countdown(void);
uint32_t test_alarm_decrement_countdown(void);

#endif /* SERVICES_TIMEBASE_H */