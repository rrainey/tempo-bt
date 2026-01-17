/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Aggregator Service Implementation
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "generated/app_version.h"

#include "services/aggregator.h"
#include "services/timebase.h"
#include "services/imu.h"
#include "services/baro.h"
#include "services/gnss.h"
#include "services/orientation.h"
#include "services/file_writer.h"
#include "app/log_format.h"

LOG_MODULE_REGISTER(aggregator, LOG_LEVEL_INF);

/* Track NMEA arrival time for PTH generation */
static uint32_t last_nmea_arrival_ms = 0;

/* Ring buffer sizes */
#define BARO_RING_SIZE      32   /* 32 samples @ 50Hz = 640ms buffer */
#define GNSS_RING_SIZE      32   /* 32 fixes @ 10Hz = 3.2s buffer (increased for 10Hz) */

/* Output timing */
#define IMU_OUTPUT_PERIOD_MS    20   /* 50 Hz */
#define ENV_OUTPUT_PERIOD_MS    250  /* 4 Hz */

char gps_date_string[14];

/* Ring buffer structures */
typedef struct {
    baro_sample_t samples[BARO_RING_SIZE];
    uint32_t head;
    uint32_t tail;
    struct k_spinlock lock;
} baro_ring_t;

typedef struct {
    gnss_fix_t samples[GNSS_RING_SIZE];
    uint32_t head;
    uint32_t tail;
    struct k_spinlock lock;
} gnss_ring_t;

/* Static data */
static baro_ring_t baro_ring;
static gnss_ring_t gnss_ring;

static aggregator_config_t config = {
    .imu_output_rate = 50,
    .env_output_rate = 4,
    .gnss_output_rate = 1,
    .mag_output_rate = 0,
    .enable_quaternion = true,
    .enable_magnetometer = false,
    .session_id = 0,
    .session_start_us = 0
};

static aggregator_output_callback_t output_callback = NULL;

/* Statistics */
static struct {
    uint32_t env_count;
    uint32_t gnss_count;
    uint32_t dropped_count;
} stats;

/* Output thread */
K_THREAD_STACK_DEFINE(aggregator_stack, 4096);
static struct k_thread aggregator_thread;
static k_tid_t aggregator_tid = NULL;
static volatile bool running = false;

/* Work items for timed output */
static struct k_work_delayable imu_output_work;
static struct k_work_delayable env_output_work;

/* Latest samples for interpolation */
static baro_sample_t last_baro_sample;
static bool have_baro_sample = false;

/* Helper: Ring buffer push */
#define RING_PUSH(ring, item) do { \
    k_spinlock_key_t key = k_spin_lock(&(ring).lock); \
    uint32_t next_head = ((ring).head + 1) % ARRAY_SIZE((ring).samples); \
    if (next_head != (ring).tail) { \
        (ring).samples[(ring).head] = (item); \
        (ring).head = next_head; \
    } else { \
        stats.dropped_count++; \
    } \
    k_spin_unlock(&(ring).lock, key); \
} while(0)

/* Helper: Ring buffer pop */
#define RING_POP(ring, item, result) do { \
    k_spinlock_key_t key = k_spin_lock(&(ring).lock); \
    if ((ring).tail != (ring).head) { \
        *(item) = (ring).samples[(ring).tail]; \
        (ring).tail = ((ring).tail + 1) % ARRAY_SIZE((ring).samples); \
        result = true; \
    } else { \
        result = false; \
    } \
    k_spin_unlock(&(ring).lock, key); \
} while(0)

/* NMEA checksum calculation */
uint8_t log_calc_checksum(const char *sentence, size_t len)
{
    uint8_t checksum = 0;
    
    for (size_t i = 0; i < len; i++) {
        checksum ^= (uint8_t)sentence[i];
    }
    
    return checksum;
}

/* Format sentence with checksum */
int log_format_sentence(char *buf, size_t buf_size, const char *fmt, ...)
{
    va_list args;
    int len;
    uint8_t checksum;
    
    /* Format the main sentence */
    va_start(args, fmt);
    len = vsnprintf(buf, buf_size - 5, fmt, args);  /* Leave room for *HH\r\n */
    va_end(args);
    
    if (len < 0 || len >= buf_size - 5) {
        return -1;
    }
    
    /* Calculate checksum (skip the $) */
    checksum = log_calc_checksum(buf + 1, len - 1);
    
    /* Append checksum and CRLF */
    len += snprintf(buf + len, buf_size - len, "*%02X\r\n", checksum);
    
    return len;
}

/* Get relative timestamp */
uint32_t log_get_timestamp_ms(uint64_t session_start_us)
{
    uint64_t now_us = time_now_us();
    uint64_t elapsed_us = now_us - session_start_us;
    return (uint32_t)(elapsed_us / 1000);
}

/* Sensor callbacks */
static void baro_data_callback(const baro_sample_t *sample)
{
    RING_PUSH(baro_ring, *sample);
    last_baro_sample = *sample;
    have_baro_sample = true;
    stats.env_count++;
}

static void gnss_fix_callback(const gnss_fix_t *fix)
{
    RING_PUSH(gnss_ring, *fix);
    stats.gnss_count++;
    
    /* Update global date string if we have valid date */
    if (fix->date_valid) {
        snprintf(gps_date_string, sizeof(gps_date_string), 
                 "%04d-%02d-%02d", fix->year, fix->month, fix->day);
    }

}

/* NMEA passthrough callback from GNSS service */
static void nmea_passthrough_callback(const char *sentence, size_t len)
{
    if (!output_callback) {
        return;
    }
    
    /* Record arrival time */
    last_nmea_arrival_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Check if this is a sentence we want to log */
    if (len > 6) {
        bool log_sentence = false;
        bool emit_pth = false;
        
        /* Check for GGA */
        if (strncmp(sentence + 3, "GGA", 3) == 0) {
            log_sentence = true;
            emit_pth = true;
        }
        /* Check for GLL */
        else if (strncmp(sentence + 3, "GLL", 3) == 0) {
            log_sentence = true;
            emit_pth = true;
        }
        /* Check for VTG */
        else if (strncmp(sentence + 3, "VTG", 3) == 0) {
            log_sentence = true;
            emit_pth = false;  /* No PTH after VTG */
        }
        
        if (log_sentence) {
            /* Output the raw NMEA sentence as-is */
            output_callback(sentence, len);
            
            /* Output PTH if needed */
            if (emit_pth) {
                char pth_line[LOG_MAX_SENTENCE_LEN];
                int pth_len = log_format_sentence(pth_line, sizeof(pth_line),
                                                  "$PTH,%u",
                                                  last_nmea_arrival_ms);
                if (pth_len > 0) {
                    output_callback(pth_line, pth_len);
                }
            }
        }
    }
}

void aggregator_write_version(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    char version_string[64];
    
    /* Format: $PVER,<description>,<numeric_version> */
    /* For Tempo V1, we use the new format with extended info */
    snprintf(version_string, sizeof(version_string),
             "Tempo %s %s (%s)",
             APP_DEVICE_TYPE,
             APP_VERSION_STRING,
             APP_GIT_COMMIT);
    
    int len = log_format_sentence(line, sizeof(line),
                                  "$PVER,\"%s\",%d",
                                  version_string,
                                  APP_VERSION_NUMERIC);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

/* Output work handlers */
static void imu_output_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!running || !output_callback) {
        goto reschedule;
    }

    /* Only output if orientation service is ready */
    if (!orientation_is_ready()) {
        goto reschedule;
    }

    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    int len;

    /* Output quaternion (orientation service now owns IMU data) */
    if (config.enable_quaternion) {
        orientation_quaternion_t quat;
        if (orientation_get_quaternion(&quat) == 0) {
            len = log_format_sentence(line, sizeof(line),
                                      "$PIM2,%u,%.4f,%.4f,%.4f,%.4f",
                                      timestamp_ms,
                                      quat.w, quat.x, quat.y, quat.z);
        } else {
            /* Fallback to identity quaternion if orientation not available */
            len = log_format_sentence(line, sizeof(line),
                                      "$PIM2,%u,1.0000,0.0000,0.0000,0.0000",
                                      timestamp_ms);
        }
        if (len > 0) {
            output_callback(line, len);
        }
    }

reschedule:
    if (running) {
        k_work_reschedule(&imu_output_work, K_MSEC(IMU_OUTPUT_PERIOD_MS));
    }
}

static void env_output_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (!running || !output_callback || !have_baro_sample) {
        goto reschedule;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Convert pressure from Pa to hPa for compatibility */
    float pressure_hpa = last_baro_sample.pressure_pa / 100.0f;
    
    /* Convert altitude from meters to feet for compatibility */
    float altitude_ft = last_baro_sample.altitude_m * 3.28084f;
    
    /* Battery voltage is -1 for V1 (no battery monitoring) */
    float battery_v = -1.0f;
    
    /* Output environmental sentence using latest sample
     * Format: $PENV,timestamp_ms,pressure_hPa,altitude_ft,battery_v
     */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PENV,%u,%.2f,%.2f,%.2f",
                                  timestamp_ms,
                                  pressure_hpa,
                                  altitude_ft,
                                  battery_v);
    
    if (len > 0) {
        output_callback(line, len);
    }
    
reschedule:
    if (running) {
        k_work_reschedule(&env_output_work, K_MSEC(ENV_OUTPUT_PERIOD_MS));
    }
}

/* Aggregator thread (future use for more complex processing) */
static void aggregator_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Aggregator thread started");
    
    while (running) {
        /* Currently all output is handled by work items */
        /* This thread reserved for future quaternion calculations, etc */
        k_sleep(K_MSEC(100));
    }
    
    LOG_INF("Aggregator thread stopped");
}

int aggregator_init(void)
{
    LOG_INF("Initializing aggregator service");

    /* Initialize ring buffers */
    memset(&baro_ring, 0, sizeof(baro_ring));
    memset(&gnss_ring, 0, sizeof(gnss_ring));

    /* Initialize work items */
    k_work_init_delayable(&imu_output_work, imu_output_handler);
    k_work_init_delayable(&env_output_work, env_output_handler);

    /* Initialize orientation tracking (uses 200Hz IMU FIFO internally) */
    orientation_config_t orient_cfg = {
        .gain = 0.5f,                    /* Fusion AHRS gain */
        .sample_period = 0.005f,         /* 200Hz */
        .use_magnetometer = false,
        .acceleration_rejection = 10.0f,  /* 10 m/s² */
        .magnetic_rejection = 10.0f,
        .recovery_trigger_period = 5.0f
    };

    int ret = orientation_init(&orient_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to initialize orientation service: %d", ret);
        /* Not fatal - continue without quaternion output */
    }

    /* Register sensor callbacks (IMU now handled by orientation service) */
    baro_register_callback(baro_data_callback);
    gnss_register_fix_callback(gnss_fix_callback);
    gnss_register_nmea_callback(nmea_passthrough_callback);

    /* Reset statistics */
    memset(&stats, 0, sizeof(stats));

    LOG_INF("Aggregator initialized");

    return 0;
}

int aggregator_configure(const aggregator_config_t *cfg)
{
    if (!cfg) {
        return -EINVAL;
    }
    
    config = *cfg;
    
    LOG_INF("Aggregator configured: IMU=%dHz, ENV=%dHz, GNSS=%dHz",
            config.imu_output_rate, config.env_output_rate, config.gnss_output_rate);
    
    return 0;
}

void aggregator_register_output_callback(aggregator_output_callback_t callback)
{
    output_callback = callback;
}

int aggregator_start(void)
{
    if (running) {
        return -EALREADY;
    }

    LOG_INF("Starting aggregator");

    /* Set session start time if not already set */
    if (config.session_start_us == 0) {
        config.session_start_us = time_now_us();
    }

    running = true;

    /* Note: Orientation service is started at boot and runs continuously.
     * We just query it here - no need to start/stop it with logging. */

    /* Start output work items */
    k_work_schedule(&imu_output_work, K_MSEC(IMU_OUTPUT_PERIOD_MS));
    k_work_schedule(&env_output_work, K_MSEC(ENV_OUTPUT_PERIOD_MS));

    /* Create aggregator thread */
    aggregator_tid = k_thread_create(&aggregator_thread,
                                     aggregator_stack,
                                     K_THREAD_STACK_SIZEOF(aggregator_stack),
                                     aggregator_thread_fn,
                                     NULL, NULL, NULL,
                                     K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(aggregator_tid, "aggregator");

    return 0;
}

int aggregator_stop(void)
{
    if (!running) {
        return -EALREADY;
    }

    LOG_INF("Stopping aggregator");

    running = false;

    /* Cancel work items */
    k_work_cancel_delayable(&imu_output_work);
    k_work_cancel_delayable(&env_output_work);

    /* Note: Orientation service keeps running - we don't stop it here.
     * It maintains continuous orientation tracking across logging sessions. */

    /* Wait for thread to exit */
    if (aggregator_tid) {
        k_thread_join(aggregator_tid, K_FOREVER);
        aggregator_tid = NULL;
    }

    return 0;
}

void aggregator_write_session_config(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    char time_str[32];
    
    /* Get current time for session start */
    uint64_t now_us = time_now_us();
    uint32_t now_ms = (uint32_t)(now_us / 1000);
    
    /* If we have GPS time, use ISO format, otherwise use milliseconds */
    if (gps_date_string[0] != '\0') {
        /* We have GPS date but might not have full time yet */
        snprintf(time_str, sizeof(time_str), "%sT00:00:00Z", gps_date_string);
    } else {
        /* No GPS yet, use relative timestamp */
        snprintf(time_str, sizeof(time_str), "T+%u", now_ms);
    }
    
    /* Format: $PSFC,session_id,time,rates,axes
     * rates format: imu:env:gnss:mag
     * axes: always NED for V1
     */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PSFC,%u,%s,%d:%d:%d:%d,NED",
                                  config.session_id,
                                  time_str,
                                  config.imu_output_rate,
                                  config.env_output_rate,
                                  config.gnss_output_rate,
                                  config.mag_output_rate);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

void aggregator_write_state_change(const char *old_state, const char *new_state,
                                   const char *trigger)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Format: $PST,timestamp_ms,old_state,new_state,trigger */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PST,%u,%s,%s,%s",
                                  timestamp_ms,
                                  old_state,
                                  new_state,
                                  trigger);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

int aggregator_set_gnss_rate(uint16_t rate_hz)
{
    if (rate_hz != 1 && rate_hz != 10) {
        LOG_ERR("Invalid GNSS rate %d Hz (must be 1 or 10)", rate_hz);
        return -EINVAL;
    }
    
    config.gnss_output_rate = rate_hz;
    
    LOG_INF("GNSS output rate changed to %d Hz", rate_hz);
    
    /* Note: The actual GNSS module configuration should be done by the GNSS service */
    /* This just updates our expected rate for statistics and logging */
    
    return 0;
}

void aggregator_get_stats(uint32_t *imu_count, uint32_t *env_count,
                          uint32_t *gnss_count, uint32_t *dropped_count)
{
    /* IMU count now comes from orientation service */
    if (imu_count) *imu_count = orientation_get_sample_count();
    if (env_count) *env_count = stats.env_count;
    if (gnss_count) *gnss_count = stats.gnss_count;
    if (dropped_count) *dropped_count = stats.dropped_count;
}

void aggregator_set_session_start_time(uint64_t start_us)
{
    config.session_start_us = start_us;
    
    /* Inform GNSS service of session start for PTH timing */
    extern void gnss_set_session_start_time(uint64_t start_us);
    gnss_set_session_start_time(start_us);
}

void aggregator_set_session_id(uint32_t id)
{
    config.session_id = id;
}

void aggregator_write_surface_altitude(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    
    /* Get ground altitude from barometer service */
    float ground_alt_ft = baro_get_ground_altitude_psfc();
    
    /* Format: $PSFC,altitude_ft */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PSFC,%d",
                                  (int)ground_alt_ft);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

/* If you need session configuration elsewhere, rename the old function */
void aggregator_write_session_info(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    char time_str[32];
    
    /* Get current time for session start */
    uint64_t now_us = time_now_us();
    uint32_t now_ms = (uint32_t)(now_us / 1000);
    
    /* If we have GPS time, use ISO format, otherwise use milliseconds */
    if (gps_date_string[0] != '\0') {
        /* We have GPS date but might not have full time yet */
        snprintf(time_str, sizeof(time_str), "%sT00:00:00Z", gps_date_string);
    } else {
        /* No GPS yet, use relative timestamp */
        snprintf(time_str, sizeof(time_str), "T+%u", now_ms);
    }
    
    /* This could be a custom sentence like $PSES (session info) if needed */
    /* Format: $PSES,session_id,time,rates,axes */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PSES,%u,%s,%d:%d:%d:%d,NED",
                                  config.session_id,
                                  time_str,
                                  config.imu_output_rate,
                                  config.env_output_rate,
                                  config.gnss_output_rate,
                                  config.mag_output_rate);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

/* Update the header writer to use the correct function */
void aggregator_write_session_header(void)
{
    /* First write version */
    aggregator_write_version();
    
    /* Then write surface altitude (not session config!) */
    aggregator_write_surface_altitude();
}
