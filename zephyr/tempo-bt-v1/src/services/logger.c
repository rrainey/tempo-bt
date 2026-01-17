/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Logger Service Implementation
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "services/logger.h"
#include "services/aggregator.h"
#include "services/file_writer.h"
#include "services/storage.h"
#include "services/timebase.h"
#include "services/baro.h"
#include "services/gnss.h"
#include "app/events.h"

LOG_MODULE_REGISTER(logger, LOG_LEVEL_INF);

/* Takeoff detection parameters */
#define TAKEOFF_CLIMB_RATE_MPS      2.0f    /* 2 m/s climb rate threshold */
#define TAKEOFF_ALTITUDE_CHANGE_M    50.0f   /* 50m altitude change to confirm */
#define TAKEOFF_DETECT_DURATION_S    5       /* Sustained for 5 seconds */
#define TAKEOFF_MIN_ALTITUDE_M       100.0f  /* Minimum altitude above ground */

/* Landing detection parameters */
#define LANDING_ALTITUDE_M           200.0f  /* Below 200m AGL */
#define LANDING_LOW_SPEED_MPS        2.0f    /* Low vertical speed */
#define LANDING_STABLE_DURATION_S    10      /* Stable for 10 seconds */

/* Executive function thresholds */
#define EXEC_TAKEOFF_CLIMB_RATE_MPS     1.016f   /* 200 ft/min - trigger ARMED->LOGGING */
#define EXEC_FREEFALL_DESCENT_RATE_MPS  (-5.08f) /* -1000 ft/min - trigger LOGGING->JUMPED */
#define EXEC_LOW_ACTIVITY_RATE_MPS      1.016f   /* |200 ft/min| - low activity threshold */
#define EXEC_JUMPED_TIMEOUT_S           60       /* 60s low activity in JUMPED -> ARMED */
#define EXEC_LOGGING_ABORT_TIMEOUT_S    360      /* 6 min low activity in LOGGING -> ARMED */

/* Alpha-beta filter parameters for climb rate estimation
 * These values provide ~1-2 second time constant for good transient rejection
 * while maintaining reasonable responsiveness to actual climb/descent.
 * Reference: https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/
 */
#define ALPHA_BETA_ALPHA                0.15f    /* Position (altitude) correction weight */
#define ALPHA_BETA_BETA                 0.005f   /* Velocity (climb rate) correction weight */

/* Median filter size for altitude pre-filtering */
#define MEDIAN_FILTER_SIZE              3

/* Session state */
static struct {
    logger_state_t state;
    logger_config_t config;

    /* Current session */
    uint32_t session_id;
    uint64_t session_start_us;
    char session_path[256];
    char log_file_path[256];

    /* Synchronization */
    struct k_mutex lock;
} logger_state;

/* Executive function state tracking */
static struct {
    /* Alpha-beta filter state for climb rate estimation */
    float altitude_est_m;           /* Estimated altitude (position state) */
    float climb_rate_est_mps;       /* Estimated climb rate (velocity state) */
    bool filter_initialized;

    /* Median pre-filter buffer for altitude */
    float median_buffer[MEDIAN_FILTER_SIZE];
    uint8_t median_index;
    uint8_t median_count;

    /* Low-activity timeout tracking */
    uint32_t low_activity_counter_s;

    /* Transition confirmation counters (debouncing) */
    uint8_t takeoff_confirm_count;
    uint8_t freefall_confirm_count;

    /* Thread safety */
    struct k_mutex exec_mutex;
} executive_state;

/* Forward declarations */
static void aggregator_output_to_file(const char *line, size_t len);
static int create_session_directory(void);
static const char *state_to_string(logger_state_t state);

/* Executive function forward declarations */
static void executive_init(void);
static void executive_reset_counters(void);
static void executive_handle_armed_state(float climb_rate);
static void executive_handle_logging_state(float climb_rate);
static void executive_handle_jumped_state(float climb_rate);

/* Ground altitude sampling work item */
static struct k_work_delayable ground_altitude_work;
static bool ground_altitude_work_initialized = false;

/* Ground altitude sampling handler */
static void ground_altitude_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* Only sample when in IDLE or ARMED states */
    if (logger_state.state != LOGGER_STATE_IDLE && 
        logger_state.state != LOGGER_STATE_ARMED) {
        return;
    }
    
    /* Get current barometer reading */
    baro_sample_t sample;
    if (baro_get_current_sample(&sample) == 0 && sample.pressure_valid) {
        float altitude_ft = sample.altitude_m * 3.28084f;
        baro_record_ground_altitude(altitude_ft);
        LOG_DBG("Recorded ground altitude: %.1f ft", altitude_ft);
    }
    
    /* Schedule next sample */
    k_work_reschedule(&ground_altitude_work, K_SECONDS(300));
}

/* Initialize ground altitude tracking (call this in logger_init) */
static void init_ground_altitude_tracking(void)
{
    if (!ground_altitude_work_initialized) {
        k_work_init_delayable(&ground_altitude_work, ground_altitude_work_handler);
        ground_altitude_work_initialized = true;
    }
}

/* Start ground altitude sampling (call when entering IDLE/ARMED) */
static void start_ground_altitude_sampling(void)
{
    /* Take immediate sample */
    ground_altitude_work_handler(&ground_altitude_work.work);
    
    /* Schedule periodic sampling */
    k_work_reschedule(&ground_altitude_work, K_SECONDS(300));
}

/* Stop ground altitude sampling (call when starting logging) */
static void stop_ground_altitude_sampling(void)
{
    k_work_cancel_delayable(&ground_altitude_work);
}

/* Public API */
int logger_init(const logger_config_t *config)
{
    LOG_INF("Initializing logger service");
    
    if (!config) {
        return -EINVAL;
    }
    
    k_mutex_init(&logger_state.lock);
    
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    
    logger_state.config = *config;
    logger_state.state = LOGGER_STATE_IDLE;
    logger_state.session_id = 0;
    
    k_mutex_unlock(&logger_state.lock);
    
    /* Initialize file writer with reasonable defaults */
    file_writer_config_t writer_config = {
        .buffer_size = 4096,
        .flush_interval_ms = 250
    };

    int ret = file_writer_init(&writer_config);
    if (ret != 0) {
        LOG_ERR("Failed to initialize file writer: %d", ret);
        return ret;
    }

    /* Initialize ground altitude tracking */
    init_ground_altitude_tracking();

    /* Initialize executive function */
    executive_init();

    /* If we start in IDLE state, begin sampling immediately */
    if (logger_state.state == LOGGER_STATE_IDLE) {
        /* Get initial altitude and start sampling */
        baro_sample_t sample;
        if (baro_get_current_sample(&sample) == 0 && sample.pressure_valid) {
            float altitude_ft = sample.altitude_m * 3.28084f;
            baro_init_ground_altitude(altitude_ft);
            start_ground_altitude_sampling();
        }
    }
    
    LOG_INF("Logger initialized");
    
    return 0;
}

int logger_start(void)
{
    int ret;
    
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    
    if (logger_state.state != LOGGER_STATE_IDLE && 
        logger_state.state != LOGGER_STATE_ARMED) {
        LOG_WRN("Cannot start logging from state %s", 
                state_to_string(logger_state.state));
        k_mutex_unlock(&logger_state.lock);
        return -EINVAL;
    }
    
    LOG_INF("Starting logging session");
    
    /* Generate session ID and timestamp */
    logger_state.session_id = sys_rand32_get();
    logger_state.session_start_us = time_now_us();
    
    /* Create session directory and file */
    ret = create_session_directory();
    if (ret != 0) {
        LOG_ERR("Failed to create session directory: %d", ret);
        k_mutex_unlock(&logger_state.lock);
        return ret;
    }
    
    /* Open log file */
    ret = file_writer_open(logger_state.log_file_path);
    if (ret != 0) {
        LOG_ERR("Failed to open log file: %d", ret);
        k_mutex_unlock(&logger_state.lock);
        return ret;
    }
    
    /* Configure aggregator */
    aggregator_config_t agg_config = {
        .imu_output_rate = logger_state.config.imu_rate_hz,
        .env_output_rate = logger_state.config.env_rate_hz,
        .gnss_output_rate = logger_state.config.gnss_rate_hz,
        .mag_output_rate = logger_state.config.enable_magnetometer ? 10 : 0,
        .enable_quaternion = logger_state.config.enable_quaternion,
        .enable_magnetometer = logger_state.config.enable_magnetometer,
        .session_id = logger_state.session_id,
        .session_start_us = logger_state.session_start_us
    };
    
    aggregator_configure(&agg_config);
    aggregator_register_output_callback(aggregator_output_to_file);

    /* Stop ground altitude sampling when we start logging */
    stop_ground_altitude_sampling();
    
    /* Write session header */
    aggregator_write_session_header();
    
    /* Start aggregator */
    ret = aggregator_start();
    if (ret != 0) {
        LOG_ERR("Failed to start aggregator: %d", ret);
        file_writer_close();
        k_mutex_unlock(&logger_state.lock);
        return ret;
    }
    
    /* Update state and emit event */
    logger_state_t old_state = logger_state.state;
    logger_state.state = LOGGER_STATE_LOGGING;
    
    k_mutex_unlock(&logger_state.lock);
    
    /* Write state change to log */
    aggregator_write_state_change(state_to_string(old_state),
                                  state_to_string(LOGGER_STATE_LOGGING),
                                  "manual_start");
    
    /* Emit event */
    app_event_t evt = {
        .type = EVT_SESSION_START,
        .payload.session.id = logger_state.session_id
    };
    event_bus_publish(&evt);
    
    LOG_INF("Logging started: session=%08x, path=%s", 
            logger_state.session_id, logger_state.session_path);
    
    return 0;
}

int logger_stop(void)
{
    k_mutex_lock(&logger_state.lock, K_FOREVER);

    if (logger_state.state != LOGGER_STATE_LOGGING &&
        logger_state.state != LOGGER_STATE_JUMPED &&
        logger_state.state != LOGGER_STATE_POSTFLIGHT) {
        LOG_WRN("Cannot stop logging from state %s",
                state_to_string(logger_state.state));
        k_mutex_unlock(&logger_state.lock);
        return -EINVAL;
    }
    
    LOG_INF("Stopping logging session");
    
    logger_state_t old_state = logger_state.state;
    logger_state.state = LOGGER_STATE_IDLE;
    
    k_mutex_unlock(&logger_state.lock);
    
    /* Write final state change */
    aggregator_write_state_change(state_to_string(old_state),
                                  state_to_string(LOGGER_STATE_IDLE),
                                  "manual_stop");
    
    /* Stop aggregator */
    aggregator_stop();
    
    /* Get final statistics */
    file_writer_stats_t stats;
    file_writer_get_stats(&stats);
    
    /* Close file */
    file_writer_close();
    
    /* Log session summary */
    LOG_INF("Session complete: %llu bytes, %u lines, %u errors",
            stats.bytes_written, stats.lines_written, stats.write_errors);
    
    /* Emit event */
    app_event_t evt = {
        .type = EVT_SESSION_STOP,
        .payload.session.id = logger_state.session_id
    };
    event_bus_publish(&evt);

    if (logger_state.state == LOGGER_STATE_IDLE) {
        start_ground_altitude_sampling();
    }
    
    return 0;
}

int logger_arm(void)
{
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    
    if (logger_state.state != LOGGER_STATE_IDLE) {
        LOG_WRN("Cannot arm from state %s", state_to_string(logger_state.state));
        k_mutex_unlock(&logger_state.lock);
        return -EINVAL;
    }
    
    /* If we haven't initialized ground altitude yet, do it now */
    baro_sample_t sample;
    if (baro_get_current_sample(&sample) == 0 && sample.pressure_valid) {
        float altitude_ft = sample.altitude_m * 3.28084f;
        baro_init_ground_altitude(altitude_ft);
    }
    
    /* Ensure ground altitude sampling is running */
    start_ground_altitude_sampling();
    
    logger_state_t old_state = logger_state.state;
    logger_state.state = LOGGER_STATE_ARMED;
    
    k_mutex_unlock(&logger_state.lock);
    
    LOG_INF("Logger armed");
    
    /* Emit event */
    app_event_t evt = {
        .type = EVT_STATE_CHANGE,
        .payload.state_change.old_state = old_state,
        .payload.state_change.new_state = LOGGER_STATE_ARMED
    };
    event_bus_publish(&evt);
    
    return 0;
}

int logger_disarm(void)
{
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    
    if (logger_state.state != LOGGER_STATE_ARMED) {
        LOG_WRN("Cannot disarm from state %s", state_to_string(logger_state.state));
        k_mutex_unlock(&logger_state.lock);
        return -EINVAL;
    }
    
    logger_state_t old_state = logger_state.state;
    logger_state.state = LOGGER_STATE_IDLE;
    
    k_mutex_unlock(&logger_state.lock);

    /* Continue ground altitude sampling in IDLE state */
    start_ground_altitude_sampling();
    
    LOG_INF("Logger disarmed");
    
    /* Emit event */
    app_event_t evt = {
        .type = EVT_STATE_CHANGE,
        .payload.state_change.old_state = old_state,
        .payload.state_change.new_state = LOGGER_STATE_IDLE
    };
    event_bus_publish(&evt);
    
    return 0;
}

logger_state_t logger_get_state(void)
{
    logger_state_t state;
    
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    state = logger_state.state;
    k_mutex_unlock(&logger_state.lock);
    
    return state;
}

int logger_get_session_info(uint32_t *session_id, uint64_t *start_time,
                            char *path, size_t path_size)
{
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    
    if (logger_state.state != LOGGER_STATE_LOGGING &&
        logger_state.state != LOGGER_STATE_POSTFLIGHT) {
        k_mutex_unlock(&logger_state.lock);
        return -EINVAL;
    }
    
    if (session_id) {
        *session_id = logger_state.session_id;
    }
    
    if (start_time) {
        *start_time = logger_state.session_start_us;
    }
    
    if (path && path_size > 0) {
        strncpy(path, logger_state.session_path, path_size - 1);
        path[path_size - 1] = '\0';
    }
    
    k_mutex_unlock(&logger_state.lock);
    
    return 0;
}

/* Internal functions */
static void aggregator_output_to_file(const char *line, size_t len)
{
    int ret = file_writer_write(line, len);
    /* Silently ignore:
     * -ENOENT: file not open (normal after logging stops)
     * -ENOSPC: buffer full (already logged by file_writer)
     */
    if (ret != 0 && ret != -ENOSPC && ret != -ENOENT) {
        LOG_ERR("Failed to write line: %d", ret);
    }
}

static int create_session_directory(void)
{
    //int ret;
    struct tm *tm;
    struct timespec ts;
    time_t now;

    /* Get actual system time (set by GNSS) instead of uptime */
    if (clock_gettime(CLOCK_REALTIME, &ts) == 0) {
        now = ts.tv_sec;
    } else {
        /* Fallback to uptime if CLOCK_REALTIME fails */
        LOG_WRN("CLOCK_REALTIME not available, using uptime for directory name");
        now = logger_state.session_start_us / 1000000;
    }

    /* Base path */
    strcpy(logger_state.session_path, logger_state.config.base_path);

    /* Add date folder if configured */
    if (logger_state.config.use_date_folders) {
        char date_str[16];
        tm = gmtime(&now);
        strftime(date_str, sizeof(date_str), "%Y%m%d", tm);

        strcat(logger_state.session_path, "/");
        strcat(logger_state.session_path, date_str);
    }

    /* Add session folder */
    strcat(logger_state.session_path, "/");

    if (logger_state.config.use_uuid_names) {
        char uuid[9];
        snprintf(uuid, sizeof(uuid), "%08X", logger_state.session_id);
        strcat(logger_state.session_path, uuid);
    } else {
        char time_str[16];
        tm = gmtime(&now);
        strftime(time_str, sizeof(time_str), "%H%M%S", tm);
        strcat(logger_state.session_path, time_str);
    }

    /* Create directories - storage layer should handle this */
    /* For now, we'll just create the file path */

    /* Build full file path */
    snprintf(logger_state.log_file_path, sizeof(logger_state.log_file_path),
             "%s/flight.txt", logger_state.session_path);

    return 0;
}

/* Helper function: compute median of 3 values */
static float median3(float a, float b, float c)
{
    if (a > b) {
        if (b > c) return b;      /* a > b > c */
        else if (a > c) return c; /* a > c >= b */
        else return a;            /* c >= a > b */
    } else {
        if (a > c) return a;      /* b >= a > c */
        else if (b > c) return c; /* b > c >= a */
        else return b;            /* c >= b >= a */
    }
}

void logger_baro_handler(const baro_sample_t *sample)
{
    static float ground_altitude_m = 0.0f;
    static uint64_t last_sample_time_us = 0;
    static int climb_samples = 0;
    static bool ground_level_set = false;

    if (!sample || !sample->pressure_valid) {
        return;
    }

    /* Set ground level on first valid reading when armed */
    if (logger_state.state == LOGGER_STATE_ARMED && !ground_level_set) {
        ground_altitude_m = sample->altitude_m;
        ground_level_set = true;
        LOG_INF("Ground altitude set: %.1f m", ground_altitude_m);
    }

    float altitude_m = sample->altitude_m;
    float dt = 0.0f;

    /* Calculate dt if we have a previous sample */
    if (last_sample_time_us > 0 && sample->timestamp_us > last_sample_time_us) {
        dt = (sample->timestamp_us - last_sample_time_us) / 1000000.0f;
    }
    last_sample_time_us = sample->timestamp_us;

    /* Protect against bad timing data */
    if (dt < 0.001f || dt > 10.0f) {
        return;
    }

    k_mutex_lock(&executive_state.exec_mutex, K_FOREVER);

    /* Step 1: Median pre-filter to reject single-sample outliers */
    executive_state.median_buffer[executive_state.median_index] = altitude_m;
    executive_state.median_index = (executive_state.median_index + 1) % MEDIAN_FILTER_SIZE;
    if (executive_state.median_count < MEDIAN_FILTER_SIZE) {
        executive_state.median_count++;
    }

    float filtered_altitude_m;
    if (executive_state.median_count >= MEDIAN_FILTER_SIZE) {
        /* Have enough samples for median filter */
        filtered_altitude_m = median3(
            executive_state.median_buffer[0],
            executive_state.median_buffer[1],
            executive_state.median_buffer[2]
        );
    } else {
        /* Not enough samples yet, use raw value */
        filtered_altitude_m = altitude_m;
    }

    /* Step 2: Alpha-beta filter for coupled altitude/velocity estimation
     * This provides much better transient rejection than a simple IIR filter
     * on the derivative, as the velocity estimate evolves based on altitude
     * residuals rather than instantaneous rate calculations.
     */
    if (!executive_state.filter_initialized) {
        /* First sample - initialize state directly */
        executive_state.altitude_est_m = filtered_altitude_m;
        executive_state.climb_rate_est_mps = 0.0f;
        executive_state.filter_initialized = true;
        k_mutex_unlock(&executive_state.exec_mutex);
        return;
    }

    /* Predict step: advance state using current velocity estimate */
    float altitude_pred = executive_state.altitude_est_m +
                          executive_state.climb_rate_est_mps * dt;

    /* Update step: correct prediction using measurement residual */
    float residual = filtered_altitude_m - altitude_pred;
    executive_state.altitude_est_m = altitude_pred + ALPHA_BETA_ALPHA * residual;
    executive_state.climb_rate_est_mps = executive_state.climb_rate_est_mps +
                                          (ALPHA_BETA_BETA / dt) * residual;

    float climb_rate = executive_state.climb_rate_est_mps;
    k_mutex_unlock(&executive_state.exec_mutex);

    /* Calculate AGL for legacy detection */
    float agl = filtered_altitude_m - ground_altitude_m;

    /* Legacy takeoff detection (kept for compatibility) */
    if (logger_state.state == LOGGER_STATE_ARMED &&
        logger_state.config.auto_start_on_takeoff) {

        if (climb_rate > TAKEOFF_CLIMB_RATE_MPS &&
            agl > TAKEOFF_MIN_ALTITUDE_M) {
            climb_samples++;

            if (climb_samples > (TAKEOFF_DETECT_DURATION_S * logger_state.config.env_rate_hz)) {
                LOG_INF("Takeoff detected! Climb rate: %.1f m/s, AGL: %.1f m",
                        climb_rate, agl);
                logger_start();
            }
        } else {
            climb_samples = 0;
        }
    }

    /* Legacy landing detection placeholder */
    if (logger_state.state == LOGGER_STATE_LOGGING &&
        logger_state.config.auto_stop_on_landing) {

        if (agl < LANDING_ALTITUDE_M &&
            fabsf(climb_rate) < LANDING_LOW_SPEED_MPS) {
            /* Landing detection now handled by executive */
        }
    }
}

static const char *state_to_string(logger_state_t state)
{
    switch (state) {
    case LOGGER_STATE_IDLE:       return "IDLE";
    case LOGGER_STATE_ARMED:      return "ARMED";
    case LOGGER_STATE_LOGGING:    return "LOGGING";
    case LOGGER_STATE_JUMPED:     return "JUMPED";
    case LOGGER_STATE_POSTFLIGHT: return "POSTFLIGHT";
    case LOGGER_STATE_ERROR:      return "ERROR";
    default:                      return "UNKNOWN";
    }
}

/*
 * Executive function implementation
 *
 * The executive manages automatic state transitions based on barometric
 * rate of climb. It should be called every 1 second from the main loop.
 */

/* Initialize executive state */
static void executive_init(void)
{
    k_mutex_init(&executive_state.exec_mutex);

    /* Alpha-beta filter state */
    executive_state.altitude_est_m = 0.0f;
    executive_state.climb_rate_est_mps = 0.0f;
    executive_state.filter_initialized = false;

    /* Median pre-filter state */
    memset(executive_state.median_buffer, 0, sizeof(executive_state.median_buffer));
    executive_state.median_index = 0;
    executive_state.median_count = 0;

    /* Executive counters */
    executive_state.low_activity_counter_s = 0;
    executive_state.takeoff_confirm_count = 0;
    executive_state.freefall_confirm_count = 0;

    LOG_INF("Logger executive initialized (alpha-beta filter: alpha=%.3f, beta=%.4f)",
            ALPHA_BETA_ALPHA, ALPHA_BETA_BETA);
}

/* Reset executive counters (called on state transitions) */
static void executive_reset_counters(void)
{
    k_mutex_lock(&executive_state.exec_mutex, K_FOREVER);
    executive_state.low_activity_counter_s = 0;
    executive_state.takeoff_confirm_count = 0;
    executive_state.freefall_confirm_count = 0;
    k_mutex_unlock(&executive_state.exec_mutex);
}

/* Handle ARMED state - looking for takeoff */
static void executive_handle_armed_state(float climb_rate)
{
    /* Check for sustained positive climb rate indicating takeoff */
    if (climb_rate > EXEC_TAKEOFF_CLIMB_RATE_MPS) {
        executive_state.takeoff_confirm_count++;
        LOG_DBG("Takeoff detection: count=%d, rate=%.2f m/s",
                executive_state.takeoff_confirm_count, climb_rate);

        /* Require 3 consecutive seconds of climb (debounce) */
        if (executive_state.takeoff_confirm_count >= 3) {
            LOG_INF("Executive: Takeoff detected! climb_rate=%.2f m/s (%.0f ft/min)",
                    climb_rate, climb_rate * 196.85f);

            /* Transition to LOGGING state */
            int ret = logger_start();
            if (ret == 0) {
                /* GNSS stays at 1 Hz during climb */
                gnss_set_rate(1);
                executive_reset_counters();
            } else {
                LOG_ERR("Failed to start logging: %d", ret);
            }
        }
    } else {
        /* Reset takeoff detection counter */
        executive_state.takeoff_confirm_count = 0;
    }
}

/* Handle LOGGING state - recording flight, looking for freefall or abort */
static void executive_handle_logging_state(float climb_rate)
{
    bool is_low_activity = fabsf(climb_rate) < EXEC_LOW_ACTIVITY_RATE_MPS;

    /* DEBUG: Periodically log climb rate and low activity counter */
    LOG_INF("LOGGING: climb=%.2f m/s (%.0f ft/min), low_act=%s, counter=%u/%u",
            climb_rate, climb_rate * 196.85f,
            is_low_activity ? "YES" : "NO",
            executive_state.low_activity_counter_s,
            EXEC_LOGGING_ABORT_TIMEOUT_S);

    /* Check for freefall (fast descent) */
    if (climb_rate < EXEC_FREEFALL_DESCENT_RATE_MPS) {
        executive_state.freefall_confirm_count++;

        /* Require 2 consecutive seconds of fast descent (debounce) */
        if (executive_state.freefall_confirm_count >= 2) {
            LOG_INF("Executive: Freefall detected! climb_rate=%.2f m/s (%.0f ft/min)",
                    climb_rate, climb_rate * 196.85f);

            /* Transition to JUMPED state */
            k_mutex_lock(&logger_state.lock, K_FOREVER);
            logger_state_t old_state = logger_state.state;
            logger_state.state = LOGGER_STATE_JUMPED;
            k_mutex_unlock(&logger_state.lock);

            /* Increase GNSS rate to 10 Hz for maximum position accuracy */
            gnss_set_rate(10);
            executive_reset_counters();

            /* Log state change */
            aggregator_write_state_change(state_to_string(old_state),
                                          state_to_string(LOGGER_STATE_JUMPED),
                                          "freefall_detected");

            /* Emit event */
            app_event_t evt = {
                .type = EVT_STATE_CHANGE,
                .payload.state_change.old_state = old_state,
                .payload.state_change.new_state = LOGGER_STATE_JUMPED
            };
            event_bus_publish(&evt);
        }
    } else {
        executive_state.freefall_confirm_count = 0;
    }

    /* Check for abort condition (no jump - aircraft landed) */
    if (is_low_activity) {
        executive_state.low_activity_counter_s++;

        if (executive_state.low_activity_counter_s >= EXEC_LOGGING_ABORT_TIMEOUT_S) {
            LOG_INF("Executive: Logging abort - no jump detected after %d seconds",
                    EXEC_LOGGING_ABORT_TIMEOUT_S);

            /* Stop logging and return to ARMED */
            logger_stop();

            /* Brief delay for stop to complete */
            k_msleep(100);

            /* Transition back to ARMED */
            logger_arm();
            gnss_set_rate(1);
            executive_reset_counters();
        }
    } else {
        /* Reset low activity counter on any significant movement */
        executive_state.low_activity_counter_s = 0;
    }
}

/* Handle JUMPED state - active jump, looking for landing */
static void executive_handle_jumped_state(float climb_rate)
{
    bool is_low_activity = fabsf(climb_rate) < EXEC_LOW_ACTIVITY_RATE_MPS;

    /* Check for landing (sustained low activity) */
    if (is_low_activity) {
        executive_state.low_activity_counter_s++;

        LOG_DBG("JUMPED low activity: %d/%d seconds",
                executive_state.low_activity_counter_s,
                EXEC_JUMPED_TIMEOUT_S);

        if (executive_state.low_activity_counter_s >= EXEC_JUMPED_TIMEOUT_S) {
            LOG_INF("Executive: Landing detected after %d seconds of low activity",
                    EXEC_JUMPED_TIMEOUT_S);

            /* Stop logging */
            logger_stop();

            /* Brief delay for stop to complete */
            k_msleep(100);

            /* Transition back to ARMED for next jump */
            logger_arm();
            gnss_set_rate(1);
            executive_reset_counters();
        }
    } else {
        /* Reset counter - still in active descent */
        executive_state.low_activity_counter_s = 0;
    }
}

/* Main executive function - call every 1 second from main loop */
void logger_executive(void)
{
    logger_state_t current_state;
    float climb_rate;
    bool rate_valid;

    /* Get current state under lock */
    k_mutex_lock(&logger_state.lock, K_FOREVER);
    current_state = logger_state.state;
    k_mutex_unlock(&logger_state.lock);

    /* Get filtered climb rate from alpha-beta filter */
    k_mutex_lock(&executive_state.exec_mutex, K_FOREVER);
    climb_rate = executive_state.climb_rate_est_mps;
    rate_valid = executive_state.filter_initialized;
    k_mutex_unlock(&executive_state.exec_mutex);

    /* Skip if filter not yet initialized */
    if (!rate_valid) {
        LOG_DBG("Executive: Filter not initialized");
        return;
    }

    /* DEBUG: Log executive tick every call */
    LOG_DBG("Executive tick: state=%s, climb_rate=%.2f m/s, rate_valid=%d",
            state_to_string(current_state), climb_rate, rate_valid);

    switch (current_state) {
    case LOGGER_STATE_ARMED:
        executive_handle_armed_state(climb_rate);
        break;

    case LOGGER_STATE_LOGGING:
        executive_handle_logging_state(climb_rate);
        break;

    case LOGGER_STATE_JUMPED:
        executive_handle_jumped_state(climb_rate);
        break;

    case LOGGER_STATE_IDLE:
    case LOGGER_STATE_POSTFLIGHT:
    case LOGGER_STATE_ERROR:
    default:
        /* Reset executive state when not in active states */
        executive_reset_counters();
        break;
    }
}