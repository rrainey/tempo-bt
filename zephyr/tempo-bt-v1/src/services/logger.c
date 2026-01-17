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

/* Forward declarations */
static void aggregator_output_to_file(const char *line, size_t len);
static int create_session_directory(void);
static const char *state_to_string(logger_state_t state);

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
    if (ret != 0 && ret != -ENOSPC) {
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

void logger_baro_handler(const baro_sample_t *sample)
{
    static float ground_altitude_m = 0.0f;
    static float last_altitude_m = 0.0f;
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
    
    /* Calculate climb rate if we have previous sample */
    if (last_sample_time_us > 0) {
        float dt = (sample->timestamp_us - last_sample_time_us) / 1000000.0f;
        float climb_rate = (sample->altitude_m - last_altitude_m) / dt;
        float agl = sample->altitude_m - ground_altitude_m;
        
        /* Check for takeoff */
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
        
        /* Check for landing */
        if (logger_state.state == LOGGER_STATE_LOGGING && 
            logger_state.config.auto_stop_on_landing) {
            
            if (agl < LANDING_ALTITUDE_M && 
                fabsf(climb_rate) < LANDING_LOW_SPEED_MPS) {
                /* Implement landing detection logic */
            }
        }
    }
    
    last_altitude_m = sample->altitude_m;
    last_sample_time_us = sample->timestamp_us;
}

static const char *state_to_string(logger_state_t state)
{
    switch (state) {
    case LOGGER_STATE_IDLE:       return "IDLE";
    case LOGGER_STATE_ARMED:      return "ARMED";
    case LOGGER_STATE_LOGGING:    return "LOGGING";
    case LOGGER_STATE_POSTFLIGHT: return "POSTFLIGHT";
    case LOGGER_STATE_ERROR:      return "ERROR";
    default:                      return "UNKNOWN";
    }
}