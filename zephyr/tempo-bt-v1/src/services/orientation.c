/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - Orientation Tracking Service Implementation
 *
 * Polls IMU FIFO and feeds samples to Fusion AHRS for quaternion output.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "services/orientation.h"
#include "services/imu.h"
#include "Fusion.h"

#if CONFIG_USE_MAG_IN_ORIENTATION
#include "services/mag.h"
#endif

LOG_MODULE_REGISTER(orientation, LOG_LEVEL_INF);

/* Polling configuration */
#define FIFO_POLL_PERIOD_MS     20      /* Poll at 50Hz */
#define MAX_SAMPLES_PER_POLL    16      /* Max samples to read per poll */
#define IMU_ODR_HZ              200     /* Expected IMU sample rate */

/* Thread configuration */
#define ORIENTATION_STACK_SIZE  2048
#define ORIENTATION_PRIORITY    5

/* Module state */
static struct {
    FusionAhrs ahrs;
    orientation_config_t config;
    bool initialized;
    bool running;
    uint64_t last_sample_us;
    uint32_t sample_count;
    uint32_t poll_count;
    imu_sample_t latest_imu;        /* Most recent raw IMU sample for $PIMU output */
    struct k_mutex lock;
} state;

/* Thread resources */
K_THREAD_STACK_DEFINE(orientation_stack, ORIENTATION_STACK_SIZE);
static struct k_thread orientation_thread;
static k_tid_t orientation_tid = NULL;

/* Sample buffer for FIFO reads */
static imu_sample_t sample_buf[MAX_SAMPLES_PER_POLL];

/* Default configuration for IMU-only operation */
static const orientation_config_t default_config = {
    .gain = 0.5f,                    /* Fusion gain */
    .sample_period = 1.0f / IMU_ODR_HZ,
    .use_magnetometer = false,
    .acceleration_rejection = 10.0f,
    .magnetic_rejection = 10.0f,
    .recovery_trigger_period = 5.0f
};

/*
 * Process a single IMU sample through Fusion AHRS
 */
static void process_sample(const imu_sample_t *sample)
{
    /* Calculate delta time */
    float delta_time;
    if (state.last_sample_us == 0) {
        delta_time = state.config.sample_period;
    } else {
        uint64_t delta_us = sample->timestamp_us - state.last_sample_us;
        delta_time = (float)delta_us / 1000000.0f;

        /* Clamp to reasonable range */
        if (delta_time < 0.0001f) {
            delta_time = 0.0001f;
        } else if (delta_time > 0.1f) {
            delta_time = 0.1f;
        }
    }
    state.last_sample_us = sample->timestamp_us;

    /* Convert units for Fusion:
     * - Gyroscope: rad/s to deg/s
     * - Accelerometer: m/s² to g
     */
    FusionVector gyroscope = {
        .axis = {
            .x = sample->gyro_x * (180.0f / (float)M_PI),
            .y = sample->gyro_y * (180.0f / (float)M_PI),
            .z = sample->gyro_z * (180.0f / (float)M_PI)
        }
    };

    FusionVector accelerometer = {
        .axis = {
            .x = sample->accel_x / 9.80665f,
            .y = sample->accel_y / 9.80665f,
            .z = sample->accel_z / 9.80665f
        }
    };

    /* Update AHRS algorithm */
#if CONFIG_USE_MAG_IN_ORIENTATION
    if (state.config.use_magnetometer && mag_is_ready()) {
        FusionVector magnetometer;
        if (mag_read_fusion(&magnetometer) == 0) {
            FusionAhrsUpdate(&state.ahrs, gyroscope, accelerometer,
                             magnetometer, delta_time);
        } else {
            /* Magnetometer read failed, fall back to IMU-only */
            FusionAhrsUpdateNoMagnetometer(&state.ahrs, gyroscope,
                                           accelerometer, delta_time);
        }
    } else {
        FusionAhrsUpdateNoMagnetometer(&state.ahrs, gyroscope,
                                       accelerometer, delta_time);
    }
#else
    FusionAhrsUpdateNoMagnetometer(&state.ahrs, gyroscope, accelerometer, delta_time);
#endif

    state.latest_imu = *sample;
    state.sample_count++;
}

/*
 * Orientation thread function - polls FIFO and processes samples
 */
static void orientation_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("Orientation thread started");

    /* Start IMU FIFO */
    int ret = imu_fifo_start();
    if (ret < 0) {
        LOG_ERR("Failed to start IMU FIFO: %d", ret);
        return;
    }

    while (state.running) {
        /* Read available samples from FIFO */
        int count = imu_fifo_read(sample_buf, MAX_SAMPLES_PER_POLL);

        if (count > 0) {
            k_mutex_lock(&state.lock, K_FOREVER);

            /* Process each sample */
            for (int i = 0; i < count; i++) {
                process_sample(&sample_buf[i]);
            }

            k_mutex_unlock(&state.lock);
        } else if (count < 0) {
            LOG_WRN("FIFO read error: %d", count);
        }

        state.poll_count++;

        /* Log status periodically (every ~2 seconds) */
        if (state.poll_count % 100 == 0) {
            orientation_quaternion_t quat;
            orientation_euler_t euler;

            orientation_get_quaternion(&quat);
            orientation_get_euler(&euler);

            if (FusionAhrsGetFlags(&state.ahrs).initialising) {
                LOG_INF("AHRS initializing... samples=%u", state.sample_count);
            } else {
                LOG_DBG("Orientation: RPY=[%.1f,%.1f,%.1f] samples=%u",
                        (double)euler.roll, (double)euler.pitch, (double)euler.yaw,
                        state.sample_count);
            }
        }

        k_sleep(K_MSEC(FIFO_POLL_PERIOD_MS));
    }

    /* Stop IMU FIFO */
    imu_fifo_stop();

    LOG_INF("Orientation thread stopped");
}

/*
 * Public API
 */

int orientation_init(const orientation_config_t *config)
{
    LOG_INF("Initializing orientation service");

    k_mutex_init(&state.lock);

    k_mutex_lock(&state.lock, K_FOREVER);

    /* Use provided config or defaults */
    if (config) {
        state.config = *config;
    } else {
        state.config = default_config;
    }

    /* Initialize Fusion AHRS */
    FusionAhrsInitialise(&state.ahrs);

    /* Configure AHRS settings */
    FusionAhrsSettings ahrs_settings = {
        .convention = FusionConventionNed,  /* North-East-Down (aviation standard) */
        .gain = state.config.gain,
        .gyroscopeRange = 2000.0f,  /* ICM-42688 configured to ±2000 dps */
        .accelerationRejection = state.config.acceleration_rejection,
        .magneticRejection = state.config.magnetic_rejection,
        .recoveryTriggerPeriod = (unsigned int)state.config.recovery_trigger_period
    };

    FusionAhrsSetSettings(&state.ahrs, &ahrs_settings);

    state.initialized = true;
    state.running = false;
    state.last_sample_us = 0;
    state.sample_count = 0;
    state.poll_count = 0;

    k_mutex_unlock(&state.lock);

    LOG_INF("Orientation service initialized (gain=%.2f)",
            (double)state.config.gain);

    return 0;
}

int orientation_start(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    if (state.running) {
        return -EALREADY;
    }

    LOG_INF("Starting orientation service");

    state.running = true;

    /* Create orientation thread */
    orientation_tid = k_thread_create(&orientation_thread,
                                      orientation_stack,
                                      K_THREAD_STACK_SIZEOF(orientation_stack),
                                      orientation_thread_fn,
                                      NULL, NULL, NULL,
                                      ORIENTATION_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(orientation_tid, "orientation");

    return 0;
}

int orientation_stop(void)
{
    if (!state.running) {
        return -EALREADY;
    }

    LOG_INF("Stopping orientation service");

    state.running = false;

    /* Wait for thread to exit */
    if (orientation_tid) {
        k_thread_join(orientation_tid, K_FOREVER);
        orientation_tid = NULL;
    }

    return 0;
}

int orientation_get_quaternion(orientation_quaternion_t *quat)
{
    if (!quat) {
        return -EINVAL;
    }

    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);

    FusionQuaternion fusion_quat = FusionAhrsGetQuaternion(&state.ahrs);

    quat->w = fusion_quat.element.w;
    quat->x = fusion_quat.element.x;
    quat->y = fusion_quat.element.y;
    quat->z = fusion_quat.element.z;

    k_mutex_unlock(&state.lock);

    return 0;
}

int orientation_get_euler(orientation_euler_t *euler)
{
    if (!euler) {
        return -EINVAL;
    }

    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);

    FusionQuaternion fusion_quat = FusionAhrsGetQuaternion(&state.ahrs);
    FusionEuler fusion_euler = FusionQuaternionToEuler(fusion_quat);

    euler->roll = fusion_euler.angle.roll;
    euler->pitch = fusion_euler.angle.pitch;
    euler->yaw = fusion_euler.angle.yaw;

    k_mutex_unlock(&state.lock);

    return 0;
}

int orientation_get_latest_imu(imu_sample_t *sample)
{
    if (!sample) {
        return -EINVAL;
    }

    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);
    *sample = state.latest_imu;
    k_mutex_unlock(&state.lock);

    return 0;
}

int orientation_reset(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);

    /* Reset to identity quaternion */
    FusionAhrsSetQuaternion(&state.ahrs, FUSION_QUATERNION_IDENTITY);

    /* Reset timing */
    state.last_sample_us = 0;

    k_mutex_unlock(&state.lock);

    LOG_INF("Orientation reset to identity");

    return 0;
}

int orientation_set_heading(float heading_deg)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);

    /* Get current orientation */
    FusionQuaternion current_quat = FusionAhrsGetQuaternion(&state.ahrs);
    FusionEuler current_euler = FusionQuaternionToEuler(current_quat);

    /* Calculate heading offset */
    float heading_offset = heading_deg - current_euler.angle.yaw;

    /* Create rotation quaternion for heading adjustment */
    float half_angle = FusionDegreesToRadians(heading_offset) * 0.5f;
    FusionQuaternion heading_rotation = {
        .element = {
            .w = cosf(half_angle),
            .x = 0.0f,
            .y = 0.0f,
            .z = sinf(half_angle)
        }
    };

    /* Apply heading rotation */
    FusionQuaternion adjusted_quat = FusionQuaternionProduct(heading_rotation, current_quat);
    FusionAhrsSetQuaternion(&state.ahrs, adjusted_quat);

    k_mutex_unlock(&state.lock);

    LOG_INF("Heading set to %.1f degrees", (double)heading_deg);

    return 0;
}

int orientation_get_config(orientation_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    k_mutex_lock(&state.lock, K_FOREVER);
    *config = state.config;
    k_mutex_unlock(&state.lock);

    return 0;
}

int orientation_set_config(const orientation_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    if (!state.initialized) {
        return -ENODEV;
    }

    k_mutex_lock(&state.lock, K_FOREVER);

    state.config = *config;

    /* Update AHRS settings */
    FusionAhrsSettings ahrs_settings = {
        .convention = FusionConventionNed,  /* North-East-Down (aviation standard) */
        .gain = config->gain,
        .gyroscopeRange = 2000.0f,  /* ICM-42688 configured to ±2000 dps */
        .accelerationRejection = config->acceleration_rejection,
        .magneticRejection = config->magnetic_rejection,
        .recoveryTriggerPeriod = (unsigned int)config->recovery_trigger_period
    };

    FusionAhrsSetSettings(&state.ahrs, &ahrs_settings);

    k_mutex_unlock(&state.lock);

    LOG_INF("Orientation config updated (gain=%.2f)", (double)config->gain);

    return 0;
}

bool orientation_is_ready(void)
{
    if (!state.initialized || !state.running) {
        return false;
    }

    return !FusionAhrsGetFlags(&state.ahrs).initialising;
}

uint32_t orientation_get_sample_count(void)
{
    return state.sample_count;
}
