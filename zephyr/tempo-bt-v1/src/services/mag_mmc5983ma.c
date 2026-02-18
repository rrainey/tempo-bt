/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - Magnetometer Service Implementation
 *
 * Provides calibrated magnetometer readings from MMC5983MA
 * in formats suitable for AHRS sensor fusion.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

#include "services/mag.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(mag_service, LOG_LEVEL_INF);

/* Device reference */
#define MAG_NODE DT_NODELABEL(mmc5983ma)

#if !DT_NODE_EXISTS(MAG_NODE)
/* Provide stub implementations when no magnetometer is present */
#warning "MMC5983MA not defined in devicetree - magnetometer service disabled"

int mag_init(void) { return -ENODEV; }
int mag_configure(const mag_config_t *config) { return -ENODEV; }
int mag_get_config(mag_config_t *config) { return -ENODEV; }
int mag_read(mag_sample_t *sample) { return -ENODEV; }
int mag_read_fusion(FusionVector *vec) { return -ENODEV; }
int mag_set(void) { return -ENODEV; }
int mag_reset(void) { return -ENODEV; }
int mag_cal_start(void) { return -ENODEV; }
int mag_cal_add_sample(void) { return -ENODEV; }
int mag_cal_get_sample_count(void) { return -ENODEV; }
int mag_cal_compute(void) { return -ENODEV; }
int mag_cal_get(mag_calibration_t *cal) { return -ENODEV; }
int mag_cal_set(const mag_calibration_t *cal) { return -ENODEV; }
int mag_cal_save(void) { return -ENODEV; }
int mag_cal_load(void) { return -ENODEV; }
int mag_cal_clear(void) { return -ENODEV; }
bool mag_is_ready(void) { return false; }
bool mag_is_calibrated(void) { return false; }

#else /* DT_NODE_EXISTS(MAG_NODE) */

static const struct device *mag_dev;

/* Service state */
static struct {
    bool initialized;
    mag_config_t config;
    mag_calibration_t cal;

    /* Calibration collection state */
    bool cal_collecting;
    int32_t cal_min_x, cal_max_x;
    int32_t cal_min_y, cal_max_y;
    int32_t cal_min_z, cal_max_z;
    uint32_t cal_sample_count;
} state;

/* Default configuration */
static const mag_config_t default_config = {
    .odr_hz = 0,              /* Single-shot mode */
    .auto_set_reset = true,   /* Enable auto SR for best accuracy */
    .bandwidth = 0,           /* 100Hz bandwidth, lowest noise */
};

/* Counts to Gauss conversion (18-bit, 16384 counts/Gauss) */
#define COUNTS_TO_GAUSS(x) ((float)(x) / 16384.0f)

/*
 * Settings subsystem handler for calibration persistence
 */
#ifdef CONFIG_SETTINGS
static int mag_settings_set(const char *name, size_t len,
                            settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "cal", &next) && !next) {
        if (len != sizeof(state.cal)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &state.cal, sizeof(state.cal));
        if (rc >= 0) {
            LOG_INF("Loaded calibration from settings");
            return 0;
        }
        return rc;
    }

    return -ENOENT;
}

static int mag_settings_export(int (*cb)(const char *name,
                                         const void *value, size_t val_len))
{
    if (state.cal.valid) {
        return cb("mag/cal", &state.cal, sizeof(state.cal));
    }
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(mag, "mag", NULL, mag_settings_set,
                               NULL, mag_settings_export);
#endif /* CONFIG_SETTINGS */

/*
 * Initialize the magnetometer service
 */
int mag_init(void)
{
    int ret;

    LOG_INF("Initializing magnetometer service");

    /* Get device reference */
    mag_dev = DEVICE_DT_GET(MAG_NODE);
    if (!device_is_ready(mag_dev)) {
        LOG_ERR("Magnetometer device not ready");
        return -ENODEV;
    }

    /* Initialize state */
    memset(&state, 0, sizeof(state));
    state.config = default_config;

    /* Initialize calibration to identity (no correction) */
    state.cal.scale_x = 32768;
    state.cal.scale_y = 32768;
    state.cal.scale_z = 32768;
    state.cal.valid = false;

    /* Load calibration from settings if available */
#ifdef CONFIG_SETTINGS
    ret = mag_cal_load();
    if (ret == 0 && state.cal.valid) {
        LOG_INF("Magnetometer calibration loaded");
    }
#endif

    state.initialized = true;
    LOG_INF("Magnetometer service initialized");

    return 0;
}

/*
 * Configure magnetometer parameters
 */
int mag_configure(const mag_config_t *config)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    if (!config) {
        return -EINVAL;
    }

    /* Apply ODR setting */
    if (config->odr_hz != state.config.odr_hz) {
        struct sensor_value val = { .val1 = config->odr_hz, .val2 = 0 };
        int ret = sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
                                  SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
        if (ret < 0) {
            LOG_WRN("Failed to set ODR: %d", ret);
        }
    }

    state.config = *config;
    return 0;
}

/*
 * Get current configuration
 */
int mag_get_config(mag_config_t *config)
{
    if (!config) {
        return -EINVAL;
    }

    *config = state.config;
    return 0;
}

/*
 * Read raw values from driver and convert to 18-bit signed counts
 */
static int mag_read_raw(int32_t *x, int32_t *y, int32_t *z)
{
    struct sensor_value val[3];
    int ret;

    /* Fetch new sample */
    ret = sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_MAGN_XYZ);
    if (ret < 0) {
        return ret;
    }

    /* Get values (driver returns Gauss as sensor_value) */
    ret = sensor_channel_get(mag_dev, SENSOR_CHAN_MAGN_XYZ, val);
    if (ret < 0) {
        return ret;
    }

    /* Convert back to counts for calibration processing
     * sensor_value is in Gauss: val1 + val2/1000000
     */
    float gx = (float)val[0].val1 + (float)val[0].val2 / 1000000.0f;
    float gy = (float)val[1].val1 + (float)val[1].val2 / 1000000.0f;
    float gz = (float)val[2].val1 + (float)val[2].val2 / 1000000.0f;

    /* Convert to raw counts (16384 counts/Gauss) */
    *x = (int32_t)(gx * 16384.0f);
    *y = (int32_t)(gy * 16384.0f);
    *z = (int32_t)(gz * 16384.0f);

    return 0;
}

/*
 * Apply calibration to raw counts
 */
static void apply_calibration(int32_t raw_x, int32_t raw_y, int32_t raw_z,
                              float *cal_x, float *cal_y, float *cal_z)
{
    int32_t x = raw_x;
    int32_t y = raw_y;
    int32_t z = raw_z;

    if (state.cal.valid) {
        /* Apply hard-iron offset correction */
        x -= state.cal.offset_x;
        y -= state.cal.offset_y;
        z -= state.cal.offset_z;

        /* Apply soft-iron scale correction (Q1.15) */
        x = (x * state.cal.scale_x) >> 15;
        y = (y * state.cal.scale_y) >> 15;
        z = (z * state.cal.scale_z) >> 15;
    }

    /* Convert to Gauss */
    *cal_x = COUNTS_TO_GAUSS(x);
    *cal_y = COUNTS_TO_GAUSS(y);
    *cal_z = COUNTS_TO_GAUSS(z);
}

/*
 * Take a single magnetometer reading
 */
int mag_read(mag_sample_t *sample)
{
    int ret;
    int32_t raw_x, raw_y, raw_z;

    if (!state.initialized) {
        return -ENODEV;
    }

    if (!sample) {
        return -EINVAL;
    }

    /* Read raw values */
    ret = mag_read_raw(&raw_x, &raw_y, &raw_z);
    if (ret < 0) {
        return ret;
    }

    /* Get timestamp */
    sample->timestamp_us = k_ticks_to_us_floor64(k_uptime_ticks());

    /* Apply calibration */
    apply_calibration(raw_x, raw_y, raw_z,
                      &sample->mag_x, &sample->mag_y, &sample->mag_z);

    /* Calculate magnitude */
    sample->magnitude = sqrtf(sample->mag_x * sample->mag_x +
                              sample->mag_y * sample->mag_y +
                              sample->mag_z * sample->mag_z);

    /* Calculate normalized vector */
    if (sample->magnitude > 0.001f) {
        float inv_mag = 1.0f / sample->magnitude;
        sample->norm_x = sample->mag_x * inv_mag;
        sample->norm_y = sample->mag_y * inv_mag;
        sample->norm_z = sample->mag_z * inv_mag;
    } else {
        sample->norm_x = 0.0f;
        sample->norm_y = 0.0f;
        sample->norm_z = 0.0f;
    }

    /* Read temperature */
    struct sensor_value temp_val;
    ret = sensor_sample_fetch_chan(mag_dev, SENSOR_CHAN_DIE_TEMP);
    if (ret == 0) {
        ret = sensor_channel_get(mag_dev, SENSOR_CHAN_DIE_TEMP, &temp_val);
        if (ret == 0) {
            sample->temperature = (float)temp_val.val1 +
                                  (float)temp_val.val2 / 1000000.0f;
        }
    }

    sample->calibrated = state.cal.valid;

    /* If collecting calibration samples, add this one */
    if (state.cal_collecting) {
        if (raw_x < state.cal_min_x) state.cal_min_x = raw_x;
        if (raw_x > state.cal_max_x) state.cal_max_x = raw_x;
        if (raw_y < state.cal_min_y) state.cal_min_y = raw_y;
        if (raw_y > state.cal_max_y) state.cal_max_y = raw_y;
        if (raw_z < state.cal_min_z) state.cal_min_z = raw_z;
        if (raw_z > state.cal_max_z) state.cal_max_z = raw_z;
        state.cal_sample_count++;
    }

    return 0;
}

/*
 * Get magnetometer reading as FusionVector
 */
int mag_read_fusion(FusionVector *vec)
{
    mag_sample_t sample;
    int ret;

    if (!vec) {
        return -EINVAL;
    }

    ret = mag_read(&sample);
    if (ret < 0) {
        return ret;
    }

    /* Return normalized vector for AHRS */
    vec->axis.x = sample.norm_x;
    vec->axis.y = sample.norm_y;
    vec->axis.z = sample.norm_z;

    return 0;
}

/*
 * Perform SET operation
 */
int mag_set(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    /* Use custom attribute if available, otherwise use driver directly */
    struct sensor_value val = { 0 };
    return sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
                           SENSOR_ATTR_PRIV_START, &val);
}

/*
 * Perform RESET operation
 */
int mag_reset(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    struct sensor_value val = { 0 };
    return sensor_attr_set(mag_dev, SENSOR_CHAN_MAGN_XYZ,
                           SENSOR_ATTR_PRIV_START + 1, &val);
}

/*
 * Start calibration sample collection
 */
int mag_cal_start(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    /* Initialize min/max trackers */
    state.cal_min_x = INT32_MAX;
    state.cal_max_x = INT32_MIN;
    state.cal_min_y = INT32_MAX;
    state.cal_max_y = INT32_MIN;
    state.cal_min_z = INT32_MAX;
    state.cal_max_z = INT32_MIN;
    state.cal_sample_count = 0;
    state.cal_collecting = true;

    LOG_INF("Calibration collection started");
    return 0;
}

/*
 * Add current reading to calibration dataset
 */
int mag_cal_add_sample(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    if (!state.cal_collecting) {
        return -EINVAL;
    }

    /* mag_read() automatically adds to calibration when cal_collecting is true */
    mag_sample_t sample;
    return mag_read(&sample);
}

/*
 * Get number of calibration samples collected
 */
int mag_cal_get_sample_count(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    return state.cal_sample_count;
}

/*
 * Compute and apply calibration
 */
int mag_cal_compute(void)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    if (!state.cal_collecting) {
        return -EINVAL;
    }

    if (state.cal_sample_count < 100) {
        LOG_WRN("Insufficient samples for calibration: %u", state.cal_sample_count);
        return -EINVAL;
    }

    state.cal_collecting = false;

    /* Compute hard-iron offsets (center of sphere) */
    state.cal.offset_x = (state.cal_min_x + state.cal_max_x) / 2;
    state.cal.offset_y = (state.cal_min_y + state.cal_max_y) / 2;
    state.cal.offset_z = (state.cal_min_z + state.cal_max_z) / 2;

    /* Compute soft-iron scale factors
     * Normalize to the average radius
     */
    int32_t range_x = state.cal_max_x - state.cal_min_x;
    int32_t range_y = state.cal_max_y - state.cal_min_y;
    int32_t range_z = state.cal_max_z - state.cal_min_z;

    if (range_x == 0 || range_y == 0 || range_z == 0) {
        LOG_ERR("Invalid calibration data (zero range)");
        return -EINVAL;
    }

    int32_t avg_range = (range_x + range_y + range_z) / 3;

    /* Scale factors in Q1.15 format */
    state.cal.scale_x = (uint16_t)((avg_range * 32768LL) / range_x);
    state.cal.scale_y = (uint16_t)((avg_range * 32768LL) / range_y);
    state.cal.scale_z = (uint16_t)((avg_range * 32768LL) / range_z);

    state.cal.valid = true;

    LOG_INF("Calibration computed from %u samples:", state.cal_sample_count);
    LOG_INF("  Offsets: [%d, %d, %d]",
            state.cal.offset_x, state.cal.offset_y, state.cal.offset_z);
    LOG_INF("  Scales:  [%u, %u, %u] (Q1.15)",
            state.cal.scale_x, state.cal.scale_y, state.cal.scale_z);

    return 0;
}

/*
 * Get current calibration data
 */
int mag_cal_get(mag_calibration_t *cal)
{
    if (!cal) {
        return -EINVAL;
    }

    *cal = state.cal;
    return 0;
}

/*
 * Set calibration data
 */
int mag_cal_set(const mag_calibration_t *cal)
{
    if (!state.initialized) {
        return -ENODEV;
    }

    if (!cal) {
        return -EINVAL;
    }

    state.cal = *cal;
    LOG_INF("Calibration set manually");
    return 0;
}

/*
 * Save calibration to persistent storage
 */
int mag_cal_save(void)
{
#ifdef CONFIG_SETTINGS
    if (!state.cal.valid) {
        LOG_WRN("No valid calibration to save");
        return -EINVAL;
    }

    int ret = settings_save_one("mag/cal", &state.cal, sizeof(state.cal));
    if (ret < 0) {
        LOG_ERR("Failed to save calibration: %d", ret);
        return ret;
    }

    LOG_INF("Calibration saved to settings");
    return 0;
#else
    return -ENOTSUP;
#endif
}

/*
 * Load calibration from persistent storage
 */
int mag_cal_load(void)
{
#ifdef CONFIG_SETTINGS
    int ret = settings_load_subtree("mag");
    if (ret < 0) {
        LOG_DBG("No calibration in settings");
        return ret;
    }
    return 0;
#else
    return -ENOTSUP;
#endif
}

/*
 * Clear calibration
 */
int mag_cal_clear(void)
{
    state.cal.offset_x = 0;
    state.cal.offset_y = 0;
    state.cal.offset_z = 0;
    state.cal.scale_x = 32768;
    state.cal.scale_y = 32768;
    state.cal.scale_z = 32768;
    state.cal.valid = false;

    LOG_INF("Calibration cleared");
    return 0;
}

/*
 * Check if magnetometer is initialized and ready
 */
bool mag_is_ready(void)
{
    return state.initialized;
}

/*
 * Check if calibration is valid
 */
bool mag_is_calibrated(void)
{
    return state.cal.valid;
}

#endif /* DT_NODE_EXISTS(MAG_NODE) */
