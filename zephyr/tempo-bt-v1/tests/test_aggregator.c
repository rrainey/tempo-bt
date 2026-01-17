/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Test Aggregator Service
 */
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "services/aggregator.h"
#include "services/timebase.h"
#include "services/imu.h"
#include "services/baro.h"
#include "services/gnss.h"

LOG_MODULE_REGISTER(test_aggregator, LOG_LEVEL_INF);

static int line_count = 0;
static int ver_count = 0;
static int sfc_count = 0;
static int imu_count = 0;
static int im2_count = 0;
static int env_count = 0;
static int th_count = 0;
static int st_count = 0;

/* Output callback - just log the lines and count them */
static void test_output_callback(const char *line, size_t len)
{
    /* Remove CRLF for logging */
    char log_line[LOG_MAX_SENTENCE_LEN];
    if (len > 2 && len < sizeof(log_line)) {
        memcpy(log_line, line, len - 2);  /* Skip CRLF */
        log_line[len - 2] = '\0';
        
        /* Count sentence types */
        if (strncmp(line, "$PVER", 5) == 0) ver_count++;
        else if (strncmp(line, "$PSFC", 5) == 0) sfc_count++;
        else if (strncmp(line, "$PIMU", 5) == 0) imu_count++;
        else if (strncmp(line, "$PIM2", 5) == 0) im2_count++;
        else if (strncmp(line, "$PENV", 5) == 0) env_count++;
        else if (strncmp(line, "$PTH", 4) == 0) th_count++;
        else if (strncmp(line, "$PST", 4) == 0) st_count++;
        
        /* Log every 10th line to reduce output */
        if (++line_count % 10 == 1) {
            LOG_INF("Line %d: %s", line_count, log_line);
        }
    }
}

int test_aggregator(void)
{
    int ret;
    aggregator_config_t config;
    
    LOG_INF("=== Aggregator Service Test ===");
    
    /* Initialize aggregator */
    ret = aggregator_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize aggregator: %d", ret);
        return ret;
    }
    
    /* Register output callback */
    aggregator_register_output_callback(test_output_callback);
    
    /* Configure aggregator */
    config.imu_output_rate = 50;     /* 50 Hz */
    config.env_output_rate = 4;      /* 4 Hz */
    config.gnss_output_rate = 1;     /* 1 Hz */
    config.mag_output_rate = 0;      /* Disabled */
    config.enable_quaternion = true;
    config.enable_magnetometer = false;
    config.session_id = 12345678;
    config.session_start_us = time_now_us();
    
    ret = aggregator_configure(&config);
    if (ret < 0) {
        LOG_ERR("Failed to configure aggregator: %d", ret);
        return ret;
    }
    
    /* Write header sentences */
    aggregator_write_version();
    aggregator_write_session_config();
    
    /* Start sensor services (if not already running) */
    LOG_INF("Starting sensors...");
    
    /* IMU might have hardware issues, so check if it's initialized */
    imu_config_t imu_cfg;
    if (imu_get_config(&imu_cfg) == 0) {
        ret = imu_start();
        if (ret < 0) {
            LOG_WRN("Failed to start IMU: %d (continuing anyway)", ret);
        }
    } else {
        LOG_WRN("IMU not initialized, skipping");
    }
    
    /* Start barometer */
    ret = baro_start();
    if (ret < 0) {
        LOG_ERR("Failed to start barometer: %d", ret);
        return ret;
    }
    
    /* GNSS is disabled for now */
    LOG_INF("GNSS disabled for this test");
    
    /* Start aggregator */
    ret = aggregator_start();
    if (ret < 0) {
        LOG_ERR("Failed to start aggregator: %d", ret);
        return ret;
    }
    
    /* Simulate state changes */
    k_sleep(K_SECONDS(1));
    aggregator_write_state_change("IDLE", "ARMED", "USER");
    
    k_sleep(K_SECONDS(1));
    aggregator_write_state_change("ARMED", "LOGGING", "FREEFALL");
    
    /* Let it run for 10 seconds */
    LOG_INF("Collecting data for 10 seconds...");
    k_sleep(K_SECONDS(10));
    
    /* Stop everything */
    LOG_INF("Stopping aggregator...");
    ret = aggregator_stop();
    if (ret < 0) {
        LOG_ERR("Failed to stop aggregator: %d", ret);
    }
    
    /* Stop sensors */
    baro_stop();
    if (imu_get_config(&imu_cfg) == 0) {
        imu_stop();
    }
    
    /* Get statistics */
    uint32_t imu_samples, env_samples, gnss_samples, dropped;
    aggregator_get_stats(&imu_samples, &env_samples, &gnss_samples, &dropped);
    
    LOG_INF("Aggregator test completed");
    LOG_INF("Statistics:");
    LOG_INF("  Sensor samples: IMU=%u, ENV=%u, GNSS=%u", 
            imu_samples, env_samples, gnss_samples);
    LOG_INF("  Dropped samples: %u", dropped);
    LOG_INF("  Output lines: Total=%d", line_count);
    LOG_INF("  By type: VER=%d, SFC=%d, IMU=%d, IM2=%d, ENV=%d, TH=%d, ST=%d",
            ver_count, sfc_count, imu_count, im2_count, env_count, th_count, st_count);
    
    /* Expected output rates over 10 seconds:
     * IMU: 40 Hz * 10s = 400 lines
     * IM2: 40 Hz * 10s = 400 lines (if quaternion enabled)
     * ENV: 4 Hz * 10s = 40 lines
     * Plus headers and state changes
     */
    LOG_INF("Expected: ~400 IMU, ~400 IM2, ~40 ENV");
    
    return 0;
}