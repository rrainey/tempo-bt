/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Orientation Test Program
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdio.h>
#include <math.h>

#include "services/imu.h"
#include "services/orientation.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(test_orientation, LOG_LEVEL_INF);

/* Test state */
static struct {
    bool running;
    bool log_quaternions;
    bool log_euler;
    uint32_t sample_count;
    uint64_t start_time;
} test_state;

/* Orientation visualization */
static void visualize_orientation(const orientation_euler_t *euler)
{
    /* Simple ASCII visualization of roll and pitch */
    const int width = 40;
    const int center = width / 2;
    
    /* Map angles to character positions (-180 to +180 degrees) */
    int roll_pos = center + (int)(euler->roll * center / 180.0f);
    int pitch_pos = center + (int)(euler->pitch * center / 180.0f);
    
    /* Clamp to valid range */
    roll_pos = MAX(0, MIN(width - 1, roll_pos));
    pitch_pos = MAX(0, MIN(width - 1, pitch_pos));
    
    /* Print roll indicator */
    printk("Roll  [");
    for (int i = 0; i < width; i++) {
        if (i == center) printk("|");
        else if (i == roll_pos) printk("*");
        else printk("-");
    }
    printk("] %6.1f°\n", euler->roll);
    
    /* Print pitch indicator */
    printk("Pitch [");
    for (int i = 0; i < width; i++) {
        if (i == center) printk("|");
        else if (i == pitch_pos) printk("*");
        else printk("-");
    }
    printk("] %6.1f°\n", euler->pitch);
    
    printk("Yaw: %6.1f°\n", euler->yaw);
}

/* IMU data callback for orientation updates */
static void orientation_test_imu_callback(const imu_sample_t *samples, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (!test_state.running) {
            break;
        }
        
        /* Update orientation */
        orientation_update(&samples[i]);
        
        test_state.sample_count++;
        
        /* Log quaternion data periodically */
        if (test_state.log_quaternions && (test_state.sample_count % 50 == 0)) {
            orientation_quaternion_t quat;
            orientation_get_quaternion(&quat);
            
            uint32_t time_ms = (samples[i].timestamp_us - test_state.start_time) / 1000;
            LOG_INF("Q: t=%u, w=%.4f, x=%.4f, y=%.4f, z=%.4f",
                    time_ms, quat.w, quat.x, quat.y, quat.z);
        }
        
        /* Log Euler angles periodically */
        if (test_state.log_euler && (test_state.sample_count % 100 == 0)) {
            orientation_euler_t euler;
            orientation_get_euler(&euler);
            
            LOG_INF("Euler: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°",
                    euler.roll, euler.pitch, euler.yaw);
        }
    }
}

/* Shell commands */
static int cmd_orient_test_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    shell_print(sh, "Starting orientation test...");
    
    /* Reset orientation */
    orientation_reset();
    
    /* Configure IMU for 200Hz */
    imu_config_t imu_cfg = {
        .accel_odr_hz = 200,
        .gyro_odr_hz = 200,
        .accel_range_g = 4,
        .gyro_range_dps = 500,
        .fifo_enabled = false
    };
    
    int ret = imu_configure(&imu_cfg);
    if (ret < 0) {
        shell_print(sh, "Failed to configure IMU: %d", ret);
        return ret;
    }
    
    /* Start IMU */
    test_state.running = true;
    test_state.sample_count = 0;
    test_state.start_time = time_now_us();
    test_state.log_quaternions = false;
    test_state.log_euler = true;
    
    ret = imu_start();
    if (ret < 0) {
        shell_print(sh, "Failed to start IMU: %d", ret);
        return ret;
    }
    
    shell_print(sh, "Orientation tracking started");
    shell_print(sh, "Move the device to see orientation changes");
    shell_print(sh, "Use 'orient test stop' to stop");
    
    return 0;
}

static int cmd_orient_test_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    test_state.running = false;
    imu_stop();
    
    shell_print(sh, "Orientation test stopped");
    shell_print(sh, "Total samples: %u", test_state.sample_count);
    
    return 0;
}

static int cmd_orient_test_reset(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    orientation_reset();
    shell_print(sh, "Orientation reset to identity");
    
    return 0;
}

static int cmd_orient_test_heading(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: orient test heading <degrees>");
        return -EINVAL;
    }
    
    float heading = atof(argv[1]);
    orientation_set_heading(heading);
    
    shell_print(sh, "Heading set to %.1f degrees", heading);
    
    return 0;
}

static int cmd_orient_test_config(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        /* Get current config */
        orientation_config_t config;
        orientation_get_config(&config);
        
        shell_print(sh, "Current configuration:");
        shell_print(sh, "  Gain: %.2f", config.gain);
        shell_print(sh, "  Sample period: %.4f s", config.sample_period);
        shell_print(sh, "  Acceleration rejection: %.1f m/s²", config.acceleration_rejection);
        shell_print(sh, "  Recovery period: %.1f s", config.recovery_trigger_period);
        
        return 0;
    }
    
    if (strcmp(argv[1], "gain") == 0 && argc >= 3) {
        orientation_config_t config;
        orientation_get_config(&config);
        config.gain = atof(argv[2]);
        orientation_set_config(&config);
        shell_print(sh, "Gain set to %.2f", config.gain);
    } else if (strcmp(argv[1], "rejection") == 0 && argc >= 3) {
        orientation_config_t config;
        orientation_get_config(&config);
        config.acceleration_rejection = atof(argv[2]);
        orientation_set_config(&config);
        shell_print(sh, "Acceleration rejection set to %.1f m/s²", config.acceleration_rejection);
    } else {
        shell_print(sh, "Usage: orient test config [gain <value>|rejection <value>]");
        return -EINVAL;
    }
    
    return 0;
}

static int cmd_orient_test_visualize(const struct shell *sh, size_t argc, char **argv)
{
    int duration = 10;  /* Default 10 seconds */
    
    if (argc >= 2) {
        duration = atoi(argv[1]);
        if (duration < 1 || duration > 60) {
            shell_print(sh, "Duration must be 1-60 seconds");
            return -EINVAL;
        }
    }
    
    shell_print(sh, "Visualizing orientation for %d seconds...", duration);
    shell_print(sh, "Press Ctrl+C to stop early");
    
    /* Make sure IMU is running */
    if (!test_state.running) {
        imu_config_t imu_cfg = {
            .accel_odr_hz = 100,
            .gyro_odr_hz = 100,
            .accel_range_g = 4,
            .gyro_range_dps = 500,
            .fifo_enabled = false
        };
        
        imu_configure(&imu_cfg);
        test_state.running = true;
        test_state.sample_count = 0;
        test_state.start_time = time_now_us();
        imu_start();
    }
    
    /* Visualize for specified duration */
    uint64_t end_time = k_uptime_get() + (duration * 1000);
    
    while (k_uptime_get() < end_time) {
        orientation_euler_t euler;
        orientation_get_euler(&euler);
        
        /* Clear screen (ANSI escape) */
        shell_print(sh, "\033[2J\033[H");
        
        /* Print visualization */
        visualize_orientation(&euler);
        
        /* Print quaternion */
        orientation_quaternion_t quat;
        orientation_get_quaternion(&quat);
        shell_print(sh, "\nQuaternion: [%.3f, %.3f, %.3f, %.3f]",
                    quat.w, quat.x, quat.y, quat.z);
        
        /* Check if still initializing */
        orientation_config_t config;
        orientation_get_config(&config);
        shell_print(sh, "\nStatus: %s (gain=%.1f)",
                    test_state.sample_count < 300 ? "Initializing" : "Tracking",
                    config.gain);
        
        k_sleep(K_MSEC(100));
    }
    
    return 0;
}

static int cmd_orient_test_log(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: orient test log <quaternion|euler|both|none>");
        return -EINVAL;
    }
    
    if (strcmp(argv[1], "quaternion") == 0) {
        test_state.log_quaternions = true;
        test_state.log_euler = false;
    } else if (strcmp(argv[1], "euler") == 0) {
        test_state.log_quaternions = false;
        test_state.log_euler = true;
    } else if (strcmp(argv[1], "both") == 0) {
        test_state.log_quaternions = true;
        test_state.log_euler = true;
    } else if (strcmp(argv[1], "none") == 0) {
        test_state.log_quaternions = false;
        test_state.log_euler = false;
    } else {
        shell_print(sh, "Invalid option. Use: quaternion, euler, both, or none");
        return -EINVAL;
    }
    
    shell_print(sh, "Logging configured");
    
    return 0;
}

/* Static calibration test */
static int cmd_orient_test_calibrate(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    shell_print(sh, "Static calibration test");
    shell_print(sh, "Place device on flat surface and keep stationary...");
    k_sleep(K_SECONDS(2));
    
    /* Collect samples */
    float accel_sum[3] = {0};
    float gyro_sum[3] = {0};
    int sample_count = 0;
    
    test_state.running = true;
    test_state.sample_count = 0;
    imu_start();
    
    /* Collect for 3 seconds */
    uint64_t start = k_uptime_get();
    while (k_uptime_get() - start < 3000) {
        k_sleep(K_MSEC(100));
        
        /* In a real implementation, we'd collect samples directly */
        /* For now, just show the concept */
        sample_count = test_state.sample_count;
    }
    
    test_state.running = false;
    imu_stop();
    
    shell_print(sh, "Collected %d samples", sample_count);
    
    /* Get current orientation */
    orientation_euler_t euler;
    orientation_get_euler(&euler);
    
    shell_print(sh, "Current orientation:");
    shell_print(sh, "  Roll: %.1f°", euler.roll);
    shell_print(sh, "  Pitch: %.1f°", euler.pitch);
    shell_print(sh, "  Yaw: %.1f°", euler.yaw);
    
    /* Check if level */
    if (fabs(euler.roll) < 5.0f && fabs(euler.pitch) < 5.0f) {
        shell_print(sh, "Device appears level - good for calibration");
    } else {
        shell_print(sh, "WARNING: Device not level!");
    }
    
    return 0;
}

/* Shell command registration */
SHELL_STATIC_SUBCMD_SET_CREATE(orient_test_cmds,
    SHELL_CMD(start, NULL, "Start orientation tracking", cmd_orient_test_start),
    SHELL_CMD(stop, NULL, "Stop orientation tracking", cmd_orient_test_stop),
    SHELL_CMD(reset, NULL, "Reset orientation to identity", cmd_orient_test_reset),
    SHELL_CMD(heading, NULL, "Set heading reference", cmd_orient_test_heading),
    SHELL_CMD(config, NULL, "Get/set configuration", cmd_orient_test_config),
    SHELL_CMD(visualize, NULL, "Visualize orientation", cmd_orient_test_visualize),
    SHELL_CMD(log, NULL, "Configure logging output", cmd_orient_test_log),
    SHELL_CMD(calibrate, NULL, "Run calibration test", cmd_orient_test_calibrate),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(orient_test, &orient_test_cmds, "Orientation test commands", NULL);

/* Initialization */
int orientation_test_init(void)
{
    int ret;
    
    LOG_INF("Initializing orientation test module");
    
    /* Initialize IMU if not already done */
    ret = imu_init();
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("Failed to initialize IMU: %d", ret);
        return ret;
    }
    
    /* Initialize orientation service if not already done */
    orientation_config_t config = {
        .gain = 2.5f,
        .sample_period = 0.005f,  /* 200Hz default */
        .use_magnetometer = false,
        .acceleration_rejection = 10.0f,
        .magnetic_rejection = 10.0f,
        .recovery_trigger_period = 5.0f
    };
    
    ret = orientation_init(&config);
    if (ret < 0 && ret != -EALREADY) {
        LOG_ERR("Failed to initialize orientation service: %d", ret);
        return ret;
    }
    
    /* DON'T register IMU callback - let aggregator handle it */
    /* The aggregator already calls orientation_update() */
    
    /* Initialize state */
    memset(&test_state, 0, sizeof(test_state));
    test_state.log_euler = true;
    
    LOG_INF("Orientation test module ready");
    LOG_INF("Commands:");
    LOG_INF("  orient_test start     - Start tracking");
    LOG_INF("  orient_test stop      - Stop tracking");
    LOG_INF("  orient_test visualize - Real-time visualization");
    LOG_INF("  orient_test reset     - Reset to identity");
    LOG_INF("  orient_test config    - View/change settings");
    
    return 0;
}

/* Call this from main() after imu_init() */
// orientation_test_init();