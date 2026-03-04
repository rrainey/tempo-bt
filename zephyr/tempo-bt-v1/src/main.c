/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Main Application Entry
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <string.h>

#include "app_init.h"
#include "app/events.h"
#include "config/settings.h"

#include "services/timebase.h"
#include "services/storage.h"
#include "services/file_writer.h"
#include "services/gnss.h"
#include "services/imu.h"
#include "services/baro.h"
#include "services/logger.h"
#include "services/led.h"
#include "services/aggregator.h"
#include "services/orientation.h"
#if CONFIG_MMC5983MA
#include "services/mag.h"
#endif
#include "util/nmea_checksum.h"
#include "app/log_format.h"
#ifdef CONFIG_USB_TTY_OUTPUT
#include "services/usb_tty.h"
#include "ble_mcumgr.h"
#endif

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

 /* Register barometer callback with logger for takeoff detection */
//extern void logger_register_baro_callback(baro_data_callback_t callback);
//extern void logger_baro_handler(const baro_sample_t *sample);

/* Also register with aggregator for logging */
extern void aggregator_register_baro_callback(baro_data_callback_t callback);

/*
 * Button handling for logger control
 */

#define BUTTON_DEBOUNCE_DELAY_MS 50
#define BUTTON_LONG_PRESS_MS     2000

/* Button GPIO specs from device tree */
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#define LED0_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button0_cb_data;
static struct k_work_delayable button0_work;
static struct k_work button0_action_work;  /* For logger start/stop (can't run in ISR) */
static int64_t button0_press_time;
static bool button0_pending_start;  /* true = start, false = stop */
#endif

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static struct gpio_callback button1_cb_data;
static struct k_work_delayable button1_work;
static struct k_work button1_action_work;
static int64_t button1_press_time;
#endif

/* Forward declaration for LED state update */
static void update_led_for_state(logger_state_t state);

/* Magnetometer calibration streaming state */
#if CONFIG_MMC5983MA && defined(CONFIG_USB_TTY_OUTPUT)
static volatile bool mag_cal_streaming;
static struct k_work button1_short_press_work;
static struct k_work_delayable mag_cal_stream_work;

#define MAG_CAL_STREAM_PERIOD_MS  50  /* 20 Hz raw mag output */

static void mag_cal_stream_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (!mag_cal_streaming) {
        return;
    }

    int32_t raw_x, raw_y, raw_z;
    float temp_c;
    int ret = mag_read_raw_counts(&raw_x, &raw_y, &raw_z, &temp_c);

    if (ret == 0) {
        char line[LOG_MAX_SENTENCE_LEN];
        uint32_t timestamp_ms = (uint32_t)(k_uptime_get() & 0xFFFFFFFF);
        int len = log_format_sentence(line, sizeof(line),
                                      "$PRMG,%u,%d,%d,%d,%.1f",
                                      timestamp_ms,
                                      raw_x, raw_y, raw_z, (double)temp_c);
        if (len > 0) {
            usb_tty_output_line(line, len);
        }
    }

    k_work_reschedule(&mag_cal_stream_work, K_MSEC(MAG_CAL_STREAM_PERIOD_MS));
}

static void button1_short_press_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (mag_cal_streaming) {
        /* Stop calibration streaming */
        mag_cal_streaming = false;
        k_work_cancel_delayable(&mag_cal_stream_work);
        usb_tty_close();
        LOG_INF("Mag calibration streaming stopped");
        ble_mcumgr_restart();
        update_led_for_state(logger_get_state());
    } else {
        /* Start calibration streaming */
        if (!mag_is_ready()) {
            LOG_WRN("Magnetometer not ready for calibration");
            return;
        }
        int ret = usb_tty_open();
        if (ret == -ENOTCONN) {
            LOG_INF("BTN1 short: no USB host connected, ignoring");
            return;
        }
        mag_cal_streaming = true;
        LOG_INF("Mag calibration streaming started (20 Hz $PRMG)");
        ble_mcumgr_stop();
        set_color_led_state(RGB_MAGENTA, true);
        k_work_schedule(&mag_cal_stream_work, K_NO_WAIT);
    }
}
#endif /* CONFIG_MMC5983MA && CONFIG_USB_TTY_OUTPUT */

/* Button work handler for long press detection */
static void button0_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    /* Check if button is still pressed */
    int val = gpio_pin_get_dt(&button0);
    if (val == 1) { /* Active low, so 1 means pressed */
        int64_t press_duration = k_uptime_get() - button0_press_time;

        if (press_duration >= BUTTON_LONG_PRESS_MS) {
            LOG_INF("Button 0 long press detected");

            /* Long press: manually start logging when armed
             * This is a safety feature - requires deliberate action to start logging */
            logger_state_t state = logger_get_state();
            if (state == LOGGER_STATE_ARMED) {
                button0_pending_start = true;
                k_work_submit(&button0_action_work);
            }
        }
    }
}

/* Button action work handler - runs logger start/stop outside ISR context */
static void button0_action_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (button0_pending_start) {
        int ret = logger_start();
        if (ret == 0) {
            /* Update LED to reflect new logging state */
            update_led_for_state(logger_get_state());
        }
    } else {
        logger_stop();
    }
}

/* Button interrupt callbacks */
static void button0_pressed(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    button0_press_time = k_uptime_get();
    
    /* Schedule work to check for long press */
    k_work_reschedule(&button0_work, K_MSEC(BUTTON_LONG_PRESS_MS));
}

static void button0_released(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Cancel long press detection */
    k_work_cancel_delayable(&button0_work);

    int64_t press_duration = k_uptime_get() - button0_press_time;

    if (press_duration < BUTTON_LONG_PRESS_MS &&
        press_duration > BUTTON_DEBOUNCE_DELAY_MS) {
        /* Short press: no effect (safety feature)
         * Only long press can start logging to prevent accidental activation */
        LOG_DBG("Button 0 short press - ignored");
    }
}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
/* Button 1 long press detection — mirrors button 0 pattern */
static void button1_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int val = gpio_pin_get_dt(&button1);
    if (val == 1) {
        int64_t press_duration = k_uptime_get() - button1_press_time;

        if (press_duration >= BUTTON_LONG_PRESS_MS) {
            LOG_INF("Button 1 long press detected");
            k_work_submit(&button1_action_work);
        }
    }
}

/* Button 1 (BTN2) action — runs outside ISR context */
static void button1_action_handler(struct k_work *work)
{
    ARG_UNUSED(work);

#ifdef CONFIG_USB_TTY_OUTPUT
    logger_state_t state = logger_get_state();

    if (state == LOGGER_STATE_LOGGING || state == LOGGER_STATE_JUMPED) {
        /* Stop current session (works for both USB and SD modes) */
        logger_stop();
        update_led_for_state(logger_get_state());
    } else if (state == LOGGER_STATE_IDLE || state == LOGGER_STATE_ARMED) {
        int ret = logger_start_usb();
        if (ret == -ENOTCONN) {
            LOG_INF("BTN1: no USB host connected, ignoring");
        } else if (ret == 0) {
            update_led_for_state(logger_get_state());
        } else {
            LOG_WRN("BTN1: USB logging start failed: %d", ret);
        }
    }
#else
    LOG_INF("Button 1 pressed (USB TTY not enabled)");
#endif
}

static void button1_isr(const struct device *dev, struct gpio_callback *cb,
                        uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int val = gpio_pin_get_dt(&button1);
    if (val == 1) {
        /* Pressed */
        button1_press_time = k_uptime_get();
        k_work_reschedule(&button1_work, K_MSEC(BUTTON_LONG_PRESS_MS));
    } else {
        /* Released — cancel long press detection */
        k_work_cancel_delayable(&button1_work);

        /* Detect short press for mag calibration toggle */
        int64_t press_duration = k_uptime_get() - button1_press_time;
        if (press_duration < BUTTON_LONG_PRESS_MS &&
            press_duration > BUTTON_DEBOUNCE_DELAY_MS) {
#if CONFIG_MMC5983MA && defined(CONFIG_USB_TTY_OUTPUT)
            /* Short press: toggle mag calibration streaming
             * (only when not actively logging) */
            logger_state_t state = logger_get_state();
            if (mag_cal_streaming ||
                state == LOGGER_STATE_IDLE ||
                state == LOGGER_STATE_ARMED) {
                k_work_submit(&button1_short_press_work);
            }
#endif
        }
    }
}
#endif

static void button0_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int val = gpio_pin_get_dt(&button0);
    if (val == 1) { /* Pressed (active low) */
        button0_pressed(dev, cb, pins);
    } else {
        button0_released(dev, cb, pins);
    }
}

/* Initialize button handling */
int buttons_init(void)
{
    int ret;
    
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    if (!device_is_ready(button0.port)) {
        LOG_ERR("Button 0 device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 0: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 0 interrupt: %d", ret);
        return ret;
    }
    
    k_work_init_delayable(&button0_work, button0_work_handler);
    k_work_init(&button0_action_work, button0_action_handler);

    gpio_init_callback(&button0_cb_data,
                       button0_isr,
                       BIT(button0.pin));
    
    gpio_add_callback(button0.port, &button0_cb_data);
    
#endif

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
    if (!device_is_ready(button1.port)) {
        LOG_ERR("Button 1 device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 1: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 1 interrupt: %d", ret);
        return ret;
    }

    k_work_init_delayable(&button1_work, button1_work_handler);
    k_work_init(&button1_action_work, button1_action_handler);
#if CONFIG_MMC5983MA && defined(CONFIG_USB_TTY_OUTPUT)
    k_work_init(&button1_short_press_work, button1_short_press_handler);
    k_work_init_delayable(&mag_cal_stream_work, mag_cal_stream_handler);
#endif

    gpio_init_callback(&button1_cb_data, button1_isr, BIT(button1.pin));
    gpio_add_callback(button1.port, &button1_cb_data);
    
#endif

    return 0;
}

/* State change handler */
static void update_led_for_state(logger_state_t state)
{
    switch (state) {
    case LOGGER_STATE_IDLE:
        /* Blue slow blink for idle */
        set_color_led_state(RGB_ORANGE, true);
        LOG_INF("LED: Orange (idle)");
        break;

    case LOGGER_STATE_ARMED:
        /* Orange slow blink for armed */
        set_color_led_state(RGB_BLUE, true);
        LOG_INF("LED: Blue (armed)");
        break;

    case LOGGER_STATE_LOGGING:
        /* Green slow blink for logging (climbing) */
        set_color_led_state(RGB_GREEN, true);
        LOG_INF("LED: Green (logging)");
        break;

    case LOGGER_STATE_JUMPED:
        /* Cyan slow blink for active jump (freefall/canopy) */
        set_color_led_state(RGB_CYAN, true);
        LOG_INF("LED: Cyan (jumped)");
        break;

    case LOGGER_STATE_ERROR:
    default:
        /* Red slow blink for error */
        set_color_led_state(RGB_RED, true);
        LOG_INF("LED: Red (error)");
        break;
    }
}

/* Event handler for state changes */
static void led_event_handler(const app_event_t *event, void *user_data)
{
    ARG_UNUSED(user_data);
    
    switch (event->type) {
    case EVT_STATE_CHANGE:
        /* Update LED based on logger state */
        update_led_for_state(logger_get_state());
        break;
        
    case EVT_STORAGE_ERROR:
    case EVT_SENSOR_ERROR:
        /* Flash red for errors */
        set_color_led_state(RGB_RED, true);
        break;
        
    case EVT_STORAGE_LOW:
        /* Could flash yellow as warning */
        break;
        
    default:
        break;
    }
}


/* Event subscriber for LED */
static event_subscriber_t led_subscriber;


/* Alternative: Manual state indication without events */
static void indicate_system_error(const char *error_msg)
{
    LOG_ERR("%s", error_msg);
    set_color_led_state(RGB_RED, true);
}

#if 0
/* Example usage in button handlers: */
static void button0_work_handler_with_led(struct k_work *work)
{
    /* ... existing code ... */
    
    if (press_duration >= BUTTON_LONG_PRESS_MS) {
        logger_state_t state = logger_get_state();
        if (state == LOGGER_STATE_IDLE) {
            logger_arm();
            set_color_led_state(RGB_GREEN, true);  /* Armed */
        } else if (state == LOGGER_STATE_ARMED) {
            logger_disarm();
            set_color_led_state(RGB_BLUE, true);   /* Back to idle */
        }
    }
}


/* Special patterns for specific conditions */
static void indicate_storage_full(void)
{
    /* Could implement a fast blink or different pattern */
    set_color_led_state(RGB_MAGENTA, true);
}

static void indicate_gnss_lock(void)
{
    /* Brief green flash when GNSS gets lock */
    set_color_led_state(RGB_GREEN, true);
    k_msleep(100);
    /* Return to current state color */
    update_led_for_state(logger_get_state());
}

static void test_version_output(const char *line, size_t len)
{
    /* Print to console for verification */
    //printk("Generated: %.*s", len, line);
    
    /* Verify checksum */
    if (!nmea_verify_checksum(line, len)) {
        printk("Checksum error!\n");
    }
}
#endif

int main(void)
{
    int ret;

    /*
     * Quiet the GNSS module VERY early - before any other initialization.
     * The SAM-M10Q continues running after CPU reset and will flood the
     * UART with NMEA data, causing buffer overruns if we wait too long.
     */
    gnss_early_quiet();

    //printk("boot\n");
    LOG_INF("Tempo-BT started");

    if (!gpio_is_ready_dt(&led)) {
		//return 0;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    ret = buttons_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize buttons: %d", ret);
    }

#ifdef CONFIG_USB_TTY_OUTPUT
    /* Initialize USB early so host can enumerate during slow SD card init */
    ret = usb_tty_init();
    if (ret < 0) {
        LOG_ERR("USB TTY init failed: %d", ret);
        /* Non-critical — continue without USB TTY capability */
    }
#endif

    // Initialize storage first
    ret = app_storage_init(); 
    if (ret < 0) {
        LOG_ERR("Failed to initialize storage: %d", ret);
        return ret;
    }

    // Now initialize app (which includes BLE/mcumgr)

    LOG_INF("Calling app_init()");
    ret = app_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize app: %d", ret);
    }

    /* Initialize event bus */
    ret = event_bus_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize event bus: %d", ret);
    }
    
#if 0
    /* Subscribe to test events */
    ret = event_bus_subscribe(&test_subscriber, 
                             test_event_handler,
                             (1U << EVT_TEST_DUMMY) | (1U << EVT_MODE_CHANGE),
                             NULL);
    if (ret < 0) {
        LOG_ERR("Failed to subscribe to events: %d", ret);
    }
    
    /* Publish a test event */
    LOG_INF("Publishing test event");
    event_bus_publish_simple(EVT_TEST_DUMMY);
    
    /* Give event thread time to process */
    k_msleep(10);
    
    /* Test settings */
    LOG_INF("About to test settings...");
    app_settings_test();
    LOG_INF("Settings test done");
#endif
    
    /* Initialize timebase */
    LOG_INF("About to init timebase...");
    ret = timebase_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize timebase: %d", ret);
    }
    LOG_INF("Timebase init done");

    ret = aggregator_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize aggregator: %d", ret);
    }
    LOG_INF("Aggregator init done");    

    //aggregator_register_output_callback(test_version_output);
    //aggregator_write_version();

    //logger_register_baro_callback(logger_baro_handler);
    
    /* Register both callbacks with barometer service */
    extern void logger_baro_handler(const baro_sample_t *sample);
    
    /* Register the logger's barometer callback for takeoff detection */
    ret = baro_register_callback(logger_baro_handler);
    if (ret < 0) {
        LOG_ERR("Failed to register logger baro callback: %d", ret);
        /* Non-critical - continue without automatic takeoff detection */
    } else {
        LOG_INF("Logger barometer callback registered for takeoff detection");
    }
    
    #if 0
    /* Test monotonic timer */
    LOG_INF("Testing monotonic timer...");
    uint64_t t1 = time_now_us();
    k_msleep(10);
    uint64_t t2 = time_now_us();
    LOG_INF("10ms sleep measured as %llu us", t2 - t1);
    
    /* Test UTC correlation (should fail initially) */
    uint64_t utc_ms;
    if (timebase_mono_to_utc(time_now_us(), &utc_ms)) {
        LOG_INF("UTC time available: %llu ms", utc_ms);
    } else {
        LOG_INF("UTC correlation not available yet: %s", 
                timebase_utc_string_placeholder());
    }
    
    LOG_INF("About to check QSPI flash...");
    
    /* Check if QSPI flash device is ready */
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("QSPI flash device not ready");
    } else {
        LOG_INF("QSPI flash device is ready");
        
        /* Get flash parameters */
        const struct flash_parameters *flash_params = flash_get_parameters(flash_dev);
        LOG_INF("Flash write block size: %d", flash_params->write_block_size);
        
        /* Get flash size */
        uint64_t flash_size;
        int res = flash_get_size(flash_dev, &flash_size);
        if (res == 0) {
            LOG_INF("Flash size: %llu bytes", flash_size);
        }
        
        /* Try to get page info */
        struct flash_pages_info info;
        ret = flash_get_page_info_by_offs(flash_dev, 0, &info);
        if (ret == 0) {
            LOG_INF("Flash page size: %d, start offset: 0x%x", 
                    info.size, (unsigned int)info.start_offset);
        }
    }
    #endif

    /* Initialize GNSS */
    LOG_INF("Initializing GNSS...");
    ret = gnss_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize GNSS: %d", ret);
        /* GNSS failure is not critical - continue without it */
    } else {
        LOG_INF("GNSS initialized successfully");

#ifdef CONFIG_GNSS_TEST_MODE
        /* Run UBX communication tests before configuring */
        ret = gnss_test_ubx_communication();
        if (ret < 0) {
            LOG_ERR("GNSS UBX communication test FAILED");
        } else {
            LOG_INF("GNSS UBX communication test PASSED");
        }
#endif

        /* Configure GNSS for skydiving operations */
        ret = gnss_init_skydiving();
        if (ret < 0) {
            LOG_WRN("Failed to configure GNSS for skydiving: %d", ret);
            /* Try manual configuration as fallback */
            ret = gnss_set_dynmodel(GNSS_DYNMODEL_AIRBORNE_4G);
            if (ret < 0) {
                LOG_ERR("Failed to set airborne 4g model: %d", ret);
            }
        }
        
        /* Set initial rate to 1Hz (will increase during freefall) */
        ret = gnss_set_rate(1);
        if (ret < 0) {
            LOG_WRN("Failed to set GNSS rate: %d", ret);
        }
    }

    /* Initialize Barometer */
    LOG_INF("Initializing BMP390 barometer...");
    ret = baro_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize barometer: %d", ret);
        /* Barometer is critical for takeoff detection */
        indicate_system_error("Barometer init failed");
        return ret;
    } else {
        LOG_INF("Barometer initialized successfully");
        
        /* Configure for flight operations - higher rate for better takeoff detection */
        baro_config_t baro_cfg = {
            .odr_hz = 8,  /* 8 Hz for good takeoff detection */
            .pressure_oversampling = 4,
            .temperature_oversampling = 1,
            .iir_filter_coeff = 3,
            .enable_data_ready_int = true
        };
        
        ret = baro_configure(&baro_cfg);
        if (ret < 0) {
            LOG_WRN("Failed to configure barometer: %d", ret);
        }
        
        /* Start barometer measurements */
        ret = baro_start();
        if (ret < 0) {
            LOG_ERR("Failed to start barometer: %d", ret);
            return ret;
        }
        
        LOG_INF("Barometer measurements started at 8 Hz");
    }

    /* Initialize IMU  */
    LOG_INF("About to init IMU...");
    ret = imu_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize IMU: %d", ret);
    } else {
        LOG_INF("IMU initialized successfully");

        /* Start orientation tracking immediately - the device is likely
         * motionless during boot, which is ideal for AHRS initialization
         * and gyroscope bias calibration. We maintain orientation
         * continuously regardless of logging state. */
        ret = orientation_start();
        if (ret < 0) {
            LOG_ERR("Failed to start orientation tracking: %d", ret);
        } else {
            LOG_INF("Orientation tracking started (AHRS initializing)");
        }
    }

    /* Initialize Magnetometer */
#if CONFIG_MMC5983MA
    LOG_INF("Initializing magnetometer...");
    ret = mag_init();
    if (ret < 0) {
        LOG_WRN("Magnetometer init failed: %d (non-critical)", ret);
    } else {
        uint8_t mag_mode = app_settings_get_mag_mode();
        if (mag_mode == 1) {
            /* Factory cal only: clear any NVM-loaded calibration */
            mag_cal_clear();
            LOG_INF("Mag mode 1: using factory calibration only");
        } else if (mag_mode == 2) {
            LOG_INF("Mag mode 2: using NVM calibration (valid=%d)",
                    mag_is_calibrated());
        } else {
            LOG_INF("Mag mode 0: magnetometer disabled for AHRS");
        }
    }
#endif

#ifdef CONFIG_SHELL

    //orientation_test_init();
    extern int imu_test_init(void);

    imu_test_init();

#endif

#if 0
    /* Test Barometer (BMP390) */
    LOG_INF("Testing BMP390 barometer...");
    extern int test_baro(void);
    ret = test_baro();
    if (ret < 0) {
        LOG_ERR("Barometer test failed: %d", ret);
    } else {
        LOG_INF("Barometer test completed successfully");
    }
#endif

#ifdef called_elsewhere
    ret = file_writer_init(NULL);  // Use default config
    if (ret < 0) {
        LOG_ERR("Failed to initialize file writer: %d", ret);
    }
#endif

    logger_config_t logger_cfg = {
        .base_path = "/logs",
        .use_date_folders = true,
        .use_uuid_names = true,
        .imu_rate_hz = 50,      /* IMU enabled with 50 Hz */
        .env_rate_hz = 4,       /* Barometer data at 4 Hz in logs */
        .gnss_rate_hz = 1,      /* GPS at 1 Hz normally */
        .enable_quaternion = true,  /* include quaternion orientation */
        .enable_magnetometer = (app_settings_get_mag_mode() > 0),
        .auto_start_on_takeoff = true,   /* ENABLE THIS for automatic detection */
        .auto_stop_on_landing = true     /* Also enable automatic stop */
    };

    ret = logger_init(&logger_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to initialize logger: %d", ret);
    }

    /* Initialize LED service */
    ret = led_service_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize LED service: %d", ret);
        /* Non-critical, continue without LED */
    } else {
        /* Subscribe to events for LED updates */
        ret = event_bus_subscribe(&led_subscriber, 
                                 led_event_handler,
                                 (1U << EVT_STATE_CHANGE) | 
                                 (1U << EVT_STORAGE_ERROR) |
                                 (1U << EVT_SENSOR_ERROR),
                                 NULL);
        if (ret < 0) {
            LOG_ERR("Failed to subscribe LED to events: %d", ret);
        }
        
        /* Set initial state */
        update_led_for_state(logger_get_state());
    }
    
    LOG_INF("System initialization complete");

    /* Arm the logger automatically at startup for flight detection */
    ret = logger_arm();
    if (ret < 0) {
        LOG_ERR("Failed to arm logger: %d", ret);
        indicate_system_error("Logger arm failed");
    } else {
        LOG_INF("Logger armed - ready for automatic flight detection");
        update_led_for_state(LOGGER_STATE_ARMED);
    }

    /* Main loop - call logger executive every 250ms for responsive freefall detection */
    while (1) {
        /* Logger executive manages automatic state transitions */
        logger_executive();

        k_sleep(K_MSEC(250));
    }
    
    return 0;
}