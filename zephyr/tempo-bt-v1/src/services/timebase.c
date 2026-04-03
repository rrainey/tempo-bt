/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Time Base Service Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/posix/time.h>

#include "services/timebase.h"
#include "services/logger.h"

LOG_MODULE_REGISTER(timebase, LOG_LEVEL_INF);

/* PPS input from GNSS (P0.27) */
#define PPS_NODE DT_NODELABEL(gnss_timepulse)
#if DT_NODE_EXISTS(PPS_NODE)
static const struct gpio_dt_spec pps_gpio = GPIO_DT_SPEC_GET(PPS_NODE, gpios);
#endif

/* Green LED for PPS visual feedback (led0 = P0.12) */
#define PPS_LED_NODE DT_ALIAS(led0)
#if DT_NODE_EXISTS(PPS_LED_NODE)
static const struct gpio_dt_spec pps_led = GPIO_DT_SPEC_GET(PPS_LED_NODE, gpios);
static bool pps_led_available = false;
#endif

/* PPS pulse counter - volatile for ISR access */
static volatile uint32_t pps_pulse_count = 0;

/* GPIO callback structure */
static struct gpio_callback pps_cb_data;

/* Delayed work for turning off LED after 100ms */
static struct k_work_delayable pps_led_off_work;

/* Work item for test alarm processing (called from PPS context) */
static struct k_work test_alarm_work;

/*
 * Dedicated work queue for test alarm processing
 * Uses larger stack (3KB) to handle FAT filesystem I/O in logger_start()
 * The system work queue stack is too small for file operations
 */
#define TEST_ALARM_STACK_SIZE 3072
#define TEST_ALARM_PRIORITY   5
static K_THREAD_STACK_DEFINE(test_alarm_stack, TEST_ALARM_STACK_SIZE);
static struct k_work_q test_alarm_workq;

/* Flag indicating PPS system is initialized */
static bool pps_initialized = false;

/* Time correlation data */
static time_correlation_t time_corr = {
    .mono_us = 0,
    .utc_ms = 0,
    .valid = false,
    .accuracy_ms = 0
};

/* Mutex for correlation updates */
static struct k_mutex corr_mutex;

/* Boot time reference */
static uint64_t boot_time_us;

int timebase_pps_init(void);

int timebase_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&corr_mutex);
    
    /* Capture boot time */
    boot_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    timebase_pps_init();

    LOG_INF("Timebase initialized, boot time: %llu us", boot_time_us);
    
    /* Test the timer resolution */
    uint64_t t1 = time_now_us();
    k_busy_wait(100);  /* Wait 100 microseconds */
    uint64_t t2 = time_now_us();
    uint64_t delta = t2 - t1;
    
    LOG_INF("Timer test: 100us busy wait measured as %llu us", delta);
    
    return 0;
}

uint64_t time_now_us(void)
{
    /* Get current uptime in ticks and convert to microseconds */
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

void timebase_update_correlation(uint64_t utc_ms, uint32_t accuracy_ms)
{
    k_mutex_lock(&corr_mutex, K_FOREVER);
    
    /* Capture current monotonic time */
    time_corr.mono_us = time_now_us();
    time_corr.utc_ms = utc_ms;
    time_corr.accuracy_ms = accuracy_ms;
    time_corr.valid = true;
    
    k_mutex_unlock(&corr_mutex);
    
    LOG_INF("Time correlation updated: mono=%llu us, UTC=%llu ms, accuracy=%u ms",
            time_corr.mono_us, time_corr.utc_ms, time_corr.accuracy_ms);
}

bool timebase_mono_to_utc(uint64_t mono_us, uint64_t *utc_ms)
{
    if (!utc_ms) {
        return false;
    }
    
    k_mutex_lock(&corr_mutex, K_FOREVER);
    
    if (!time_corr.valid) {
        k_mutex_unlock(&corr_mutex);
        LOG_DBG("No valid time correlation available");
        return false;
    }
    
    /* Calculate time offset from correlation point */
    int64_t offset_us = (int64_t)(mono_us - time_corr.mono_us);
    int64_t offset_ms = offset_us / 1000;
    
    /* Apply offset to UTC time */
    *utc_ms = time_corr.utc_ms + offset_ms;
    
    k_mutex_unlock(&corr_mutex);
    
    return true;
}

bool timebase_get_correlation(time_correlation_t *corr)
{
    if (!corr) {
        return false;
    }
    
    k_mutex_lock(&corr_mutex, K_FOREVER);
    *corr = time_corr;
    k_mutex_unlock(&corr_mutex);
    
    return corr->valid;
}

/*
 * Get current UTC time in milliseconds
 * Returns 0 if no valid correlation
 */
static uint64_t get_current_utc_ms(void)
{
    uint64_t utc_ms = 0;
    if (timebase_mono_to_utc(time_now_us(), &utc_ms)) {
        return utc_ms;
    }
    return 0;
}

/*
 * Test alarm work handler
 * Called from work queue context on each PPS pulse to check alarm state
 * and trigger logger state transitions at the appropriate times.
 */
static void test_alarm_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    int state = test_alarm_get_state();
    if (state == TEST_ALARM_IDLE) {
        return;  /* No alarm active */
    }

    uint64_t current_utc_ms = get_current_utc_ms();
    if (current_utc_ms == 0) {
        LOG_WRN("Test alarm: no valid UTC time");
        return;
    }

    if (state == TEST_ALARM_WAITING_START) {
        uint64_t target_ms = test_alarm_get_target_utc_ms();

        if (current_utc_ms >= target_ms) {
            LOG_INF("Test alarm: start time reached, triggering LOGGING");

            /* Auto-arm if needed, then start logging */
            logger_state_t logger_state = logger_get_state();
            int ret = 0;

            if (logger_state == LOGGER_STATE_IDLE) {
                ret = logger_arm();
                if (ret != 0) {
                    LOG_ERR("Test alarm: failed to arm logger: %d", ret);
                    test_alarm_set_state(TEST_ALARM_IDLE);
                    return;
                }
            }

            ret = logger_start("test_alarm");
            if (ret != 0) {
                LOG_ERR("Test alarm: failed to start logger: %d", ret);
                test_alarm_set_state(TEST_ALARM_IDLE);
                return;
            }

            /* Transition to waiting for jump */
            uint32_t jump_delay = test_alarm_get_jump_delay();
            if (jump_delay > 0) {
                test_alarm_start_jump_countdown();
                test_alarm_set_state(TEST_ALARM_WAITING_JUMP);
                LOG_INF("Test alarm: now counting down %u seconds to JUMPED",
                        jump_delay);
            } else {
                /* No jump delay, we're done */
                test_alarm_set_state(TEST_ALARM_IDLE);
                LOG_INF("Test alarm: complete (no jump delay)");
            }
        }
    } else if (state == TEST_ALARM_WAITING_JUMP) {
        uint32_t remaining = test_alarm_decrement_countdown();

        if (remaining == 0) {
            LOG_INF("Test alarm: jump countdown complete, triggering JUMPED");

            int ret = logger_jumped();
            if (ret != 0) {
                LOG_ERR("Test alarm: failed to trigger jumped: %d", ret);
            }

            /* Alarm sequence complete */
            test_alarm_set_state(TEST_ALARM_IDLE);
            LOG_INF("Test alarm: sequence complete");
        } else if ((remaining % 10) == 0 || remaining <= 5) {
            LOG_INF("Test alarm: %u seconds until JUMPED", remaining);
        }
    }
}

/*
 * PPS LED off work handler
 * Called 100ms after PPS pulse to turn off the green LED
 */
static void pps_led_off_handler(struct k_work *work)
{
    ARG_UNUSED(work);

#if DT_NODE_EXISTS(PPS_LED_NODE)
    if (pps_led_available) {
        gpio_pin_set_dt(&pps_led, 1);  /* Active low: 1 = LED off */
    }
#endif
}

/*
 * PPS GPIO interrupt callback
 * Called on rising edge of GNSS time pulse (1 Hz)
 */
static void pps_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                              uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Increment pulse counter */
    pps_pulse_count++;

    /* Turn on green LED */
#if DT_NODE_EXISTS(PPS_LED_NODE)
    if (pps_led_available) {
        gpio_pin_set_dt(&pps_led, 0);  /* Active low: 0 = LED on */
    }
#endif

    /* Schedule LED off after 100ms */
    k_work_reschedule(&pps_led_off_work, K_MSEC(100));

    /* Schedule test alarm check (runs in dedicated work queue for larger stack) */
    k_work_submit_to_queue(&test_alarm_workq, &test_alarm_work);
}

int timebase_pps_init(void)
{
    int ret;

#if !DT_NODE_EXISTS(PPS_NODE)
    LOG_WRN("PPS GPIO node not found in devicetree");
    return -ENODEV;
#else
    /* Check if PPS GPIO device is ready */
    if (!gpio_is_ready_dt(&pps_gpio)) {
        LOG_ERR("PPS GPIO device not ready");
        return -ENODEV;
    }

    /* Configure PPS pin as input */
    ret = gpio_pin_configure_dt(&pps_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure PPS GPIO: %d", ret);
        return ret;
    }

    /* Configure interrupt on rising edge */
    ret = gpio_pin_interrupt_configure_dt(&pps_gpio, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        LOG_ERR("Failed to configure PPS interrupt: %d", ret);
        return ret;
    }

    /* Initialize and add callback */
    gpio_init_callback(&pps_cb_data, pps_gpio_callback, BIT(pps_gpio.pin));
    ret = gpio_add_callback(pps_gpio.port, &pps_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add PPS callback: %d", ret);
        return ret;
    }

    LOG_INF("PPS input configured on P%d.%02d",
            pps_gpio.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
            pps_gpio.pin);
#endif

    /* Initialize green LED for visual feedback */
#if DT_NODE_EXISTS(PPS_LED_NODE)
    if (gpio_is_ready_dt(&pps_led)) {
        ret = gpio_pin_configure_dt(&pps_led, GPIO_OUTPUT_INACTIVE);
        if (ret == 0) {
            pps_led_available = true;
            LOG_INF("PPS LED feedback configured on led0 (green)");
        } else {
            LOG_WRN("Failed to configure PPS LED: %d", ret);
        }
    }
#endif

    /* Initialize delayed work for LED off */
    k_work_init_delayable(&pps_led_off_work, pps_led_off_handler);

    /* Initialize dedicated work queue for test alarm (needs larger stack for FAT I/O) */
    k_work_queue_init(&test_alarm_workq);
    k_work_queue_start(&test_alarm_workq, test_alarm_stack,
                       K_THREAD_STACK_SIZEOF(test_alarm_stack),
                       TEST_ALARM_PRIORITY, NULL);

    /* Initialize test alarm work */
    k_work_init(&test_alarm_work, test_alarm_work_handler);

    pps_initialized = true;
    LOG_INF("PPS subsystem initialized");

    return 0;
}

void timebase_get_pps_data(timebase_pps_data_t *data)
{
    if (!data) {
        return;
    }

    /* Read counter and timestamp as close together as possible */
    data->pulse_count = pps_pulse_count;
    data->timestamp_us = time_now_us();
}

bool timebase_pps_active(void)
{
    return pps_initialized && (pps_pulse_count > 0);
}

/* UTC placeholder function for when GNSS is not available */
const char *timebase_utc_string_placeholder(void)
{
    return "unknown";
}

int timebase_get_utc_iso8601(char *buf, size_t buf_size)
{
    if (!buf || buf_size < 25) {
        return -EINVAL;
    }

    uint64_t utc_ms = get_current_utc_ms();
    if (utc_ms == 0) {
        return -EAGAIN;  /* No valid UTC correlation */
    }

    /* Convert milliseconds since epoch to time components */
    time_t utc_sec = (time_t)(utc_ms / 1000);
    uint32_t tenths = (uint32_t)((utc_ms % 1000) / 100);  /* 0.1s resolution */

    struct tm tm_buf;
    struct tm *tm = gmtime_r(&utc_sec, &tm_buf);
    if (!tm) {
        return -EINVAL;
    }

    /* Format as ISO 8601: YYYY-MM-DDTHH:MM:SS.dZ */
    int len = snprintf(buf, buf_size, "%04d-%02d-%02dT%02d:%02d:%02d.%uZ",
                       tm->tm_year + 1900,
                       tm->tm_mon + 1,
                       tm->tm_mday,
                       tm->tm_hour,
                       tm->tm_min,
                       tm->tm_sec,
                       tenths);

    if (len < 0 || (size_t)len >= buf_size) {
        return -ENOMEM;
    }

    return 0;
}