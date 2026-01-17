/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - RGB LED Service Implementation
 *
 * Controls an RGB LED using PWM, providing a 50ms blink once per second.
 * Also controls led1 (red GPIO LED) to indicate GPS lock status.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>

#include "services/led.h"
#include "services/gnss.h"

LOG_MODULE_REGISTER(led_service, LOG_LEVEL_INF);

/* GPIO LED for GPS status indicator (led1 = red LED) */
#define GPS_LED_NODE DT_ALIAS(led1)
#if DT_NODE_EXISTS(GPS_LED_NODE)
static const struct gpio_dt_spec gps_led = GPIO_DT_SPEC_GET(GPS_LED_NODE, gpios);
static bool gps_led_available = false;
#endif

/* PWM configuration */
#define LED_PWM_NODE DT_NODELABEL(pwm0)
#define LED_PWM_CHANNEL_R 0  /* Red on channel 0 */
#define LED_PWM_CHANNEL_G 1  /* Green on channel 1 */
#define LED_PWM_CHANNEL_B 2  /* Blue on channel 2 */

/* Timing configuration */
#define BLINK_PERIOD_MS    2000  /* 2 second period */ 
#define BLINK_ON_TIME_MS   50    /* 50ms on time */
#define PWM_PERIOD_NS      1000000  /* 1ms PWM period for smooth dimming */ 

/* LED state */
static struct {
    const struct device *pwm_dev;
    rgb_color_t current_color;
    bool enabled;
    struct k_timer blink_timer;
    bool led_on;
    struct k_mutex mutex;
    rgb_color_t override_color;
    bool override_enabled;
    rgb_color_t app_color;      
    bool app_enabled;           
} led_state = {
    .pwm_dev = NULL,
    .current_color = RGB_BLACK,
    .enabled = false,
    .led_on = false,
    .override_color = RGB_BLACK,
    .override_enabled = false,
    .app_color = RGB_BLACK,
    .app_enabled = false
};

/* Forward declaration */
static void blink_timer_handler(struct k_timer *timer);
static void led_off(void);

/* Timer work item for turning LED off */
static struct k_work_delayable led_off_work;

/* Work handler to turn LED off */
static void led_off_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    led_off();
}

/**
 * @brief Set PWM duty cycle for a color channel
 */
static int set_pwm_channel(uint32_t channel, uint8_t brightness)
{
    uint32_t pulse_ns;
    int ret;
    
    // For common anode LEDs, we need to invert the brightness
    // 0 brightness = full PWM duty (LED OFF)
    // 255 brightness = 0 PWM duty (LED ON)
    uint8_t inverted_brightness = 255 - brightness;
    
    // Still use the bit-shifted value if you want dimmer LEDs
    // For full brightness, remove the >> 3
    inverted_brightness = inverted_brightness >> 3;  // Scale to 0-31 range
    
    pulse_ns = ((uint32_t)inverted_brightness * PWM_PERIOD_NS) / 31;
    
    LOG_DBG("Setting PWM ch%d: brightness=%d (inverted=%d), pulse=%d ns", 
            channel, brightness, inverted_brightness, pulse_ns);
    
    ret = pwm_set(led_state.pwm_dev, channel, PWM_PERIOD_NS, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM channel %d: %d", channel, ret);
    }
    
    return ret;
}

/**
 * @brief Turn LED on with current color
 */
static void led_on(void)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);

    if (led_state.enabled) {
        set_pwm_channel(LED_PWM_CHANNEL_R, led_state.current_color.r >> 3);
        set_pwm_channel(LED_PWM_CHANNEL_G, led_state.current_color.g >> 3);
        set_pwm_channel(LED_PWM_CHANNEL_B, led_state.current_color.b >> 3);
        led_state.led_on = true;

        /* Turn on GPS status LED (red) if no GPS fix */
#if DT_NODE_EXISTS(GPS_LED_NODE)
        if (gps_led_available && !gnss_has_fix()) {
            gpio_pin_set_dt(&gps_led, 0);  /* Active low: 0 = LED on */
        }
#endif
    }

    k_mutex_unlock(&led_state.mutex);
}

/**
 * @brief Turn LED off
 */
static void led_off(void)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);

    set_pwm_channel(LED_PWM_CHANNEL_R, 0);
    set_pwm_channel(LED_PWM_CHANNEL_G, 0);
    set_pwm_channel(LED_PWM_CHANNEL_B, 0);
    led_state.led_on = false;

    /* Always turn off GPS status LED when RGB LED turns off */
#if DT_NODE_EXISTS(GPS_LED_NODE)
    if (gps_led_available) {
        gpio_pin_set_dt(&gps_led, 1);  /* Active low: 1 = LED off */
    }
#endif

    k_mutex_unlock(&led_state.mutex);
}

/**
 * @brief Blink timer handler
 */
static void blink_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    
    /* Only turn on if currently off */
    if (!led_state.led_on) {
        /* Turn LED on */
        led_on();
        
        /* Schedule LED off after 50ms */
        k_work_reschedule(&led_off_work, K_MSEC(BLINK_ON_TIME_MS));
    }
}

/**
 * @brief Initialize the LED service
 */
int led_service_init(void)
{
    /* Get PWM device */
    led_state.pwm_dev = DEVICE_DT_GET(LED_PWM_NODE);
    if (!device_is_ready(led_state.pwm_dev)) {
        LOG_ERR("PWM device not ready");
        return -ENODEV;
    }

    /* Initialize GPS status LED (led1 - red LED) */
#if DT_NODE_EXISTS(GPS_LED_NODE)
    if (gpio_is_ready_dt(&gps_led)) {
        int gps_led_ret = gpio_pin_configure_dt(&gps_led, GPIO_OUTPUT_INACTIVE);
        if (gps_led_ret == 0) {
            gps_led_available = true;
            LOG_INF("GPS status LED (led1) initialized");
        } else {
            LOG_WRN("Failed to configure GPS LED: %d", gps_led_ret);
        }
    } else {
        LOG_WRN("GPS LED device not ready");
    }
#endif

    /* Initialize mutex */
    k_mutex_init(&led_state.mutex);

    /* Initialize timer */
    k_timer_init(&led_state.blink_timer, blink_timer_handler, NULL);

    /* Initialize work item */
    k_work_init_delayable(&led_off_work, led_off_work_handler);

    /* Turn off all channels initially */
    led_off();

    LOG_INF("LED service initialized");

    return 0;
}

/**
 * @brief Set RGB LED color and state
 */
int set_color_led_state(rgb_color_t color, bool state)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    /* Store application state */
    led_state.app_color = color;
    led_state.app_enabled = state;
    
    /* Only update actual LED if override is not active */
    if (!led_state.override_enabled) {
        led_state.current_color = color;
        led_state.enabled = state;
        
        if (state) {
            /* Start blinking timer */
            k_timer_start(&led_state.blink_timer, 
                        K_NO_WAIT,
                        K_MSEC(BLINK_PERIOD_MS));
        } else {
            /* Stop timer and turn off LED */
            k_timer_stop(&led_state.blink_timer);
            k_work_cancel_delayable(&led_off_work);
            led_off();
        }
    }
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}

/**
 * @brief Get current LED state
 */
int led_service_get_state(rgb_color_t *color, bool *state)
{
    if (!color || !state) {
        return -EINVAL;
    }
    
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    *color = led_state.current_color;
    *state = led_state.enabled;
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}

/**
 * @brief Test individual PWM channels
 */
void led_test_channels(void)
{
    LOG_INF("Testing individual PWM channels...");
    
    // Test each channel individually at different brightness levels
    uint8_t test_values[] = {0, 1, 10, 50, 100, 200, 255};
    
    for (int ch = 0; ch < 3; ch++) {
        const char *color_name = (ch == 0) ? "RED" : (ch == 1) ? "GREEN" : "BLUE";
        LOG_INF("Testing %s channel", color_name);
        
        for (int i = 0; i < ARRAY_SIZE(test_values); i++) {
            LOG_INF("  Setting %s to %d", color_name, test_values[i]);
            
            // Turn off all channels first
            set_pwm_channel(LED_PWM_CHANNEL_R, 0);
            set_pwm_channel(LED_PWM_CHANNEL_G, 0);
            set_pwm_channel(LED_PWM_CHANNEL_B, 0);
            
            // Set only the test channel
            set_pwm_channel(ch, test_values[i]);
            
            k_sleep(K_SECONDS(2));
        }
    }
    
    // Turn off all
    set_pwm_channel(LED_PWM_CHANNEL_R, 0);
    set_pwm_channel(LED_PWM_CHANNEL_G, 0);
    set_pwm_channel(LED_PWM_CHANNEL_B, 0);
}

/**
 * @brief Test PWM period variations
 */
void led_test_pwm_periods(void)
{
    uint32_t test_periods[] = {
        1000000,    // 1ms (1kHz) - current
        10000000,   // 10ms (100Hz)
        20000000,   // 20ms (50Hz)
        100000000   // 100ms (10Hz)
    };
    
    LOG_INF("Testing different PWM periods...");
    
    for (int i = 0; i < ARRAY_SIZE(test_periods); i++) {
        uint32_t period = test_periods[i];
        uint32_t pulse = period / 2; // 50% duty cycle
        
        LOG_INF("Testing period: %d ns (%d Hz)", period, 1000000000 / period);
        
        // Test on red channel
        pwm_set(led_state.pwm_dev, LED_PWM_CHANNEL_R, period, pulse, 0);
        pwm_set(led_state.pwm_dev, LED_PWM_CHANNEL_G, period, 0, 0);
        pwm_set(led_state.pwm_dev, LED_PWM_CHANNEL_B, period, 0, 0);
        
        k_sleep(K_SECONDS(3));
    }
    
    // Turn off
    pwm_set(led_state.pwm_dev, LED_PWM_CHANNEL_R, test_periods[0], 0, 0);
}

/**
 * @brief Test timing accuracy
 */
void led_test_timing(void)
{
    LOG_INF("Testing blink timing accuracy...");
    
    // Manually control LED timing to verify
    for (int i = 0; i < 5; i++) {
        int64_t start = k_uptime_get();
        
        // Turn on
        set_pwm_channel(LED_PWM_CHANNEL_R, 255);
        k_sleep(K_MSEC(410));
        
        // Turn off
        set_pwm_channel(LED_PWM_CHANNEL_R, 0);
        
        int64_t on_time = k_uptime_get() - start;
        LOG_INF("Cycle %d: ON time = %lld ms (expected 410ms)", i, on_time);
        
        k_sleep(K_MSEC(590)); // Rest of the 1 second period
    }
}

/**
 * @brief Modified set_pwm_channel with better debugging
 */
static int set_pwm_channel_debug(uint32_t channel, uint8_t brightness)
{
    uint32_t pulse_ns;
    int ret;
    
    // Try different scaling methods
    #ifdef USE_LINEAR_SCALING
        pulse_ns = ((uint32_t)brightness * PWM_PERIOD_NS) / 255;
    #elif defined(USE_FULL_RANGE)
        // Use full brightness value without scaling
        pulse_ns = ((uint32_t)brightness * PWM_PERIOD_NS) / 255;
    #else
        // Original scaling
        pulse_ns = ((uint32_t)(brightness >> 3) * PWM_PERIOD_NS) / 31;
    #endif
    
    LOG_INF("PWM ch%d: bright=%d, pulse=%d ns (%.1f%%)", 
            channel, brightness, pulse_ns, 
            (float)pulse_ns * 100.0f / PWM_PERIOD_NS);
    
    ret = pwm_set(led_state.pwm_dev, channel, PWM_PERIOD_NS, pulse_ns, 0);
    if (ret < 0) {
        LOG_ERR("Failed to set PWM channel %d: %d", channel, ret);
    }
    
    return ret;
}

int led_service_set_override(rgb_color_t color, bool enable)
{
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    led_state.override_color = color;
    led_state.override_enabled = enable;
    
    if (enable) {
        /* Apply override settings */
        led_state.current_color = color;
        led_state.enabled = true;
        
        /* Start blinking with override color */
        k_timer_start(&led_state.blink_timer, 
                    K_NO_WAIT,
                    K_MSEC(BLINK_PERIOD_MS));
                    
        LOG_INF("LED override enabled: R=%d G=%d B=%d", 
                color.r, color.g, color.b);
    } else {
        /* Return to application control */
        led_state.current_color = led_state.app_color;
        led_state.enabled = led_state.app_enabled;
        
        if (led_state.app_enabled) {
            /* Resume app's blinking pattern */
            k_timer_start(&led_state.blink_timer, 
                        K_NO_WAIT,
                        K_MSEC(BLINK_PERIOD_MS));
        } else {
            /* App had LED off, so turn it off */
            k_timer_stop(&led_state.blink_timer);
            k_work_cancel_delayable(&led_off_work);
            led_off();
        }
        
        LOG_INF("LED override disabled, returning to app control");
    }
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}

int led_service_get_override(rgb_color_t *color, bool *enabled)
{
    if (!color || !enabled) {
        return -EINVAL;
    }
    
    k_mutex_lock(&led_state.mutex, K_FOREVER);
    
    *color = led_state.override_color;
    *enabled = led_state.override_enabled;
    
    k_mutex_unlock(&led_state.mutex);
    
    return 0;
}