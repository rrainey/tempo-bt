/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - USB CDC-ACM TTY Service
 *
 * Bidirectional USB serial for ground testing and calibration.
 * TX: interrupt-driven ring buffer for non-blocking writes.
 * RX: ISR line accumulation, dispatched via system workqueue.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include "services/usb_tty.h"

LOG_MODULE_REGISTER(usb_tty, LOG_LEVEL_INF);

#define USB_TTY_RING_SIZE 4096
#define USB_TTY_RX_LINE_SIZE 128

/* CDC-ACM device from devicetree */
static const struct device *cdc_dev = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

/* State */
static bool usb_enabled;
static bool tty_open;
static volatile bool dtr_state;
static volatile bool usb_configured;  /* Tracks USB_DC_CONFIGURED state */
static usb_tty_disconnect_cb_t disconnect_cb;

/* TX ring buffer */
RING_BUF_DECLARE(tx_ring, USB_TTY_RING_SIZE);

/* RX line accumulation (ISR writes rx_line_buf, work handler reads rx_complete_buf) */
static char rx_line_buf[USB_TTY_RX_LINE_SIZE];
static char rx_complete_buf[USB_TTY_RX_LINE_SIZE];
static size_t rx_pos;
static size_t rx_complete_len;
static usb_tty_rx_callback_t rx_callback;

static void rx_line_work_handler(struct k_work *work);
static K_WORK_DEFINE(rx_line_work, rx_line_work_handler);

/* Work item for disconnect notification — cannot call logger_stop from ISR */
static void disconnect_work_handler(struct k_work *work);
static K_WORK_DEFINE(disconnect_work, disconnect_work_handler);

/* Work item for DTR polling (CDC-ACM doesn't have a line-ctrl callback) */
static void dtr_poll_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dtr_poll_work, dtr_poll_handler);

#define DTR_POLL_INTERVAL_MS 250

/*
 * RX line work handler — called from system workqueue when a complete
 * line has been accumulated by the ISR.
 */
static void rx_line_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (rx_callback && rx_complete_len > 0) {
        rx_callback(rx_complete_buf, rx_complete_len);
    }
}

/*
 * UART interrupt callback — handles both TX drain and RX line accumulation.
 */
static void uart_irq_callback(const struct device *dev, void *user_data)
{
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    /* TX: drain ring buffer into CDC-ACM FIFO */
    if (uart_irq_tx_ready(dev)) {
        uint8_t *data;
        uint32_t len;

        len = ring_buf_get_claim(&tx_ring, &data, 256);
        if (len > 0) {
            int sent = uart_fifo_fill(dev, data, len);
            if (sent > 0) {
                ring_buf_get_finish(&tx_ring, sent);
            } else {
                ring_buf_get_finish(&tx_ring, 0);
            }
        } else {
            /* Ring buffer empty — disable TX interrupt */
            uart_irq_tx_disable(dev);
        }
    }

    /* RX: accumulate bytes into line buffer */
    if (uart_irq_rx_ready(dev)) {
        uint8_t byte;

        while (uart_fifo_read(dev, &byte, 1) == 1) {
            if (byte == '\n') {
                /* Complete line received */
                rx_line_buf[rx_pos] = '\0';
                /* Strip trailing \r */
                if (rx_pos > 0 && rx_line_buf[rx_pos - 1] == '\r') {
                    rx_line_buf[rx_pos - 1] = '\0';
                    rx_pos--;
                }
                if (rx_pos > 0 && rx_callback) {
                    memcpy(rx_complete_buf, rx_line_buf, rx_pos + 1);
                    rx_complete_len = rx_pos;
                    k_work_submit(&rx_line_work);
                }
                rx_pos = 0;
            } else if (rx_pos < sizeof(rx_line_buf) - 1) {
                rx_line_buf[rx_pos++] = byte;
            }
            /* else: overflow — discard byte */
        }
    }
}

/*
 * DTR polling — Zephyr CDC-ACM doesn't provide a line-control change callback,
 * so we poll DTR periodically while the TTY is open.
 */
static void dtr_poll_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    uint32_t dtr_val = 0;
    int ret = uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr_val);

    if (ret == 0) {
        bool new_dtr = (dtr_val != 0);

        if (dtr_state && !new_dtr && tty_open) {
            /* DTR dropped while open — host disconnected */
            LOG_INF("USB host disconnected (DTR dropped)");
            dtr_state = false;
            k_work_submit(&disconnect_work);
            return; /* Stop polling — disconnect handler will close */
        }

        dtr_state = new_dtr;
    }

    /* Continue polling while open */
    if (tty_open) {
        k_work_reschedule(&dtr_poll_work, K_MSEC(DTR_POLL_INTERVAL_MS));
    }
}

static void disconnect_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (disconnect_cb) {
        disconnect_cb();
    }
}

/*
 * USB device status callback — tracks configured/suspended/disconnected state.
 * Called from USB stack context (not ISR-safe for heavy work).
 */
static void usb_status_callback(enum usb_dc_status_code status, const uint8_t *param)
{
    ARG_UNUSED(param);

    switch (status) {
    case USB_DC_CONFIGURED:
        usb_configured = true;
        LOG_INF("USB TTY: host configured");
        break;
    case USB_DC_DISCONNECTED:
    case USB_DC_SUSPEND:
        if (usb_configured && tty_open) {
            LOG_INF("USB TTY: host disconnected/suspended");
            usb_configured = false;
            k_work_submit(&disconnect_work);
        }
        usb_configured = false;
        break;
    default:
        break;
    }
}

int usb_tty_init(void)
{
    int ret;

    if (!device_is_ready(cdc_dev)) {
        LOG_ERR("CDC-ACM device not ready");
        return -ENODEV;
    }

    /* Set up interrupt-driven callback */
    uart_irq_callback_set(cdc_dev, uart_irq_callback);

    /* Enable the USB device stack with status callback */
    ret = usb_enable(usb_status_callback);
    if (ret != 0 && ret != -EALREADY) {
        LOG_ERR("Failed to enable USB: %d", ret);
        return ret;
    }

    usb_enabled = true;
    LOG_INF("USB TTY initialized");

    return 0;
}

bool usb_tty_is_connected(void)
{
    if (!usb_enabled) {
        return false;
    }

    /* Check DTR first — the reliable indicator when a terminal asserts it */
    uint32_t dtr_val = 0;
    int ret = uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr_val);

    if (ret == 0 && dtr_val != 0) {
        return true;
    }

    /* Fall back to USB configured state — many Windows terminals don't
     * assert DTR. If the host has configured the device, it's connected.
     */
    return usb_configured;
}

int usb_tty_open(void)
{
    if (!usb_enabled) {
        return -ENODEV;
    }

    if (!usb_tty_is_connected()) {
        return -ENOTCONN;
    }

    /* Reset ring buffer and RX state */
    ring_buf_reset(&tx_ring);
    rx_pos = 0;

    dtr_state = true;
    tty_open = true;

    /* Enable RX interrupts for incoming commands */
    uart_irq_rx_enable(cdc_dev);

    /* Start DTR polling for disconnect detection */
    k_work_reschedule(&dtr_poll_work, K_MSEC(DTR_POLL_INTERVAL_MS));

    LOG_INF("USB TTY opened");
    return 0;
}

void usb_tty_output_line(const char *line, size_t len)
{
    if (!tty_open || !dtr_state) {
        return;
    }

    uint32_t written = ring_buf_put(&tx_ring, (const uint8_t *)line, len);
    if (written < len) {
        LOG_DBG("USB TTY ring full, dropped %u bytes", (unsigned int)(len - written));
    }

    if (written > 0) {
        uart_irq_tx_enable(cdc_dev);
    }
}

void usb_tty_close(void)
{
    tty_open = false;

    /* Cancel DTR polling */
    k_work_cancel_delayable(&dtr_poll_work);

    /* Disable TX and RX interrupts */
    uart_irq_tx_disable(cdc_dev);
    uart_irq_rx_disable(cdc_dev);

    /* Reset buffers */
    ring_buf_reset(&tx_ring);
    rx_pos = 0;

    LOG_INF("USB TTY closed");
}

void usb_tty_register_disconnect_callback(usb_tty_disconnect_cb_t callback)
{
    disconnect_cb = callback;
}

void usb_tty_register_rx_callback(usb_tty_rx_callback_t callback)
{
    rx_callback = callback;
}
