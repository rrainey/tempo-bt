/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - USB CDC-ACM TTY Service
 *
 * Provides NMEA sentence output over USB serial for ground testing.
 */

#ifndef SERVICES_USB_TTY_H
#define SERVICES_USB_TTY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Initialize the USB TTY service
 *
 * Enables the USB device stack and prepares the CDC-ACM UART.
 * Must be called once during system initialization.
 *
 * @return 0 on success, negative error code on failure
 */
int usb_tty_init(void);

/**
 * @brief Check if a USB host is connected (DTR asserted)
 *
 * @return true if USB host is connected and DTR is asserted
 */
bool usb_tty_is_connected(void);

/**
 * @brief Open the USB TTY for output
 *
 * Prepares the CDC-ACM UART for writing.
 *
 * @return 0 on success, -ENOTCONN if no USB host connected
 */
int usb_tty_open(void);

/**
 * @brief Write a line to the USB TTY
 *
 * Matches the aggregator_output_callback_t signature. Writes data
 * to the USB CDC-ACM UART using interrupt-driven TX. If the internal
 * buffer is full, the data is silently dropped.
 *
 * @param line  Formatted NMEA sentence (with CRLF)
 * @param len   Length of the line
 */
void usb_tty_output_line(const char *line, size_t len);

/**
 * @brief Close the USB TTY
 *
 * Flushes remaining data and disables TX.
 */
void usb_tty_close(void);

/**
 * @brief Register a callback for USB disconnect events
 *
 * The callback is invoked from work queue context (not ISR)
 * when the USB host disconnects (DTR de-asserted while open).
 *
 * @param callback  Function to call on disconnect, or NULL to clear
 */
typedef void (*usb_tty_disconnect_cb_t)(void);
void usb_tty_register_disconnect_callback(usb_tty_disconnect_cb_t callback);

#endif /* SERVICES_USB_TTY_H */
