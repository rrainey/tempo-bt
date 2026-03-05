/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - USB Command Protocol
 *
 * NMEA-style command/response protocol over USB CDC-ACM.
 * Host sends $PCMD,<verb>[,args]*HH, device responds $PRSP,<verb>,<result>*HH.
 * Active only during magnetometer calibration streaming mode.
 */

#ifndef SERVICES_USB_CMD_H
#define SERVICES_USB_CMD_H

#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Initialize the USB command handler
 *
 * Registers itself as the USB TTY RX line callback.
 * Call once after usb_tty_init().
 */
void usb_cmd_init(void);

/**
 * @brief Enable or disable command processing
 *
 * When disabled, received lines are silently ignored.
 * Call with true when entering mag calibration mode,
 * false when leaving it.
 *
 * @param active  true to enable command processing
 */
void usb_cmd_set_active(bool active);

#endif /* SERVICES_USB_CMD_H */
