/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - USB Command Protocol
 *
 * Processes $PCMD NMEA sentences received over USB CDC-ACM and sends
 * $PRSP responses.  Commands are gated by usb_cmd_set_active() so they
 * are only processed during magnetometer calibration streaming mode.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "services/usb_cmd.h"
#include "services/usb_tty.h"
#include "services/mag.h"
#include "config/settings.h"
#include "app/log_format.h"
#include "util/nmea_checksum.h"

LOG_MODULE_REGISTER(usb_cmd, LOG_LEVEL_INF);

static volatile bool cmd_active;

/* Forward declarations */
static void usb_cmd_process_line(const char *line, size_t len);
static void cmd_cal_get(void);
static void cmd_cal_set(const char *args, size_t len);
static void cmd_mode_get(void);
static void cmd_mode_set(const char *args, size_t len);

/*
 * Send a $PRSP response sentence over USB TTY.
 * Uses log_format_sentence() which appends checksum + CRLF.
 */
static void send_response(const char *fmt, ...)
{
    char buf[LOG_MAX_SENTENCE_LEN];
    va_list ap;

    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0 || (size_t)len >= sizeof(buf)) {
        return;
    }

    /* Append NMEA checksum and CRLF */
    if (nmea_append_checksum(buf, sizeof(buf)) == 0) {
        usb_tty_output_line(buf, strlen(buf));
    }
}

/*
 * Process a received line — called from system workqueue via usb_tty RX callback.
 */
static void usb_cmd_process_line(const char *line, size_t len)
{
    if (!cmd_active) {
        return;
    }

    /* Only process $PCMD sentences */
    if (len < 6 || strncmp(line, "$PCMD,", 6) != 0) {
        return;
    }

    /* Verify checksum */
    if (!nmea_verify_checksum(line, len)) {
        send_response("$PRSP,ERR,CHECKSUM");
        LOG_WRN("USB CMD: bad checksum: %s", line);
        return;
    }

    /* Find the command content between "$PCMD," and "*" */
    const char *cmd_start = line + 6;
    const char *star = strchr(cmd_start, '*');
    if (!star) {
        return;
    }
    size_t cmd_len = star - cmd_start;

    /* Dispatch based on command verb */
    if (cmd_len == 7 && strncmp(cmd_start, "CAL_GET", 7) == 0) {
        cmd_cal_get();
    } else if (cmd_len > 8 && strncmp(cmd_start, "CAL_SET,", 8) == 0) {
        cmd_cal_set(cmd_start + 8, cmd_len - 8);
    } else if (cmd_len == 8 && strncmp(cmd_start, "MODE_GET", 8) == 0) {
        cmd_mode_get();
    } else if (cmd_len > 9 && strncmp(cmd_start, "MODE_SET,", 9) == 0) {
        cmd_mode_set(cmd_start + 9, cmd_len - 9);
    } else {
        send_response("$PRSP,ERR,UNKNOWN");
        LOG_WRN("USB CMD: unknown command");
    }
}

static void cmd_cal_get(void)
{
    mag_calibration_t cal;
    mag_cal_get(&cal);

    send_response("$PRSP,CAL_GET,%d,%d,%d,%d,%u,%u,%u",
                  cal.valid ? 1 : 0,
                  cal.offset_x, cal.offset_y, cal.offset_z,
                  (unsigned)cal.scale_x, (unsigned)cal.scale_y,
                  (unsigned)cal.scale_z);

    LOG_INF("USB CMD: CAL_GET -> valid=%d", cal.valid);
}

static void cmd_cal_set(const char *args, size_t len)
{
    ARG_UNUSED(len);

    int32_t ox, oy, oz;
    unsigned int sx, sy, sz;

    if (sscanf(args, "%d,%d,%d,%u,%u,%u", &ox, &oy, &oz, &sx, &sy, &sz) != 6) {
        send_response("$PRSP,CAL_SET,ERR,%d", -EINVAL);
        LOG_WRN("USB CMD: CAL_SET parse error");
        return;
    }

    if (sx > UINT16_MAX || sy > UINT16_MAX || sz > UINT16_MAX) {
        send_response("$PRSP,CAL_SET,ERR,%d", -ERANGE);
        LOG_WRN("USB CMD: CAL_SET scale out of range");
        return;
    }

    mag_calibration_t cal = {
        .offset_x = ox,
        .offset_y = oy,
        .offset_z = oz,
        .scale_x = (uint16_t)sx,
        .scale_y = (uint16_t)sy,
        .scale_z = (uint16_t)sz,
        .valid = true,
    };

    int ret = mag_cal_set(&cal);
    if (ret != 0) {
        send_response("$PRSP,CAL_SET,ERR,%d", ret);
        LOG_ERR("USB CMD: mag_cal_set failed: %d", ret);
        return;
    }

    ret = mag_cal_save();
    if (ret != 0) {
        send_response("$PRSP,CAL_SET,ERR,%d", ret);
        LOG_ERR("USB CMD: mag_cal_save failed: %d", ret);
        return;
    }

    send_response("$PRSP,CAL_SET,OK");
    LOG_INF("USB CMD: CAL_SET OK [%d,%d,%d] [%u,%u,%u]",
            ox, oy, oz, sx, sy, sz);
}

static void cmd_mode_get(void)
{
    uint8_t mode = app_settings_get_mag_mode();

    send_response("$PRSP,MODE_GET,%u", (unsigned)mode);
    LOG_INF("USB CMD: MODE_GET -> %u", mode);
}

static void cmd_mode_set(const char *args, size_t len)
{
    ARG_UNUSED(len);

    unsigned int mode;

    if (sscanf(args, "%u", &mode) != 1 || mode > 2) {
        send_response("$PRSP,MODE_SET,ERR,%d", -EINVAL);
        LOG_WRN("USB CMD: MODE_SET invalid: %s", args);
        return;
    }

    int ret = app_settings_set_mag_mode((uint8_t)mode);
    if (ret != 0) {
        send_response("$PRSP,MODE_SET,ERR,%d", ret);
        LOG_ERR("USB CMD: app_settings_set_mag_mode failed: %d", ret);
        return;
    }

    send_response("$PRSP,MODE_SET,OK");
    LOG_INF("USB CMD: MODE_SET -> %u", mode);
}

void usb_cmd_init(void)
{
    usb_tty_register_rx_callback(usb_cmd_process_line);
    LOG_INF("USB command handler registered");
}

void usb_cmd_set_active(bool active)
{
    cmd_active = active;
    LOG_INF("USB commands %s", active ? "enabled" : "disabled");
}
