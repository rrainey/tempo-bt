/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - BLE and mcumgr Initialization
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/mgmt/mcumgr/grp/fs_mgmt/fs_mgmt.h>
//#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/stat_mgmt/stat_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/shell_mgmt/shell_mgmt.h>

#include "app/events.h"
#include "config/settings.h"

LOG_MODULE_REGISTER(ble_mcumgr, LOG_LEVEL_INF);

static struct k_work advertising_work;

/* Deferred BLE stop work item (500 ms delay) */
static struct k_work_delayable ble_stop_work;

/* Track whether BLE radio is currently active */
static bool ble_radio_active;

/* Storage for dynamic device name */
static char device_name[32] = "Tempo-BT";  /* Default fallback */

/* BLE advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
                  0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d), /* SMP/mcumgr UUID */
};

/* Scan response data - will be updated with dynamic name */
static struct bt_data sd[1];

/* Connection reference */
static struct bt_conn *current_conn;

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    
    LOG_INF("Connected");

    /* Update connection parameters for better throughput */
    struct bt_le_conn_param param = {
        .interval_min = 6,    /* 7.5 ms */
        .interval_max = 12,   /* 15 ms */
        .latency = 0,
        .timeout = 400,       /* 4 s */
    };

    int ret = bt_conn_le_param_update(conn, &param);
    if (ret) {
        LOG_WRN("Failed to request connection parameter update: %d", ret);
    }

    /* Emit connection event */
    app_event_t evt = {
        .type = EVT_BLE_CONNECTED
    };
    event_bus_publish(&evt);
}

static void advertising_work_handler(struct k_work *work)
{
    /* Don't restart advertising if radio has been intentionally stopped */
    if (!ble_radio_active) {
        LOG_INF("BLE radio stopped, not restarting advertising");
        return;
    }

    int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, 
                              ad, ARRAY_SIZE(ad), 
                              sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to restart advertising: %d", ret);
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason 0x%02x)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    /* Emit disconnection event */
    app_event_t evt = {
        .type = EVT_BLE_DISCONNECTED
    };
    event_bus_publish(&evt);

}

static void recycled_cb(void)
{
    LOG_INF("Connection recycled - safe to restart\n");
    k_work_submit(&advertising_work);
}


static void param_updated(struct bt_conn *conn, uint16_t interval,
                          uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);
    
    LOG_INF("Connection parameters updated: interval=%d, latency=%d, timeout=%d",
            interval, latency, timeout);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = param_updated,
    .recycled = recycled_cb,
};

/* File system access hooks for mcumgr */
static enum mgmt_cb_return fs_mgmt_upload_action(uint32_t event, enum mgmt_cb_return prev_status,
				       int32_t *rc, uint16_t *group, bool *abort_more, void *data,
				       size_t data_size)
{
    const char *path;
    
    if (event == MGMT_EVT_OP_FS_MGMT_FILE_ACCESS) {
        struct fs_mgmt_file_access *access = (struct fs_mgmt_file_access *)data;
        path = access->filename;
        
        /* Check write permissions */
        if (access->access == FS_MGMT_FILE_ACCESS_WRITE) {
            /* Allow write access to upload directory */
            if (strstr(path, "/lfs/upload/") == path) {
                return MGMT_CB_OK;
            }
            
            /* Deny all other writes */
            *abort_more = true;
            return MGMT_CB_ERROR_RC;
        } else {
            /* Read access checks */
            /* Deny access to system files */
            if (strstr(path, "/lfs/system/") == path) {
                *abort_more = true;
                return MGMT_CB_ERROR_RC;
            }
            
            /* Allow read access to everything else */
            return MGMT_CB_OK;
        }
    }
    
    return MGMT_CB_OK;
}

static struct mgmt_callback fs_mgmt_cb = {
    .callback = fs_mgmt_upload_action,
    .event_id = MGMT_EVT_OP_FS_MGMT_FILE_ACCESS,
};

/* Deferred BLE stop handler */
static void ble_stop_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    LOG_INF("Deferred BLE stop executing");
    ble_mcumgr_stop();
}

/* Initialize BLE and mcumgr */
int ble_mcumgr_init(void)
{
    int ret;

    LOG_INF("Initializing BLE and mcumgr");

    /* Initialize settings first to get the device name */
    ret = app_settings_init();
    if (ret) {
        LOG_WRN("Failed to initialize settings: %d, using default name", ret);
    } else {
        /* Get the configured device name */
        const char *configured_name = app_settings_get_ble_name();
        if (configured_name && strlen(configured_name) > 0) {
            strncpy(device_name, configured_name, sizeof(device_name) - 1);
            device_name[sizeof(device_name) - 1] = '\0';
            LOG_INF("Using configured device name: %s", device_name);
        } else {
            LOG_INF("No configured name, using default: %s", device_name);
        }
    }

    /* Update scan response data with the device name */
    sd[0].type = BT_DATA_NAME_COMPLETE;
    sd[0].data_len = strlen(device_name);
    sd[0].data = (const uint8_t *)device_name;

    /* Initialize Bluetooth */
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Failed to enable Bluetooth: %d", ret);
        return ret;
    }

    LOG_INF("Bluetooth initialized");

    /* Set the device name in the Bluetooth stack */
    ret = bt_set_name(device_name);
    if (ret) {
        LOG_WRN("Failed to set Bluetooth device name: %d", ret);
    }

    k_work_init(&advertising_work, advertising_work_handler);
    k_work_init_delayable(&ble_stop_work, ble_stop_work_handler);

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Initialize mcumgr transports - no need to call smp_bt_register() in newer versions */
    /* It's done automatically when CONFIG_MCUMGR_TRANSPORT_BT is enabled */

    /* Register mcumgr command groups - these are now done automatically via Kconfig */
    /* The groups are registered when their respective CONFIG options are enabled:
     * - CONFIG_MCUMGR_GRP_FS for filesystem
     * - CONFIG_MCUMGR_GRP_IMG for image management
     * - CONFIG_MCUMGR_GRP_OS for OS management
     * - CONFIG_MCUMGR_GRP_STAT for statistics
     * - CONFIG_MCUMGR_GRP_SHELL for shell
     */

    /* Register file system access callback */
    mgmt_callback_register(&fs_mgmt_cb);

    /* Start advertising */
    ble_radio_active = true;

    ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to start advertising: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising started with name: %s", device_name);

    return 0;
}

/* Stop BLE radio: disconnect + stop advertising */
int ble_mcumgr_stop(void)
{
    int ret;

    if (!ble_radio_active) {
        LOG_DBG("BLE radio already stopped");
        return 0;
    }

    LOG_INF("Stopping BLE radio for logging session");

    ble_radio_active = false;

    /* Stop advertising first so no new connections arrive */
    ret = bt_le_adv_stop();
    if (ret && ret != -EALREADY) {
        LOG_WRN("Failed to stop advertising: %d", ret);
    }

    /* Disconnect any active connection */
    if (current_conn) {
        ret = bt_conn_disconnect(current_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        if (ret) {
            LOG_WRN("Failed to disconnect: %d", ret);
        }
        /* The disconnected callback will unref current_conn */
    }

    LOG_INF("BLE radio stopped");
    return 0;
}

/* Restart BLE advertising */
int ble_mcumgr_restart(void)
{
    int ret;

    if (ble_radio_active) {
        LOG_DBG("BLE radio already active");
        return 0;
    }

    LOG_INF("Restarting BLE radio after logging session");

    ble_radio_active = true;

    ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (ret && ret != -EALREADY) {
        LOG_ERR("Failed to restart advertising: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising restarted");
    return 0;
}

/* Schedule BLE stop after delay (for BLE-initiated logging starts) */
void ble_mcumgr_stop_deferred(void)
{
    LOG_INF("Scheduling BLE stop in 500 ms");
    k_work_schedule(&ble_stop_work, K_MSEC(500));
}

/* Get current connection status */
bool ble_is_connected(void)
{
    return current_conn != NULL;
}

/* Get connection info */
int ble_get_conn_info(struct bt_conn_info *info)
{
    if (!current_conn || !info) {
        return -EINVAL;
    }

    return bt_conn_get_info(current_conn, info);
}

/* Get current device name */
const char *ble_get_device_name(void)
{
    return device_name;
}
