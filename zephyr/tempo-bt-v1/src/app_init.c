/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application Initialization
 */

#define PM_lfs_storage_ID PM_littlefs_storage_ID

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

LOG_MODULE_REGISTER(app_init, LOG_LEVEL_INF);

#include "ble_mcumgr.h"
#include "services/logger.h"
#include "services/storage.h"

extern void tempo_mgmt_register(void);

/* DFU safety callback */
static enum mgmt_cb_return dfu_mgmt_action(uint32_t event, enum mgmt_cb_return prev_status,
				       int32_t *rc, uint16_t *group, bool *abort_more, void *data,
				       size_t data_size)
{
    if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK) {
        /* Check if logging is active */
        logger_state_t state = logger_get_state();
        if (state == LOGGER_STATE_LOGGING) {
            LOG_WRN("DFU blocked: logging in progress");
            *abort_more = true;
            return MGMT_CB_ERROR_RC;
        }
    }
    
    return MGMT_CB_OK;
}

static struct mgmt_callback dfu_mgmt_cb = {
    .callback = dfu_mgmt_action,
    .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

/* Register DFU safety callback */
void dfu_safety_init(void)
{
    mgmt_callback_register(&dfu_mgmt_cb);
    LOG_INF("DFU safety callback registered");
}

/* Get the partition ID directly */
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(lfs_storage)

/* LittleFS work buffer and cache configuration */
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(littlefs_storage);

#if 0
/* Mount point - renamed to avoid conflict with lfs_mount function */
static struct fs_mount_t storage_mount_point = {
    .type = FS_LITTLEFS,
    .fs_data = &littlefs_storage,
    .storage_dev = (void *)STORAGE_PARTITION_ID,
    .mnt_point = "/lfs",
};
#endif

int app_storage_init(void)
{
    int ret;

    /* Try to initialize with auto-detection */
    ret = storage_auto_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize storage: %d", ret);
        return ret;
    }

    /* Get backend info */
    storage_backend_t backend = storage_get_backend();
    const char *backend_name = (backend == STORAGE_BACKEND_FATFS) ? "SD card" : "internal flash";
    
    LOG_INF("Storage initialized using %s", backend_name);

    /* Create logs directory based on backend if it doesn't exist */
    const char *logs_path = (backend == STORAGE_BACKEND_LITTLEFS) ? "/lfs/logs" : "/SD:/logs";
    struct fs_dirent entry;

    ret = fs_stat(logs_path, &entry);
    if (ret == 0) {
        LOG_INF("Logs directory exists");
    } else {
        ret = fs_mkdir(logs_path);
        if (ret < 0) {
            LOG_ERR("Error creating logs directory: %d", ret);
            return ret;
        }
        LOG_INF("Created logs directory");
    }

    /* Get filesystem statistics */
    uint64_t free_bytes, total_bytes;
    ret = storage_get_free_space(&free_bytes, &total_bytes);
    if (ret == 0) {
        LOG_INF("Storage: %llu MB free of %llu MB total",
                free_bytes / (1024*1024), 
                total_bytes / (1024*1024));
    }

    return 0;
}

int app_init(void)
{
    int ret;

    LOG_INF("Initializing Tempo-BT application");

    /* Note: Settings are now initialized inside ble_mcumgr_init() */
    /* to ensure the device name is available before BLE starts */

    /* Initialize BLE and mcumgr - this also starts advertising */
    ret = ble_mcumgr_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize BLE/mcumgr: %d", ret);
        return ret;
    }

    /* Note: BLE advertising is already started by ble_mcumgr_init() */
    /* No need to start it again here */

    //LOG_INF("Bluetooth initialized and advertising");

    /* Register custom mcumgr handlers */
    tempo_mgmt_register();

    /* Initialize DFU safety */
    dfu_safety_init();

    LOG_INF("app_init() complete");

    return 0;
}