/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - FAT/exFAT Storage Backend for SD Card
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <ff.h>
#include <string.h>
#include <stdio.h>

#include "services/storage.h"

LOG_MODULE_REGISTER(storage_fatfs, LOG_LEVEL_INF);

/* FAT filesystem mount point */
#define FATFS_MOUNT_POINT "/SD:"
#define STORAGE_PARTITION "SD" 

/* Maximum path length */
#define MAX_PATH_LEN 256

/* SD Card detect GPIO */
#define SD_DETECT_NODE DT_PATH(sdcard_detect, sdcard_detect_pin)
#if DT_NODE_HAS_STATUS(SD_DETECT_NODE, okay)
static const struct gpio_dt_spec sd_detect = GPIO_DT_SPEC_GET(SD_DETECT_NODE, gpios);
#endif

/* Storage state */
static struct {
    bool mounted;
    struct fs_mount_t mount_point;
    FATFS fat_fs;
} storage_state = {
    .mounted = false,
    .mount_point = {
        .type = FS_FATFS,
        .fs_data = &storage_state.fat_fs,
        .mnt_point = FATFS_MOUNT_POINT,
    }
};

/* Forward declarations */
static int ensure_mounted(void);
static int create_directory_path(const char *path);
static const char *translate_path(const char *log_path);

/* ILogSink implementation for FAT */
static int fatfs_open(void *ctx, const char *path, storage_file_t *file);
static int fatfs_write(void *ctx, storage_file_t *file, const void *data, size_t len);
static int fatfs_sync(void *ctx, storage_file_t *file);
static int fatfs_close(void *ctx, storage_file_t *file);
static int fatfs_get_free_space(void *ctx, uint64_t *free_bytes, uint64_t *total_bytes);
static int fatfs_delete(void *ctx, const char *path);
static int fatfs_exists(void *ctx, const char *path);
static int fatfs_list_dir(void *ctx, const char *path, storage_list_cb_t cb, void *cb_ctx);

/* Storage interface */
static const storage_interface_t fatfs_interface = {
    .open = fatfs_open,
    .write = fatfs_write,
    .sync = fatfs_sync,
    .close = fatfs_close,
    .get_free_space = fatfs_get_free_space,
    .delete = fatfs_delete,
    .exists = fatfs_exists,
    .list_dir = fatfs_list_dir,
};

/* Public API */
int storage_fatfs_init(void)
{
    int ret;
    uint32_t block_count;
    //struct disk_info info;

    /* Check if disk is present */
    ret = disk_access_init(STORAGE_PARTITION);
    if (ret != 0) {
        LOG_ERR("Failed to initialize disk access: %d", ret);
        return ret;
    }

    /* Get disk info */
    /*
    ret = disk_access_ioctl(STORAGE_PARTITION, DISK_IOCTL_GET_DISK_INFO, &info);
    if (ret != 0) {
        LOG_ERR("Failed to get disk info: %d", ret);
        return ret;
    }

    LOG_INF("SD Card detected: %s", info.name ? info.name : "Unknown");
    */
    /* Get disk size */
    ret = disk_access_ioctl(STORAGE_PARTITION, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
    if (ret == 0) {
        uint32_t block_size;
        ret = disk_access_ioctl(STORAGE_PARTITION, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
        if (ret == 0) {
            uint64_t total_size = (uint64_t)block_count * block_size;
            LOG_INF("SD Card size: %llu MB", total_size / (1024 * 1024));
        }
    }

    /* Try to mount */
    ret = ensure_mounted();
    if (ret != 0) {
        LOG_ERR("Failed to mount FAT filesystem: %d", ret);
        return ret;
    }

    return 0;
}

int storage_fatfs_deinit(void)
{
    int ret = 0;

    if (storage_state.mounted) {
        ret = fs_unmount(&storage_state.mount_point);
        if (ret == 0) {
            storage_state.mounted = false;
            LOG_INF("FAT filesystem unmounted");
        } else {
            LOG_ERR("Failed to unmount: %d", ret);
        }
    }

    return ret;
}

const storage_interface_t *storage_fatfs_get_interface(void)
{
    return &fatfs_interface;
}

/* Internal functions */
static int ensure_mounted(void)
{
    int ret;

    if (storage_state.mounted) {
        return 0;
    }

    /* Mount filesystem */
    ret = fs_mount(&storage_state.mount_point);
    if (ret != 0) {
        LOG_ERR("Failed to mount FAT filesystem: %d", ret);
        return ret;
    }

    storage_state.mounted = true;
    LOG_INF("FAT filesystem mounted at %s", FATFS_MOUNT_POINT);

    return 0;
}

static int create_directory_path(const char *full_path)
{
    char path[MAX_PATH_LEN];
    char *p;
    int ret;

    LOG_INF("create directory path %s", full_path);

    /* Make a working copy */
    strncpy(path, full_path, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';

    /* Find the last slash to separate directory from filename */
    p = strrchr(path, '/');
    if (p == NULL) {
        return 0;  /* No directory part */
    }

    /* Terminate at the last directory */
    *p = '\0';

    /* Create each directory in the path */
    p = path;
    while ((p = strchr(p + 1, '/')) != NULL) {
        *p = '\0';

        /* Try to create directory - ignore EEXIST */
        ret = fs_mkdir(path);
        if (ret != 0 && ret != -EEXIST) {
            LOG_ERR("Failed to create directory %s: %d", path, ret);
            return ret;
        }

        *p = '/';
    }

    /* Create the final directory */
    ret = fs_mkdir(path);
    if (ret != 0 && ret != -EEXIST) {
        LOG_ERR("Failed to create directory %s: %d", path, ret);
        return ret;
    }

    return 0;
}

/* Translate logical paths to FAT paths */
static const char *translate_path(const char *log_path)
{
    static char fat_path[MAX_PATH_LEN];
    
    /* Convert paths to /SD:/... */
    if (strncmp(log_path, "/SD:/", 5) == 0) {
        /* do nothing */
        strcpy(fat_path, log_path);
    } else if (strncmp(log_path, "/lfs/", 5) == 0) {
        /* Legacy /lfs/ prefix - strip it */
        snprintf(fat_path, sizeof(fat_path), "%s%s", FATFS_MOUNT_POINT, log_path + 4);
    } else if (log_path[0] == '/') {
        /* Absolute path without /lfs prefix */
        snprintf(fat_path, sizeof(fat_path), "%s%s", FATFS_MOUNT_POINT, log_path);
    } else {
        /* Relative path */
        snprintf(fat_path, sizeof(fat_path), "%s/%s", FATFS_MOUNT_POINT, log_path);
    }
    
    return fat_path;
}

/* ILogSink implementation */
static int fatfs_open(void *ctx, const char *path, storage_file_t *file)
{
    int ret;
    const char *fat_path;

    ARG_UNUSED(ctx);

    ret = ensure_mounted();
    if (ret != 0) {
        return ret;
    }

    fat_path = translate_path(path);
    
    /* Create directory structure if needed */
    ret = create_directory_path(fat_path);
    if (ret != 0) {
        return ret;
    }

    /* Open file for append/create */
    fs_file_t_init(&file->zfp);
    ret = fs_open(&file->zfp, fat_path, FS_O_CREATE | FS_O_APPEND | FS_O_WRITE);
    if (ret != 0) {
        LOG_ERR("Failed to open %s: %d", fat_path, ret);
        return ret;
    }

    file->is_open = true;
    LOG_DBG("Opened file: %s", fat_path);

    return 0;
}

static int fatfs_write(void *ctx, storage_file_t *file, const void *data, size_t len)
{
    ssize_t written;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    written = fs_write(&file->zfp, data, len);
    if (written < 0) {
        LOG_ERR("Write failed: %zd", written);
        return (int)written;
    }

    if (written != len) {
        LOG_WRN("Partial write: %zd/%zu bytes", written, len);
        return -EIO;
    }

    return 0;
}

static int fatfs_sync(void *ctx, storage_file_t *file)
{
    int ret;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    ret = fs_sync(&file->zfp);
    if (ret != 0) {
        LOG_ERR("Sync failed: %d", ret);
    }

    return ret;
}

static int fatfs_close(void *ctx, storage_file_t *file)
{
    int ret;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    ret = fs_close(&file->zfp);
    if (ret != 0) {
        LOG_ERR("Close failed: %d", ret);
    } else {
        file->is_open = false;
        LOG_DBG("File closed");
    }

    return ret;
}

static int fatfs_get_free_space(void *ctx, uint64_t *free_bytes, uint64_t *total_bytes)
{
    struct fs_statvfs stats;
    int ret;

    ARG_UNUSED(ctx);

    ret = ensure_mounted();
    if (ret != 0) {
        return ret;
    }

    ret = fs_statvfs(FATFS_MOUNT_POINT, &stats);
    if (ret != 0) {
        LOG_ERR("Failed to get filesystem stats: %d", ret);
        return ret;
    }

    if (free_bytes) {
        *free_bytes = (uint64_t)stats.f_bfree * stats.f_frsize;
    }

    if (total_bytes) {
        *total_bytes = (uint64_t)stats.f_blocks * stats.f_frsize;
    }

    return 0;
}

static int fatfs_delete(void *ctx, const char *path)
{
    const char *fat_path;
    int ret;

    ARG_UNUSED(ctx);

    ret = ensure_mounted();
    if (ret != 0) {
        return ret;
    }

    fat_path = translate_path(path);
    ret = fs_unlink(fat_path);
    
    if (ret != 0 && ret != -ENOENT) {
        LOG_ERR("Failed to delete %s: %d", fat_path, ret);
    }

    return ret;
}

static int fatfs_exists(void *ctx, const char *path)
{
    struct fs_dirent entry;
    const char *fat_path;
    int ret;

    ARG_UNUSED(ctx);

    ret = ensure_mounted();
    if (ret != 0) {
        return ret;
    }

    fat_path = translate_path(path);
    ret = fs_stat(fat_path, &entry);
    
    return (ret == 0) ? 1 : 0;
}

static int fatfs_list_dir(void *ctx, const char *path, storage_list_cb_t cb, void *cb_ctx)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    const char *fat_path;
    int ret;

    ARG_UNUSED(ctx);

    if (!cb) {
        return -EINVAL;
    }

    ret = ensure_mounted();
    if (ret != 0) {
        return ret;
    }

    fat_path = translate_path(path);
    fs_dir_t_init(&dir);

    ret = fs_opendir(&dir, fat_path);
    if (ret != 0) {
        LOG_ERR("Failed to open directory %s: %d", fat_path, ret);
        return ret;
    }

    while (fs_readdir(&dir, &entry) == 0) {
        if (entry.name[0] == '\0') {
            break;  /* End of directory */
        }

        /* Skip . and .. */
        if (strcmp(entry.name, ".") == 0 || strcmp(entry.name, "..") == 0) {
            continue;
        }

        /* Build full path for callback */
        char full_path[MAX_PATH_LEN];
        strncpy(full_path, path, sizeof(full_path));
        size_t len = strlen(path);
        size_t flen = strlen(entry.name);
        strcat(full_path, (len > 0 && path[len - 1] == '/') ? "" : "/");
        strncat(full_path, entry.name, sizeof(full_path)-1);
        full_path[MAX_PATH_LEN-1] = '\0';
        if (len + flen >= 254) {
            LOG_ERR("Full path exceeds maximum length: %s", full_path);
        }

        //snprintf(full_path, sizeof(full_path), "%s/%s", path, entry.name);

        /* Call the callback */
        ret = cb(full_path, entry.type == FS_DIR_ENTRY_DIR, entry.size, cb_ctx);
        if (ret != 0) {
            break;  /* Callback requested stop */
        }
    }

    fs_closedir(&dir);
    return ret;
}

/* Card detect helper */
bool storage_fatfs_card_present(void)
{
#if DT_NODE_HAS_STATUS(SD_DETECT_NODE, okay)
    int ret;
    
    /* Check if the GPIO device is ready */
    if (!device_is_ready(sd_detect.port)) {
        LOG_WRN("SD card detect GPIO not ready, falling back to disk status check");
        goto fallback_check;
    }
    
    /* Configure the GPIO pin as input with pull-up (card detect is active low) */
    ret = gpio_pin_configure_dt(&sd_detect, GPIO_INPUT);
    if (ret < 0) {
        LOG_WRN("Failed to configure SD detect pin: %d, falling back to disk status", ret);
        goto fallback_check;
    }
    
    /* Read the pin state - active low, so 0 = card present, 1 = no card */
    ret = gpio_pin_get_dt(&sd_detect);
    if (ret < 0) {
        LOG_WRN("Failed to read SD detect pin: %d", ret);
        goto fallback_check;
    }
    
    /* Return true if pin is low (card present) */
    bool card_present = (ret == 0);
    LOG_DBG("SD card detect GPIO: %s", card_present ? "card present" : "no card");
    return card_present;

fallback_check:
#endif
    /* Fallback: try to access the disk to detect presence */
    int ret1 = disk_access_status(STORAGE_PARTITION);
    bool disk_ok = (ret1 == DISK_STATUS_OK);
    LOG_DBG("SD card disk status: %s (code %d)", disk_ok ? "OK" : "Not OK", ret1);
    return disk_ok;
}