/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/random/random.h>
#include <string.h>

LOG_MODULE_REGISTER(app_settings, LOG_LEVEL_INF);

/* Settings values with defaults */
static struct {
    /* BLE settings */
    char ble_name[32];
    
    /* Device UUIDs (zeroes means "Assign random id at startup")*/
    struct bt_uuid_128 device_uuid;
    
    /* Features */
    bool    pps_enabled;     /* PPS sync enabled (always false for V1) */
    uint8_t pcb_variant;     /* PCB revision level, e.g. 0x01 = V1, etc. */
    
    /* Storage */
    char log_backend[12]; /* "littlefs" or "fatfs" */

    /* Magnetometer */
    uint8_t mag_mode;    /* 0=disabled, 1=factory cal, 2=NVM cal */
} app_settings = {
    /* Default values */
    .ble_name = "Tempo-BT",
    .device_uuid = BT_UUID_INIT_128(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
    .pps_enabled = false, /* No PPS on V1 */
    .pcb_variant = 0x01,  /* V1 hardware */
    .log_backend = "littlefs",
    .mag_mode = 0         /* Disabled by default */
};

/* Generate a new random UUID */
int app_settings_generate_uuid(struct bt_uuid_128 *uuid)
{
    if (!uuid) {
        return -EINVAL;
    }

    /* Generate 16 random bytes */
    sys_rand_get(uuid->val, sizeof(uuid->val));
    
    /* Set UUID version to 4 (random) and variant bits */
    uuid->val[6] = (uuid->val[6] & 0x0F) | 0x40;  /* Version 4 */
    uuid->val[8] = (uuid->val[8] & 0x3F) | 0x80;  /* Variant bits */
    
    return 0;
}

/* Check if UUID is all zeros (uninitialized) */
static bool is_uuid_empty(const struct bt_uuid_128 *uuid)
{
    for (int i = 0; i < 16; i++) {
        if (uuid->val[i] != 0) {
            return false;
        }
    }
    return true;
}

/* Settings handler - must return 0 on success, negative on error */
static int settings_set_handler(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "ble_name", &next) && !next) {
        if (len >= sizeof(app_settings.ble_name)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.ble_name, len);
        if (rc < 0) {
            return rc;
        }
        app_settings.ble_name[len] = '\0';
        LOG_INF("Set ble_name: %s", app_settings.ble_name);
        return 0;
    }

    if (settings_name_steq(name, "device_uuid", &next) && !next) {
        if (len != sizeof(app_settings.device_uuid.val)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.device_uuid.val, len);
        if (rc < 0) {
            return rc;
        }
        char uuid_str[BT_UUID_STR_LEN];
        bt_uuid_to_str((const struct bt_uuid *)&app_settings.device_uuid,
                      uuid_str, sizeof(uuid_str));
        LOG_INF("Set device_uuid: %s", uuid_str);
        return 0;
    }

    if (settings_name_steq(name, "pps", &next) && !next) {
        if (len != sizeof(app_settings.pps_enabled)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.pps_enabled, len);
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Set pps_enabled: %d", app_settings.pps_enabled);
        return 0;
    }

    if (settings_name_steq(name, "pcb", &next) && !next) {
        if (len != sizeof(app_settings.pcb_variant)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.pcb_variant, len);
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Set pcb_variant: 0x%02X", app_settings.pcb_variant);
        return 0;
    }

    if (settings_name_steq(name, "log", &next) && !next) {
        if (len >= sizeof(app_settings.log_backend)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.log_backend, len);
        if (rc < 0) {
            return rc;
        }
        app_settings.log_backend[len] = '\0';
        LOG_INF("Set log_backend: %s", app_settings.log_backend);
        return 0;
    }

    if (settings_name_steq(name, "mag_mode", &next) && !next) {
        if (len != sizeof(app_settings.mag_mode)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.mag_mode, len);
        if (rc < 0) {
            return rc;
        }
        LOG_INF("Set mag_mode: %u", app_settings.mag_mode);
        return 0;
    }

    /* Unknown setting */
    return -ENOENT;
}

static struct settings_handler app_settings_handler = {
    .name = "app",
    .h_set = settings_set_handler
};

int app_settings_init(void)
{
    int rc;

    /* Initialize settings subsystem FIRST - settings_init() inside
     * settings_subsys_init() reinitializes the handlers list, so any
     * handlers registered before this call would be wiped out. */
    rc = settings_subsys_init();
    if (rc) {
        LOG_ERR("Failed to initialize settings subsystem: %d", rc);
        return rc;
    }

    /* Register settings handler AFTER subsystem is initialized */
    rc = settings_register(&app_settings_handler);
    if (rc) {
        LOG_ERR("Failed to register settings handler: %d", rc);
        return rc;
    }

    /* Load settings from NVS */
    rc = settings_load();
    if (rc) {
        LOG_ERR("Failed to load settings: %d", rc);
        return rc;
    }

    /* Generate UUIDs if they haven't been set */
    bool need_save = false;

    if (is_uuid_empty(&app_settings.device_uuid)) {
        LOG_INF("Generating new device UUID");
        rc = app_settings_generate_uuid(&app_settings.device_uuid);
        if (rc == 0) {
            need_save = true;
        } else {
            LOG_ERR("Failed to generate device UUID: %d", rc);
        }
    }

    if (need_save) {
        rc = settings_save_one("app/device_uuid",
                               app_settings.device_uuid.val,
                               sizeof(app_settings.device_uuid.val));
        if (rc) {
            LOG_ERR("Failed to save device UUID: %d", rc);
        }
    }

    LOG_INF("Settings loaded successfully");
    return 0;
}

/* Getter functions */
const char *app_settings_get_ble_name(void)
{
    return app_settings.ble_name;
}

const struct bt_uuid_128 *app_settings_get_device_uuid(void)
{
    return &app_settings.device_uuid;
}

const char *app_settings_get_log_backend(void)
{
    return app_settings.log_backend;
}

bool app_settings_get_pps_enabled(void)
{
    return app_settings.pps_enabled;
}

uint8_t app_settings_get_pcb_variant(void)
{
    return app_settings.pcb_variant;
}   

/* Setter functions with persistence */
int app_settings_set_ble_name(const char *name)
{
    if (!name || strlen(name) >= sizeof(app_settings.ble_name)) {
        return -EINVAL;
    }
    
    strcpy(app_settings.ble_name, name);
    return settings_save_one("app/ble_name", name, strlen(name));
}

#if 0
int app_settings_set_device_uuid(const struct bt_uuid_128 *uuid)
{
    if (!uuid) {
        return -EINVAL;
    }
    
    memcpy(&app_settings.device_uuid, uuid, sizeof(struct bt_uuid_128));
    return settings_save_one("app/device_uuid", uuid->val, sizeof(uuid->val));
}
#endif

int app_settings_set_pps_enabled(bool enabled)
{
    app_settings.pps_enabled = enabled;
    return settings_save_one("app/pps", &enabled, sizeof(enabled));
}   

int app_settings_set_pcb_variant(uint8_t variant)
{
    app_settings.pcb_variant = variant;
    return settings_save_one("app/pcb", &variant, sizeof(variant));
}

int app_settings_set_log_backend(const char *backend)
{
    if (!backend || strlen(backend) >= sizeof(app_settings.log_backend)) {
        return -EINVAL;
    }

    strcpy(app_settings.log_backend, backend);
    return settings_save_one("app/log", backend, strlen(backend));
}

uint8_t app_settings_get_mag_mode(void)
{
    return app_settings.mag_mode;
}

int app_settings_set_mag_mode(uint8_t mode)
{
    if (mode > 2) {
        return -EINVAL;
    }

    app_settings.mag_mode = mode;
    return settings_save_one("app/mag_mode", &mode, sizeof(mode));
}

/* Test function to demonstrate settings */
void app_settings_test(void)
{
    char uuid_str[BT_UUID_STR_LEN];
    
    LOG_INF("Current settings:");
    LOG_INF("  BLE name: %s", app_settings.ble_name);
    LOG_INF("  Log backend: %s", app_settings.log_backend);
    LOG_INF("  PPS enabled: %s", app_settings.pps_enabled ? "Yes" : "No");
    LOG_INF("  PCB variant: 0x%02X", app_settings.pcb_variant); 

    
    /* Display device UUID */
    bt_uuid_to_str((const struct bt_uuid *)&app_settings.device_uuid, 
                   uuid_str, sizeof(uuid_str));
    LOG_INF("  Device UUID: %s", uuid_str);
    
    /* Test UUID generation */
    struct bt_uuid_128 test_uuid;
    int rc = app_settings_generate_uuid(&test_uuid);
    if (rc == 0) {
        bt_uuid_to_str((const struct bt_uuid *)&test_uuid, 
                       uuid_str, sizeof(uuid_str));
        LOG_INF("  Generated test UUID: %s", uuid_str);
    } else {
        LOG_ERR("  Failed to generate test UUID: %d", rc);
    }
    
    LOG_INF("Settings test complete");
}