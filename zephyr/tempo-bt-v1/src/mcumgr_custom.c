/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Custom mcumgr Handlers
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
//#include <zephyr/mgmt/mcumgr/util/zcbor_bulk.h>

#include <zcbor_encode.h>
#include <zcbor_decode.h>

#include "services/storage.h"
#include "services/logger.h"
#include "services/led.h"
#include "config/settings.h"

LOG_MODULE_REGISTER(mcumgr_custom, LOG_LEVEL_INF);

/* Custom group ID */
#define MGMT_GROUP_ID_TEMPO    64

/* Command IDs */
#define TEMPO_MGMT_ID_SESSION_LIST     0
#define TEMPO_MGMT_ID_SESSION_INFO     1
#define TEMPO_MGMT_ID_STORAGE_INFO     2
#define TEMPO_MGMT_ID_LED_CONTROL      3
#define TEMPO_MGMT_ID_LOGGER_CONTROL   4
#define TEMPO_MGMT_ID_SESSION_DELETE   5
#define TEMPO_MGMT_ID_SETTINGS_GET     6
#define TEMPO_MGMT_ID_SETTINGS_SET     7

/* List callback context */
struct list_context {
    zcbor_state_t *zse;
    int count;
    bool error;
};


static int list_callback(const char *path, bool is_dir, size_t size, void *ctx)
{
    struct list_context *context = (struct list_context *)ctx;

    if (context->error) {
        return -1;  /* Stop on error */
    }

    if (is_dir) {
        /* Extract relative path - handle both /lfs/logs/ and /logs/ */
        const char *rel_path = path;
        if (strncmp(path, "/lfs/logs/", 10) == 0) {
            rel_path = path + 10;
        } else if (strncmp(path, "/logs/", 6) == 0) {
            rel_path = path + 6;
        }

        /* Start session object */
        bool ok = zcbor_map_start_encode(context->zse, 3) &&
                  zcbor_tstr_put_lit(context->zse, "name") &&
                  zcbor_tstr_put_term(context->zse, rel_path, 256) &&  /* Increased size */
                  zcbor_tstr_put_lit(context->zse, "is_dir") &&
                  zcbor_bool_put(context->zse, true) &&
                  zcbor_tstr_put_lit(context->zse, "size") &&
                  zcbor_uint32_put(context->zse, (uint32_t)size) &&
                  zcbor_map_end_encode(context->zse, 3);

        if (!ok) {
            context->error = true;
            return -1;  /* Stop on error */
        }

        context->count++;
    }

    return 0;  /* Continue */
}

/* List logging sessions */
static int tempo_mgmt_session_list(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    struct list_context list_ctx;
    bool ok;

    /* Start map */
    ok = zcbor_tstr_put_lit(zse, "sessions") &&
         zcbor_list_start_encode(zse, 20);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    /* Initialize list context */
    list_ctx.zse = zse;
    list_ctx.count = 0;
    list_ctx.error = false;

    /* List sessions from storage - use generic path */
    storage_list_dir("/logs", list_callback, &list_ctx);

    if (list_ctx.error) {
        return MGMT_ERR_EMSGSIZE;
    }

    /* End sessions array and add count */
    ok = zcbor_list_end_encode(zse, 20) &&
         zcbor_tstr_put_lit(zse, "count") &&
         zcbor_int32_put(zse, list_ctx.count);

    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    return 0;
}

struct delete_context {
    int error;
    int files_deleted;
};

/* Callback for deleting files in a directory */
static int delete_files_callback(const char *path, bool is_dir, size_t size, void *ctx)
{
    struct delete_context *del_ctx = (struct delete_context *)ctx;
    
    if (!is_dir) {
        /* Delete file */
        int ret = storage_delete(path);
        if (ret != 0 && ret != -ENOENT) {
            LOG_ERR("Failed to delete file %s: %d", path, ret);
            del_ctx->error = ret;
            return -1;  /* Stop on error */
        }
        del_ctx->files_deleted++;
        LOG_DBG("Deleted file: %s", path);
    }
    return 0;  /* Continue */
}

/* Delete a logging session */
static int tempo_mgmt_session_delete(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;  /* Decoder for request */
    zcbor_state_t *zse = ctxt->writer->zs;  /* Encoder for response */
    
    bool ok;
    struct zcbor_string key;
    struct zcbor_string session_name;
    bool has_session = false;
    
    /* Start decoding the map */
    ok = zcbor_map_start_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to start decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Decode the session parameter */
    while (zcbor_tstr_decode(zsd, &key)) {
        if (key.len == 7 && memcmp(key.value, "session", 7) == 0) {
            ok = zcbor_tstr_decode(zsd, &session_name);
            if (ok) {
                has_session = true;
            }
        } else {
            /* Skip unknown keys */
            ok = zcbor_any_skip(zsd, NULL);
        }
        
        if (!ok) {
            LOG_ERR("Failed to decode value");
            return MGMT_ERR_EINVAL;
        }
    }
    
    ok = zcbor_map_end_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to end decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    if (!has_session) {
        LOG_ERR("Missing session parameter");
        return MGMT_ERR_EINVAL;
    }
    
    /* Build the full path to the session directory */
    char session_path[256];
    snprintf(session_path, sizeof(session_path), "/logs/%.*s", 
             (int)session_name.len, session_name.value);
    
    LOG_INF("Deleting session: %s", session_path);
    
    /* Check if currently logging to this session */
    logger_state_t current_state = logger_get_state();
    if (current_state == LOGGER_STATE_LOGGING) {
        uint32_t session_id;
        uint64_t start_time;
        char current_session_path[256];
        
        if (logger_get_session_info(&session_id, &start_time, 
                                   current_session_path, sizeof(current_session_path)) == 0) {
            /* Check if trying to delete active session */
            if (strstr(current_session_path, session_path) != NULL) {
                LOG_ERR("Cannot delete active logging session");
                
                /* Return error response */
                ok = zcbor_tstr_put_lit(zse, "success") &&
                     zcbor_bool_put(zse, false) &&
                     zcbor_tstr_put_lit(zse, "error") &&
                     zcbor_tstr_put_lit(zse, "Session is currently active");
                
                if (!ok) {
                    return MGMT_ERR_EMSGSIZE;
                }
                
                return 0;  /* Return success with error in response */
            }
        }
    }
    
    /* Delete all files in the session directory */
    struct delete_context del_ctx = {0, 0};
    
    /* First pass: delete all files in the directory */
    storage_list_dir(session_path, delete_files_callback, &del_ctx);
    
    if (del_ctx.error != 0) {
        LOG_ERR("Error deleting session files");
        return MGMT_ERR_EUNKNOWN;
    }
    
    /* Delete the session directory itself */
    int ret = storage_delete(session_path);
    if (ret != 0 && ret != -ENOENT) {
        LOG_ERR("Failed to delete session directory: %d", ret);
        /* Continue anyway - we deleted the files */
    }
    
    /* Build success response */
    ok = zcbor_tstr_put_lit(zse, "success") &&
         zcbor_bool_put(zse, true) &&
         zcbor_tstr_put_lit(zse, "files_deleted") &&
         zcbor_int32_put(zse, del_ctx.files_deleted);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    LOG_INF("Session deleted successfully: %.*s (%d files)", 
            (int)session_name.len, session_name.value, del_ctx.files_deleted);
    
    return 0;
}

/* Get storage info */
static int tempo_mgmt_storage_info(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    uint64_t free_bytes, total_bytes;
    int ret;

    ret = storage_get_free_space(&free_bytes, &total_bytes);
    if (ret != 0) {
        return MGMT_ERR_EUNKNOWN;
    }

    uint32_t used_percent = (uint32_t)(100 - (free_bytes * 100 / total_bytes));
    
    /* Get backend type */
    storage_backend_t backend = storage_get_backend();
    const char *backend_str = (backend == STORAGE_BACKEND_FATFS) ? "sdcard" : "internal";

    bool ok = zcbor_tstr_put_lit(zse, "backend") &&
              zcbor_tstr_put_term(zse, backend_str, 10) &&
              zcbor_tstr_put_lit(zse, "free_bytes") &&
              zcbor_uint64_put(zse, free_bytes) &&
              zcbor_tstr_put_lit(zse, "total_bytes") &&
              zcbor_uint64_put(zse, total_bytes) &&
              zcbor_tstr_put_lit(zse, "used_percent") &&
              zcbor_uint32_put(zse, used_percent);

    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }

    return 0;
}

/* Logger control command handler */
static int tempo_mgmt_logger_control(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;  /* Decoder for request */
    zcbor_state_t *zse = ctxt->writer->zs;  /* Encoder for response */
    
    bool ok;
    struct zcbor_string key;
    struct zcbor_string action_str;
    bool has_action = false;
    
    /* Start decoding the map */
    ok = zcbor_map_start_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to start decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Decode the action parameter */
    while (zcbor_tstr_decode(zsd, &key)) {
        if (key.len == 6 && memcmp(key.value, "action", 6) == 0) {
            ok = zcbor_tstr_decode(zsd, &action_str);
            if (ok) {
                has_action = true;
            }
        } else {
            /* Skip unknown keys */
            ok = zcbor_any_skip(zsd, NULL);
        }
        
        if (!ok) {
            LOG_ERR("Failed to decode value");
            return MGMT_ERR_EINVAL;
        }
    }
    
    ok = zcbor_map_end_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to end decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    if (!has_action) {
        LOG_ERR("Missing action parameter");
        return MGMT_ERR_EINVAL;
    }
    
    /* Process the action */
    int ret = 0;
    logger_state_t current_state = logger_get_state();
    logger_state_t new_state = current_state;
    
    if (action_str.len == 5 && memcmp(action_str.value, "start", 5) == 0) {
        /* Start logging */
        if (current_state == LOGGER_STATE_ARMED) {
            ret = logger_start();
            if (ret == 0) {
                new_state = LOGGER_STATE_LOGGING;
            }
        } else if (current_state == LOGGER_STATE_IDLE) {
            /* Auto-arm then start */
            ret = logger_arm();
            if (ret == 0) {
                ret = logger_start();
                if (ret == 0) {
                    new_state = LOGGER_STATE_LOGGING;
                }
            }
        } else {
            LOG_WRN("Cannot start logging from state %d", current_state);
            ret = -EINVAL;
        }
    } else if (action_str.len == 4 && memcmp(action_str.value, "stop", 4) == 0) {
        /* Stop logging */
        if (current_state == LOGGER_STATE_LOGGING) {
            ret = logger_stop();
            if (ret == 0) {
                new_state = LOGGER_STATE_IDLE;
            }
        } else {
            LOG_WRN("Not logging, cannot stop");
            ret = -EINVAL;
        }
    } else if (action_str.len == 3 && memcmp(action_str.value, "arm", 3) == 0) {
        /* Arm logger */
        if (current_state == LOGGER_STATE_IDLE) {
            ret = logger_arm();
            if (ret == 0) {
                new_state = LOGGER_STATE_ARMED;
            }
        } else {
            LOG_WRN("Cannot arm from state %d", current_state);
            ret = -EINVAL;
        }
    } else if (action_str.len == 6 && memcmp(action_str.value, "disarm", 6) == 0) {
        /* Disarm logger */
        if (current_state == LOGGER_STATE_ARMED) {
            ret = logger_disarm();
            if (ret == 0) {
                new_state = LOGGER_STATE_IDLE;
            }
        } else {
            LOG_WRN("Cannot disarm from state %d", current_state);
            ret = -EINVAL;
        }
    } else {
        LOG_ERR("Unknown action: %.*s", action_str.len, action_str.value);
        return MGMT_ERR_EINVAL;
    }
    
    if (ret != 0) {
        return MGMT_ERR_EUNKNOWN;
    }
    
    /* Build response with current state and session info */
    const char *state_str = "unknown";
    switch (new_state) {
        case LOGGER_STATE_IDLE:       state_str = "idle"; break;
        case LOGGER_STATE_ARMED:      state_str = "armed"; break;
        case LOGGER_STATE_LOGGING:    state_str = "logging"; break;
        case LOGGER_STATE_JUMPED:    state_str = "jumped"; break;
        case LOGGER_STATE_POSTFLIGHT: state_str = "postflight"; break;
        case LOGGER_STATE_ERROR:      state_str = "error"; break;
    }
    
    ok = zcbor_tstr_put_lit(zse, "state") &&
         zcbor_tstr_put_term(zse, state_str, 10) &&
         zcbor_tstr_put_lit(zse, "success") &&
         zcbor_bool_put(zse, true);
    
    /* If logging, add session info */
    if (new_state == LOGGER_STATE_LOGGING) {
        uint32_t session_id;
        uint64_t start_time;
        char path[256];
        
        if (logger_get_session_info(&session_id, &start_time, path, sizeof(path)) == 0) {
            ok = ok && zcbor_tstr_put_lit(zse, "session_id") &&
                 zcbor_uint32_put(zse, session_id) &&
                 zcbor_tstr_put_lit(zse, "session_path") &&
                 zcbor_tstr_put_term(zse, path, sizeof(path));
        }
    }
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    LOG_INF("Logger control: action=%.*s, new_state=%s", 
            action_str.len, action_str.value, state_str);
    
    return 0;
}

/* LED control command handler */
static int tempo_mgmt_led_control(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;  /* Decoder for request */
    zcbor_state_t *zse = ctxt->writer->zs;  /* Encoder for response */
    
    bool ok;
    bool enable = false;
    uint32_t r = 0, g = 0, b = 0;
    bool has_enable = false;
    bool has_r = false, has_g = false, has_b = false;
    struct zcbor_string key;
    
    /* Start decoding the map */
    ok = zcbor_map_start_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to start decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Decode each key-value pair */
    while (zcbor_tstr_decode(zsd, &key)) {
        if (key.len == 6 && memcmp(key.value, "enable", 6) == 0) {
            ok = zcbor_bool_decode(zsd, &enable);
            if (ok) has_enable = true;
        }
        else if (key.len == 1 && key.value[0] == 'r') {
            ok = zcbor_uint32_decode(zsd, &r);
            if (ok) has_r = true;
        }
        else if (key.len == 1 && key.value[0] == 'g') {
            ok = zcbor_uint32_decode(zsd, &g);
            if (ok) has_g = true;
        }
        else if (key.len == 1 && key.value[0] == 'b') {
            ok = zcbor_uint32_decode(zsd, &b);
            if (ok) has_b = true;
        }
        else {
            /* Skip unknown keys */
            ok = zcbor_any_skip(zsd, NULL);
        }
        
        if (!ok) {
            LOG_ERR("Failed to decode value");
            return MGMT_ERR_EINVAL;
        }
    }
    
    ok = zcbor_map_end_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to end decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    LOG_INF("LED control: enable=%d, R=%d G=%d B=%d", 
            enable, (uint8_t)r, (uint8_t)g, (uint8_t)b);
    
    /* Validate color values */
    if (r > 255 || g > 255 || b > 255) {
        LOG_ERR("Invalid color values");
        return MGMT_ERR_EINVAL;
    }
    
    /* Apply the command */
    rgb_color_t color = {
        .r = (uint8_t)r,
        .g = (uint8_t)g,
        .b = (uint8_t)b
    };
    
    int ret = led_service_set_override(color, enable);
    if (ret != 0) {
        return MGMT_ERR_EUNKNOWN;
    }
    
    /* Build response with current state */
    rgb_color_t current_color;
    bool override_enabled;
    led_service_get_override(&current_color, &override_enabled);
    
    ok = zcbor_tstr_put_lit(zse, "enabled") &&
         zcbor_bool_put(zse, override_enabled) &&
         zcbor_tstr_put_lit(zse, "r") &&
         zcbor_uint32_put(zse, current_color.r) &&
         zcbor_tstr_put_lit(zse, "g") &&
         zcbor_uint32_put(zse, current_color.g) &&
         zcbor_tstr_put_lit(zse, "b") &&
         zcbor_uint32_put(zse, current_color.b);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    return 0;
}

/* Get NVM settings */
static int tempo_mgmt_settings_get(struct smp_streamer *ctxt)
{
    zcbor_state_t *zse = ctxt->writer->zs;
    
    /* Get current settings values */
    const char *ble_name = app_settings_get_ble_name();
    bool pps_enabled = app_settings_get_pps_enabled();
    uint8_t pcb_variant = app_settings_get_pcb_variant();
    const char *log_backend = app_settings_get_log_backend();
    
    /* Build response with all settings */
    bool ok = zcbor_tstr_put_lit(zse, "ble_name") &&
              zcbor_tstr_put_term(zse, ble_name, 32) &&
              zcbor_tstr_put_lit(zse, "pps_enabled") &&
              zcbor_bool_put(zse, pps_enabled) &&
              zcbor_tstr_put_lit(zse, "pcb_variant") &&
              zcbor_uint32_put(zse, pcb_variant) &&
              zcbor_tstr_put_lit(zse, "log_backend") &&
              zcbor_tstr_put_term(zse, log_backend, 12);
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    LOG_INF("Settings get: ble_name=%s, pps=%d, pcb=0x%02X, backend=%s", 
            ble_name, pps_enabled, pcb_variant, log_backend);
    
    return 0;
}

/* Set NVM settings */
static int tempo_mgmt_settings_set(struct smp_streamer *ctxt)
{
    zcbor_state_t *zsd = ctxt->reader->zs;  /* Decoder for request */
    zcbor_state_t *zse = ctxt->writer->zs;  /* Encoder for response */
    
    bool ok;
    struct zcbor_string key;
    struct zcbor_string str_value;
    uint32_t uint_value;
    int ret = 0;
    
    /* Values to update */
    bool has_ble_name = false;
    bool has_pps = false;
    bool has_pcb = false;
    bool has_backend = false;
    
    char new_ble_name[32];
    bool new_pps;
    uint8_t new_pcb = 1;
    char new_backend[12];
    
    /* Start decoding the map */
    ok = zcbor_map_start_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to start decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Decode each setting */
    while (zcbor_tstr_decode(zsd, &key)) {
        if (key.len == 8 && memcmp(key.value, "ble_name", 8) == 0) {
            ok = zcbor_tstr_decode(zsd, &str_value);
            if (ok && str_value.len < sizeof(new_ble_name)) {
                memcpy(new_ble_name, str_value.value, str_value.len);
                new_ble_name[str_value.len] = '\0';
                has_ble_name = true;
            }
        }
        else if (key.len == 11 && memcmp(key.value, "pps_enabled", 11) == 0) {
            ok = zcbor_bool_decode(zsd, &new_pps);
            if (ok) has_pps = true;
        }
        else if (key.len == 11 && memcmp(key.value, "pcb_variant", 11) == 0) {
            ok = zcbor_uint32_decode(zsd, &uint_value);
            if (ok && uint_value <= 255) {
                new_pcb = (uint8_t)uint_value;
                has_pcb = true;
            }
        }
        else if (key.len == 11 && memcmp(key.value, "log_backend", 11) == 0) {
            ok = zcbor_tstr_decode(zsd, &str_value);
            if (ok && str_value.len < sizeof(new_backend)) {
                memcpy(new_backend, str_value.value, str_value.len);
                new_backend[str_value.len] = '\0';
                has_backend = true;
            }
        }
        else {
            /* Skip unknown keys */
            ok = zcbor_any_skip(zsd, NULL);
        }
        
        if (!ok) {
            LOG_ERR("Failed to decode value");
            return MGMT_ERR_EINVAL;
        }
    }
    
    ok = zcbor_map_end_decode(zsd);
    if (!ok) {
        LOG_ERR("Failed to end decoding map");
        return MGMT_ERR_EINVAL;
    }
    
    /* Apply the settings */
    if (has_ble_name) {
        ret = app_settings_set_ble_name(new_ble_name);
        if (ret != 0) {
            LOG_ERR("Failed to set ble_name: %d", ret);
            return MGMT_ERR_EUNKNOWN;
        }
        LOG_INF("Set ble_name: %s", new_ble_name);
    }
    
    if (has_pps) {
        ret = app_settings_set_pps_enabled(new_pps);
        if (ret != 0) {
            LOG_ERR("Failed to set pps_enabled: %d", ret);
            return MGMT_ERR_EUNKNOWN;
        }
        LOG_INF("Set pps_enabled: %d", new_pps);
    }
    
    if (has_pcb) {
        ret = app_settings_set_pcb_variant(new_pcb);
        if (ret != 0) {
            LOG_ERR("Failed to set pcb_variant: %d", ret);
            return MGMT_ERR_EUNKNOWN;
        }
        LOG_INF("Set pcb_variant: 0x%02X", new_pcb);
    }
    
    if (has_backend) {
        ret = app_settings_set_log_backend(new_backend);
        if (ret != 0) {
            LOG_ERR("Failed to set log_backend: %d", ret);
            return MGMT_ERR_EUNKNOWN;
        }
        LOG_INF("Set log_backend: %s", new_backend);
    }
    
    /* Build response with all current settings (after updates) */
    const char *ble_name = app_settings_get_ble_name();
    bool pps_enabled = app_settings_get_pps_enabled();
    uint8_t pcb_variant = app_settings_get_pcb_variant();
    const char *log_backend = app_settings_get_log_backend();
    
    ok = zcbor_tstr_put_lit(zse, "ble_name") &&
         zcbor_tstr_put_term(zse, ble_name, 32) &&
         zcbor_tstr_put_lit(zse, "pps_enabled") &&
         zcbor_bool_put(zse, pps_enabled) &&
         zcbor_tstr_put_lit(zse, "pcb_variant") &&
         zcbor_uint32_put(zse, pcb_variant) &&
         zcbor_tstr_put_lit(zse, "log_backend") &&
         zcbor_tstr_put_term(zse, log_backend, 12) &&
         zcbor_tstr_put_lit(zse, "success") &&
         zcbor_bool_put(zse, true);
    
    if (has_ble_name) {
        ok = ok && zcbor_tstr_put_lit(zse, "note") &&
             zcbor_tstr_put_lit(zse, "BLE name changes require reboot");
    }
    
    if (!ok) {
        return MGMT_ERR_EMSGSIZE;
    }
    
    return 0;
}

/* Command handlers table */
static const struct mgmt_handler tempo_mgmt_handlers[] = {
    [TEMPO_MGMT_ID_SESSION_LIST] = {
        .mh_read = tempo_mgmt_session_list,
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_SESSION_INFO] = {
        .mh_read = NULL,  /* TODO: Implement */
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_STORAGE_INFO] = {
        .mh_read = tempo_mgmt_storage_info,
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_LED_CONTROL] = {
        .mh_read = NULL,
        .mh_write = tempo_mgmt_led_control,  
    },
    [TEMPO_MGMT_ID_LOGGER_CONTROL] = {
        .mh_read = NULL,
        .mh_write = tempo_mgmt_logger_control,  
    },
    [TEMPO_MGMT_ID_SESSION_DELETE] = {
        .mh_read = NULL,
        .mh_write = tempo_mgmt_session_delete,
    },
    [TEMPO_MGMT_ID_SETTINGS_GET] = {
        .mh_read = tempo_mgmt_settings_get,
        .mh_write = NULL,
    },
    [TEMPO_MGMT_ID_SETTINGS_SET] = {
        .mh_read = NULL,
        .mh_write = tempo_mgmt_settings_set,
    },
};

static struct mgmt_group tempo_mgmt_group = {
    .mg_handlers = tempo_mgmt_handlers,
    .mg_handlers_count = ARRAY_SIZE(tempo_mgmt_handlers),
    .mg_group_id = MGMT_GROUP_ID_TEMPO,
};

/* Register custom handlers */
void tempo_mgmt_register(void)
{
    mgmt_register_group(&tempo_mgmt_group);
    LOG_INF("Tempo custom mcumgr handlers registered");
}