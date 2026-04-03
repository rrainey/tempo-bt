/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Logger Shell Commands
 */

#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>

#include "services/logger.h"
#include "services/file_writer.h"
#include "services/aggregator.h"
#include "services/storage.h"

/* Logger shell commands */
static int cmd_log_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    int ret = logger_start("shell_commanded_start");
    if (ret == 0) {
        shell_print(sh, "Logging started");
    } else {
        shell_error(sh, "Failed to start logging: %d", ret);
    }
    
    return ret;
}

static int cmd_log_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    int ret = logger_stop();
    if (ret == 0) {
        shell_print(sh, "Logging stopped");
    } else {
        shell_error(sh, "Failed to stop logging: %d", ret);
    }
    
    return ret;
}

static int cmd_log_status(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    logger_state_t state = logger_get_state();
    const char *state_str;
    
    switch (state) {
    case LOGGER_STATE_IDLE:       state_str = "IDLE"; break;
    case LOGGER_STATE_ARMED:      state_str = "ARMED"; break;
    case LOGGER_STATE_LOGGING:    state_str = "LOGGING"; break;
    case LOGGER_STATE_POSTFLIGHT: state_str = "POSTFLIGHT"; break;
    case LOGGER_STATE_ERROR:      state_str = "ERROR"; break;
    default:                      state_str = "UNKNOWN"; break;
    }
    
    shell_print(sh, "Logger state: %s", state_str);
    
    /* If logging, show session info */
    if (state == LOGGER_STATE_LOGGING || state == LOGGER_STATE_POSTFLIGHT) {
        uint32_t session_id;
        uint64_t start_time;
        char path[256];
        
        if (logger_get_session_info(&session_id, &start_time, path, sizeof(path)) == 0) {
            shell_print(sh, "Session ID: %08X", session_id);
            shell_print(sh, "Path: %s", path);
            
            /* Show file writer stats */
            file_writer_stats_t stats;
            file_writer_get_stats(&stats);
            shell_print(sh, "Lines written: %u", stats.lines_written);
            shell_print(sh, "Bytes written: %llu", stats.bytes_written);
            shell_print(sh, "Queue usage: %d%%", file_writer_get_queue_usage());
            
            if (stats.write_errors > 0) {
                shell_warn(sh, "Write errors: %u", stats.write_errors);
            }
            
            /* Show dropped counts */
            bool any_dropped = false;
            for (int i = 0; i < WRITE_PRIORITY_COUNT; i++) {
                if (stats.dropped[i] > 0) {
                    any_dropped = true;
                    break;
                }
            }
            
            if (any_dropped) {
                shell_warn(sh, "Dropped samples:");
                shell_warn(sh, "  Critical (IMU): %u", stats.dropped[WRITE_PRIORITY_CRITICAL]);
                shell_warn(sh, "  High (Baro): %u", stats.dropped[WRITE_PRIORITY_HIGH]);
                shell_warn(sh, "  Medium (Mag): %u", stats.dropped[WRITE_PRIORITY_MEDIUM]);
                shell_warn(sh, "  Low (GNSS): %u", stats.dropped[WRITE_PRIORITY_LOW]);
            }
        }
    }
    
    /* Show aggregator stats */
    uint32_t imu_count, env_count, gnss_count, agg_dropped;
    aggregator_get_stats(&imu_count, &env_count, &gnss_count, &agg_dropped);
    
    shell_print(sh, "\nAggregator stats:");
    shell_print(sh, "  IMU samples: %u", imu_count);
    shell_print(sh, "  ENV samples: %u", env_count);
    shell_print(sh, "  GNSS fixes: %u", gnss_count);
    if (agg_dropped > 0) {
        shell_warn(sh, "  Dropped: %u", agg_dropped);
    }
    
    return 0;
}

static int cmd_log_arm(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    int ret = logger_arm();
    if (ret == 0) {
        shell_print(sh, "Logger armed");
    } else {
        shell_error(sh, "Failed to arm logger: %d", ret);
    }
    
    return ret;
}

static int cmd_log_disarm(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    int ret = logger_disarm();
    if (ret == 0) {
        shell_print(sh, "Logger disarmed");
    } else {
        shell_error(sh, "Failed to disarm logger: %d", ret);
    }
    
    return ret;
}

/* Health monitoring command */
static int cmd_health_show(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    /* Storage health */
    uint64_t free_bytes, total_bytes;
    if (storage_get_free_space(&free_bytes, &total_bytes) == 0) {
        uint32_t free_mb = free_bytes / (1024 * 1024);
        uint32_t total_mb = total_bytes / (1024 * 1024);
        uint32_t used_percent = 100 - (free_bytes * 100 / total_bytes);
        
        shell_print(sh, "Storage: %u/%u MB used (%u%%)", 
                    total_mb - free_mb, total_mb, used_percent);
    }
    
    /* TODO: Add sensor health status when implemented */
    shell_print(sh, "IMU: OK");
    shell_print(sh, "Baro: OK");
    shell_print(sh, "GNSS: OK");
    
    return 0;
}

/* Storage commands */
static int cmd_storage_list(const struct shell *sh, size_t argc, char **argv)
{
    const char *path = "/lfs/logs";
    
    if (argc > 1) {
        path = argv[1];
    }
    
    shell_print(sh, "Listing: %s", path);
    
    struct list_context {
        const struct shell *sh;
        int count;
    } ctx = { .sh = sh, .count = 0 };
    
    int ret = storage_list_dir(path, 
        [](const char *name, bool is_dir, size_t size, void *ctx) -> int {
            struct list_context *lctx = ctx;
            if (is_dir) {
                shell_print(lctx->sh, "  [DIR] %s", name);
            } else {
                shell_print(lctx->sh, "  %s (%zu bytes)", name, size);
            }
            lctx->count++;
            return 0;
        }, &ctx);
    
    if (ret != 0) {
        shell_error(sh, "Failed to list directory: %d", ret);
    } else {
        shell_print(sh, "Total: %d entries", ctx.count);
    }
    
    return ret;
}

static int cmd_storage_delete(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_error(sh, "Usage: storage delete <path>");
        return -EINVAL;
    }
    
    int ret = storage_delete(argv[1]);
    if (ret == 0) {
        shell_print(sh, "Deleted: %s", argv[1]);
    } else {
        shell_error(sh, "Failed to delete %s: %d", argv[1], ret);
    }
    
    return ret;
}

/* Sub-command structures */
SHELL_STATIC_SUBCMD_SET_CREATE(log_cmds,
    SHELL_CMD(start, NULL, "Start logging", cmd_log_start),
    SHELL_CMD(stop, NULL, "Stop logging", cmd_log_stop),
    SHELL_CMD(status, NULL, "Show logging status", cmd_log_status),
    SHELL_CMD(arm, NULL, "Arm logger", cmd_log_arm),
    SHELL_CMD(disarm, NULL, "Disarm logger", cmd_log_disarm),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(storage_cmds,
    SHELL_CMD(list, NULL, "List directory contents", cmd_storage_list),
    SHELL_CMD(delete, NULL, "Delete file or directory", cmd_storage_delete),
    SHELL_SUBCMD_SET_END
);

/* Register commands */
SHELL_CMD_REGISTER(log, &log_cmds, "Logging commands", NULL);
SHELL_CMD_REGISTER(health, NULL, "Show system health", cmd_health_show);
SHELL_CMD_REGISTER(storage, &storage_cmds, "Storage commands", NULL);