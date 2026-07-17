/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - File Writer Service Implementation
 *
 * Simplified async buffered writer using a ring buffer and worker thread.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>

#include "services/file_writer.h"
#include "services/storage.h"

LOG_MODULE_REGISTER(file_writer, LOG_LEVEL_INF);

/* Default configuration */
#define DEFAULT_BUFFER_SIZE     4096
#define DEFAULT_FLUSH_INTERVAL  250

/* Ring buffer size - should be larger than write buffer for headroom */
#define RING_BUFFER_SIZE        8192

/* Thread configuration */
#define WRITER_THREAD_STACK_SIZE 2048  /* Increased from 1024 for FAT FS operations */
#define WRITER_THREAD_PRIORITY   K_PRIO_PREEMPT(10)

/* Static allocations */
static K_THREAD_STACK_DEFINE(writer_thread_stack, WRITER_THREAD_STACK_SIZE);
static uint8_t ring_buffer_data[RING_BUFFER_SIZE];
static uint8_t write_buffer[DEFAULT_BUFFER_SIZE];

/* File writer state */
static struct {
    /* Configuration */
    size_t buffer_size;
    uint32_t flush_interval_ms;

    /* File state */
    storage_file_t file;
    bool file_open;
    char current_path[256];

    /* Ring buffer for incoming data.
     *
     * Producers (system workqueue, GNSS thread, main thread) are serialized
     * by ring_lock; the writer thread is the single unlocked consumer, which
     * Zephyr's ring_buf supports. Lines are enqueued whole or not at all —
     * a partial line in the ring corrupts the log stream for downstream
     * NMEA parsers.
     */
    struct ring_buf ring;
    struct k_spinlock ring_lock;
    uint32_t pending_drops;     /* Lines dropped in the current buffer-full episode */

    /* Thread */
    struct k_thread thread;
    k_tid_t tid;
    volatile bool running;
    struct k_sem data_ready;

    /* Flush work */
    struct k_work_delayable flush_work;

    /* Statistics */
    file_writer_stats_t stats;
    struct k_mutex stats_mutex;
} writer;

/* Forward declarations */
static void writer_thread_fn(void *p1, void *p2, void *p3);
static void flush_work_handler(struct k_work *work);
static int do_flush(void);

int file_writer_init(const file_writer_config_t *config)
{
    LOG_INF("Initializing file writer");

    memset(&writer, 0, sizeof(writer));

    /* Apply configuration */
    if (config) {
        writer.buffer_size = config->buffer_size > 0 ?
                             config->buffer_size : DEFAULT_BUFFER_SIZE;
        writer.flush_interval_ms = config->flush_interval_ms > 0 ?
                                   config->flush_interval_ms : DEFAULT_FLUSH_INTERVAL;
    } else {
        writer.buffer_size = DEFAULT_BUFFER_SIZE;
        writer.flush_interval_ms = DEFAULT_FLUSH_INTERVAL;
    }

    /* Clamp buffer size to our static buffer */
    if (writer.buffer_size > DEFAULT_BUFFER_SIZE) {
        writer.buffer_size = DEFAULT_BUFFER_SIZE;
    }

    /* Initialize ring buffer */
    ring_buf_init(&writer.ring, sizeof(ring_buffer_data), ring_buffer_data);

    /* Initialize synchronization */
    k_sem_init(&writer.data_ready, 0, 1);
    k_mutex_init(&writer.stats_mutex);

    /* Initialize flush work */
    k_work_init_delayable(&writer.flush_work, flush_work_handler);

    /* Start writer thread */
    writer.running = true;
    writer.tid = k_thread_create(&writer.thread,
                                 writer_thread_stack,
                                 WRITER_THREAD_STACK_SIZE,
                                 writer_thread_fn,
                                 NULL, NULL, NULL,
                                 WRITER_THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(writer.tid, "file_writer");

    LOG_INF("File writer initialized (buffer=%zu, flush=%ums)",
            writer.buffer_size, writer.flush_interval_ms);
    return 0;
}

int file_writer_deinit(void)
{
    LOG_INF("Deinitializing file writer");

    /* Stop thread */
    writer.running = false;
    k_sem_give(&writer.data_ready);  /* Wake thread */

    if (writer.tid) {
        k_thread_join(writer.tid, K_FOREVER);
        writer.tid = NULL;
    }

    /* Close file if open */
    if (writer.file_open) {
        file_writer_close();
    }

    /* Cancel flush work */
    k_work_cancel_delayable(&writer.flush_work);

    LOG_INF("File writer deinitialized");
    return 0;
}

int file_writer_open(const char *path)
{
    int ret;

    if (!path) {
        return -EINVAL;
    }

    if (writer.file_open) {
        LOG_WRN("File already open, closing first");
        file_writer_close();
    }

    LOG_INF("Opening file: %s", path);

    /* Open file through storage layer */
    ret = storage_open(path, &writer.file);
    if (ret != 0) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }

    writer.file_open = true;
    strncpy(writer.current_path, path, sizeof(writer.current_path) - 1);

    /* Reset statistics for new file */
    k_mutex_lock(&writer.stats_mutex, K_FOREVER);
    memset(&writer.stats, 0, sizeof(writer.stats));
    k_mutex_unlock(&writer.stats_mutex);

    /* Reset ring buffer */
    ring_buf_reset(&writer.ring);
    writer.pending_drops = 0;

    /* Schedule periodic flush */
    k_work_schedule(&writer.flush_work, K_MSEC(writer.flush_interval_ms));

    return 0;
}

int file_writer_write(const void *data, size_t len)
{
    uint32_t drops_ended = 0;
    bool new_episode = false;
    bool dropped = false;

    if (!data || len == 0) {
        return -EINVAL;
    }

    if (!writer.file_open) {
        return -ENOENT;
    }

    k_spinlock_key_t key = k_spin_lock(&writer.ring_lock);

    if (ring_buf_space_get(&writer.ring) < len) {
        new_episode = (writer.pending_drops == 0);
        writer.pending_drops++;
        dropped = true;
    } else {
        drops_ended = writer.pending_drops;
        writer.pending_drops = 0;
        ring_buf_put(&writer.ring, data, len);
    }

    k_spin_unlock(&writer.ring_lock, key);

    if (dropped) {
        k_mutex_lock(&writer.stats_mutex, K_FOREVER);
        writer.stats.lines_dropped++;
        if (new_episode) {
            writer.stats.buffer_overflows++;
        }
        k_mutex_unlock(&writer.stats_mutex);

        /* Wake writer thread; the line is gone but drain the backlog */
        k_sem_give(&writer.data_ready);
        return -ENOSPC;
    }

    /* Count lines (outside the spinlock; data is already enqueued whole) */
    uint32_t newlines = 0;
    const char *p = data;
    for (size_t i = 0; i < len; i++) {
        if (p[i] == '\n') {
            newlines++;
        }
    }

    k_mutex_lock(&writer.stats_mutex, K_FOREVER);
    writer.stats.lines_written += newlines;
    k_mutex_unlock(&writer.stats_mutex);

    if (drops_ended > 0) {
        /* Episode is over — pressure has eased enough to log about it */
        LOG_WRN("Buffer full: dropped %u lines", drops_ended);
    }

    /* Signal writer thread if buffer getting full */
    if (ring_buf_size_get(&writer.ring) > RING_BUFFER_SIZE / 2) {
        k_sem_give(&writer.data_ready);
    }

    return 0;
}

int file_writer_flush(void)
{
    if (!writer.file_open) {
        return -ENOENT;
    }

    /* Cancel scheduled flush */
    k_work_cancel_delayable(&writer.flush_work);

    /* Do immediate flush */
    return do_flush();
}

int file_writer_close(void)
{
    int ret;

    if (!writer.file_open) {
        return -ENOENT;
    }

    LOG_INF("Closing file: %s", writer.current_path);

    /* Cancel flush work */
    k_work_cancel_delayable(&writer.flush_work);

    /* Final flush */
    do_flush();

    /* Close file */
    writer.file_open = false;
    ret = storage_close(&writer.file);
    if (ret != 0) {
        LOG_ERR("Failed to close file: %d", ret);
    }

    /* Log final statistics */
    LOG_INF("File closed: %llu bytes, %u lines, %u flushes",
            writer.stats.bytes_written,
            writer.stats.lines_written,
            writer.stats.flushes);

    if (writer.stats.write_errors > 0) {
        LOG_WRN("Write errors: %u", writer.stats.write_errors);
    }
    if (writer.stats.buffer_overflows > 0) {
        LOG_WRN("Buffer overflows: %u (%u lines dropped)",
                writer.stats.buffer_overflows, writer.stats.lines_dropped);
    }

    return ret;
}

void file_writer_get_stats(file_writer_stats_t *stats)
{
    if (!stats) {
        return;
    }

    k_mutex_lock(&writer.stats_mutex, K_FOREVER);
    *stats = writer.stats;
    k_mutex_unlock(&writer.stats_mutex);
}

/* Writer thread - processes ring buffer and writes to storage */
static void writer_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("File writer thread started");

    while (writer.running) {
        /* Wait for data or timeout */
        k_sem_take(&writer.data_ready, K_MSEC(100));

        if (!writer.running) {
            break;
        }

        /* Process ring buffer if file is open */
        if (writer.file_open && ring_buf_size_get(&writer.ring) > 0) {
            do_flush();
        }
    }

    LOG_INF("File writer thread exiting");
}

/* Flush ring buffer contents to storage */
static int do_flush(void)
{
    uint32_t len;
    int ret = 0;

    if (!writer.file_open) {
        return -ENOENT;
    }

    /* Get data from ring buffer */
    len = ring_buf_get(&writer.ring, write_buffer, writer.buffer_size);

    if (len == 0) {
        return 0;  /* Nothing to flush */
    }

    /* Write to storage */
    ret = storage_write(&writer.file, write_buffer, len);
    if (ret == 0) {
        /* Sync to ensure data is on storage */
        storage_sync(&writer.file);

        k_mutex_lock(&writer.stats_mutex, K_FOREVER);
        writer.stats.bytes_written += len;
        writer.stats.flushes++;
        k_mutex_unlock(&writer.stats_mutex);
    } else {
        LOG_ERR("Write failed: %d", ret);
        k_mutex_lock(&writer.stats_mutex, K_FOREVER);
        writer.stats.write_errors++;
        k_mutex_unlock(&writer.stats_mutex);
    }

    /* If there's more data, process it */
    if (ring_buf_size_get(&writer.ring) > 0) {
        k_sem_give(&writer.data_ready);
    }

    return ret;
}

/* Periodic flush work handler */
static void flush_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);

    if (writer.file_open) {
        /* Signal thread to flush */
        k_sem_give(&writer.data_ready);

        /* Reschedule */
        k_work_schedule(&writer.flush_work, K_MSEC(writer.flush_interval_ms));
    }
}
