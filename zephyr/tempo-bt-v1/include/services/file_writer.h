/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Tempo-BT V1 - File Writer Service
 *
 * Asynchronous buffered file writing with periodic flush
 */

#ifndef SERVICES_FILE_WRITER_H
#define SERVICES_FILE_WRITER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* File writer configuration */
typedef struct {
    size_t buffer_size;         /* Write buffer size (default 4KB) */
    uint32_t flush_interval_ms; /* Time-based flush interval (default 250ms) */
} file_writer_config_t;

/* File writer statistics */
typedef struct {
    uint32_t lines_written;
    uint64_t bytes_written;
    uint32_t flushes;
    uint32_t write_errors;
    uint32_t buffer_overflows;  /* Buffer-full episodes (consecutive drops count once) */
    uint32_t lines_dropped;     /* Whole lines discarded because buffer was full */
} file_writer_stats_t;

/**
 * @brief Initialize file writer service
 *
 * @param config Configuration parameters (NULL for defaults)
 * @return 0 on success, negative error code on failure
 */
int file_writer_init(const file_writer_config_t *config);

/**
 * @brief Deinitialize file writer service
 *
 * @return 0 on success, negative error code on failure
 */
int file_writer_deinit(void);

/**
 * @brief Open file for writing
 *
 * @param path File path
 * @return 0 on success, negative error code on failure
 */
int file_writer_open(const char *path);

/**
 * @brief Write data to buffer
 *
 * Data is buffered and written asynchronously. If buffer is full,
 * an immediate flush is triggered.
 *
 * @param data Data to write
 * @param len Length of data
 * @return 0 on success, negative error code on failure
 */
int file_writer_write(const void *data, size_t len);

/**
 * @brief Force flush of buffered data
 *
 * @return 0 on success, negative error code on failure
 */
int file_writer_flush(void);

/**
 * @brief Close current file
 *
 * Flushes all buffered data and closes the file
 *
 * @return 0 on success, negative error code on failure
 */
int file_writer_close(void);

/**
 * @brief Get writer statistics
 *
 * @param stats Output statistics structure
 */
void file_writer_get_stats(file_writer_stats_t *stats);

#endif /* SERVICES_FILE_WRITER_H */
