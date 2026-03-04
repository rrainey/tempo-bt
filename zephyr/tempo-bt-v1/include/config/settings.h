/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management Header
 */

#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/uuid.h>

/**
 * @brief Initialize settings subsystem
 * 
 * Registers handlers and loads saved settings from NVS
 * 
 * @return 0 on success, negative error code on failure
 */
int app_settings_init(void);

/**
 * @brief Get BLE device name
 * 
 * @return Current BLE name string
 */
const char *app_settings_get_ble_name(void);

/**
 * @brief Get device UUID
 * 
 * @return Pointer to device UUID
 */
const struct bt_uuid_128 *app_settings_get_device_uuid(void);

/**
 * @brief Get log backend type
 * 
 * @return Log backend string ("littlefs" or "fatfs")
 */
const char *app_settings_get_log_backend(void);

/**
 * @brief Get PPS enabled state
 * 
 * @return true if PPS is enabled (always false for V1)
 */
bool app_settings_get_pps_enabled(void);

/**
 * @brief Get PCB variant
 * 
 * @return PCB variant/revision number
 */
uint8_t app_settings_get_pcb_variant(void);

/**
 * @brief Set BLE device name
 * 
 * @param name New BLE name (max 31 chars)
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_ble_name(const char *name);

/**
 * @brief Set PPS enabled state
 * 
 * @param enabled true to enable PPS (will be forced to false on V1)
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_pps_enabled(bool enabled);

/**
 * @brief Set PCB variant
 * 
 * @param variant PCB variant/revision number
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_pcb_variant(uint8_t variant);

/**
 * @brief Set log backend type
 *
 * @param backend Backend type ("littlefs" or "fatfs")
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_log_backend(const char *backend);

/**
 * @brief Get magnetometer integration mode
 *
 * @return 0=disabled, 1=factory calibration, 2=NVM calibration
 */
uint8_t app_settings_get_mag_mode(void);

/**
 * @brief Set magnetometer integration mode
 *
 * @param mode 0=disabled, 1=factory calibration, 2=NVM calibration
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_mag_mode(uint8_t mode);

/**
 * @brief Generate a new random UUID
 * 
 * @param uuid Output UUID structure
 * @return 0 on success, negative error code on failure
 */
int app_settings_generate_uuid(struct bt_uuid_128 *uuid);

/**
 * @brief Test function to demonstrate settings
 */
void app_settings_test(void);

#endif /* CONFIG_SETTINGS_H */