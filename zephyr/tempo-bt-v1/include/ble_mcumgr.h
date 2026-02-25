#ifndef BLE_MCUMGR_H
#define BLE_MCUMGR_H

/**
 * @brief Initialize BLE stack, mcumgr transport, and start advertising
 *
 * @return 0 on success, negative error code on failure
 */
extern int ble_mcumgr_init(void);

/**
 * @brief Stop BLE advertising and disconnect any active connection
 *
 * Call this when entering an active logging session to eliminate
 * radio interference and save power during flight.  Safe to call
 * if BLE is already stopped.
 *
 * @return 0 on success, negative error code on failure
 */
int ble_mcumgr_stop(void);

/**
 * @brief Restart BLE advertising after a logging session ends
 *
 * Re-enables advertising so the device is discoverable again.
 * Safe to call if BLE is already advertising.
 *
 * @return 0 on success, negative error code on failure
 */
int ble_mcumgr_restart(void);

/**
 * @brief Schedule BLE stop after a short delay
 *
 * Used when logging is started via a BLE command so the mcumgr
 * response packet can be transmitted before the radio is disabled.
 * The default delay is 500 ms.
 */
void ble_mcumgr_stop_deferred(void);

#endif /* BLE_MCUMGR_H */