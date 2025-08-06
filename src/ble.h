#ifndef BLE_H
#define BLE_H

#include <zephyr/types.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum size of the notification buffer
#define SENSOR_NOTIFY_BUF_SIZE 160

// Public shared state and synchronization objects
extern struct bt_conn *current_conn;             // Pointer to active BLE connection
extern volatile bool notify_enabled;             // Whether notifications are enabled
extern char sensor_notify_buf[SENSOR_NOTIFY_BUF_SIZE];  // Global notify buffer
extern struct k_mutex notify_buf_mutex;          // Mutex to protect the buffer
//extern struct bt_gatt_service_static sensor_svc;

/**
 * @brief Initializes the BLE stack and starts advertising.
 *        This wraps `bt_enable(bt_ready)` and prints BLE MAC address.
 */
void ble_init(void);

/**
 * @brief (Optional API if needed externally) Sends the current buffer as a BLE notification.
 *        Not used directly in current code but can be exposed for clarity.
 */
void ble_send_notification(void); // Optional: not implemented yet

void bt_ready(int err);


#ifdef __cplusplus
}
#endif

#endif // BLE_H
