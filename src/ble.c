#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include "ble.h"

// Defines a custom 128-bit UUID for the Bluetooth service
struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);


// Defines a custom 128-bit UUID for the Bluetooth characteristic inside the service
struct bt_uuid_128 char_uuid = BT_UUID_INIT_128(
    0x01, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);


struct bt_conn *current_conn = NULL;     // Pointer to the current active Bluetooth connection
volatile bool notify_enabled = false;    // Flag to check if notifications are enabled by the client
char sensor_notify_buf[160];             // Buffer to hold the sensor data to be sent over BLE
struct k_mutex notify_buf_mutex;         // Mutex to protect access to the notify buffer from multiple threads




/*This function retrieves and prints the device’s BLE MAC address in a readable string format via printk().*/
void print_ble_address(void)
{
    bt_addr_le_t addr;              // Declares a variable addr to hold the BLE address
    size_t count = 1;               // Sets the number of BLE identity addresses to retrieve.
    bt_id_get(&addr, &count);       // Retrieves the device's local BLE identity address and stores it in addr.
    char buf[BT_ADDR_LE_STR_LEN];   // Declares a character buffer buf large enough to hold the string version of the BLE address.
    bt_addr_le_to_str(&addr, buf, sizeof(buf));     // Converts the binary address data in addr into a readable string format and stores it in buf
    printk("[BLE] Device BT Address: %s\n", buf);
}


/*This function is the BLE GATT read handler for a custom characteristic.
When a connected BLE client (e.g., smartphone or app) reads the characteristic, this function is called.
The function:

-> Locks a mutex to ensure thread-safe access to the buffer.

-> Reads the buffer content using Zephyr's helper.

-> Unlocks the mutex.

-> Logs the read operation.

-> Returns the result to the BLE stack.
*/
ssize_t read_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                void *buf, uint16_t len, uint16_t offset)
{
    // Lock the mutex to safely access the shared buffer (prevent race conditions)
    k_mutex_lock(&notify_buf_mutex, K_FOREVER);

    // Perform the actual BLE characteristic read from the buffer
    ssize_t ret = bt_gatt_attr_read(
        conn,               // The BLE connection from which read is requested
        attr,               // The attribute (characteristic) being read
        buf,                // Output buffer to send data to client
        len,                // Maximum length the client can accept
        offset,             // Offset if doing partial read
        sensor_notify_buf,  // Actual data to send
        strnlen(sensor_notify_buf, sizeof(sensor_notify_buf))       // Size of valid data in the buffer (excluding null terminator)
    );

    // Unlock the mutex after reading is complete
    k_mutex_unlock(&notify_buf_mutex);

    // Debug log showing read request and its parameters
    printk("[BLE] GATT: read_sensor (len=%u, offset=%u)\n", len, offset);
    return ret;     // Return number of bytes read to the BLE stack
}

/*This function is a callback handler that gets called whenever a Bluetooth Client Characteristic Configuration Descriptor (CCCD) value changes.
It checks if the client has enabled notifications.*/
void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("[BLE] CCCD config changed: notifications %s (value=0x%x)\n",
           notify_enabled ? "ENABLED" : "DISABLED", value);
}


/*This macro defines a custom Bluetooth GATT service called sensor_svc.
It exposes a characteristic that supports both:

-> Read operations (READ) from a BLE client

-> Notifications (NOTIFY) to a BLE client when new data is available (e.g., sensor values)

The service and characteristic use custom 128-bit UUIDs (service_uuid, char_uuid), and a CCCD (Client Characteristic Configuration Descriptor) is included so clients can enable/disable notifications dynamically.*/
BT_GATT_SERVICE_DEFINE(sensor_svc,                          // Define a custom GATT service named 'sensor_svc'
    BT_GATT_PRIMARY_SERVICE(&service_uuid),                 // Declare this as a primary BLE service with custom 128-bit UUID
    BT_GATT_CHARACTERISTIC(&char_uuid.uuid,                 // Define a characteristic with custom 128-bit UUID
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,            // This characteristic supports READ and NOTIFY properties
        BT_GATT_PERM_READ,                                  // Clients are allowed to READ the characteristic value
        read_sensor,                                        // Function that handles read requests from clients
        NULL,                                               // No write handler (write not supported)
        sensor_notify_buf),                                 // Pointer to the data buffer exposed via this characteristic
    BT_GATT_CCC(ccc_cfg_changed,                            // Define the CCCD (Client Characteristic Configuration) for enabling/disabling notifications
        BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)             // Clients can READ and WRITE the CCCD to enable/disable notifications
);




/*This defines a static advertising data array (ad) used for broadcasting over Bluetooth Low Energy (BLE) when the device is in advertising mode.*/
const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "ADPD_BLE", 8),
};


/*This is a callback function that gets called after the Bluetooth stack is initialized.
It's passed as an argument to bt_enable(bt_ready) during the Bluetooth setup process.

-> To check if Bluetooth initialization succeeded.

-> To start advertising if Bluetooth is ready.

-> To print the device’s BLE MAC address.

-> To log errors if anything fails.
*/
void bt_ready(int err) {
    printk("[MAIN] bt_ready callback, err=%d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("[BLE] Bluetooth initialized\n");
    print_ble_address();
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        printk("[BLE] Advertising failed to start (err %d)\n", err);
    } else {
        printk("[BLE] Advertising started successfully!\n");
    }
}


/*This function is a Bluetooth connection callback. It gets called automatically by the Zephyr Bluetooth stack when:

A BLE central (client) (e.g., phone or PC) connects to your device (peripheral) or when a connection attempt fails.*/
void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("[BLE] Connection failed (err %u)\n", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    printk("[BLE] Connected\n");
}


/*This function is a Bluetooth disconnection callback — it is automatically called by the Zephyr Bluetooth stack when a connected BLE central device (e.g., a phone or computer) disconnects from your device (the BLE peripheral).*/
void disconnected(struct bt_conn *conn, uint8_t reason) {
    printk("[BLE] Disconnected (reason 0x%02x)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled = false;
}



BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};
