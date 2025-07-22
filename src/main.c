#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <zephyr/types.h>

#define I2C_NODE DT_NODELABEL(i2c0)    // Refers to the I2C0 hardware device using device tree label
#define ADPD144RI_I2C_ADDR 0x64        // I2C slave address
#define NUM_CHANNELS 3                 // Number of sensor data channels to read
#define MSGQ_SIZE 10                   // Size of message queue

#define UNIX_TIME_START 1718000000UL   // Unix timestamp (set it manually)
#define LFCLK_FREQ 32768UL             // Low-frequency clock 32768 Hz
#define RTC2_PRESCALER 0
#define RTC_TICKS_PER_SEC (LFCLK_FREQ / (RTC2_PRESCALER + 1))  // Number of RTC ticks per second

typedef struct {
    uint32_t sample_id;                    // Unique sample id for each sensor sample 
    uint64_t timestamp_ms;                 // Timestamp when the sample is taken
    uint32_t channels[NUM_CHANNELS];       // Array to store multiple channel data
} SampleEntry;


// Defines a message queue named 'sensor_msgq'
// Each message is the size of a 'SampleEntry'
// Queue can hold up to 'MSGQ_SIZE' (10) messages
// Aligns each message on a 4-byte boundary for memory safety
K_MSGQ_DEFINE(sensor_msgq, sizeof(SampleEntry), MSGQ_SIZE, 4);


// Defines a custom 128-bit UUID for the Bluetooth service
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);


// Defines a custom 128-bit UUID for the Bluetooth characteristic inside the service
static struct bt_uuid_128 char_uuid = BT_UUID_INIT_128(
    0x01, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xcd, 0xab);


static struct bt_conn *current_conn = NULL;     // Pointer to the current active Bluetooth connection
static volatile bool notify_enabled = false;    // Flag to check if notifications are enabled by the client
static char sensor_notify_buf[128];             // Buffer to hold the sensor data to be sent over BLE
static struct k_mutex notify_buf_mutex;         // Mutex to protect access to the notify buffer from multiple threads


/*This function initializes the Real-Time Counter 2 (RTC2) on the Nordic SoC to function as a system timer for timestamping sensor data. It performs the following steps:

-> Starts the low-frequency clock (LFCLK) using an external crystal.

-> Stops, resets, and sets the prescaler for RTC2.

-> Starts RTC2 to run continuously.

-> It is used later to provide precise time stamps in milliseconds for sampled sensor data.*/

static void rtc2_init(void) {
    // Print a message to indicate RTC2 initialization has started
    printk("[RTC2] Initializing RTC2...\n");

    // Set the LFCLK (low-frequency clock) source to external crystal oscillator
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;

    // Clear the 'LFCLK started' event flag
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;

    // Trigger the task to start the LFCLK
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    // Wait until the LFCLK has started (polling the event flag)
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED) {}

    NRF_RTC2->TASKS_STOP = 1;               // Stop RTC2 before reconfiguration
    NRF_RTC2->PRESCALER = RTC2_PRESCALER;   // Set the RTC2 prescaler to control tick frequency (0 = no division)
    NRF_RTC2->TASKS_CLEAR = 1;              // Clear the RTC2 counter (reset to 0)
    NRF_RTC2->TASKS_START = 1;              // Start the RTC2 counter
    
    // Print a message indicating RTC2 is now running with configured settings
    printk("[RTC2] RTC2 running (prescaler=%d, unix start=%lu).\n", RTC2_PRESCALER, (unsigned long)UNIX_TIME_START);
}


/*This function returns the current timestamp in milliseconds based on the number of ticks from the RTC2 hardware counter.

It calculates the elapsed time since a defined starting point (UNIX_TIME_START) and adds the milliseconds computed from the counter ticks.*/
static uint64_t get_timestamp_ms(void) {
    uint32_t ticks = NRF_RTC2->COUNTER;               // Read the current tick count from the RTC2 hardware counter (increments every ~30.5 µs)
    uint64_t ms = ((uint64_t)ticks * 1000UL) / RTC_TICKS_PER_SEC;       // Convert tick count to milliseconds using the tick rate (32768 ticks/sec)
    return ((uint64_t)UNIX_TIME_START * 1000UL) + ms;           // Return the full timestamp = initial UNIX time (in ms) + elapsed milliseconds
}


/*This function converts a timestamp into a human-readable format (dd/mm/yyyy  hr:min:sec:ms)*/
static void format_timestamp(uint64_t timestamp_ms, char *buffer, size_t size) {
    time_t sec = timestamp_ms / 1000;       // Convert milliseconds to seconds (standard Unix timestamp format)
    uint16_t ms = timestamp_ms % 1000;      // Extract the milliseconds part (remainder after dividing by 1000)
    struct tm t;                            // Structure to hold broken-down time (year, month, day, hour, etc.)
    gmtime_r(&sec, &t);                     // Convert 'sec' into UTC time represented in 'struct tm' (thread-safe version)

    // Format the date & time into the buffer as: dd/mm/yyyy hh:mm:ss.mmm
    snprintf(buffer, size, "%02d/%02d/%04d %02d:%02d:%02d.%03d",
        t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
        t.tm_hour, t.tm_min, t.tm_sec, ms);
}


/*This function retrieves and prints the device’s BLE MAC address in a readable string format via printk().*/
static void print_ble_address(void)
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
static ssize_t read_sensor(struct bt_conn *conn, const struct bt_gatt_attr *attr,
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
static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
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

/*This function writes a 16-bit value to a register of the ADPD144RI sensor via the I2C interface.*/
static int adpd144ri_write_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = { reg, value >> 8, value & 0xFF };
    int ret = i2c_write(i2c_dev, buf, sizeof(buf), ADPD144RI_I2C_ADDR);
    if (ret)
        printk("[I2C] Write reg 0x%02X failed: %d\n", reg, ret);
    return ret;
}


/*This function reads a 16-bit register value from the ADPD144RI optical sensor using the I2C interface.*/
static int adpd144ri_read_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    int ret = i2c_write_read(i2c_dev, ADPD144RI_I2C_ADDR, &reg, 1, data, 2);
    if (ret)
        printk("[I2C] Read reg 0x%02X failed: %d\n", reg, ret);
    else
        *value = (data[0] << 8) | data[1];
    return ret;
}


/*This function sets up (initializes and configures) the ADPD144RI sensor module via I2C for SPO2 (blood oxygen) and heart rate (HR) monitoring.*/
static int adpd144ri_configure_spo2_hr(const struct device *i2c_dev) {
    printk("[I2C] Configuring ADPD144RI sensor ...\n");
    int rc = 0;
    rc += adpd144ri_write_reg16(i2c_dev, 0x10, 0x0001);
    rc += adpd144ri_write_reg16(i2c_dev, 0x02, 0x0005);     
    rc += adpd144ri_write_reg16(i2c_dev, 0x01, 0x009F);
    rc += adpd144ri_write_reg16(i2c_dev, 0x11, 0x30A9);
    rc += adpd144ri_write_reg16(i2c_dev, 0x12, 0x0384);
    rc += adpd144ri_write_reg16(i2c_dev, 0x15, 0x0330);
    rc += adpd144ri_write_reg16(i2c_dev, 0x14, 0x0116);
    rc += adpd144ri_write_reg16(i2c_dev, 0x18, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x19, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1A, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1B, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1E, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x1F, 0x3FFF);
    rc += adpd144ri_write_reg16(i2c_dev, 0x20, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x21, 0x1FF0);
    rc += adpd144ri_write_reg16(i2c_dev, 0x23, 0x3005);
    rc += adpd144ri_write_reg16(i2c_dev, 0x24, 0x3007);
    rc += adpd144ri_write_reg16(i2c_dev, 0x25, 0x0207);
    rc += adpd144ri_write_reg16(i2c_dev, 0x30, 0x0319);
    rc += adpd144ri_write_reg16(i2c_dev, 0x31, 0x0813);
    rc += adpd144ri_write_reg16(i2c_dev, 0x35, 0x0319);
    rc += adpd144ri_write_reg16(i2c_dev, 0x36, 0x0813);
    rc += adpd144ri_write_reg16(i2c_dev, 0x39, 0x21F3);
    rc += adpd144ri_write_reg16(i2c_dev, 0x3B, 0x21F3);
    rc += adpd144ri_write_reg16(i2c_dev, 0x42, 0x1C36);
    rc += adpd144ri_write_reg16(i2c_dev, 0x44, 0x1C36);
    rc += adpd144ri_write_reg16(i2c_dev, 0x4E, 0x0040);
    rc += adpd144ri_write_reg16(i2c_dev, 0x4B, 0x0080);
    rc += adpd144ri_write_reg16(i2c_dev, 0x10, 0x0002);
    k_msleep(50);
    if (rc)
        printk("[I2C] One or more sensor config writes failed: rc=%d\n", rc);
    else
        printk("[I2C] ADPD144RI configuration done.\n");
    return rc;
}


/*This function reads three 24-bit sensor channel values from the ADPD144RI sensor using I2C*/
static int adpd144ri_read_3ch(const struct device *i2c_dev, uint32_t *data) {
    int rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0006);
    if (rc) { printk("[I2C] Measurement reg set failed: %d\n", rc); return rc; }

    uint16_t ch3_l=0, ch3_h=0, ch4_l=0, ch4_h=0, bch3_l=0, bch3_h=0;

    rc |= adpd144ri_read_reg16(i2c_dev, 0x72, &ch3_l);      // Read CH3 LSB 
    rc |= adpd144ri_read_reg16(i2c_dev, 0x76, &ch3_h);      // Read CH3 MSB 
    rc |= adpd144ri_read_reg16(i2c_dev, 0x73, &ch4_l);      // Read CH4 LSB
    rc |= adpd144ri_read_reg16(i2c_dev, 0x77, &ch4_h);      // Read CH4 MSB
    rc |= adpd144ri_read_reg16(i2c_dev, 0x7A, &bch3_l);     // Read background CH3 LSB
    rc |= adpd144ri_read_reg16(i2c_dev, 0x7E, &bch3_h);     // Read background CH3 MSB

    if (rc) { printk("[I2C] One or more channel reads failed: rc=%d\n", rc); return rc; }

    data[0] = ((uint32_t)ch3_h << 16) | ch3_l;      // CH3 = MSB << 16 | LSB
    data[1] = ((uint32_t)ch4_h << 16) | ch4_l;      // CH4 = MSB << 16 | LSB  
    data[2] = ((uint32_t)bch3_h << 16) | bch3_l;    // Background CH3 = MSB << 16 | LSB

    rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0000);
    if (rc) { printk("[I2C] Measurement reg reset failed: %d\n", rc); return rc; }
    return 0;
}


/*This defines a static advertising data array (ad) used for broadcasting over Bluetooth Low Energy (BLE) when the device is in advertising mode.*/
static const struct bt_data ad[] = {
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
static void bt_ready(int err) {
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
static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        printk("[BLE] Connection failed (err %u)\n", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    printk("[BLE] Connected\n");
}


/*This function is a Bluetooth disconnection callback — it is automatically called by the Zephyr Bluetooth stack when a connected BLE central device (e.g., a phone or computer) disconnects from your device (the BLE peripheral).*/
static void disconnected(struct bt_conn *conn, uint8_t reason) {
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



/*This function runs inside a dedicated Zephyr thread and is responsible for:

-> Initializing the I2C-connected ADPD144RI optical sensor

-> Reading sensor data (3 channels)

-> Time-stamping each sample

-> Sending the data to a message queue (sensor_msgq) for further processing or transmission via BLE

-> Reacting to BLE notifications being enabled/disabled

It operates in a state-driven, event-based loop and works only when a BLE central (like a phone app) has enabled notifications.*/
void sensor_thread(void) {
    static uint32_t sample_id = 1;
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    printk("[THREAD] sensor_thread started\n");

    if (!device_is_ready(i2c_dev)) {
        printk("[I2C] device %s not ready!\n", i2c_dev->name);
        printk("[SENSOR] Thread will now sleep forever (does not block system).\n");
        while (1) k_msleep(1000);
    }

    rtc2_init();

    while (1) {
        while (!notify_enabled) {
            k_msleep(200);
        }

        while (adpd144ri_configure_spo2_hr(i2c_dev) != 0) {
            printk("[SENSOR] Sensor config failed, sleeping and retrying...\n");
            k_msleep(200);
            if (!notify_enabled) break;
        }
        if (!notify_enabled) continue;

        printk("[SENSOR] Measurement loop started\n");

        while (notify_enabled) {
            SampleEntry entry;
            entry.sample_id = sample_id++;
            entry.timestamp_ms = get_timestamp_ms();
            int rc = adpd144ri_read_3ch(i2c_dev, entry.channels);
            if (rc) {
                printk("[SENSOR] I2C sensor read failed (rc=%d). Yielding...\n", rc);
                k_msleep(100);
                continue;
            }

            int ret = k_msgq_put(&sensor_msgq, &entry, K_NO_WAIT);
            if (ret) printk("[SENSOR] k_msgq_put failed: %d\n", ret);

            k_msleep(1000); // 1Hz
        }
        printk("[SENSOR] Measurement loop paused (notifications off)\n");
    }
}


/*The printer_thread() is a dedicated background thread that:

-> Waits for sensor data from the message queue (sensor_msgq)

-> Formats each data sample by adding a human-readable timestamp

-> Prints the formatted sample to the console (for debugging)

-> Sends the formatted data over Bluetooth Low Energy (BLE) via GATT notifications to the connected central device (e.g., a phone or smartwatch)

-> Handles buffer-overflow errors and retries notification sending when needed*/
void printer_thread(void) {
    printk("[THREAD] printer_thread started\n");
    SampleEntry entry;
    char ts[64];
    int dropped_count = 0;

    while (1) {
        int kret = k_msgq_get(&sensor_msgq, &entry, K_FOREVER);
        if (kret != 0) {
            printk("[PRINTER] k_msgq_get failed: %d\n", kret);
            continue;
        }
        format_timestamp(entry.timestamp_ms, ts, sizeof(ts));
        k_mutex_lock(&notify_buf_mutex, K_FOREVER);
        snprintf(sensor_notify_buf, sizeof(sensor_notify_buf), "%lu %s %lu %lu %lu",
            (unsigned long)entry.sample_id, ts,
            (unsigned long)entry.channels[0], (unsigned long)entry.channels[1], (unsigned long)entry.channels[2]);
        sensor_notify_buf[sizeof(sensor_notify_buf)-1] = '\0';
        k_mutex_unlock(&notify_buf_mutex);

        printk("[PRINTER] BLE Sample: %s\n", sensor_notify_buf);
        if (notify_enabled && current_conn) {
            int attempts = 0;
            int err;
            do {
                err = bt_gatt_notify(current_conn, &sensor_svc.attrs[1],
                                     sensor_notify_buf,
                                     strnlen(sensor_notify_buf, sizeof(sensor_notify_buf)));
                if (err == -ENOMEM) {
                    attempts++;
                    dropped_count++;
                    if (dropped_count <= 5 || dropped_count % 100 == 0) {
                        printk("[BLE] Notify error: buffer full - notification dropped (count=%d)\n", dropped_count);
                    }
                    k_msleep(100); // Wait a bit before retrying
                } else if (err) {
                    printk("[BLE] Notify error: %d\n", err);
                    break;
                } else {
                    dropped_count = 0;
                    break;
                }
            } while (err == -ENOMEM && attempts < 5);
        }
    }
}

K_THREAD_DEFINE(sensor_tid, 2048, sensor_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(printer_tid, 2048, printer_thread, NULL, NULL, NULL, 5, 0, 0);

void main(void) {
    printk("\n********* BOOTING ADPD144RI BLE SENSOR DEMO *********\n");
    k_mutex_init(&notify_buf_mutex);
    memset(sensor_notify_buf, 0, sizeof(sensor_notify_buf)); // Make sure buffer initialized
    int err = bt_enable(bt_ready);
    printk("[MAIN] Called bt_enable(), returned %d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
    }
}
