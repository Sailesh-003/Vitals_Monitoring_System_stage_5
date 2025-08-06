#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/types.h>
#include "adpd144ri.h"
#include "rtc.h"
#include "ble.h"
#include "temp.h"
#include "lis3dh.h"

#define I2C_NODE DT_NODELABEL(i2c0)
#define MSGQ_SIZE 10

// Add lis3dh_data_t to the combined sensor sample
typedef struct {
    PPGSample ppg;
    TempSample temp;
    lis3dh_data_t accel;       // accelerometer data sample
} CombinedSensorSample;

extern const struct bt_gatt_service_static sensor_svc;

extern struct k_mutex notify_buf_mutex;
extern char sensor_notify_buf[SENSOR_NOTIFY_BUF_SIZE];

K_MSGQ_DEFINE(sensor_msgq, sizeof(CombinedSensorSample), MSGQ_SIZE, 4);

static uint32_t ppg_sample_id = 1;
static uint32_t temp_sample_id = 1;
static uint32_t accel_sample_id = 1;

static lis3dh_sensor_t lis3dh_dev;

void sensor_thread(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);
    printk("[THREAD] sensor_thread started\n");

    if (!device_is_ready(i2c_dev)) {
        printk("[I2C] device %s not ready!\n", i2c_dev->name);
        while (1) k_msleep(1000);
    }

    // Initialize LIS3DH sensor
    if (!lis3dh_init(&lis3dh_dev, i2c_dev, 0x18)) {  
        printk("[LIS3DH] Accelerometer sensor initialization failed\n");
    }

    // Initialize temperature sensor
    if (temp_sensor_init() != 0) {
        printk("[TEMP] Temperature sensor initialization failed\n");
    }

    rtc2_init();

    while (1) {
        while (!notify_enabled) {
            k_msleep(200);
        }

        while (adpd144ri_configure_spo2_hr(i2c_dev) != 0) {
            printk("[SENSOR] PPG sensor config failed, retrying...\n");
            k_msleep(200);
            if (!notify_enabled) {
                break;
            }
        }
        if (!notify_enabled) {
            continue;
        }

        printk("[SENSOR] Measurement loop started\n");

        while (notify_enabled) {
            CombinedSensorSample sample;

            // Read PPG sensor data
            sample.ppg.timestamp_ms = get_timestamp_ms();
            int rc = adpd144ri_read_2ch(i2c_dev, sample.ppg.channels);
            if (rc) {
                printk("[SENSOR] PPG sensor read failed (rc=%d)\n", rc);
                k_msleep(100);
                continue;
            }

            // Read temperature sensor data
            if (temp_sensor_read(&sample.temp) != 0) {
                printk("[SENSOR] Temperature sensor read failed\n");
            }

            // Read accelerometer data
            if (!lis3dh_read_data(&lis3dh_dev, sample.accel.data)) {
                printk("[SENSOR] LIS3DH accelerometer read failed\n");
            } else {
                sample.accel.timestamp_ms = get_timestamp_ms();
            }

            int ret = k_msgq_put(&sensor_msgq, &sample, K_NO_WAIT);
            if (ret) {
                printk("[SENSOR] Message queue put failed: %d\n", ret);
            }

            k_msleep(30);  // Approx 33 Hz sample rate
        }
        printk("[SENSOR] Measurement loop paused (notifications off)\n");
    }
}

void printer_thread(void) {
    printk("[THREAD] printer_thread started\n");
    CombinedSensorSample sample;
    char ts_ppg[64];
    char ts_temp[64];
    char ts_accel[64];
    int dropped_count = 0;

    while (1) {
        int kret = k_msgq_get(&sensor_msgq, &sample, K_FOREVER);
        if (kret != 0) {
            printk("[PRINTER] k_msgq_get failed: %d\n", kret);
            continue;
        }

        format_timestamp(sample.ppg.timestamp_ms, ts_ppg, sizeof(ts_ppg));
        //format_timestamp(sample.temp.timestamp_ms, ts_temp, sizeof(ts_temp));
        format_timestamp(sample.accel.timestamp_ms, ts_accel, sizeof(ts_accel));

        k_mutex_lock(&notify_buf_mutex, K_FOREVER);
        snprintf(sensor_notify_buf, sizeof(sensor_notify_buf),
                 "PPG: Time=%s Red=%lu IR=%lu; "
                 "Accel: Time=%s X=%d Y=%d Z=%d; "
                 "Temp=%.2f°C;",
                 ts_ppg,
                 (unsigned long)sample.ppg.channels[0], (unsigned long)sample.ppg.channels[1],
                 ts_accel,
                 sample.accel.data[0], sample.accel.data[1], sample.accel.data[2],
                 sample.temp.temperature_c);
        sensor_notify_buf[sizeof(sensor_notify_buf) - 1] = '\0';
        k_mutex_unlock(&notify_buf_mutex);

        printk("%s\n", sensor_notify_buf);

        if (notify_enabled && current_conn) {
            int attempts = 0;
            int err;
            do {
                err = bt_gatt_notify(current_conn, &sensor_svc.attrs[1],
                                     sensor_notify_buf,
                                     strnlen(sensor_notify_buf, sizeof(sensor_notify_buf)));
                if (err == -ENOMEM) {
                    dropped_count++;
                    if (dropped_count <= 5 || dropped_count % 100 == 0) {
                        printk("[BLE] Notify buffer full (dropped=%d)\n", dropped_count);
                    }
                    k_msleep(100);
                } else {
                    dropped_count = 0;
                    break;
                }
            } while (err == -ENOMEM && attempts++ < 5);
        }
    }
}

K_THREAD_DEFINE(sensor_tid, 2048, sensor_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(printer_tid, 2048, printer_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void) {
    printk("\n********* BOOTING ADPD144RI BLE SENSOR DEMO *********\n");
    k_mutex_init(&notify_buf_mutex);
    memset(sensor_notify_buf, 0, sizeof(sensor_notify_buf));

    int err = bt_enable(bt_ready);
    printk("[MAIN] Called bt_enable(), returned %d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
    }
    return 0;
}












