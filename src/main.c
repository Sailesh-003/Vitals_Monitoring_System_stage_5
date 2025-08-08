#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/types.h>
#include <time.h>
#include "adpd144ri.h"
#include "rtc.h"
#include "ble.h"
#include "temp.h"
#include "lis3dh.h"

#define I2C_NODE DT_NODELABEL(i2c0)

#define BATCH_SIZE 1
#define MSGQ_SIZE 20

K_MSGQ_DEFINE(ppg_msgq, sizeof(PPGSample), MSGQ_SIZE, 4);
K_MSGQ_DEFINE(temp_msgq, sizeof(TempSample), MSGQ_SIZE, 4);
K_MSGQ_DEFINE(accel_msgq, sizeof(lis3dh_data_t), MSGQ_SIZE, 4);

extern struct k_mutex notify_buf_mutex;
extern volatile bool notify_enabled_ppg;
extern volatile bool notify_enabled_temp;
extern volatile bool notify_enabled_accel;

extern struct bt_conn *current_conn;
extern const struct bt_gatt_service_static sensor_svc;
extern const struct bt_gatt_attr *ppg_char_attr;
extern const struct bt_gatt_attr *temp_char_attr;
extern const struct bt_gatt_attr *accel_char_attr;

static lis3dh_sensor_t lis3dh_dev;

// Custom timegm() replacement (UTC time -> timestamp)
time_t custom_timegm(struct tm *tm) {
    static const int days_in_month[] = {
        31, 28, 31, 30, 31, 30,
        31, 31, 30, 31, 30, 31
    };

    int year = tm->tm_year + 1900;
    int month = tm->tm_mon;  // 0-11
    int day = tm->tm_mday;

    // Count leap years
    int y = year - (month < 2);
    int leap_days = y / 4 - y / 100 + y / 400;

    // Days since epoch
    int days = (year - 1970) * 365 + leap_days;
    for (int i = 0; i < month; i++) {
        days += days_in_month[i];
    }

    // Add one more day for leap year if after Feb
    if (month > 1 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
        days += 1;
    }

    days += day - 1;

    // Return seconds since epoch
    return days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

uint64_t parse_timestamp_ms(const char *timestamp_str) {
    struct tm t = {0};
    int ms = 0;

    // Parse: "dd/mm/yyyy hh:mm:ss.mmm"
    sscanf(timestamp_str, "%d/%d/%d %d:%d:%d.%d",
           &t.tm_mday, &t.tm_mon, &t.tm_year,
           &t.tm_hour, &t.tm_min, &t.tm_sec, &ms);

    t.tm_mon -= 1;
    t.tm_year -= 1900;

    time_t seconds = custom_timegm(&t);
    return ((uint64_t)seconds * 1000) + ms;
}

void sensor_thread(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    printk("[THREAD] sensor_thread started\n");

    if (!device_is_ready(i2c_dev)) {
        printk("[I2C] device %s not ready!\n", i2c_dev->name);
        while (1) {
            k_msleep(1000);
        }
    }

    if (!lis3dh_init(&lis3dh_dev, i2c_dev, 0x18)) {
        printk("[LIS3DH] Accelerometer sensor initialization failed\n");
    }

    if (temp_sensor_init() != 0) {
        printk("[TEMP] Temperature sensor initialization failed\n");
    }

    rtc2_init();

    while (1) {
        while (!(notify_enabled_ppg || notify_enabled_temp || notify_enabled_accel)) {
            k_msleep(200);
        }

        while (adpd144ri_configure_spo2_hr(i2c_dev) != 0) {
            printk("[SENSOR] PPG sensor config failed, retrying...\n");
            k_msleep(200);
            if (!(notify_enabled_ppg || notify_enabled_temp || notify_enabled_accel)) {
                break;
            }
        }
        if (!(notify_enabled_ppg || notify_enabled_temp || notify_enabled_accel)) {
            continue;
        }

        printk("[SENSOR] Measurement loop started\n");

        while (notify_enabled_ppg || notify_enabled_temp || notify_enabled_accel) {
            PPGSample ppg_sample = {0};
            TempSample temp_sample = {0};
            lis3dh_data_t accel_sample = {0};

            //ppg_sample.timestamp_ms = get_timestamp_ms();
            int rc = adpd144ri_read_2ch(i2c_dev, ppg_sample.channels);
            if (rc) {
                printk("[SENSOR] PPG sensor read failed (rc=%d)\n", rc);
            } else {
                ppg_sample.timestamp_ms = get_timestamp_ms();
            }

            if (temp_sensor_read(&temp_sample) != 0) {
                printk("[SENSOR] Temperature sensor read failed\n");
            } else {
                temp_sample.timestamp_ms = get_timestamp_ms();
            }

            if (!lis3dh_read_data(&lis3dh_dev, accel_sample.data)) {
                printk("[SENSOR] LIS3DH accelerometer read failed\n");
            } else {
                accel_sample.timestamp_ms = get_timestamp_ms();
            }

            if (notify_enabled_ppg && k_msgq_put(&ppg_msgq, &ppg_sample, K_NO_WAIT)) {
                printk("[SENSOR] PPG msgq full, dropping sample\n");
            }

            if (notify_enabled_temp && k_msgq_put(&temp_msgq, &temp_sample, K_NO_WAIT)) {
                printk("[SENSOR] Temp msgq full, dropping sample\n");
            }

            if (notify_enabled_accel && k_msgq_put(&accel_msgq, &accel_sample, K_NO_WAIT)) {
                printk("[SENSOR] Accel msgq full, dropping sample\n");
            }

            k_msleep(30);
        }
        printk("[SENSOR] Measurement loop paused (notifications off)\n");
    }
}
/*
void printer_thread(void)
{
    printk("[THREAD] printer_thread started\n");

    PPGSample ppg_batch[BATCH_SIZE];
    TempSample temp_batch[BATCH_SIZE];
    lis3dh_data_t accel_batch[BATCH_SIZE];
    char ts[64];

    while (1) {
        if (notify_enabled_ppg) {
            for (int i = 0; i < BATCH_SIZE; i++) {
                if (k_msgq_get(&ppg_msgq, &ppg_batch[i], K_NO_WAIT)) {
                    printk("[PRINTER] Failed to get PPG sample\n");
                }
            }
        }

        if (notify_enabled_temp) {
            for (int i = 0; i < BATCH_SIZE; i++) {
                if (k_msgq_get(&temp_msgq, &temp_batch[i], K_NO_WAIT)) {
                    printk("[PRINTER] Failed to get Temp sample\n");
                }
            }
        }

        if (notify_enabled_accel) {
            for (int i = 0; i < BATCH_SIZE; i++) {
                if (k_msgq_get(&accel_msgq, &accel_batch[i], K_NO_WAIT)) {
                    printk("[PRINTER] Failed to get Accel sample\n");
                }
            }
        }

        k_mutex_lock(&notify_buf_mutex, K_FOREVER);

        
        if (notify_enabled_ppg) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintf(sensor_notify_buf_ppg + pos, len - pos, "PPG_BATCH:");
            for (int i = 0; i < BATCH_SIZE && pos < len; i++) {
                format_timestamp(ppg_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_ppg + pos, len - pos,
                                "#%llu,%x,%x", (unsigned long long)ts_ms,
                                (unsigned long)ppg_batch[i].channels[0],
                                (unsigned long)ppg_batch[i].channels[1]);
                if (w < 0 || w >= len - pos) {
                    break;
                }
                pos += w;
            }
            sensor_notify_buf_ppg[len - 1] = '\0';
        }

        if (notify_enabled_temp) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintf(sensor_notify_buf_temp + pos, len - pos, "TEMP_BATCH:");
            for (int i = 0; i < BATCH_SIZE && pos < len; i++) {
                format_timestamp(temp_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_temp + pos, len - pos,
                                "#%llu,%.2f", (unsigned long long)ts_ms, (double)temp_batch[i].temperature_c);
                if (w < 0 || w >= len - pos) {
                    break;
                }
                pos += w;
            }
            sensor_notify_buf_temp[len - 1] = '\0';
        }

        if (notify_enabled_accel) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintk(sensor_notify_buf_accel + pos, len - pos, "ACCEL_BATCH:");
            for (int i = 0; i < BATCH_SIZE && pos < len; i++) {
                format_timestamp(accel_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_accel + pos, len - pos,
                                "#%llu,%d,%d,%d", (unsigned long long)ts_ms,
                                accel_batch[i].data[0], accel_batch[i].data[1], accel_batch[i].data[2]);
                if (w < 0 || w >= len - pos) {
                    break;
                }
                pos += w;
            }
            sensor_notify_buf_accel[len - 1] = '\0';
        }

        k_mutex_unlock(&notify_buf_mutex);

        if (notify_enabled_ppg) {
            printk("[PRINTER] Notify Data (PPG):\n%s\n", sensor_notify_buf_ppg);
        }
        if (notify_enabled_temp) {
            printk("[PRINTER] Notify Data (TEMP):\n%s\n", sensor_notify_buf_temp);
        }
        if (notify_enabled_accel) {
            printk("[PRINTER] Notify Data (ACCEL):\n%s\n", sensor_notify_buf_accel);
        }

        if (current_conn) {
            int err, attempts;

            if (notify_enabled_ppg) {
                attempts = 0;
                do {
                    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
                    err = bt_gatt_notify(current_conn, ppg_char_attr,
                                         sensor_notify_buf_ppg, strlen(sensor_notify_buf_ppg));
                    k_mutex_unlock(&notify_buf_mutex);
                    if (err == -ENOMEM) {
                        k_msleep(100);
                    } else {
                        break;
                    }
                    attempts++;
                } while (err == -ENOMEM && attempts < 5);
            }

            if (notify_enabled_temp) {
                attempts = 0;
                do {
                    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
                    err = bt_gatt_notify(current_conn, temp_char_attr,
                                         sensor_notify_buf_temp, strlen(sensor_notify_buf_temp));
                    k_mutex_unlock(&notify_buf_mutex);
                    if (err == -ENOMEM) {
                        k_msleep(100);
                    } else {
                        break;
                    }
                    attempts++;
                } while (err == -ENOMEM && attempts < 5);
            }

            if (notify_enabled_accel) {
                attempts = 0;
                do {
                    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
                    err = bt_gatt_notify(current_conn, accel_char_attr,
                                         sensor_notify_buf_accel, strlen(sensor_notify_buf_accel));
                    k_mutex_unlock(&notify_buf_mutex);
                    if (err == -ENOMEM) {
                        k_msleep(100);
                    } else {
                        break;
                    }
                    attempts++;
                } while (err == -ENOMEM && attempts < 5);
            }
        } else {
            k_msleep(50);
        }
    }
} */
#define WAIT_MS 10  // how long to wait for at least one sample

void printer_thread(void)
{
    printk("[THREAD] printer_thread started\n");

    PPGSample ppg_batch[BATCH_SIZE];
    TempSample temp_batch[BATCH_SIZE];
    lis3dh_data_t accel_batch[BATCH_SIZE];
    char ts[64];

    while (1) {
        int ppg_count = 0, temp_count = 0, accel_count = 0;

        // Grab as many as available for each sensor, but don't block forever
        if (notify_enabled_ppg) {
            while (ppg_count < BATCH_SIZE &&
                   k_msgq_get(&ppg_msgq, &ppg_batch[ppg_count], K_NO_WAIT) == 0) {
                ppg_count++;
            }
        }

        if (notify_enabled_temp) {
            while (temp_count < BATCH_SIZE &&
                   k_msgq_get(&temp_msgq, &temp_batch[temp_count], K_NO_WAIT) == 0) {
                temp_count++;
            }
        }

        if (notify_enabled_accel) {
            while (accel_count < BATCH_SIZE &&
                   k_msgq_get(&accel_msgq, &accel_batch[accel_count], K_NO_WAIT) == 0) {
                accel_count++;
            }
        }

        // If nothing to send, wait a bit before checking again
        if (ppg_count == 0 && temp_count == 0 && accel_count == 0) {
            k_msleep(WAIT_MS);
            continue;
        }

        k_mutex_lock(&notify_buf_mutex, K_FOREVER);

        if (notify_enabled_ppg && ppg_count > 0) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintf(sensor_notify_buf_ppg + pos, len - pos, "PPG_BATCH:");
            for (int i = 0; i < ppg_count && pos < len; i++) {
                format_timestamp(ppg_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_ppg + pos, len - pos,
                                "#%llu,%x,%x",
                                (unsigned long long)ts_ms,
                                (unsigned long)ppg_batch[i].channels[0],
                                (unsigned long)ppg_batch[i].channels[1]);
                if (w < 0 || w >= len - pos) break;
                pos += w;
            }
            sensor_notify_buf_ppg[len - 1] = '\0';
        }

        if (notify_enabled_temp && temp_count > 0) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintf(sensor_notify_buf_temp + pos, len - pos, "TEMP_BATCH:");
            for (int i = 0; i < temp_count && pos < len; i++) {
                format_timestamp(temp_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_temp + pos, len - pos,
                                "#%llu,%.2f",
                                (unsigned long long)ts_ms,
                                (double)temp_batch[i].temperature_c);
                if (w < 0 || w >= len - pos) break;
                pos += w;
            }
            sensor_notify_buf_temp[len - 1] = '\0';
        }

        if (notify_enabled_accel && accel_count > 0) {
            int pos = 0, len = SENSOR_NOTIFY_BUF_SIZE;
            pos += snprintk(sensor_notify_buf_accel + pos, len - pos, "ACCEL_BATCH:");
            for (int i = 0; i < accel_count && pos < len; i++) {
                format_timestamp(accel_batch[i].timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = parse_timestamp_ms(ts);
                int w = snprintk(sensor_notify_buf_accel + pos, len - pos,
                                "#%llu,%d,%d,%d",
                                (unsigned long long)ts_ms,
                                accel_batch[i].data[0],
                                accel_batch[i].data[1],
                                accel_batch[i].data[2]);
                if (w < 0 || w >= len - pos) break;
                pos += w;
            }
            sensor_notify_buf_accel[len - 1] = '\0';
        }

        k_mutex_unlock(&notify_buf_mutex);

        // Print to console (optional)
        if (notify_enabled_ppg && ppg_count > 0)
            printk("[PRINTER] Notify Data (PPG):\n%s\n", sensor_notify_buf_ppg);
        if (notify_enabled_temp && temp_count > 0)
            printk("[PRINTER] Notify Data (TEMP):\n%s\n", sensor_notify_buf_temp);
        if (notify_enabled_accel && accel_count > 0)
            printk("[PRINTER] Notify Data (ACCEL):\n%s\n", sensor_notify_buf_accel);

        // Send via BLE
        if (current_conn) {
            if (notify_enabled_ppg && ppg_count > 0)
                bt_gatt_notify(current_conn, ppg_char_attr, sensor_notify_buf_ppg, strlen(sensor_notify_buf_ppg));
            if (notify_enabled_temp && temp_count > 0)
                bt_gatt_notify(current_conn, temp_char_attr, sensor_notify_buf_temp, strlen(sensor_notify_buf_temp));
            if (notify_enabled_accel && accel_count > 0)
                bt_gatt_notify(current_conn, accel_char_attr, sensor_notify_buf_accel, strlen(sensor_notify_buf_accel));
        }
    }
}


K_THREAD_DEFINE(sensor_tid, 1280, sensor_thread, NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(printer_tid, 2560, printer_thread, NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    printk("\n********* BOOTING ADPD144RI BLE SENSOR DEMO *********\n");
    k_mutex_init(&notify_buf_mutex);

    memset(sensor_notify_buf_ppg, 0, sizeof(sensor_notify_buf_ppg));
    memset(sensor_notify_buf_temp, 0, sizeof(sensor_notify_buf_temp));
    memset(sensor_notify_buf_accel, 0, sizeof(sensor_notify_buf_accel));

    int err = bt_enable(bt_ready);
    printk("[MAIN] Called bt_enable(), returned %d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
    }

    return 0;
}
