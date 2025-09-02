
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <zephyr/types.h>
#include <time.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include "adpd144ri.h"
#include "rtc.h"
#include "ble.h"
#include "temp.h"
#include "lis3dh.h"

#define I2C_NODE DT_NODELABEL(i2c0)

#define BATCH_SIZE 6
#define MSGQ_SIZE 30
#define BT_ATT_MTU_DEFAULT 23

#define MAX_SAMPLES 1998

#define I2C_MUTEX_TIMEOUT  K_MSEC(5)  // Max wait time for I2C access

LOG_MODULE_REGISTER(main);

#define LFS_MOUNT_POINT "/lfs"

/* --- global flash device binding --- */
static const struct device *flash_dev;

/* --- LittleFS callbacks (use Zephyr flash API) --- */
int w25q16_read(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, void *buffer, lfs_size_t size)
{
    off_t addr = (off_t)block * c->block_size + off;
    return flash_read(flash_dev, addr, buffer, size);
}

int w25q16_prog(const struct lfs_config *c, lfs_block_t block,
                lfs_off_t off, const void *buffer, lfs_size_t size)
{
    off_t addr = (off_t)block * c->block_size + off;
    return flash_write(flash_dev, addr, buffer, size);
}

int w25q16_erase(const struct lfs_config *c, lfs_block_t block)
{
    off_t addr = (off_t)block * c->block_size;
    return flash_erase(flash_dev, addr, c->block_size);
}

int w25q16_sync(const struct lfs_config *c)
{
    ARG_UNUSED(c);
    /* No-op for SPI NOR */
    return 0;
}

/* --- buffers for LittleFS runtime config --- */
static uint8_t read_buffer[256];
static uint8_t prog_buffer[256];
static uint32_t lookahead_buffer[32 / sizeof(uint32_t)];

/* --- lfs config (4KB blocks, 2MB total) --- */
static struct lfs_config littlefs_cfg = {
    .read = w25q16_read,
    .prog = w25q16_prog,
    .erase = w25q16_erase,
    .sync = w25q16_sync,

    .read_size  = 256,
    .prog_size  = 256,
    .block_size = 4096,
    .block_count = 512, /* 2MB / 4KB = 512 */

    .cache_size = 256,
    .lookahead_size = 32,

    .block_cycles = 512,

    .read_buffer = read_buffer,
    .prog_buffer = prog_buffer,
    .lookahead_buffer = lookahead_buffer,
};

/* --- Zephyr FS wrappers --- */
static struct fs_littlefs littlefs_data;
static struct fs_mount_t littlefs_mnt = {
    .type = FS_LITTLEFS,
    .mnt_point = LFS_MOUNT_POINT,
    .fs_data = &littlefs_data,
    .storage_dev = (void *)FLASH_AREA_ID(littlefs_storage),
    .flags = FS_MOUNT_FLAG_AUTOMOUNT,
};

static void check_storage_usage(void)
{
    struct fs_statvfs stat;
    int ret = fs_statvfs(LFS_MOUNT_POINT, &stat);
    if (ret == 0) {
        uint32_t total = stat.f_blocks * stat.f_frsize;
        uint32_t free  = stat.f_bfree  * stat.f_frsize;
        uint32_t used  = total - free;

        LOG_INF("===== Storage Info =====");
        LOG_INF("Total: %u bytes", total);
        LOG_INF("Used : %u bytes", used);
        LOG_INF("Free : %u bytes", free);
        LOG_INF("========================");
    } else {
        LOG_ERR("Failed to get storage info (err %d)", ret);
    }
}

/* --- msgqs, mutexes and semaphores --- */
K_MUTEX_DEFINE(i2c_bus_mutex);

K_MSGQ_DEFINE(ppg_msgq, sizeof(PPGSample), MSGQ_SIZE, 4);
K_MSGQ_DEFINE(temp_msgq, sizeof(TempSample), MSGQ_SIZE, 4);
K_MSGQ_DEFINE(accel_msgq, sizeof(lis3dh_data_t), MSGQ_SIZE, 4);

/* External symbols from other modules (BLE etc.) */
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

/* Synchronization for end-of-collection and completion */
static K_SEM_DEFINE(collection_sem, 0, 1); /* given by sensor when final temp enqueued */
static K_SEM_DEFINE(done_sem, 0, 1);       /* given by printer when finished */

static volatile bool collection_done = false;

/* local helper: safe msgq count get (atomic-ish) */
static inline size_t msgq_used_count_kmsgq(struct k_msgq *q)
{
    return k_msgq_num_used_get(q);
}

/* --- sensor thread: collects PPG and ACCEL up to MAX_SAMPLES each, then reads TEMP once --- */
void sensor_thread(void *p1, void *p2, void *p3)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    printk("[THREAD] sensor_thread started\n");

    if (!device_is_ready(i2c_dev)) {
        printk("[I2C] device %s not ready!\n", i2c_dev->name);
        return;
    }

    /* Init LIS3DH (short lock) */
    if (k_mutex_lock(&i2c_bus_mutex, I2C_MUTEX_TIMEOUT) == 0) {
        if (!lis3dh_init(&lis3dh_dev, i2c_dev, 0x18)) {
            printk("[LIS3DH] Accelerometer sensor initialization failed\n");
        }
        k_mutex_unlock(&i2c_bus_mutex);
    }

    /* Init PPG (short lock) */
    if (k_mutex_lock(&i2c_bus_mutex, I2C_MUTEX_TIMEOUT) == 0) {
        if (adpd144ri_configure_spo2_hr(i2c_dev) != 0) {
            printk("[PPG] PPG Sensor failed\n");
        }
        k_mutex_unlock(&i2c_bus_mutex);
    }

    /* Re-init LIS3DH after PPG if needed */
    if (k_mutex_lock(&i2c_bus_mutex, I2C_MUTEX_TIMEOUT) == 0) {
        if (!lis3dh_init(&lis3dh_dev, i2c_dev, 0x18)) {
            printk("[LIS3DH] Accelerometer sensor re-init failed\n");
        }
        k_mutex_unlock(&i2c_bus_mutex);
    }

    if (temp_sensor_init() != 0) {
        printk("[TEMP] Temperature sensor initialization failed\n");
    }

    rtc2_init();

    PPGSample ppg_sample;
    lis3dh_data_t accel_sample;
    TempSample temp_sample;

    uint32_t p_cnt = 0;
    uint32_t a_cnt = 0;

    /* Run until we collect MAX_SAMPLES for both PPG and ACCEL */
    while (1) {
        /* wait until notifications enabled (reduce useless polling) */
        while (!(notify_enabled_ppg || notify_enabled_accel || notify_enabled_temp)) {
            k_msleep(200);
        }

        /* Collect PPG if enabled and not reached MAX_SAMPLES */
        if (notify_enabled_ppg && p_cnt < MAX_SAMPLES) {
            if (k_mutex_lock(&i2c_bus_mutex, I2C_MUTEX_TIMEOUT) == 0) {
                if (adpd144ri_read_2ch(i2c_dev, ppg_sample.channels) == 0) {
                    ppg_sample.timestamp_ms = get_timestamp_ms();
                    /* If msgq full, drop oldest to keep moving (avoid blocking sensor) */
                    if (k_msgq_put(&ppg_msgq, &ppg_sample, K_NO_WAIT) != 0) {
                        /* try removing one and push */
                        PPGSample tmp;
                        k_msgq_get(&ppg_msgq, &tmp, K_NO_WAIT);
                        k_msgq_put(&ppg_msgq, &ppg_sample, K_NO_WAIT);
                    }
                    p_cnt++;
                }
                k_mutex_unlock(&i2c_bus_mutex);
            }
        }

        /* Collect ACCEL if enabled and not reached MAX_SAMPLES */
        if (notify_enabled_accel && a_cnt < MAX_SAMPLES) {
            if (k_mutex_lock(&i2c_bus_mutex, I2C_MUTEX_TIMEOUT) == 0) {
                if (lis3dh_read_data(&lis3dh_dev, accel_sample.data)) {
                    accel_sample.timestamp_ms = get_timestamp_ms();
                    if (k_msgq_put(&accel_msgq, &accel_sample, K_NO_WAIT) != 0) {
                        lis3dh_data_t tmp;
                        k_msgq_get(&accel_msgq, &tmp, K_NO_WAIT);
                        k_msgq_put(&accel_msgq, &accel_sample, K_NO_WAIT);
                    }
                    a_cnt++;
                }
                k_mutex_unlock(&i2c_bus_mutex);
            }
        }

        /* If both counts reached MAX_SAMPLES, read a TEMP sample and enqueue it, signal printer */
        if (p_cnt >= MAX_SAMPLES && a_cnt >= MAX_SAMPLES) {
            /* read temperature once (don't block) */
            if (temp_sensor_read(&temp_sample) == 0) {
                temp_sample.timestamp_ms = get_timestamp_ms();
                /* ensure space for temp */
                if (k_msgq_put(&temp_msgq, &temp_sample, K_NO_WAIT) != 0) {
                    TempSample tmp;
                    k_msgq_get(&temp_msgq, &tmp, K_NO_WAIT);
                    k_msgq_put(&temp_msgq, &temp_sample, K_NO_WAIT);
                }
                collection_done = true;
                k_sem_give(&collection_sem); /* notify printer that final temp is available */
                printk("[SENSOR] Collected MAX samples (PPG=%u, ACCEL=%u). Temp queued.\n", p_cnt, a_cnt);
                break;
            } else {
                /* If temp read failed, wait and retry briefly */
                k_msleep(50);
            }
        }

        /* small sleep to yield CPU and reduce I2C bus contention */
        k_msleep(30);
    }

    /* sensor thread done */
    printk("[SENSOR] sensor_thread exiting after collection.\n");
    return;
}

/* --- printer thread: reads in batches of BATCH_SIZE and writes to external flash --- */
void printer_thread(void *p1, void *p2, void *p3)
{
    printk("[THREAD] printer_thread started\n");

    PPGSample ppg_batch[BATCH_SIZE];
    TempSample temp_sample;
    lis3dh_data_t accel_batch[BATCH_SIZE];
    //char ts[64];

    uint32_t total_ppg_written = 0;
    uint32_t total_accel_written = 0;
    bool temp_written = false;

    //int p_count = 0, a_count = 0;
    while (1) {
        int ppg_count = 0, accel_count = 0, t_count = 0;

        /* --- Try to gather PPG batch (wait up to 200ms total to accumulate) --- */
        for (int i = 0; i < BATCH_SIZE; i++) {
            if (k_msgq_get(&ppg_msgq, &ppg_batch[i], K_MSEC(200)) == 0) {
                ppg_count++;
            } else {
                break;
            }
        }

        /* --- Try to gather ACCEL batch --- */
        for (int i = 0; i < BATCH_SIZE; i++) {
            if (k_msgq_get(&accel_msgq, &accel_batch[i], K_MSEC(200)) == 0) {
                accel_count++;
            } else {
                break;
            }
        }

        /* --- Try to get TEMP if available (no long wait) --- */
        if (k_msgq_get(&temp_msgq, &temp_sample, K_NO_WAIT) == 0) {
            t_count = 1;
        }

        /* If nothing to write, wait for either collection_sem (final) or a small timeout */
        if (ppg_count == 0 && accel_count == 0 && t_count == 0) {
            /* If collection_done was signalled, check if queues have drained */
            if (collection_done) {
                /* small grace period for remaining data to arrive */
                k_msleep(100);
                /* If queues empty and temp was written, finish */
                if (msgq_used_count_kmsgq(&ppg_msgq) == 0 &&
                    msgq_used_count_kmsgq(&accel_msgq) == 0 &&
                    (temp_written || msgq_used_count_kmsgq(&temp_msgq) > 0)) {
                    /* Try to take remaining temp if still present */
                    if (!temp_written && k_msgq_get(&temp_msgq, &temp_sample, K_NO_WAIT) == 0) {
                        t_count = 1;
                    }
                }
            }

            /* If still nothing actionable, wait for final signal or timeout */
            if (ppg_count == 0 && accel_count == 0 && t_count == 0) {
                /* wait for final collection signal but with timeout */
                k_sem_take(&collection_sem, K_MSEC(500));
                /* loop to re-check queues */
                continue;
            }
        }

        /* --- Write PPG batch if we have samples --- */
        if (ppg_count > 0) {
            struct fs_file_t file;
            fs_file_t_init(&file);
            if (fs_open(&file, LFS_MOUNT_POINT "/ppg.txt", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND) == 0) {
                char buf[512];
                int pos = 0, len = sizeof(buf);

                pos += snprintf(buf + pos, len - pos, "PPG_BATCH:");
                for (int i = 0; i < ppg_count; i++) {
                    //format_timestamp(ppg_batch[i].timestamp_ms, ts, sizeof(ts));
                    uint64_t ts_ms = ppg_batch[i].timestamp_ms;
                    pos += snprintk(buf + pos, len - pos,
                                    "#%llu,%lx,%lx",
                                    (unsigned long long)ts_ms,
                                    (unsigned long)ppg_batch[i].channels[0],
                                    (unsigned long)ppg_batch[i].channels[1]);
                }
                buf[len - 1] = '\0';
                fs_write(&file, buf, strlen(buf));
                fs_write(&file, "\n", 1);

                fs_sync(&file);
                fs_close(&file);
                total_ppg_written += ppg_count;
                printk("[FS] Wrote %d PPG samples (total %u)\n", ppg_count, total_ppg_written);
            } else {
                LOG_ERR("Failed to open ppg.txt for append");
            }
        }

        /* --- Write ACCEL batch if we have samples --- */
        if (accel_count > 0) {
            struct fs_file_t file;
            fs_file_t_init(&file);
            if (fs_open(&file, LFS_MOUNT_POINT "/accel.txt", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND) == 0) {
                char buf[512];
                int pos = 0, len = sizeof(buf);

                pos += snprintf(buf + pos, len - pos, "ACCEL_BATCH:");
                for (int i = 0; i < accel_count; i++) {
                    //format_timestamp(accel_batch[i].timestamp_ms, ts, sizeof(ts));
                    uint64_t ts_ms = accel_batch[i].timestamp_ms;
                    pos += snprintk(buf + pos, len - pos,
                                    "#%llu,%d,%d,%d",
                                    (unsigned long long)ts_ms,
                                    accel_batch[i].data[0],
                                    accel_batch[i].data[1],
                                    accel_batch[i].data[2]);
                }
                buf[len - 1] = '\0';
                fs_write(&file, buf, strlen(buf));
                fs_write(&file, "\n", 1);

                fs_sync(&file);
                fs_close(&file);
                total_accel_written += accel_count;
                printk("[FS] Wrote %d ACCEL samples (total %u)\n", accel_count, total_accel_written);
            } else {
                LOG_ERR("Failed to open accel.txt for append");
            }
        }

        /* --- Write TEMP sample if available --- */
        if (t_count > 0) {
            struct fs_file_t file;
            fs_file_t_init(&file);
            if (fs_open(&file, LFS_MOUNT_POINT "/temp.txt", FS_O_CREATE | FS_O_WRITE | FS_O_APPEND) == 0) {
                char buf[256];
                int pos = 0, len = sizeof(buf);

                pos += snprintf(buf + pos, len - pos, "TEMP_BATCH:");
                //format_timestamp(temp_sample.timestamp_ms, ts, sizeof(ts));
                uint64_t ts_ms = temp_sample.timestamp_ms;
                pos += snprintk(buf + pos, len - pos,
                                "#%llu,%d,%d",
                                (unsigned long long)ts_ms,
                                temp_sample.temperature_c,
                                (int)temp_sample.battery_pct);

                buf[len - 1] = '\0';
                fs_write(&file, buf, strlen(buf));
                fs_write(&file, "\n", 1);

                fs_sync(&file);
                fs_close(&file);
                temp_written = true;
                printk("[FS] Wrote TEMP sample\n");
            } else {
                LOG_ERR("Failed to open temp.txt for append");
            }
        }

        /* --- Check if we're done: both totals reached MAX_SAMPLES and temp_written --- */
        if (total_ppg_written >= MAX_SAMPLES &&
            total_accel_written >= MAX_SAMPLES &&
            temp_written &&
            collection_done) {

            printk("[PRINTER] All data written: PPG=%u, ACCEL=%u, TEMP=1\n",
                   total_ppg_written, total_accel_written);

            /* Sync any remaining filesystem buffers (redundant but safe) */
            check_storage_usage();

            /* Signal main that we are done */
            k_sem_give(&done_sem);

            /* Exit thread */
            printk("[PRINTER] printer_thread exiting.\n");
            return;
        }

        /* small yield */
        k_msleep(10);
    }
}

/* Thread stacks and data */
K_THREAD_STACK_DEFINE(sensor_stack, 2048);
K_THREAD_STACK_DEFINE(printer_stack, 2048);
static struct k_thread sensor_thread_data;
static struct k_thread printer_thread_data;

/* extern helper functions assumed provided in other modules */
extern void format_timestamp(uint64_t ts_ms, char *buf, size_t buf_len);
extern uint64_t get_timestamp_ms(void);

K_THREAD_STACK_DEFINE(ble_tx_stack, 1536);
static struct k_thread ble_tx_thread_data;


static int notify_chunk(const struct bt_gatt_attr *attr,
                        const uint8_t *buf, size_t len)
{
    if (!current_conn || !attr) {
        return -ENOTCONN;
    }

    /* Use the simple notify API */
    int rc = bt_gatt_notify(current_conn, attr, (void *)buf, len);
    return rc;
}


static int stream_file_over_ble(const char *path,
                                const struct bt_gatt_attr *attr,
                                volatile bool *notify_enabled_flag)
{
    struct fs_file_t file;
    fs_file_t_init(&file);

    if (fs_open(&file, path, FS_O_READ) < 0) {
        printk("[BLE_TX] Failed to open file: %s\n", path);
        return -EIO;
    }

    /* Determine maximum BLE payload size */
    size_t mtu = bt_gatt_get_mtu(current_conn);
    size_t max_payload_len = (mtu > 3) ? (mtu - 3) : 20;

    printk("[BLE_TX] Start streaming file: %s (MTU=%u, payload=%u)\n",
           path, (unsigned int)mtu, (unsigned int)max_payload_len);

    char line_buf[256];   /* enough to hold one line */
    char c;
    size_t idx = 0;

    while (1) {
        /* Stop streaming if notifications disabled */
        if (!(*notify_enabled_flag)) {
            printk("[BLE_TX] Notifications disabled, stopping stream\n");
            break;
        }

        int rc = fs_read(&file, &c, 1);
        if (rc == 0) {
            /* EOF reached */
            break;
        } else if (rc < 0) {
            printk("[BLE_TX] File read error: %d\n", rc);
            break;
        }

        /* Build line until newline */
        if (c != '\n' && idx < sizeof(line_buf) - 1) {
            line_buf[idx++] = c;
        } else {
            line_buf[idx] = '\0';  /* terminate string */
            size_t line_len = strlen(line_buf);

            /* --- Send this line in BLE-sized chunks --- */
            size_t offset = 0;
            while (offset < line_len) {
                size_t chunk_len = MIN(max_payload_len, line_len - offset);

                int tries = 0;
                while (tries < 5) {
                    rc = bt_gatt_notify(current_conn, attr,
                                        &line_buf[offset], chunk_len);
                    if (rc == 0) {
                        break;  /* success */
                    }
                    if (rc == -EAGAIN) {
                        tries++;
                        k_msleep(5);
                        continue;
                    }
                    printk("[BLE_TX] notify failed (%s) rc=%d\n", path, rc);
                    fs_close(&file);
                    return rc;
                }

                offset += chunk_len;
                k_msleep(2);  /* pacing */
            }

            idx = 0; /* reset for next line */
        }
    }

    fs_close(&file);
    printk("[BLE_TX] Completed streaming file: %s\n", path);
    return 0;
}



void ble_tx_thread(void *p1, void *p2, void *p3)
{
    printk("[THREAD] ble_tx_thread started\n");

    /* Wait a bit to let client connect and enable notifications (if it will) */
    k_msleep(500);

    int rc;
    rc = stream_file_over_ble(LFS_MOUNT_POINT "/ppg.txt", ppg_char_attr, &notify_enabled_ppg);
    if (rc) printk("[BLE_TX] notify failed /lfs/ppg.txt rc=%d\n", rc);

    rc = stream_file_over_ble(LFS_MOUNT_POINT "/accel.txt", accel_char_attr, &notify_enabled_accel);
    if (rc) printk("[BLE_TX] notify failed /lfs/accel.txt rc=%d\n", rc);

    rc = stream_file_over_ble(LFS_MOUNT_POINT "/temp.txt", temp_char_attr, &notify_enabled_temp);
    if (rc) printk("[BLE_TX] notify failed /lfs/temp.txt rc=%d\n", rc);

    printk("[BLE_TX] All files streamed (or attempted). Thread exiting.\n");
}

/* main */
int main(void)
{
    printk("\n********* BOOTING ADPD144RI BLE SENSOR DEMO *********\n");
    k_mutex_init(&notify_buf_mutex);

    memset(sensor_notify_buf_ppg, 0, sizeof(sensor_notify_buf_ppg));
    memset(sensor_notify_buf_temp, 0, sizeof(sensor_notify_buf_temp));
    memset(sensor_notify_buf_accel, 0, sizeof(sensor_notify_buf_accel));

    /* bind flash device (driver provided by devicetree / board) */
    flash_dev = device_get_binding("w25q16");
    if (!flash_dev) {
        LOG_ERR("Failed to bind flash device");
        return -ENODEV;
    }

    /* Erase entire external flash (2MB) before formatting/mounting */
    LOG_INF("Erasing entire external flash...");
    int rc = flash_erase(flash_dev, 0, 512 * 4096);
    if (rc < 0) {
        LOG_ERR("Flash erase failed [%d]", rc);
        return rc;
    }
    LOG_INF("Erase completed.");

    /* --- Inject runtime lfs config & buffers into Zephyr fs_littlefs struct --- */
    littlefs_data.cfg = littlefs_cfg;
    littlefs_data.read_buffer = read_buffer;
    littlefs_data.prog_buffer = prog_buffer;
    memcpy(littlefs_data.lookahead_buffer, lookahead_buffer, sizeof(lookahead_buffer));

    /* Now mount using Zephyr API (fs_mount) — wrapper will use our cfg */
    rc = fs_mount(&littlefs_mnt);
    if (rc < 0) {
        LOG_ERR("Failed to mount LittleFS [%d]", rc);
        return rc;
    }
    LOG_INF("Mounted LittleFS at %s", LFS_MOUNT_POINT);

    /* Print storage info right after mounting */
    check_storage_usage();

    int err = bt_enable(bt_ready);
    printk("[MAIN] Called bt_enable(), returned %d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
    }

    //for(;;){
        /* Start threads */
    k_thread_create(&sensor_thread_data, sensor_stack,
                    K_THREAD_STACK_SIZEOF(sensor_stack),
                    (k_thread_entry_t)sensor_thread,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    k_thread_create(&printer_thread_data, printer_stack,
                    K_THREAD_STACK_SIZEOF(printer_stack),
                    (k_thread_entry_t)printer_thread,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    /* Wait for printer to signal completion (writing finished) */
    k_sem_take(&done_sem, K_FOREVER);

    /* Print storage info after completion */
    LOG_INF("Final storage snapshot after collection & writes:");
    check_storage_usage();

    /* Start BLE transmit thread AFTER printer completes */
    k_thread_create(&ble_tx_thread_data, ble_tx_stack,
                    K_THREAD_STACK_SIZEOF(ble_tx_stack),
                    (k_thread_entry_t)ble_tx_thread,
                    NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

        //k_msleep(50000);

    //}
    

    printk("[MAIN] All done. Main returning.\n");
    return 0;
}









