#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include "ble.h"
#include "ws2812.h"


uint16_t patient_id;
/* Backing storage for ACK characteristic */
uint8_t ack_value[10];  
static struct bt_gatt_attr *ack_attr_ref;

/* Read callback */
static ssize_t read_ack(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    const char *value = (const char *)ack_value;
    return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

/* Write callback (from mobile side) */
static ssize_t write_ack(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len >= sizeof(ack_value)) {
        return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
    }

    memset(ack_value, 0, sizeof(ack_value));
    memcpy(ack_value, buf, len);

    printk("ACK updated from mobile: %s\n", ack_value);

    return len;
}

/* --- Patient ID Characteristic --- */
ssize_t write_patient_id(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf, uint16_t len,
                         uint16_t offset, uint8_t flags)
{
    if (len == 2) {
        memcpy(&patient_id, buf, sizeof(patient_id));
        printk("Patient ID received: %u\n", patient_id);
    }
    return len;
}

uint32_t UNIX_TIME_START = 0;

ssize_t write_timestamp(struct bt_conn *conn,
                        const struct bt_gatt_attr *attr,
                        const void *buf, uint16_t len,
                        uint16_t offset, uint8_t flags)
{
    if (len == 4) {
        uint32_t unix_time =
              ((uint32_t)((uint8_t*)buf)[0]) |
              ((uint32_t)((uint8_t*)buf)[1] << 8) |
              ((uint32_t)((uint8_t*)buf)[2] << 16) |
              ((uint32_t)((uint8_t*)buf)[3] << 24);

        printk("Received UNIX time: %u (s)\n", unix_time);

        rtc2_init(unix_time);  

        uint64_t now = get_timestamp_ms();
        printk("Full timestamp (ms): %llu\n", (uint64_t)now);

        char buf_time[32];
        format_timestamp(now, buf_time, sizeof(buf_time));
        printk("Formatted time: %s\n", buf_time);
    }
    return len;
}

char sensor_notify_buf_ppg[SENSOR_NOTIFY_BUF_SIZE];
char sensor_notify_buf_temp[SENSOR_NOTIFY_BUF_SIZE];
char sensor_notify_buf_accel[SENSOR_NOTIFY_BUF_SIZE];

struct bt_conn *current_conn = NULL;
volatile bool notify_enabled_ppg = false;
volatile bool notify_enabled_temp = false;
volatile bool notify_enabled_accel = false;
volatile bool ack_notify_enabled = false;
struct k_mutex notify_buf_mutex;

/* ---------- UUIDs ---------- */
/* One Service UUID */
static struct bt_uuid_128 vitals_service_uuid = BT_UUID_INIT_128(
    0xac, 0xdf, 0xd2, 0x00, 0x4b, 0xd6, 0x69, 0xac,
    0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6);

/* Three Characteristic UUIDs */
static struct bt_uuid_128 ppg_char_uuid = BT_UUID_INIT_128(
    0xb7, 0x5c, 0xb4, 0x7b, 0xb8, 0x27, 0x5a, 0xa1,
    0x20, 0x44, 0x4b, 0x9c, 0xd4, 0x4a, 0x95, 0x4c);

static struct bt_uuid_128 temp_char_uuid = BT_UUID_INIT_128(
    0xd2, 0xf8, 0x67, 0x5a, 0xce, 0x5a, 0x64, 0x86,
    0xfe, 0x43, 0xf6, 0x7a, 0x90, 0x67, 0xf6, 0x66);

static struct bt_uuid_128 accel_char_uuid = BT_UUID_INIT_128(
    0x98, 0x65, 0x12, 0xa1, 0xd8, 0x03, 0xb3, 0x93,
    0x42, 0xf1, 0x61, 0x43, 0x96, 0x21, 0xf5, 0x0a);


/* -------- Custom Service UUID -------- */
static struct bt_uuid_128 input_service_uuid =
    BT_UUID_INIT_128(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x56, 0x12);

/* -------- ACK Characteristic UUID -------- */
static struct bt_uuid_128 ack_char_uuid =
    BT_UUID_INIT_128(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x56, 0x12);

/* -------- Patient ID Characteristic UUID -------- */
static struct bt_uuid_128 patient_id_char_uuid =
    BT_UUID_INIT_128(0xf2, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x56, 0x12);

/* -------- Timestamp Characteristic UUID -------- */
static struct bt_uuid_128 timestamp_char_uuid =
    BT_UUID_INIT_128(0xf3, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x56, 0x12);

const struct bt_gatt_attr *ppg_char_attr = NULL;
const struct bt_gatt_attr *temp_char_attr = NULL;
const struct bt_gatt_attr *accel_char_attr = NULL;

/* ---------- Read Callbacks ---------- */
static ssize_t read_ppg(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                        void *buf, uint16_t len, uint16_t offset)
{
    ssize_t ret;
    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
    ret = bt_gatt_attr_read(conn, attr, buf, len, offset,
                            sensor_notify_buf_ppg, strlen(sensor_notify_buf_ppg));
    k_mutex_unlock(&notify_buf_mutex);
    return ret;
}

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                         void *buf, uint16_t len, uint16_t offset)
{
    ssize_t ret;
    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
    ret = bt_gatt_attr_read(conn, attr, buf, len, offset,
                            sensor_notify_buf_temp, strlen(sensor_notify_buf_temp));
    k_mutex_unlock(&notify_buf_mutex);
    return ret;
}

static ssize_t read_accel(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                          void *buf, uint16_t len, uint16_t offset)
{
    ssize_t ret;
    k_mutex_lock(&notify_buf_mutex, K_FOREVER);
    ret = bt_gatt_attr_read(conn, attr, buf, len, offset,
                            sensor_notify_buf_accel, strlen(sensor_notify_buf_accel));
    k_mutex_unlock(&notify_buf_mutex);
    return ret;
}

/* ---------- CCCD Callbacks ---------- */
static void ccc_cfg_changed_ppg(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled_ppg = (value == BT_GATT_CCC_NOTIFY);
}

static void ccc_cfg_changed_temp(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled_temp = (value == BT_GATT_CCC_NOTIFY);
}

static void ccc_cfg_changed_accel(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled_accel = (value == BT_GATT_CCC_NOTIFY);
}

static void ack_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ack_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/* ---------- Single Service with 3 Characteristics ---------- */
BT_GATT_SERVICE_DEFINE(vitals_svc,
    BT_GATT_PRIMARY_SERVICE(&vitals_service_uuid),

    /* PPG Characteristic */
    BT_GATT_CHARACTERISTIC(&ppg_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_ppg, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_ppg,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

     /* Accel Characteristic */
    BT_GATT_CHARACTERISTIC(&accel_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_accel, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_accel,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Temp Characteristic */
    BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_temp, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_temp,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);


BT_GATT_SERVICE_DEFINE(input_svc,
    BT_GATT_PRIMARY_SERVICE(&input_service_uuid),

    /* ACK Characteristic */
    BT_GATT_CHARACTERISTIC(&ack_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                           read_ack, write_ack, &ack_value),
    BT_GATT_CCC(ack_ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

    /* Patient ID Characteristic */
    BT_GATT_CHARACTERISTIC(&patient_id_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_patient_id, NULL),

    /* Timestamp Characteristic */
    BT_GATT_CHARACTERISTIC(&timestamp_char_uuid.uuid,
                           BT_GATT_CHRC_WRITE,
                           BT_GATT_PERM_WRITE,
                           NULL, write_timestamp, NULL)
);



/* Assign characteristic pointers */
static void assign_attr_pointers(void)
{
    ppg_char_attr   = &vitals_svc.attrs[2];
    accel_char_attr = &vitals_svc.attrs[5]; 
    temp_char_attr  = &vitals_svc.attrs[8];
}

/* ---------- BLE Helpers ---------- */
void print_ble_address(void)
{
    bt_addr_le_t addr;
    size_t count = 1;
    bt_id_get(&addr, &count);
    char buf[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(&addr, buf, sizeof(buf));
    printk("[BLE] Device BT Address: %s\n", buf);
}

void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        printk("[BLE] Connection failed (err %u)\n", err);
        return;
    }
    current_conn = bt_conn_ref(conn);
    printk("[BLE] Device Connected\n");
    ws2812_set_color(&BLUE);
       k_timer_stop(&ble_disconnect_timer);
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("[BLE] Disconnected (reason 0x%02x)\n", reason);
    ws2812_set_color(&GREEN);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled_ppg = false;
    notify_enabled_temp = false;
    notify_enabled_accel = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_PARAM(
    (BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY), 78, 78, NULL);

const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, "Vitals", 6),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0xac, 0xdf, 0xd2, 0x00, 0x4b, 0xd6, 0x69, 0xac,
                  0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6)
};

LOG_MODULE_REGISTER(ble_module, LOG_LEVEL_INF);

struct k_work ble_disconnect_work;

void ble_disconnect_work_handler(struct k_work *work)
{
    LOG_INF("No connection for 3 minutes. Powering off.");
    
    ws2812_blink_violet();
}

void ble_disconnect_timeout(struct k_timer *dummy)
{
    // Schedule the work instead of doing it in timer context
    k_work_submit(&ble_disconnect_work);
}

void bt_ready(int err)
{
    printk("[MAIN] bt_ready callback, err=%d\n", err);
    if (err) {
        printk("[BLE] Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("[BLE] Bluetooth initialized\n");
    print_ble_address();

    assign_attr_pointers();

    int ret = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
    if (ret) {
        printk("[BLE] Advertising failed to start (err %d)\n", ret);
    } else {
        printk("[BLE] Advertising started successfully!\n");
        k_timer_init(&ble_disconnect_timer, ble_disconnect_timeout, NULL);
        k_timer_start(&ble_disconnect_timer, K_MSEC(BLE_TIMEOUT_MS), K_NO_WAIT);
    }
}

void send_ack_to_mobile(const char *ack_msg)
{
    // Copy the message to the backing buffer
    strncpy(ack_value, ack_msg, sizeof(ack_value) - 1);
    ack_value[sizeof(ack_value) - 1] = '\0';

    // Send notification only if ACK notify is enabled and connection exists
    if (ack_notify_enabled && current_conn) {
        bt_gatt_notify(current_conn, &input_svc.attrs[1], ack_value, strlen(ack_value));
        printk("ACK sent to mobile: %s\n", ack_value);
    }
}