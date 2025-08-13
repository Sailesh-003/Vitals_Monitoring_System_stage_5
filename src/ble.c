#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <string.h>
#include "ble.h"

char sensor_notify_buf_ppg[SENSOR_NOTIFY_BUF_SIZE];
char sensor_notify_buf_temp[SENSOR_NOTIFY_BUF_SIZE];
char sensor_notify_buf_accel[SENSOR_NOTIFY_BUF_SIZE];

struct bt_conn *current_conn = NULL;
volatile bool notify_enabled_ppg = false;
volatile bool notify_enabled_temp = false;
volatile bool notify_enabled_accel = false;
struct k_mutex notify_buf_mutex;

static struct bt_uuid_128 ppg_service_uuid = BT_UUID_INIT_128(
    0xac, 0xdf, 0xd2, 0x00, 0x4b, 0xd6, 0x69, 0xac,
    0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6);

static struct bt_uuid_128 ppg_char_uuid = BT_UUID_INIT_128(
    0xb7, 0x5c, 0xb4, 0x7b, 0xb8, 0x27, 0x5a, 0xa1,
    0x20, 0x44, 0x4b, 0x9c, 0xd4, 0x4a, 0x95, 0x4c);

static struct bt_uuid_128 temp_service_uuid = BT_UUID_INIT_128(
    0xbc, 0xdf, 0xd2, 0x00, 0x4b, 0xd6, 0x69, 0xac,
    0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6);

static struct bt_uuid_128 temp_char_uuid = BT_UUID_INIT_128(
    0xd2, 0xf8, 0x67, 0x5a, 0xce, 0x5a, 0x64, 0x86,
    0xfe, 0x43, 0xf6, 0x7a, 0x90, 0x67, 0xf6, 0x66);

static struct bt_uuid_128 accel_service_uuid = BT_UUID_INIT_128(
    0xcc, 0xdf, 0xd2, 0x00, 0x4b, 0xd6, 0x69, 0xac,
    0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6);

static struct bt_uuid_128 accel_char_uuid = BT_UUID_INIT_128(
    0x98, 0x65, 0x12, 0xa1, 0xd8, 0x03, 0xb3, 0x93,
    0x42, 0xf1, 0x61, 0x43, 0x96, 0x21, 0xf5, 0x0a);

const struct bt_gatt_attr *ppg_char_attr = NULL;
const struct bt_gatt_attr *temp_char_attr = NULL;
const struct bt_gatt_attr *accel_char_attr = NULL;

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

/* GATT service definitions */
BT_GATT_SERVICE_DEFINE(ppg_svc,
    BT_GATT_PRIMARY_SERVICE(&ppg_service_uuid),
    BT_GATT_CHARACTERISTIC(&ppg_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_ppg, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_ppg,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

BT_GATT_SERVICE_DEFINE(temp_svc,
    BT_GATT_PRIMARY_SERVICE(&temp_service_uuid),
    BT_GATT_CHARACTERISTIC(&temp_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_temp, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_temp,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

BT_GATT_SERVICE_DEFINE(accel_svc,
    BT_GATT_PRIMARY_SERVICE(&accel_service_uuid),
    BT_GATT_CHARACTERISTIC(&accel_char_uuid.uuid,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ,
                           read_accel, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed_accel,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

static void assign_attr_pointers(void)
{
    ppg_char_attr = &ppg_svc.attrs[2];
    temp_char_attr = &temp_svc.attrs[2];
    accel_char_attr = &accel_svc.attrs[2];
}

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
    printk("[BLE] Besquare-Yantram Device Connected\n");
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("[BLE] Disconnected (reason 0x%02x)\n", reason);
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
                  0x97, 0x42, 0x94, 0x26, 0x3e, 0x24, 0x9e, 0xf6)};

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
    }
}
