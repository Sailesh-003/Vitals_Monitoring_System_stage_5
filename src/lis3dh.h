#ifndef LIS3DH_H
#define LIS3DH_H

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdbool.h>

#define LIS3DH_REG_WHO_AM_I         0x0F
#define LIS3DH_REG_CTRL1            0x20
#define LIS3DH_REG_CTRL3            0x22
#define LIS3DH_REG_CTRL5            0x24
#define LIS3DH_REG_CTRL6            0x25
#define LIS3DH_REG_CLICK_CFG        0x38
#define LIS3DH_REG_CLICK_SRC        0x39
#define LIS3DH_REG_CLICK_THS        0x3A
#define LIS3DH_REG_TIME_LIMIT       0x3B
#define LIS3DH_REG_TIME_LATENCY     0x3C
#define LIS3DH_REG_TIME_WINDOW      0x3D

typedef struct {
    const struct device *i2c_dev;
    uint8_t i2c_addr;
} lis3dh_sensor_t;

typedef struct {
    bool x_single;
    bool x_double;
    bool y_single;
    bool y_double;
    bool z_single;
    bool z_double;
    uint8_t threshold;
    uint8_t time_limit;
    uint8_t time_latency;
    uint8_t time_window;
    bool latch;
} lis3dh_int_click_config_t;

bool lis3dh_init(lis3dh_sensor_t *dev, const struct device *i2c_dev, uint8_t addr);
bool lis3dh_set_int_click_config(lis3dh_sensor_t *dev, const lis3dh_int_click_config_t *cfg);

typedef struct {
    uint16_t accel_sample_id;
    uint64_t timestamp_ms;
    int16_t data[3];
} lis3dh_data_t;

bool lis3dh_read_data(lis3dh_sensor_t *dev, int16_t *data);


#endif