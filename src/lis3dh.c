#include "lis3dh.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2c.h>

static bool lis3dh_write_reg(const lis3dh_sensor_t *dev, uint8_t reg, uint8_t val) {
    return i2c_reg_write_byte(dev->i2c_dev, dev->i2c_addr, reg, val) == 0;
}

static bool lis3dh_read_reg(const lis3dh_sensor_t *dev, uint8_t reg, uint8_t *val) {
    return i2c_reg_read_byte(dev->i2c_dev, dev->i2c_addr, reg, val) == 0;
}

bool lis3dh_init(lis3dh_sensor_t *dev, const struct device *i2c_dev, uint8_t addr) {
    dev->i2c_dev = i2c_dev;
    dev->i2c_addr = addr;

    uint8_t who_am_i = 0;
    if (!lis3dh_read_reg(dev, LIS3DH_REG_WHO_AM_I, &who_am_i)) {
        printk("[LIS3DH] Failed to read WHO_AM_I\n");
        return false;
    }

    if (who_am_i != 0x33) {
        printk("[LIS3DH] Unexpected WHO_AM_I: 0x%02X\n", who_am_i);
        return false;
    }

    // Enable XYZ axes and set 100Hz data rate
    if (!lis3dh_write_reg(dev, LIS3DH_REG_CTRL1, 0x57)) return false;

    // Enable click interrupt on INT1
    if (!lis3dh_write_reg(dev, LIS3DH_REG_CTRL3, 0x80)) return false;

    // Latch interrupt if required
    if (!lis3dh_write_reg(dev, LIS3DH_REG_CTRL5, 0x08)) return false;

    return true;
}

bool lis3dh_set_int_click_config(lis3dh_sensor_t *dev, const lis3dh_int_click_config_t *cfg) {
    uint8_t click_cfg = 0;
    if (cfg->x_single) click_cfg |= BIT(0);
    if (cfg->x_double) click_cfg |= BIT(4);
    if (cfg->y_single) click_cfg |= BIT(1);
    if (cfg->y_double) click_cfg |= BIT(5);
    if (cfg->z_single) click_cfg |= BIT(2);
    if (cfg->z_double) click_cfg |= BIT(6);

    if (!lis3dh_write_reg(dev, LIS3DH_REG_CLICK_CFG, click_cfg)) return false;
    if (!lis3dh_write_reg(dev, LIS3DH_REG_CLICK_THS, cfg->threshold)) return false;
    if (!lis3dh_write_reg(dev, LIS3DH_REG_TIME_LIMIT, cfg->time_limit)) return false;
    if (!lis3dh_write_reg(dev, LIS3DH_REG_TIME_LATENCY, cfg->time_latency)) return false;
    if (!lis3dh_write_reg(dev, LIS3DH_REG_TIME_WINDOW, cfg->time_window)) return false;

    // Enable latching interrupt
    uint8_t ctrl5_val = cfg->latch ? 0x08 : 0x00;
    return lis3dh_write_reg(dev, LIS3DH_REG_CTRL5, ctrl5_val);
}

// LIS3DH Register for output data
#define LIS3DH_OUT_X_L  0x28
#define LIS3DH_AUTO_INCREMENT 0x80

bool lis3dh_read_data(lis3dh_sensor_t *dev, int16_t *data)
{
    uint8_t raw_data[6];
    uint8_t start_reg = LIS3DH_OUT_X_L | LIS3DH_AUTO_INCREMENT;

    int ret = i2c_write_read(dev->i2c_dev, dev->i2c_addr,
                             &start_reg, 1,
                             raw_data, 6);
    if (ret < 0) {
        printk("[LIS3DH] i2c_read failed: %d\n", ret);
        return false;
    }

    // Little endian: Low byte first
    data[0] = (int16_t)(raw_data[1] << 8 | raw_data[0]);
    data[1] = (int16_t)(raw_data[3] << 8 | raw_data[2]);
    data[2] = (int16_t)(raw_data[5] << 8 | raw_data[4]);

    return true;
}