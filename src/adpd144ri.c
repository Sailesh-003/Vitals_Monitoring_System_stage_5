#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "adpd144ri.h"

#define ADPD144RI_I2C_ADDR 0x64        // I2C slave address

/*This function writes a 16-bit value to a register of the ADPD144RI sensor via the I2C interface.*/
int adpd144ri_write_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3] = { reg, value >> 8, value & 0xFF };
    int ret = i2c_write(i2c_dev, buf, sizeof(buf), ADPD144RI_I2C_ADDR);
    if (ret)
        printk("[I2C] Write reg 0x%02X failed: %d\n", reg, ret);
    return ret;
}


/*This function reads a 16-bit register value from the ADPD144RI optical sensor using the I2C interface.*/
int adpd144ri_read_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    int ret = i2c_write_read(i2c_dev, ADPD144RI_I2C_ADDR, &reg, 1, data, 2);
    if (ret)
        printk("[I2C] Read reg 0x%02X failed: %d\n", reg, ret);
    else
        *value = (data[0] << 8) | data[1];
    return ret;
}


/*This function sets up (initializes and configures) the ADPD144RI sensor module via I2C for SPO2 (blood oxygen) and heart rate (HR) monitoring.*/
int adpd144ri_configure_spo2_hr(const struct device *i2c_dev) {
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


/*This function reads two 24-bit sensor channel values from the ADPD144RI sensor using I2C*/
int adpd144ri_read_2ch(const struct device *i2c_dev, uint32_t *data) {
    int rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0006);
    if (rc) { printk("[I2C] Measurement reg set failed: %d\n", rc); return rc; }

    uint16_t ch_red_l=0, ch_red_h=0, ch_ir_l=0, ch_ir_h=0;

    rc |= adpd144ri_read_reg16(i2c_dev, 0x72, &ch_red_l);      // Read CH3 LSB 
    rc |= adpd144ri_read_reg16(i2c_dev, 0x76, &ch_red_h);      // Read CH3 MSB 
    rc |= adpd144ri_read_reg16(i2c_dev, 0x73, &ch_ir_l);      // Read CH4 LSB
    rc |= adpd144ri_read_reg16(i2c_dev, 0x77, &ch_ir_h);      // Read CH4 MSB

    if (rc) { printk("[I2C] One or more channel reads failed: rc=%d\n", rc); return rc; }

    data[0] = ((uint32_t)ch_red_h << 16) | ch_red_l;      // CH3 = MSB << 16 | LSB
    data[1] = ((uint32_t)ch_ir_h << 16) | ch_ir_l;      // CH4 = MSB << 16 | LSB  
    
    rc = adpd144ri_write_reg16(i2c_dev, 0x5F, 0x0000);
    if (rc) { printk("[I2C] Measurement reg reset failed: %d\n", rc); return rc; }
    return 0;
}