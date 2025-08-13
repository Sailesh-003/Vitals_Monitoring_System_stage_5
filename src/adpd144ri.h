#ifndef ADPD144RI_H
#define ADPD144RI_H
#include <zephyr/device.h>
#include <stdint.h>

#define PPG_NUM_CHANNELS 2                 // Number of sensor data channels to read

typedef struct {
    uint64_t timestamp_ms;                // Timestamp when sample was taken
    uint32_t channels[PPG_NUM_CHANNELS]; // Array to hold red and IR LED values
} PPGSample;

int adpd144ri_write_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t value);
int adpd144ri_read_reg16(const struct device *i2c_dev, uint8_t reg, uint16_t *value);
int adpd144ri_configure_spo2_hr(const struct device *i2c_dev);
int adpd144ri_read_2ch(const struct device *i2c_dev, uint32_t *data);

#endif
