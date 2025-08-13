#ifndef TEMP_H_
#define TEMP_H_

#include <zephyr/types.h>

typedef struct {
    int16_t  adc_raw_value;   // Raw ADC value for temperature channel
    int16_t  temperature_c;   // Temperature in 0.1 °C units
    int16_t  battery_mv;      // Battery voltage in millivolts
    uint8_t  battery_pct;     // Battery percentage (0–100)
    uint64_t timestamp_ms;    // Measurement timestamp in milliseconds
} TempSample;


int temp_sensor_init(void);

int temp_sensor_read(TempSample *sample);


int read_battery_mv(void);

uint8_t battery_percent(int mv);

#endif 
