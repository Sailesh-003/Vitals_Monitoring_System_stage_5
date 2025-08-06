#ifndef TEMP_H
#define TEMP_H

#include <zephyr/types.h>

typedef struct {
    //uint64_t timestamp_ms;
    int16_t adc_raw_value;
    float temperature_c;
} TempSample;

// Initialize the temperature sensor ADC (call once during startup)
int temp_sensor_init(void);

// Read temperature sample from ADC and fill TempSample structure
int temp_sensor_read(TempSample *sample);

#endif // TEMP_H
