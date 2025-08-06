#include "temp.h"
#include "rtc.h"  // for get_timestamp_ms()

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <nrfx_saadc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(temp_sensor, LOG_LEVEL_INF);

#define ADC_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION      12
#define ADC_GAIN            ADC_GAIN_1_6
#define ADC_REFERENCE       ADC_REF_INTERNAL
#define ADC_ACQ_TIME        ADC_ACQ_TIME_DEFAULT
#define ADC_CHANNEL_ID      1
#define ADC_INPUT_POS       NRF_SAADC_INPUT_AIN2  // P0.03


static const struct device *adc_dev;

// Convert signed 12-bit ADC raw value to voltage (volts)
static float adc_raw_to_voltage(int16_t adc_raw) {
    // SAADC 12-bit signed: value range -2048 to +2047, full scale voltage 3.6V (gain 1/6)
    return ((float)adc_raw / 2048.0f) * 3.6f;
}

// Convert voltage to temperature in Celsius for TMP36-like sensor
static float voltage_to_temperature(float voltage) {
    // Clamp voltage to sensor max rating (TMP36 max output ~1.75V near 125°C)
    if (voltage > 1.75f) {
        voltage = 1.75f;
    }
    if (voltage < 0.0f) {
        voltage = 0.0f;  // clamp negative voltage to zero
    }
    // TMP36: 10 mV per °C with 500 mV offset
    return (voltage - 0.5f) * 100.0f;
}

int temp_sensor_init(void) {
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("ADC device not ready\n");
        return -ENODEV;
    }

    struct adc_channel_cfg channel_cfg = {
        .gain             = ADC_GAIN,
        .reference        = ADC_REFERENCE,
        .acquisition_time = ADC_ACQ_TIME,
        .channel_id       = ADC_CHANNEL_ID,
        .input_positive   = ADC_INPUT_POS,
        .differential     = 0,
    };

    int err = adc_channel_setup(adc_dev, &channel_cfg);
    if (err) {
        printk("ADC channel setup error: %d\n", err);
        return err;
    }

    printk("Temp sensor ADC initialized\n");
    return 0;
}

int temp_sensor_read(TempSample *sample) {
    int16_t sample_buffer = 0;
    struct adc_sequence sequence = {
        .channels    = BIT(ADC_CHANNEL_ID),
        .buffer      = &sample_buffer,
        .buffer_size = sizeof(sample_buffer),
        .resolution  = ADC_RESOLUTION,
    };

    int err = adc_read(adc_dev, &sequence);
    if (err) {
        printk("ADC read error: %d\n", err);
        return err;
    }

    float voltage = adc_raw_to_voltage(sample_buffer);
    float temperature = voltage_to_temperature(voltage);

    sample->adc_raw_value = sample_buffer;
    sample->temperature_c = temperature;
    //sample->timestamp_ms = get_timestamp_ms();


    return 0;
}
