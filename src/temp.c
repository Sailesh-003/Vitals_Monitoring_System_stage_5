#include "temp.h"
#include "rtc.h"  

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <nrfx_saadc.h> 

LOG_MODULE_REGISTER(temp_sensor, LOG_LEVEL_INF);

/* === ADC Configuration === */
#define ADC_NODE              DT_NODELABEL(adc)
#define ADC_RESOLUTION        12
#define ADC_GAIN_TEMP         ADC_GAIN_1_4
#define ADC_GAIN_BATTERY      ADC_GAIN_1_6
#define ADC_REFERENCE_TEMP    ADC_REF_VDD_1_4
#define ADC_REFERENCE_BATTERY ADC_REF_INTERNAL
#define ADC_ACQ_TIME_CFG      ADC_ACQ_TIME_DEFAULT

#define ADC_CHANNEL_TEMP      1
#define ADC_INPUT_POS_TEMP    NRF_SAADC_INPUT_AIN2  // AIN2 = P0.03

#define ADC_CHANNEL_BATT      6
#define ADC_INPUT_POS_BATT    NRF_SAADC_INPUT_AIN6  // AIN6 = P0.30

#define RES_12BIT             4096
#define VDD_VOLT              3.0f   // V
#define ADC_REF_VOLTAGE_BATT  600
#define ADC_MAX_VALUE         4095

#define MAX_VOL_TEMP          (3000 / 1000.0f) // 3.0 V
#define FIRST_SAMPLING_WINDOW 200
#define SECOND_SAMPLING_WINDOW 400
#define MATCHES               10
#define MOVING_AVG_WINDOW     10

/* Steinhart–Hart coefficients */
#define A_COEFF 0.0011574933f
#define B_COEFF 0.0002290130f
#define C_COEFF 0.0000001117f
#define KELVIN  273.15f

/* Static state variables */
static const struct device *adc_dev;
static int16_t adc_sample_temp;
static int16_t adc_sample_batt;

static float calc_volt_temp = 0;
static float Temp_C = 0;

/* Moving average buffers */
static float moving_avg_buffer[MOVING_AVG_WINDOW];
static int moving_avg_index = 0;
static bool moving_avg_buffer_filled = false;
static float moving_avg_sum = 0;

/* Stabilization sampling */
static unsigned int i_window = 0;
static long int base_value_sum = 0;
static long int new_value_sum = 0;
static double base_value_average = 0;
static double new_value_average = 0;
static unsigned int stable_count = 0;

/* --- Moving average functions --- */
static void moving_avg_init(void) {
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        moving_avg_buffer[i] = 0;
    }
    moving_avg_sum = 0;
    moving_avg_index = 0;
    moving_avg_buffer_filled = false;
}

static float apply_moving_avg(float new_sample) {
    moving_avg_sum -= moving_avg_buffer[moving_avg_index];
    moving_avg_buffer[moving_avg_index] = new_sample;
    moving_avg_sum += new_sample;
    moving_avg_index = (moving_avg_index + 1) % MOVING_AVG_WINDOW;
    if (!moving_avg_buffer_filled && moving_avg_index == 0) {
        moving_avg_buffer_filled = true;
    }
    return moving_avg_buffer_filled
           ? (moving_avg_sum / MOVING_AVG_WINDOW)
           : (moving_avg_sum / moving_avg_index);
}

/* Helper to read ADC channel */
static int adc_read_channel(uint8_t channel, int16_t *buf) {
    struct adc_sequence sequence = {
        .channels    = BIT(channel),
        .buffer      = buf,
        .buffer_size = sizeof(*buf),
        .resolution  = ADC_RESOLUTION,
    };
    return adc_read(adc_dev, &sequence);
}

/* Fetch stabilized averaged ADC value for temperature */
static float adc_compare_temp(void) {
    while (1) {
        if (adc_read_channel(ADC_CHANNEL_TEMP, &adc_sample_temp) != 0)
            return 0;

        float adc_val = apply_moving_avg((float)adc_sample_temp);

        if (i_window <= FIRST_SAMPLING_WINDOW) {
            base_value_sum += adc_val;
            if (i_window == FIRST_SAMPLING_WINDOW) {
                base_value_average = base_value_sum / FIRST_SAMPLING_WINDOW;
            }
            i_window++;
        } else if (i_window <= SECOND_SAMPLING_WINDOW) {
            new_value_sum += adc_val;
            if (i_window == SECOND_SAMPLING_WINDOW) {
                new_value_average = new_value_sum / FIRST_SAMPLING_WINDOW;
            }
            i_window++;
        } else {
            i_window = 1;
            base_value_sum = 0;
            new_value_sum = 0;

            float threshold = new_value_average - base_value_average;
            stable_count = (threshold >= -1 && threshold <= 1) ? stable_count + 1 : 0;
            base_value_average = new_value_average;

            if (stable_count == MATCHES) {
                stable_count = 0;
                return new_value_average;
            }
        }
        k_sleep(K_MSEC(1));
    }
}

/* Fetch and store temperature voltage */
static void fetch_temp(void) {
    double adc_result = adc_compare_temp();
    calc_volt_temp = (adc_result * MAX_VOL_TEMP) / RES_12BIT;
    LOG_INF("Temp voltage: %.3f V",(double) calc_volt_temp);
}

/* Voltage -> Temperature */
static float voltage_to_temperature(float volt) {
    if (volt <= 0.0f) {
        return -273.15f; // invalid
    }
    float Rntc = ((30.0f / volt) - 10.0f) * 1000.0f;
    float lnR = logf(Rntc);
    float invT = A_COEFF + (B_COEFF * lnR) + (C_COEFF * powf(lnR, 3));
    float tempC = (1.0f / invT) - KELVIN;
    tempC += 0.05f; // calibration
    tempC -= 0.3f;
    return tempC;
}

/* Process last fetched voltage into temperature */
static void get_temp(void) {
    if (calc_volt_temp <= 0) return;
    Temp_C = voltage_to_temperature(calc_volt_temp);
    Temp_C = roundf(Temp_C * 10) / 10;
}

/* Battery read */
int read_battery_mv(void) {
    if (adc_read_channel(ADC_CHANNEL_BATT, &adc_sample_batt) != 0)
        return -1;
    int32_t mv = adc_sample_batt;
    mv = mv * ADC_REF_VOLTAGE_BATT * 6 / ADC_MAX_VALUE;
    return mv;
}

/* Battery percent */
uint8_t battery_percent(int mv) {
    const int full = 4200, empty = 3400;
    if (mv >= full) return 100;
    if (mv <= empty) return 5;
    return (uint8_t)(((mv - empty) * 95) / (full - empty) + 5);
}

/* Init ADC */
int temp_sensor_init(void) {
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    if (!device_is_ready(adc_dev)) {
        printk("[TEMP] ADC device not ready!\n");
        return -ENODEV;
    }

    struct adc_channel_cfg temp_cfg = {
        .gain           = ADC_GAIN_TEMP,
        .reference      = ADC_REFERENCE_TEMP,
        .acquisition_time = ADC_ACQ_TIME_CFG,
        .channel_id     = ADC_CHANNEL_TEMP,
        .input_positive = ADC_INPUT_POS_TEMP,
        .differential   = 0,
    };

    int err = adc_channel_setup(adc_dev, &temp_cfg);
    if (err) {
        printk("[TEMP] ADC temp channel setup failed: %d\n", err);
        return err;
    }

    struct adc_channel_cfg batt_cfg = {
        .gain           = ADC_GAIN_BATTERY,
        .reference      = ADC_REFERENCE_BATTERY,
        .acquisition_time = ADC_ACQ_TIME_CFG,
        .channel_id     = ADC_CHANNEL_BATT,
        .input_positive = ADC_INPUT_POS_BATT,
        .differential   = 0,
    };

    err = adc_channel_setup(adc_dev, &batt_cfg);
    if (err) {
        printk("[TEMP] ADC battery channel setup failed: %d\n", err);
        return err;
    }

    moving_avg_init();

    printk("[TEMP] ADC initialized with moving average filter\n");
    return 0;
}

/* Read temperature and battery (updated to use stabilized average) */
int temp_sensor_read(TempSample *sample) {
    fetch_temp();
    get_temp();

    sample->temperature_c = (int16_t)lroundf(Temp_C * 10.0f); // store as 0.1°C

    int mv = read_battery_mv();
    sample->battery_mv = mv;
    sample->battery_pct = (mv >= 0) ? battery_percent(mv) : 0;

    sample->adc_raw_value = adc_sample_temp;

    return 0;
}
