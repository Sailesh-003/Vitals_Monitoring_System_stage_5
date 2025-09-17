
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include "gpio.h"

const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct gpio_callback charging_cb_data;

void gpio_high(void)
{
      if (!device_is_ready(gpio0_dev)) {
        printk("Error: GPIO device not ready\n");
        return;
    }

    // Configure pin as output and set high
    int x=gpio_pin_configure(gpio0_dev, GPIO_PIN, GPIO_OUTPUT_ACTIVE);

    if(!x){printk("GPIO 16 set HIGH\n");}
    else{printk("gpio fail\n");}
} 


void charging_connected_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("[CHARGING] Interrupt triggered! Charger connected.\n");

    if (!device_is_ready(gpio0_dev)) {
        printk("Error: GPIO device not ready\n");
        return;
    }

    // Set GPIO 16 low to cut power
    int ret = gpio_pin_configure(gpio0_dev, GPIO_PIN, GPIO_OUTPUT_LOW);
    if (ret == 0) {
        printk("GPIO 16 set LOW (cutting power)\n");
    } else {
        printk("Failed to set GPIO 16 LOW: %d\n", ret);
    }

    printk("[CHARGING] Shutting down the device...\n");
    nrf_power_system_off(NRF_POWER);
}


// Setup charging pin as input with interrupt
void setup_charging_interrupt(void)
{
    int ret;

    if (!device_is_ready(gpio0_dev)) {
        printk("Error: GPIO device not ready\n");
        return;
    }


    // Configure the pin as input with pull-down
    ret = gpio_pin_configure(gpio0_dev, CHARGING_GPIO_PIN, GPIO_INPUT | GPIO_PULL_DOWN);
    if (ret < 0) {
        printk("[CHARGING] Error configuring pin: %d\n", ret);
        return;
    }

    // Configure interrupt on rising edge
    ret = gpio_pin_interrupt_configure(gpio0_dev, CHARGING_GPIO_PIN, GPIO_INT_EDGE_RISING);
    if (ret < 0) {
        printk("[CHARGING] Error configuring interrupt: %d\n", ret);
        return;
    }

    // Initialize the callback
    gpio_init_callback(&charging_cb_data, charging_connected_callback, BIT(CHARGING_GPIO_PIN));
    ret = gpio_add_callback(gpio0_dev, &charging_cb_data);
    if (ret < 0) {
        printk("[CHARGING] Error adding callback: %d\n", ret);
        return;
    }

    printk("[CHARGING] Charging interrupt configured on GPIO %d\n", CHARGING_GPIO_PIN);
}
