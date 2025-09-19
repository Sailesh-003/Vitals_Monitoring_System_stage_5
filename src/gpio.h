#pragma once
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_power.h>

#define GPIO_PIN 16
#define CHARGING_GPIO_PIN 7
#define CHARGING_GPIO_FLAGS (GPIO_INPUT | GPIO_INT_EDGE_RISING)
extern const struct device *gpio0_dev;

void gpio_high(void);
void charging_connected_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
void setup_charging_interrupt(void);
