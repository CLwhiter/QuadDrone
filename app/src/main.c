/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#define LED_NODE DT_ALIAS(led0)

#if !DT_NODE_HAS_STATUS(LED_NODE, okay)
#error "LED node is not enabled in device tree"
#endif

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

int main(void)
{
	int ret;

	if (!device_is_ready(led.port)) {
		printk("Error: LED device %s is not ready\n", led.port->name);
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure LED pin %d\n", ret, led.pin);
		return ret;
	}

	printk("QuadDrone ANO Remote Controller - LED Test\n");
	printk("LED initialized on pin %d (%s)\n", led.pin, led.port->name);

	while (1) {
		gpio_pin_set_dt(&led, 1);
		printk("LED ON\n");
		k_msleep(500);

		gpio_pin_set_dt(&led, 0);
		printk("LED OFF\n");
		k_msleep(500);
	}

	return 0;
}
