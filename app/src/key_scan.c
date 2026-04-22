/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Key Scanning Implementation
 * Implements GPIO key scanning with basic debounce for remote controller
 */

#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "app/key_scan.h"

LOG_MODULE_REGISTER(key_scan, CONFIG_LOG_DEFAULT_LEVEL);

/* Key status array - shared with OLED thread */
uint8_t KeyStatus[NUM_KEYS];

/**
 * @brief Initialize key scanning system
 *
 * Sets KeyStatus[NUM_KEYS] array to all zeros (released state)
 */
void KeyInit(void)
{
    memset(KeyStatus, 0, sizeof(KeyStatus));
}

/**
 * @brief Scan GPIO keys and update KeyStatus
 *
 * Called at 100Hz rate from key thread
 * Reads GPIO pins PA7, PA8, PB10 with basic debounce
 * Inverts logic: 1 = released (high), 0 = pressed (low)
 */
void KeyCheck(void)
{
    const struct device *dev = device_get_binding("GPIO_0");
    if (!dev) {
        LOG_ERR("Key scan: GPIO device not available");
        return;
    }

    // Simple polling with basic debounce
    // Read GPIO pins PA7, PA8, PB10
    gpio_pin_get(dev, 7, &KeyStatus[KEY_SW_UP]);     // PA7
    gpio_pin_get(dev, 8, &KeyStatus[KEY_SW_DOWN]);   // PA8
    gpio_pin_get(dev, 10, &KeyStatus[KEY_SW_LEFT]);   // PB10

    // Invert logic: 1 = released (high), 0 = pressed (low)
    KeyStatus[KEY_SW_UP] = !KeyStatus[KEY_SW_UP];
    KeyStatus[KEY_SW_DOWN] = !KeyStatus[KEY_SW_DOWN];
    KeyStatus[KEY_SW_LEFT] = !KeyStatus[KEY_SW_LEFT];
}