/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Battery Voltage Calculation Implementation
 * Implements ANO-compatible battery voltage monitoring
 */

#include "app/battery_monitor.h"

/* Battery voltage and percentage - shared with OLED thread */
float battery_voltage = 0.0f;
uint8_t battery_level_percent = 0;

/**
 * @brief Initialize battery monitoring system
 *
 * Sets battery_voltage to 0.0 and battery_level_percent to 0
 */
void BatteryInit(void)
{
    battery_voltage = 0.0f;
    battery_level_percent = 0;
}

/**
 * @brief Sample ADC and calculate battery voltage
 *
 * Called at 200Hz rate from battery thread
 * Uses DataRaw[BATTERY_ADC_CHANNEL] with ANO formula:
 * (ADC * 3.3 / 4096) * 2
 * Updates battery_voltage and battery_level_percent
 */
void BatteryCheck(void)
{
    uint16_t adc_value;

    // Read battery voltage ADC channel (PA4)
    adc_value = (uint16_t)DataRaw[BATTERY_ADC_CHANNEL];

    // ANO voltage calculation: (ADC * 3.3 / 4096) * 2
    battery_voltage = (adc_value * BATTERY_REF_VOLTAGE / 4096.0f) * BATTERY_DIVIDER_RATIO;

    // Calculate battery percentage (simple linear)
    if (battery_voltage >= BATTERY_FULL_VOLTAGE) {
        battery_level_percent = 100;
    } else if (battery_voltage <= BATTERY_EMPTY_VOLTAGE) {
        battery_level_percent = 0;
    } else {
        battery_level_percent = (uint8_t)((battery_voltage - BATTERY_EMPTY_VOLTAGE) /
                                         (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100);
    }
}

/**
 * @brief Get current battery voltage
 *
 * @param voltage Pointer to store voltage value
 */
void BatteryGetVoltage(float *voltage)
{
    if (voltage) {
        *voltage = battery_voltage;
    }
}

/**
 * @brief Get battery percentage (0-100)
 *
 * @return uint8_t Battery percentage
 */
uint8_t BatteryGetPercent(void)
{
    return battery_level_percent;
}