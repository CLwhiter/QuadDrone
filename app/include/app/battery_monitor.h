/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Battery Monitoring Module
 * Implements battery voltage calculation for remote controller
 */

#ifndef APP_BATTERY_MONITOR_H_
#define APP_BATTERY_MONITOR_H_

#include <stdint.h>

/* Battery voltage calculation constants */
#define BATTERY_ADC_CHANNEL     4  // PA4 - Battery voltage
#define BATTERY_REF_VOLTAGE     3.3f  // ADC reference voltage (V)
#define BATTERY_DIVIDER_RATIO  2.0f  // 2:1 voltage divider ratio
#define BATTERY_FULL_VOLTAGE    4.2f  // Li-ion fully charged voltage (V)
#define BATTERY_EMPTY_VOLTAGE  3.0f  // Li-ion empty voltage (V)

/* Battery status - shared with OLED thread */
extern float battery_voltage;           // Battery voltage in volts
extern uint8_t battery_level_percent;   // Battery percentage (0-100)

/**
 * @brief Initialize battery monitoring system
 *
 * Sets battery_voltage to 0.0 and battery_level_percent to 0
 */
void BatteryInit(void);

/**
 * @brief Sample ADC and calculate battery voltage
 *
 * Called at 200Hz rate from battery thread
 * Uses DataRaw[BATTERY_ADC_CHANNEL] with ANO formula:
 * (ADC * 3.3 / 4096) * 2
 * Updates battery_voltage and battery_level_percent
 */
void BatteryCheck(void);

/**
 * @brief Get current battery voltage
 *
 * @param voltage Pointer to store voltage value
 */
void BatteryGetVoltage(float *voltage);

/**
 * @brief Get battery percentage (0-100)
 *
 * @return uint8_t Battery percentage
 */
uint8_t BatteryGetPercent(void);

#endif /* APP_BATTERY_MONITOR_H_ */