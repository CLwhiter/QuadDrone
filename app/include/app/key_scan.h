/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Key Scanning Module
 * Implements GPIO key scanning for remote controller buttons
 */

#ifndef APP_KEY_SCAN_H_
#define APP_KEY_SCAN_H_

#include <stdint.h>

/* Key switch definitions matching device tree */
#define KEY_SW_UP      0  // PA7 - Up switch
#define KEY_SW_DOWN    1  // PA8 - Down switch
#define KEY_SW_LEFT    2  // PB10 - Left switch
#define NUM_KEYS      3

/* Key states */
#define KEY_RELEASED   0  // Key not pressed (high impedance, pull-up enabled)
#define KEY_PRESSED    1  // Key pressed (low)

/**
 * @brief Key status array
 *
 * Shared with OLED thread for key state display
 * Updated by Key thread at 100Hz
 * 0 = released, 1 = pressed
 */
extern uint8_t KeyStatus[NUM_KEYS];

/**
 * @brief Initialize key scanning system
 *
 * Sets KeyStatus[NUM_KEYS] array to all zeros (released state)
 */
void KeyInit(void);

/**
 * @brief Scan GPIO keys and update KeyStatus
 *
 * Called at 100Hz rate from key thread
 * Reads GPIO pins PA7, PA8, PB10 with basic debounce
 * Inverts logic: 1 = released (high), 0 = pressed (low)
 */
void KeyCheck(void);

#endif /* APP_KEY_SCAN_H_ */