/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Flight Display Module for OLED Display
 * Implements ANO-style flight status display layout
 */

#ifndef APP_FLIGHT_DISPLAY_H_
#define APP_FLIGHT_DISPLAY_H_

#include <stdint.h>

/* Display mode constants */
#define DISPLAY_MODE_FLIGHT     0  // Flight mode only (v1 scope)
#define DISPLAY_MODE_CONFIG     1  // Future v2
#define DISPLAY_MODE_PID        2  // Future v2

/* Display layout constants (6x8 font, 128x64 OLED) */
#define DISPLAY_ROW_HEIGHT     8
#define MAX_ROW_CHARS         21
#define DISPLAY_ROWS          8

/**
 * @brief Initialize OLED flight display
 *
 * Clears screen and prepares display for flight mode updates
 */
void FlightDisplayInit(void);

/**
 * @brief Update flight display with current data
 *
 * Renders 8-row ANO flight status layout:
 * - Row 0: THR:xxxx  YAW:xxxx
 * - Row 1: PIT:xxxx  ROL:xxxx
 * - Row 2: BAT:x.xxV
 * - Row 3: NRF:OK or NRF:FAIL
 * - Row 4: KNOB_L:xxxx
 * - Row 5: KNOB_R:xxxx
 * - Row 6: Frame counter (F:xxx)
 * - Row 7: Key status (Keys:xyz)
 *
 * @note Called at 30Hz rate from OLED thread
 */
void FlightDisplayUpdate(void);

#endif /* APP_FLIGHT_DISPLAY_H_ */