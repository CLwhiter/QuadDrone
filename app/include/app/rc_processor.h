/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Remote Controller Data Processing Module
 * Implements ANO-compatible RC data filtering and scaling
 */

#ifndef APP_RC_PROCESSOR_H_
#define APP_RC_PROCESSOR_H_

#include <stdint.h>

/* RC channel definitions */
#define RC_CHANNEL_THROTTLE     0
#define RC_CHANNEL_YAW         1
#define RC_CHANNEL_PITCH       2
#define RC_CHANNEL_ROLL        3
#define RC_CHANNEL_KNOB_L      4
#define RC_CHANNEL_KNOB_R      5

/* Data structures */
extern float DataRaw[8];          /* Raw filtered ADC values */
extern uint16_t Data[6];          /* Processed RC channels (1000-2000 range) */
extern int16_t DataTrim[6];       /* Trim offsets (initialized to 0 for v1) */

/**
 * @brief Initialize RC data structures
 *
 * Sets DataRaw[8] to zeros, Data[6] to 1500 (neutral position),
 * and DataTrim[6] to zeros (no trim in v1)
 */
void DataInit(void);

/**
 * @brief Apply 1kHz IIR low-pass filter to ADC values
 *
 * Implements ANO RemoteV1.25 DataGet formula:
 * DataRaw[i] = DataRaw[i] * 0.99 + ADC_value * 0.01
 *
 * @note Called at 1kHz rate from ADC thread
 */
void DataGet(void);

/**
 * @brief Convert filtered ADC values to standard RC channels
 *
 * Implements ANO RemoteV1.25 DataCalculate formulas:
 * - ROLL: (val-1500)*0.8+1500
 * - PIT: (val-1500)*0.8+1500
 * - YAW: (val-1500)*0.8+1500
 * - THR: (val-1500)*0.85+1500-75
 * - KNOB_L/R: direct mapping from raw values
 *
 * @note Called at 200Hz rate from Calc thread
 */
void DataCalculate(void);

#endif /* APP_RC_PROCESSOR_H_ */