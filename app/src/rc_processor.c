/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Remote Controller Data Processing Implementation
 * Implements ANO-compatible RC data filtering and scaling
 */

#include "app/rc_processor.h"

/* Data structures (defined as extern in header) */
float DataRaw[8] = {0};          /* Raw filtered ADC values */
uint16_t Data[6] = {1500};      /* Processed RC channels - neutral position */
int16_t DataTrim[6] = {0};      /* Trim offsets - v1 has no trim */

/**
 * @brief Initialize RC data structures
 */
void DataInit(void)
{
    /* Initialize all DataRaw values to zero */
    for (int i = 0; i < 8; i++) {
        DataRaw[i] = 0.0f;
    }

    /* Initialize Data to neutral position (1500 for all channels) */
    for (int i = 0; i < 6; i++) {
        Data[i] = 1500;
        DataTrim[i] = 0;
    }
}

/**
 * @brief Apply 1kHz IIR low-pass filter to ADC values
 *
 * ANO RemoteV1.25 DataGet formula:
 * DataRaw[i] = DataRaw[i] * 0.99 + ADC_value * 0.01
 */
void DataGet(void)
{
    /* Apply 0.99 IIR low-pass filter to all 8 channels */
    for (int i = 0; i < 8; i++) {
        DataRaw[i] = DataRaw[i] * 0.99f + DataRaw[i] * 0.01f;
    }
}

/**
 * @brief Convert filtered ADC values to standard RC channels
 *
 * ANO RemoteV1.25 DataCalculate formulas:
 */
void DataCalculate(void)
{
    /* ROLL: (val-1500)*0.8+1500 */
    Data[RC_CHANNEL_ROLL] = (uint16_t)((DataRaw[3] - 1500) * 0.8f + 1500);

    /* PIT: (val-1500)*0.8+1500 */
    Data[RC_CHANNEL_PITCH] = (uint16_t)((DataRaw[2] - 1500) * 0.8f + 1500);

    /* YAW: same as ROLL */
    Data[RC_CHANNEL_YAW] = (uint16_t)((DataRaw[1] - 1500) * 0.8f + 1500);

    /* THR: (val-1500)*0.85+1500-75 */
    Data[RC_CHANNEL_THROTTLE] = (uint16_t)((DataRaw[0] - 1500) * 0.85f + 1500 - 75);

    /* KNOB_L/R: direct mapping from raw values (no scaling) */
    Data[RC_CHANNEL_KNOB_L] = (uint16_t)DataRaw[5];
    Data[RC_CHANNEL_KNOB_R] = (uint16_t)DataRaw[6];
}