/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Multi-channel ADC driver for QuadDrone Remote Controller
 * Implements 9-channel ADC reading with DMA support
 */

#ifndef APP_DRIVERS_ADC_MULTI_H_
#define APP_DRIVERS_ADC_MULTI_H_

#include <stdint.h>
#include <zephyr/drivers/adc.h>

/* ADC channel configuration - matches ANO RemoteV1.25 */
#define ADC_NUM_CHANNELS      9

/* ADC channel assignments */
#define ADC_CHANNEL_THR       0  /* PA0 - Throttle */
#define ADC_CHANNEL_YAW       1  /* PA1 - Yaw */
#define ADC_CHANNEL_PITCH     2  /* PA2 - Pitch */
#define ADC_CHANNEL_ROLL      3  /* PA3 - Roll */
#define ADC_CHANNEL_POWER     4  /* PA4 - Battery voltage */
#define ADC_CHANNEL_AUX5      5  /* PA5 - Aux 5 */
#define ADC_CHANNEL_AUX6      6  /* PA6 - Aux 6 */
#define ADC_CHANNEL_KEY_L     8  /* PB0 - Left key analog */
#define ADC_CHANNEL_KEY_R     9  /* PB1 - Right key analog */

/* ADC buffer size for circular DMA */
#define ADC_BUFFER_SIZE      ADC_NUM_CHANNELS

/**
 * @brief Initialize multi-channel ADC with DMA
 *
 * @return int 0 on success, negative error code on failure
 */
int adc_multi_init(void);

/**
 * @brief Read all 9 ADC channels at once
 *
 * @param buffer Buffer to store ADC values (must be ADC_NUM_CHANNELS size)
 * @return int Number of channels read, or negative error code
 */
int adc_multi_read(uint16_t *buffer);

/**
 * @brief Get ADC channel configuration bit mask
 *
 * @return uint32_t Bit mask of enabled channels
 */
uint32_t adc_multi_get_channel_mask(void);

#endif /* APP_DRIVERS_ADC_MULTI_H_ */