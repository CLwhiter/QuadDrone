/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Multi-channel ADC driver for QuadDrone Remote Controller
 * Implements 9-channel ADC reading with DMA support
 */

#define DT_DRV_COMPAT adc_multi

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "adc_multi.h"

LOG_MODULE_REGISTER(adc_multi, CONFIG_LOG_DEFAULT_LEVEL);

/* ADC device name */
#define ADC_DEV_NAME "ADC_0"

/* ADC configuration */
static const struct device *adc_dev;
static struct adc_sequence adc_seq;
static uint16_t adc_buffer[ADC_BUFFER_SIZE];

/* Channel configuration - matches ANO RemoteV1.25 */
static const uint32_t adc_channels[] = {
	BIT(ADC_CHANNEL_THR),    /* PA0 */
	BIT(ADC_CHANNEL_YAW),    /* PA1 */
	BIT(ADC_CHANNEL_PITCH),  /* PA2 */
	BIT(ADC_CHANNEL_ROLL),   /* PA3 */
	BIT(ADC_CHANNEL_POWER),  /* PA4 */
	BIT(ADC_CHANNEL_AUX5),   /* PA5 */
	BIT(ADC_CHANNEL_AUX6),   /* PA6 */
	BIT(ADC_CHANNEL_KEY_L),  /* PB0 */
	BIT(ADC_CHANNEL_KEY_R)   /* PB1 */
};

int adc_multi_init(void)
{
	/* Get ADC device */
	adc_dev = device_get_binding(ADC_DEV_NAME);
	if (!adc_dev) {
		LOG_ERR("ADC device %s not found", ADC_DEV_NAME);
		return -ENODEV;
	}

	/* Configure ADC sequence for multi-channel reading */
	adc_seq.channels = 0;
	for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
		if (i < (sizeof(adc_channels) / sizeof(adc_channels[0]))) {
			adc_seq.channels |= adc_channels[i];
		} else {
			LOG_WRN("ADC channel %d not configured", i);
		}
	}

	adc_seq.buffer = adc_buffer;
	adc_seq.buffer_size = sizeof(adc_buffer);
	adc_seq.resolution = 12;
	adc_seq.oversampling = 0;
	adc_seq.calibrate = false;

	LOG_INF("ADC multi-channel driver initialized (%d channels)", ADC_NUM_CHANNELS);
	return 0;
}

int adc_multi_read(uint16_t *buffer)
{
	if (!adc_dev || !buffer) {
		return -EINVAL;
	}

	/* Read all channels using Zephyr ADC sequence API */
	int ret = adc_read(adc_dev, &adc_seq);
	if (ret < 0) {
		LOG_ERR("ADC read failed (%d)", ret);
		return ret;
	}

	/* Copy buffer to output */
	for (int i = 0; i < ADC_NUM_CHANNELS; i++) {
		buffer[i] = adc_buffer[i];
	}

	return ADC_NUM_CHANNELS;
}

uint32_t adc_multi_get_channel_mask(void)
{
	return adc_seq.channels;
}