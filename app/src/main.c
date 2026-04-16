/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Main application for QuadDrone Remote Controller
 * Tests sensor drivers: ADC multi-channel and SSD1306 OLED
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "drivers/adc_multi/adc_multi.h"
#include "drivers/display/ssd1306_spi/ssd1306_spi.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

void main(void)
{
    int ret;

    LOG_INF("QuadDrone Remote Controller - Sensor Test");

    /* Initialize ADC multi-channel driver */
    ret = adc_multi_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize ADC driver (%d)", ret);
        return;
    }

    /* Initialize SSD1306 OLED display */
    const struct device *spi_dev = device_get_binding("oled_spi");
    if (!spi_dev) {
        LOG_ERR("Failed to get SPI device");
        return;
    }

    ret = ssd1306_spi_init(spi_dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize OLED display (%d)", ret);
        return;
    }

    /* Clear display and show test message */
    ssd1306_spi_clear();
    ssd1306_spi_putstr(32, 2, "QuadDrone", 8);

    k_sleep(K_SECONDS(2));

    /* Show ADC values test */
    uint16_t adc_values[ADC_NUM_CHANNELS];
    ret = adc_multi_read(adc_values);
    if (ret > 0) {
        LOG_INF("ADC values: THR=%d YAW=%d PITCH=%d ROLL=%d",
                adc_values[ADC_CHANNEL_THR],
                adc_values[ADC_CHANNEL_YAW],
                adc_values[ADC_CHANNEL_PITCH],
                adc_values[ADC_CHANNEL_ROLL]);
    }

    /* Display a simple status */
    ssd1306_spi_clear();
    ssd1306_spi_putstr(48, 2, "OK", 6);

    LOG_INF("Sensor test completed");
}