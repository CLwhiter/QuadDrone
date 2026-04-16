/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Multi-threaded sensor application for QuadDrone Remote Controller
 * ADC samples at 1kHz, OLED updates at 30Hz
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "drivers/adc_multi/adc_multi.h"
#include "drivers/display/ssd1306_spi/ssd1306_spi.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread configurations */
#define ADC_THREAD_PRIORITY    7  /* High priority for 1kHz sampling */
#define ADC_THREAD_STACK_SIZE  512
#define OLED_THREAD_PRIORITY    6  /* Medium priority for 30Hz display */
#define OLED_THREAD_STACK_SIZE  512

#define ADC_SAMPLE_INTERVAL_MS  1   /* 1kHz sampling */
#define OLED_UPDATE_INTERVAL_MS 33  /* ~30Hz display */

/* Shared ADC values */
static uint16_t adc_values[ADC_NUM_CHANNELS];
static struct k_mutex adc_mutex;
static bool adc_ready = false;

/* Thread functions */
void adc_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("ADC thread started");

    while (1) {
        /* Read ADC values */
        int ret = adc_multi_read(adc_values);
        if (ret > 0) {
            k_mutex_lock(&adc_mutex, K_FOREVER);
            adc_ready = true;
            k_mutex_unlock(&adc_mutex);
        }

        /* Wait for next sample */
        k_sleep(K_MSEC(ADC_SAMPLE_INTERVAL_MS));
    }
}

void oled_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("OLED thread started");
    int frame_count = 0;

    while (1) {
        /* Update display at 30Hz */
        k_sleep(K_MSEC(OLED_UPDATE_INTERVAL_MS));

        /* Get latest ADC values */
        k_mutex_lock(&adc_mutex, K_FOREVER);
        if (adc_ready) {
            /* Clear display */
            ssd1306_spi_clear();

            /* Display ADC channel values */
            char buf[32];

            /* Channel 1-4 (main controls) */
            sprintf(buf, "THR:%04d", adc_values[ADC_CHANNEL_THR]);
            ssd1306_spi_putstr(0, 0, buf, 6);

            sprintf(buf, "YAW:%04d", adc_values[ADC_CHANNEL_YAW]);
            ssd1306_spi_putstr(64, 0, buf, 6);

            sprintf(buf, "PIT:%04d", adc_values[ADC_CHANNEL_PITCH]);
            ssd1306_spi_putstr(0, 2, buf, 6);

            sprintf(buf, "ROL:%04d", adc_values[ADC_CHANNEL_ROLL]);
            ssd1306_spi_putstr(64, 2, buf, 6);

            /* Battery voltage */
            sprintf(buf, "BAT:%04d", adc_values[ADC_CHANNEL_POWER]);
            ssd1306_spi_putstr(32, 4, buf, 6);

            /* Frame counter */
            frame_count++;
            sprintf(buf, "F:%03d", frame_count);
            ssd1306_spi_putstr(100, 6, buf, 6);
        }
        k_mutex_unlock(&adc_mutex);
    }
}

void main(void)
{
    int ret;

    LOG_INF("QuadDrone Remote Controller - Multi-threaded Sensor Test");

    /* Initialize mutex */
    k_mutex_init(&adc_mutex);

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

    /* Create threads */
    k_thread_create(&adc_thread_data, adc_thread_stack,
                    ADC_THREAD_STACK_SIZE,
                    adc_thread, NULL, NULL, NULL,
                    ADC_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&oled_thread_data, oled_thread_stack,
                    OLED_THREAD_STACK_SIZE,
                    oled_thread, NULL, NULL, NULL,
                    OLED_THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("Sensor threads started - monitoring in background");

    /* Main thread just maintains the application */
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}

/* Thread data storage */
K_THREAD_DEFINE(adc_thread_data, ADC_THREAD_STACK_SIZE,
                adc_thread, NULL, NULL, NULL,
                ADC_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(oled_thread_data, OLED_THREAD_STACK_SIZE,
                oled_thread, NULL, NULL, NULL,
                OLED_THREAD_PRIORITY, 0, 0);

static char adc_thread_stack[ADC_THREAD_STACK_SIZE];
static char oled_thread_stack[OLED_THREAD_STACK_SIZE];