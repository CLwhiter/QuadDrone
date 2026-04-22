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

/* ANO protocol header */
#include "drivers/protocol_ano/protocol_ano.h"

/* ANO communication headers */
#include "drivers/uart_ano/uart_ano.h"
#include "drivers/nrf24l01_ano/nrf24l01_ano.h"

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

/* Communication test data */
static uint8_t test_rc_data[8] = {0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80};

/* UART ANO thread */
static void uart_ano_thread(void *p1, void *p2, void *p3)
{
	/* Initialize UART ANO driver */
	int ret = uart_ano_init("UART_1");
	if (ret < 0) {
		printk("UART ANO init failed: %d\n", ret);
		return;
	}

	printk("UART ANO thread started\n");

	/* Test ANO frame transmission */
	struct ano_frame frame;
	int counter = 0;

	while (1) {
		/* Create heartbeat frame */
		ret = protocol_frame_build(&frame, ANO_CMD_HEARTBEAT,
		                           (uint8_t *)&counter, sizeof(counter));
		if (ret < 0) {
			printk("Protocol frame build failed: %d\n", ret);
			k_sleep(K_MSEC(1000));
			continue;
		}

		/* Send via UART */
		ret = uart_ano_send_frame(&frame);
		if (ret < 0) {
			printk("UART send failed: %d\n", ret);
		} else {
			printk("UART heartbeat sent (counter=%d)\n", counter);
		}

		counter++;
		k_sleep(K_MSEC(1000)); // 1Hz heartbeat
	}
}

/* NRF24L01 ANO thread */
static void nrf24l01_ano_thread(void *p1, void *p2, void *p3)
{
	/* Initialize NRF24L01 ANO driver */
	int ret = nrf24l01_ano_init("SPI_2", "nrf24_ce", "nrf24_csn");
	if (ret < 0) {
		printk("NRF24L01 ANO init failed: %d\n", ret);
		return;
	}

	printk("NRF24L01 ANO thread started\n");

	/* Test ANO frame transmission */
	struct ano_frame frame;
	int counter = 0;

	while (1) {
		/* Create RC data frame */
		ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA,
		                           test_rc_data, sizeof(test_rc_data));
		if (ret < 0) {
			printk("Protocol frame build failed: %d\n", ret);
			k_sleep(K_MSEC(100));
			continue;
		}

		/* Send via NRF24L01 */
		ret = nrf24l01_ano_send_frame(&frame);
		if (ret < 0) {
			printk("NRF24L01 send failed: %d\n", ret);
		} else {
			printk("NRF24L01 RC data sent (counter=%d)\n", counter);
		}

		counter++;
		k_sleep(K_MSEC(100)); // 10Hz transmission
	}
}

/* Define communication threads */
K_THREAD_DEFINE(uart_ano_thread_id, 1024, uart_ano_thread, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(nrf24l01_ano_thread_id, 1024, nrf24l01_ano_thread, NULL, NULL, NULL, 6, 0, 0);

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