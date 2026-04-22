/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Multi-threaded Remote Controller Application
 * ADC: 1kHz sampling, Calc: 200Hz processing, Keys: 100Hz, NRF: 50Hz, OLED: 30Hz
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "drivers/adc_multi/adc_multi.h"
#include "drivers/display/ssd1306_spi/ssd1306_spi.h"
#include "app/rc_processor.h"
#include "app/key_scan.h"
#include "app/battery_monitor.h"
#include "app/flight_display.h"

/* ANO protocol header */
#include "drivers/protocol_ano/protocol_ano.h"

/* ANO communication headers */
#include "drivers/uart_ano/uart_ano.h"
#include "drivers/nrf24l01_ano/nrf24l01_ano.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Thread configurations */
#define ADC_THREAD_PRIORITY    7
#define ADC_THREAD_STACK_SIZE  256
#define CALC_THREAD_PRIORITY   8
#define CALC_THREAD_STACK_SIZE 256
#define KEY_THREAD_PRIORITY    9
#define KEY_THREAD_STACK_SIZE  256
#define NRF_TX_THREAD_PRIORITY 9
#define NRF_TX_THREAD_STACK_SIZE 256
#define OLED_THREAD_PRIORITY   10
#define OLED_THREAD_STACK_SIZE 256

/* Thread intervals */
#define ADC_INTERVAL_MS        1    /* 1kHz sampling */
#define CALC_INTERVAL_MS       5    /* 200Hz processing */
#define KEY_INTERVAL_MS        10   /* 100Hz key scanning */
#define NRF_TX_INTERVAL_MS    20   /* 50Hz transmission */
#define OLED_INTERVAL_MS       33   /* ~30Hz display */

/* Thread function prototypes */
void adc_thread(void *p1, void *p2, void *p3);
void calc_thread(void *p1, void *p2, void *p3);
void key_thread(void *p1, void *p2, void *p3);
void battery_thread(void *p1, void *p2, void *p3);
void nrf_tx_thread(void *p1, void *p2, void *p3);
void oled_thread(void *p1, void *p2, void *p3);

/* Thread data storage */
static K_THREAD_STACK_DEFINE(adc_thread_stack, ADC_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(calc_thread_stack, CALC_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(key_thread_stack, KEY_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(battery_thread_stack, CALC_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(nrf_tx_thread_stack, NRF_TX_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(oled_thread_stack, OLED_THREAD_STACK_SIZE);

static struct k_thread adc_thread_data;
static struct k_thread calc_thread_data;
static struct k_thread key_thread_data;
static struct k_thread battery_thread_data;
static struct k_thread nrf_tx_thread_data;
static struct k_thread oled_thread_data;

/* Thread data storage */
static K_THREAD_DEFINE(nrf24l01_tx_thread_data, NRF_TX_THREAD_STACK_SIZE,
                        nrf_tx_thread, NULL, NULL, NULL,
                        NRF_TX_THREAD_PRIORITY, 0, 0);

/* ADC buffer for reading values */
static uint16_t adc_buffer[ADC_NUM_CHANNELS];

void adc_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("ADC thread started (1kHz)");

    while (1) {
        /* Read ADC values from DMA buffer */
        adc_multi_read(adc_buffer);

        /* Copy to DataRaw for filtering */
        memcpy(DataRaw, adc_buffer, sizeof(adc_buffer));

        /* Apply 1kHz IIR filter */
        DataGet();

        k_sleep(K_MSEC(ADC_INTERVAL_MS));
    }
}

void calc_thread(void *p1, void *p2, void *p3)
{
    LOG_INF("Calc thread started (200Hz)");

    while (1) {
        /* Apply ANO scaling formulas to convert to standard RC channels */
        DataCalculate();

        k_sleep(K_MSEC(CALC_INTERVAL_MS));
    }
}

void key_thread(void *p1, void *p2, void *p3)
{
    KeyInit();
    LOG_INF("Key thread started (100Hz)");

    while (1) {
        KeyCheck();
        k_sleep(K_MSEC(KEY_INTERVAL_MS));
    }
}

void battery_thread(void *p1, void *p2, void *p3)
{
    BatteryInit();
    LOG_INF("Battery thread started (200Hz)");

    while (1) {
        BatteryCheck();
        k_sleep(K_MSEC(5));  /* 200Hz */
    }
}

void nrf_tx_thread(void *p1, void *p2, void *p3)
{
    int ret;
    LOG_INF("NRF TX thread started (50Hz)");

    /* Initialize NRF24L01 ANO driver */
    ret = nrf24l01_ano_init("SPI_2", "nrf24_ce", "nrf24_csn");
    if (ret < 0) {
        LOG_ERR("NRF24L01 ANO init failed: %d", ret);
        return;
    }

    struct ano_frame frame;
    int counter = 0;

    while (1) {
        /* Create RC data frame with 6 channels */
        ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA,
                                   (uint8_t *)Data, sizeof(Data));
        if (ret < 0) {
            LOG_ERR("Protocol frame build failed: %d", ret);
            k_sleep(K_MSEC(NRF_TX_INTERVAL_MS));
            continue;
        }

        /* Send via NRF24L01 */
        ret = nrf24l01_ano_send_frame(&frame);
        if (ret < 0) {
            LOG_WRN("NRF24L01 send failed: %d", ret);
        } else {
            LOG_DBG("NRF24L01 RC data sent (ch=%d,%d,%d,%d,%d,%d)",
                    Data[0], Data[1], Data[2], Data[3], Data[4], Data[5]);
        }

        counter++;
        k_sleep(K_MSEC(NRF_TX_INTERVAL_MS));
    }
}

void oled_thread(void *p1, void *p2, void *p3)
{
    FlightDisplayInit();
    LOG_INF("OLED thread started (30Hz)");
    int frame_count = 0;

    while (1) {
        /* Update display at 30Hz */
        k_sleep(K_MSEC(OLED_INTERVAL_MS));

        FlightDisplayUpdate();
        frame_count++;
    }
}

void main(void)
{
    int ret;

    LOG_INF("QuadDrone Remote Controller - Multi-threaded RC Application");

    /* Initialize RC data structures */
    DataInit();

    /* Initialize key and battery monitoring */
    KeyInit();
    BatteryInit();

    /* Initialize multi-channel ADC */
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

    /* Initialize flight display */
    FlightDisplayInit();

    /* Create all threads with proper priorities */
    k_thread_create(&adc_thread_data, adc_thread_stack,
                    ADC_THREAD_STACK_SIZE,
                    adc_thread, NULL, NULL, NULL,
                    ADC_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&calc_thread_data, calc_thread_stack,
                    CALC_THREAD_STACK_SIZE,
                    calc_thread, NULL, NULL, NULL,
                    CALC_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&key_thread_data, key_thread_stack,
                    KEY_THREAD_STACK_SIZE,
                    key_thread, NULL, NULL, NULL,
                    KEY_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&battery_thread_data, battery_thread_stack,
                    CALC_THREAD_STACK_SIZE,
                    battery_thread, NULL, NULL, NULL,
                    CALC_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&nrf_tx_thread_data, nrf_tx_thread_stack,
                    NRF_TX_THREAD_STACK_SIZE,
                    nrf_tx_thread, NULL, NULL, NULL,
                    NRF_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

    k_thread_create(&oled_thread_data, oled_thread_stack,
                    OLED_THREAD_STACK_SIZE,
                    oled_thread, NULL, NULL, NULL,
                    OLED_THREAD_PRIORITY, 0, K_NO_WAIT);

    LOG_INF("All RC threads started");

    /* Main thread just maintains the application */
    while (1) {
        k_sleep(K_SECONDS(1));
    }
}