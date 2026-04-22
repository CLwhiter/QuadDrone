/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Flight Display Implementation for OLED Display
 * Implements ANO-style flight status display rendering
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <drivers/display/ssd1306_spi/ssd1306_spi.h>
#include <drivers/nrf24l01_ano/nrf24l01_ano.h>
#include "app/rc_processor.h"
#include "app/key_scan.h"
#include "app/battery_monitor.h"

/* Frame counter for display */
static int frame_count = 0;

/* Mutex for thread-safe display data access */
static struct k_mutex display_data_mutex;

/**
 * @brief Initialize OLED flight display
 */
void FlightDisplayInit(void)
{
    /* Initialize mutex for thread-safe display data access */
    k_mutex_init(&display_data_mutex);

    const struct device *dev = device_get_binding("oled_spi");
    if (!dev) {
        return;
    }

    ssd1306_spi_init(dev);
    ssd1306_spi_clear();
}

/**
 * @brief Update flight display with current data
 */
void FlightDisplayUpdate(void)
{
    char buf[MAX_ROW_CHARS];
    uint16_t temp_data[6];
    float temp_voltage;

    /* Copy shared data to local variables for thread safety */
    k_mutex_lock(&display_data_mutex, K_FOREVER);
    memcpy(temp_data, Data, sizeof(temp_data));
    temp_voltage = battery_voltage;
    k_mutex_unlock(&display_data_mutex);

    /* Clear display */
    ssd1306_spi_clear();

    /* Row 0: THR:xxxx  YAW:xxxx */
    sprintf(buf, "THR:%04d  YAW:%04d", temp_data[RC_CHANNEL_THROTTLE], temp_data[RC_CHANNEL_YAW]);
    ssd1306_spi_putstr(0, 0, buf, 6);

    /* Row 1: PIT:xxxx  ROL:xxxx */
    sprintf(buf, "PIT:%04d  ROL:%04d", temp_data[RC_CHANNEL_PITCH], temp_data[RC_CHANNEL_ROLL]);
    ssd1306_spi_putstr(0, 2, buf, 6);

    /* Row 2: BAT:x.xxV */
    sprintf(buf, "BAT:%.2fV", temp_voltage);
    ssd1306_spi_putstr(32, 4, buf, 6);

    /* Row 3: NRF status */
    if (nrf24l01_is_connected()) {
        sprintf(buf, "NRF:OK");
    } else {
        sprintf(buf, "NRF:FAIL");
    }
    ssd1306_spi_putstr(40, 6, buf, 6);

    /* Row 4: KNOB_L:xxxx */
    sprintf(buf, "L:%04d", temp_data[RC_CHANNEL_KNOB_L]);
    ssd1306_spi_putstr(0, 8, buf, 6);

    /* Row 5: KNOB_R:xxxx */
    sprintf(buf, "R:%04d", temp_data[RC_CHANNEL_KNOB_R]);
    ssd1306_spi_putstr(64, 8, buf, 6);

    /* Row 6: Frame counter */
    frame_count++;
    sprintf(buf, "F:%03d", frame_count % 1000);
    ssd1306_spi_putstr(100, 10, buf, 6);

    /* Row 7: Key status */
    sprintf(buf, "Keys:%d%d%d", KeyStatus[0], KeyStatus[1], KeyStatus[2]);
    ssd1306_spi_putstr(0, 12, buf, 6);
}