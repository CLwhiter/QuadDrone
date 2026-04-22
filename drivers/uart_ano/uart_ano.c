/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "uart_ano.h"

LOG_MODULE_REGISTER(uart_ano, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_UART_ANO_DEBUG
#define UART_LOG_DBG(...) LOG_DBG(__VA_ARGS__)
#else
#define UART_LOG_DBG(...)
#endif

static const struct device *uart_dev;
static uint8_t uart_buffer[UART_ANO_BUFFER_SIZE];
static struct k_mutex uart_mutex;
static uart_ano_error_callback_t error_callback;

int uart_ano_init(const char *uart_name)
{
    LOG_INF("UART ANO: Initializing driver on %s", uart_name);

    /* Get UART device */
    uart_dev = device_get_binding(uart_name);
    if (!uart_dev) {
        LOG_ERR("UART ANO: Device %s not found", uart_name);
        return -ENODEV;
    }

    /* Initialize mutex for thread safety */
    k_mutex_init(&uart_mutex);

    /* Configure UART for 115200 baud, 8N1 */
    int ret = uart_basic_config(uart_dev, UART_ANO_BAUD_RATE,
                               UART_CFG_DATA_BITS_8 |
                               UART_CFG_PARITY_NONE |
                               UART_CFG_STOP_BITS_1,
                               UART_FLOW_CONTROL_NONE);
    if (ret < 0) {
        LOG_ERR("UART ANO: Basic config failed (%d)", ret);
        return ret;
    }

    LOG_INF("UART ANO: Driver initialized successfully on %s", uart_name);
    return 0;
}

int uart_ano_send_frame(const struct ano_frame *frame)
{
    if (!uart_dev || !frame) {
        LOG_ERR("UART ANO: Invalid parameters (uart_dev=%p, frame=%p)", uart_dev, frame);
        return -EINVAL;
    }

    UART_LOG_DBG("UART ANO: Sending frame type=%d, length=%d", frame->type, frame->length);

    k_mutex_lock(&uart_mutex, K_FOREVER);

    /* Send complete ANO frame using polling API */
    int ret = 0;

    /* Send header (0xAA) */
    ret = uart_poll_out(uart_dev, frame->header);
    if (ret < 0) {
        LOG_ERR("UART ANO: Failed to send header (%d)", ret);
        goto unlock;
    }

    /* Send length (little-endian) */
    ret = uart_poll_out(uart_dev, frame->length & 0xFF);
    if (ret < 0) {
        LOG_ERR("UART ANO: Failed to send length low byte (%d)", ret);
        goto unlock;
    }

    ret = uart_poll_out(uart_dev, (frame->length >> 8) & 0xFF);
    if (ret < 0) {
        LOG_ERR("UART ANO: Failed to send length high byte (%d)", ret);
        goto unlock;
    }

    /* Send type and data */
    for (int i = 0; i < frame->length; i++) {
        ret = uart_poll_out(uart_dev, frame->data[i]);
        if (ret < 0) {
            LOG_ERR("UART ANO: Failed to send data byte %d (%d)", i, ret);
            goto unlock;
        }
    }

    if (ret >= 0) {
        /* Send CRC and tail */
        ret = uart_poll_out(uart_dev, frame->crc & 0xFF);
        if (ret < 0) {
            LOG_ERR("UART ANO: Failed to send CRC low byte (%d)", ret);
            goto unlock;
        }

        ret = uart_poll_out(uart_dev, (frame->crc >> 8) & 0xFF);
        if (ret < 0) {
            LOG_ERR("UART ANO: Failed to send CRC high byte (%d)", ret);
            goto unlock;
        }

        ret = uart_poll_out(uart_dev, frame->tail);
        if (ret < 0) {
            LOG_ERR("UART ANO: Failed to send tail (%d)", ret);
            goto unlock;
        }
    }

    UART_LOG_DBG("UART ANO: Frame sent successfully (type=%d)", frame->type);

unlock:
    k_mutex_unlock(&uart_mutex);

    if (ret < 0 && error_callback) {
        error_callback(TRANSMIT_ERROR, ret);
    }

    return ret;
}

bool uart_ano_is_ready(void)
{
    bool ready = uart_dev != NULL;
    UART_LOG_DBG("UART ANO: Ready check = %d", ready);
    return ready;
}

int uart_ano_set_error_callback(uart_ano_error_callback_t callback)
{
    LOG_INF("UART ANO: Error callback registered");
    error_callback = callback;
    return 0;
}