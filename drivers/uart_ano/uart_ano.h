/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * UART ANO protocol driver for QuadDrone Remote Controller
 * Implements ANO frame transmission over UART with CRC checksum
 */

#ifndef APP_DRIVERS_UART_ANO_H_
#define APP_DRIVERS_UART_ANO_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include "../protocol_ano/protocol_ano.h"

/* UART ANO protocol constants */
#define UART_ANO_FRAME_MAX_SIZE    64
#define UART_ANO_BUFFER_SIZE       256
#define UART_ANO_BAUD_RATE         115200

/**
 * @brief Initialize UART ANO driver
 *
 * @param uart_name UART device name from device tree
 * @return int 0 on success, negative error code on failure
 */
int uart_ano_init(const char *uart_name);

/**
 * @brief Send ANO frame via UART
 *
 * @param frame Complete ANO frame structure
 * @return int 0 on success, negative error code on failure
 */
int uart_ano_send_frame(const struct ano_frame *frame);

/**
 * @brief Get UART device status
 *
 * @return bool True if UART is ready
 */
bool uart_ano_is_ready(void);

/**
 * @brief UART error callback
 *
 * @param error_type Type of error (transmission, timeout, etc.)
 * @param error_code Error code from Zephyr API
 */
typedef void (*uart_ano_error_callback_t)(int error_type, int error_code);

int uart_ano_set_error_callback(uart_ano_error_callback_t callback);

#endif /* APP_DRIVERS_UART_ANO_H_ */