/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * NRF24L01 ANO protocol driver for QuadDrone Remote Controller
 * Implements wireless data transmission using Zephyr SPI API
 */

#ifndef APP_DRIVERS_NRF24L01_ANO_H_
#define APP_DRIVERS_NRF24L01_ANO_H_

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "../protocol_ano/protocol_ano.h"

/* NRF24L01 constants */
#define NRF24L01_TX_PIPE          0
#define NRF24L01_PAYLOAD_SIZE      32
#define NRF24L01_CHANNEL           76
#define NRF24L01_RF_POWER          0
#define NRF24L01_DATA_RATE         1  // 1Mbps

/* NRF24L01 Register Addresses */
#define NRF_CONFIG        0x00
#define NRF_EN_AA        0x01
#define NRF_SETUP_AW     0x03
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_TX_ADDR      0x10
#define NRF_RX_ADDR_P0   0x0A
#define NRF_RX_PW_P0    0x11
#define NRF_FIFO_STATUS  0x17

/* NRF24L01 Commands */
#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_R_TX_PAYLOAD  0x60
#define NRF_CMD_W_TX_PAYLOAD  0xA0
#define NRF_CMD_FLUSH_TX      0xE1
#define NRF_CMD_FLUSH_RX      0xE2
#define NRF_CMD_REUSE_TX_PL   0xE3
#define NRF_CMD_NOP          0xFF

/**
 * @brief Initialize NRF24L01 ANO driver
 *
 * @param spi_dev_name SPI device name from device tree
 * @param ce_pin_name CE pin device name
 * @param csn_pin_name CSN pin device name
 * @return int 0 on success, negative error code on failure
 */
int nrf24l01_ano_init(const char *spi_dev_name,
                      const char *ce_pin_name,
                      const char *csn_pin_name);

/**
 * @brief Send ANO frame via NRF24L01
 *
 * @param frame Complete ANO frame structure
 * @return int 0 on success, negative error code on failure
 */
int nrf24l01_ano_send_frame(const struct ano_frame *frame);

/**
 * @brief Check NRF24L01 transmitter status
 *
 * @return bool True if transmitter is ready
 */
bool nrf24l01_ano_is_ready(void);

/* Error types */
#define NRF24L01_ERROR_SPI    1
#define NRF24L01_ERROR_TIMEOUT 2

/**
 * @brief NRF24L01 error callback
 *
 * @param error_type Type of error (SPI, timeout, etc.)
 * @param error_code Error code from Zephyr API
 */
typedef void (*nrf24l01_ano_error_callback_t)(int error_type, int error_code);

int nrf24l01_ano_set_error_callback(nrf24l01_ano_error_callback_t callback);

#endif /* APP_DRIVERS_NRF24L01_ANO_H_ */