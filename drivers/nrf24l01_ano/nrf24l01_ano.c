/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "nrf24l01_ano.h"
#include "../protocol_ano/protocol_ano.h"

LOG_MODULE_REGISTER(nrf24l01_ano, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *spi_dev;
static const struct device *ce_dev;
static const struct device *csn_dev;
static struct k_mutex nrf_mutex;
static nrf24l01_ano_error_callback_t error_callback;

/* NRF24L01 Register Configuration Values (ported from ANO) */
static const uint8_t nrf_config_value = 0x0E;       // PTX, CRC 2 bytes, TX pipe 0
static const uint8_t nrf_en_aa_value = 0x01;         // Enable Auto ACK on pipe 0
static const uint8_t nrf_setup_aw_value = 0x03;      // 5-byte address width
static const uint8_t nrf_rf_ch_value = NRF24L01_CHANNEL; // Channel 76
static const uint8_t nrf_rf_setup_value = 0x06;     // 1Mbps, 0dBm
static const uint8_t nrf_tx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // TX address
static const uint8_t nrf_rx_addr_p0[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // RX address for pipe 0
static const uint8_t nrf_rx_pw_p0 = NRF24L01_PAYLOAD_SIZE; // Payload size

static int nrf24l01_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg | NRF_CMD_W_REGISTER, value};
    struct spi_buf spi_buf = {
        .buf = buf,
        .len = sizeof(buf)
    };
    struct spi_buf_set spi_buf_set = {
        .buffers = &spi_buf,
        .count = 1
    };

    /* CSN low */
    gpio_pin_set_dt(csn_dev, 0);

    /* Write register */
    int ret = spi_transceive(spi_dev, &spi_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI write failed (%d)", ret);
        return ret;
    }

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);

    return 0;
}

static int nrf24l01_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t cmd_buf[1] = {reg | NRF_CMD_R_REGISTER};
    uint8_t rx_buf[1] = {0};
    struct spi_buf cmd_buf_struct = {
        .buf = cmd_buf,
        .len = sizeof(cmd_buf)
    };
    struct spi_buf rx_buf_struct = {
        .buf = rx_buf,
        .len = sizeof(rx_buf)
    };
    struct spi_buf_set spi_buf_set = {
        .buffers = (struct spi_buf[]){cmd_buf_struct, rx_buf_struct},
        .count = 2
    };

    /* CSN low */
    gpio_pin_set_dt(csn_dev, 0);

    /* Read register */
    int ret = spi_transceive(spi_dev, &spi_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI read failed (%d)", ret);
        return ret;
    }

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);

    *value = rx_buf[0];
    return 0;
}

static void nrf24l01_setup_registers(void)
{
    uint8_t status;

    /* Soft reset */
    nrf24l01_write_reg(NRF_CONFIG, 0);
    k_sleep(K_MSEC(1));

    /* Configure registers (ported from ANO_Drv_Nrf24l01.cpp) */
    nrf24l01_write_reg(NRF_CONFIG, nrf_config_value);
    nrf24l01_write_reg(NRF_EN_AA, nrf_en_aa_value);
    nrf24l01_write_reg(NRF_SETUP_AW, nrf_setup_aw_value);
    nrf24l01_write_reg(NRF_RF_CH, nrf_rf_ch_value);
    nrf24l01_write_reg(NRF_RF_SETUP, nrf_rf_setup_value);
    nrf24l01_write_reg(NRF_RX_PW_P0, nrf_rx_pw_p0);

    /* Set TX address */
    nrf24l01_write_reg(NRF_TX_ADDR, nrf_tx_addr[0]);
    nrf24l01_write_reg(NRF_TX_ADDR + 1, nrf_tx_addr[1]);
    nrf24l01_write_reg(NRF_TX_ADDR + 2, nrf_tx_addr[2]);
    nrf24l01_write_reg(NRF_TX_ADDR + 3, nrf_tx_addr[3]);
    nrf24l01_write_reg(NRF_TX_ADDR + 4, nrf_tx_addr[4]);

    /* Set RX address for pipe 0 */
    nrf24l01_write_reg(NRF_RX_ADDR_P0, nrf_rx_addr_p0[0]);
    nrf24l01_write_reg(NRF_RX_ADDR_P0 + 1, nrf_rx_addr_p0[1]);
    nrf24l01_write_reg(NRF_RX_ADDR_P0 + 2, nrf_rx_addr_p0[2]);
    nrf24l01_write_reg(NRF_RX_ADDR_P0 + 3, nrf_rx_addr_p0[3]);
    nrf24l01_write_reg(NRF_RX_ADDR_P0 + 4, nrf_rx_addr_p0[4]);

    /* Clear status register */
    nrf24l01_read_reg(NRF_STATUS, &status);

    LOG_INF("NRF24L01 registers configured");
}

int nrf24l01_ano_init(const char *spi_dev_name,
                      const char *ce_pin_name,
                      const char *csn_pin_name)
{
    spi_dev = device_get_binding(spi_dev_name);
    ce_dev = device_get_binding(ce_pin_name);
    csn_dev = device_get_binding(csn_pin_name);

    if (!spi_dev || !ce_dev || !csn_dev) {
        LOG_ERR("NRF24L01 devices not ready");
        return -ENODEV;
    }

    /* Configure GPIO pins */
    int ret = gpio_pin_configure_dt(GPIO_DT_SPEC_GET(DT_NODELABEL(nrf24_ce), gpios), GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CE pin (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(GPIO_DT_SPEC_GET(DT_NODELABEL(nrf24_csn), gpios), GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure CSN pin (%d)", ret);
        return ret;
    }

    /* Initial state: CE low, CSN high */
    gpio_pin_set_dt(GPIO_DT_SPEC_GET(DT_NODELABEL(nrf24_ce), gpios), 0);
    gpio_pin_set_dt(GPIO_DT_SPEC_GET(DT_NODELABEL(nrf24_csn), gpios), 1);

    k_mutex_init(&nrf_mutex);

    /* Initialize NRF24L01 registers (ported from ANO code) */
    nrf24l01_setup_registers();

    LOG_INF("NRF24L01 ANO driver initialized");
    return 0;
}

int nrf24l01_ano_send_frame(const struct ano_frame *frame)
{
    if (!spi_dev || !frame) {
        return -EINVAL;
    }

    k_mutex_lock(&nrf_mutex, K_FOREVER);

    /* Load TX payload */
    uint8_t payload[NRF24L01_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));

    /* Copy ANO frame header, length, type, data, and CRC (excluding tail) */
    uint8_t frame_size = frame->length + 5; // header(1) + length(2) + type(1) + crc(2)
    memcpy(payload, frame, frame_size > NRF24L01_PAYLOAD_SIZE ? NRF24L01_PAYLOAD_SIZE : frame_size);

    /* Write to TX FIFO */
    uint8_t cmd = NRF_CMD_W_TX_PAYLOAD;
    struct spi_buf cmd_buf = {
        .buf = &cmd,
        .len = 1
    };
    struct spi_buf payload_buf = {
        .buf = payload,
        .len = sizeof(payload)
    };
    struct spi_buf_set spi_buf_set = {
        .buffers = (struct spi_buf[]){cmd_buf, payload_buf},
        .count = 2
    };

    /* CSN low */
    gpio_pin_set_dt(csn_dev, 0);

    /* Transmit packet */
    int ret = spi_transceive(spi_dev, &spi_buf_set);

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);

    /* Pulse CE to send packet (Enhanced ShockBurst) */
    gpio_pin_set_dt(ce_dev, 1);
    k_sleep(K_USEC(10));
    gpio_pin_set_dt(ce_dev, 0);

    k_mutex_unlock(&nrf_mutex);

    if (ret < 0) {
        LOG_ERR("NRF24L01 send failed (%d)", ret);
        if (error_callback) {
            error_callback(NRF24L01_ERROR_SPI, ret);
        }
    }

    return ret;
}

bool nrf24l01_ano_is_ready(void)
{
    uint8_t status;
    int ret = nrf24l01_read_reg(NRF_STATUS, &status);
    return ret == 0 && (status & 0x20) == 0; // Check TX_DS (data sent)
}

int nrf24l01_ano_set_error_callback(nrf24l01_ano_error_callback_t callback)
{
    error_callback = callback;
    return 0;
}