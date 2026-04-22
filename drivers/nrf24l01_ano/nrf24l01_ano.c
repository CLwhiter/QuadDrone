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

/* Enable detailed logging based on Kconfig */
#ifdef CONFIG_NRF24L01_DEBUG
#define NRF_LOG_DBG(...) LOG_DBG(__VA_ARGS__)
#define NRF_LOG_HEXDUMP(...) LOG_HEXDUMP_DBG(__VA_ARGS__, __VA_ARGS__##_len, __VA_ARGS__##_name)
#else
#define NRF_LOG_DBG(...)
#define NRF_LOG_HEXDUMP(...)
#endif

static const struct device *spi_dev;
static const struct device *ce_dev;
static const struct device *csn_dev;
static struct k_mutex nrf_mutex;
static nrf24l01_ano_error_callback_t error_callback;

/* Connection status tracking */
static uint32_t last_ack_time = 0;
static uint8_t ack_count = 0;

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

    LOG_DBG("NRF24L01: Writing reg 0x%02X = 0x%02X", reg, value);

    /* CSN low */
    gpio_pin_set_dt(csn_dev, 0);

    /* Write register */
    int ret = spi_transceive(spi_dev, &spi_buf_set);
    if (ret < 0) {
        LOG_ERR("NRF24L01: SPI write failed (%d) for reg 0x%02X", ret, reg);
        return ret;
    }

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);

    LOG_DBG("NRF24L01: Register 0x%02X written successfully", reg);
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

    LOG_DBG("NRF24L01: Reading reg 0x%02X", reg);

    /* CSN low */
    gpio_pin_set_dt(csn_dev, 0);

    /* Read register */
    int ret = spi_transceive(spi_dev, &spi_buf_set);
    if (ret < 0) {
        LOG_ERR("NRF24L01: SPI read failed (%d) for reg 0x%02X", ret, reg);
        return ret;
    }

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);

    *value = rx_buf[0];
    LOG_DBG("NRF24L01: Register 0x%02X = 0x%02X", reg, *value);
    return 0;
}

static void nrf24l01_dump_status(void)
{
    uint8_t status, fifo_status, rf_ch;

    if (nrf24l01_read_reg(NRF_STATUS, &status) == 0) {
        LOG_INF("NRF24L01: STATUS = 0x%02X (TX_DS:%d, MAX_RT:%d, RX_DR:%d)",
                status,
                (status & 0x20) != 0,
                (status & 0x10) != 0,
                (status & 0x40) != 0);
    }

    if (nrf24l01_read_reg(NRF_RF_CH, &rf_ch) == 0) {
        LOG_INF("NRF24L01: RF_CH = %d", rf_ch);
    }

    if (nrf24l01_read_reg(NRF_FIFO_STATUS, &fifo_status) == 0) {
        LOG_INF("NRF24L01: FIFO_STATUS = 0x%02X (TX_EMPTY:%d, RX_EMPTY:%d)",
                fifo_status,
                (fifo_status & 0x20) != 0,
                (fifo_status & 0x01) != 0);
    }
}

static void nrf24l01_setup_registers(void)
{
    uint8_t status;

    LOG_INF("NRF24L01: Setting up registers");

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

    LOG_INF("NRF24L01: Registers configured");
    nrf24l01_dump_status();
}

int nrf24l01_ano_init(const char *spi_dev_name,
                      const char *ce_pin_name,
                      const char *csn_pin_name)
{
    LOG_INF("NRF24L01: Initializing ANO driver");
    LOG_INF("NRF24L01: SPI device = %s, CE = %s, CSN = %s",
            spi_dev_name, ce_pin_name, csn_pin_name);

    spi_dev = device_get_binding(spi_dev_name);
    ce_dev = device_get_binding(ce_pin_name);
    csn_dev = device_get_binding(csn_pin_name);

    if (!spi_dev) {
        LOG_ERR("NRF24L01: SPI device %s not found", spi_dev_name);
        return -ENODEV;
    }

    if (!ce_dev) {
        LOG_ERR("NRF24L01: CE device %s not found", ce_pin_name);
        return -ENODEV;
    }

    if (!csn_dev) {
        LOG_ERR("NRF24L01: CSN device %s not found", csn_pin_name);
        return -ENODEV;
    }

    LOG_INF("NRF24L01: All devices bound successfully");

    /* Configure GPIO pins */
    int ret = gpio_pin_configure(ce_dev, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("NRF24L01: Failed to configure CE pin (%d)", ret);
        return ret;
    }

    ret = gpio_pin_configure(csn_dev, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("NRF24L01: Failed to configure CSN pin (%d)", ret);
        return ret;
    }

    /* Initial state: CE low, CSN high */
    gpio_pin_set_dt(ce_dev, 0);
    gpio_pin_set_dt(csn_dev, 1);

    LOG_INF("NRF24L01: GPIO pins configured (CE=low, CSN=high)");

    k_mutex_init(&nrf_mutex);

    /* Initialize NRF24L01 registers (ported from ANO code) */
    nrf24l01_setup_registers();

    LOG_INF("NRF24L01: ANO driver initialized successfully");
    return 0;
}

int nrf24l01_ano_send_frame(const struct ano_frame *frame)
{
    if (!spi_dev || !frame) {
        LOG_ERR("NRF24L01: Invalid parameters (spi_dev=%p, frame=%p)", spi_dev, frame);
        return -EINVAL;
    }

    LOG_DBG("NRF24L01: Sending frame type=%d, length=%d", frame->type, frame->length);

    k_mutex_lock(&nrf_mutex, K_FOREVER);

    /* Load TX payload */
    uint8_t payload[NRF24L01_PAYLOAD_SIZE];
    memset(payload, 0, sizeof(payload));

    /* Copy ANO frame header, length, type, data, and CRC (excluding tail) */
    uint8_t frame_size = frame->length + 5; // header(1) + length(2) + type(1) + crc(2)
    if (frame_size > NRF24L01_PAYLOAD_SIZE) {
        frame_size = NRF24L01_PAYLOAD_SIZE;
    }
    memcpy(payload, frame, frame_size);

    LOG_DBG("NRF24L01: Payload (%d bytes):", frame_size);
    for (int i = 0; i < frame_size && i < 16; i++) {
        LOG_DBG("  [%02d] 0x%02X", i, payload[i]);
    }

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
    LOG_DBG("NRF24L01: CSN low");

    /* Transmit packet */
    int ret = spi_transceive(spi_dev, &spi_buf_set);

    /* CSN high */
    gpio_pin_set_dt(csn_dev, 1);
    LOG_DBG("NRF24L01: CSN high, SPI result=%d", ret);

    /* Pulse CE to send packet (Enhanced ShockBurst) */
    LOG_DBG("NRF24L01: Pulse CE (10us)");
    gpio_pin_set_dt(ce_dev, 1);
    k_sleep(K_USEC(10));
    gpio_pin_set_dt(ce_dev, 0);
    LOG_DBG("NRF24L01: CE low");

    /* Track ACK if transmission was successful */
    if (ret == 0) {
        ack_count++;
        last_ack_time = k_uptime_get_32();
    }

    k_mutex_unlock(&nrf_mutex);

    if (ret < 0) {
        LOG_ERR("NRF24L01: Send failed (%d)", ret);
        nrf24l01_dump_status();  // Log status on failure
        if (error_callback) {
            error_callback(SPI_ERROR, ret);
        }
    } else {
        LOG_INF("NRF24L01: Frame sent successfully (type=%d)", frame->type);
        k_sleep(K_MSEC(10));  // Brief delay between packets
    }

    return ret;
}

bool nrf24l01_ano_is_ready(void)
{
    uint8_t status;
    int ret = nrf24l01_read_reg(NRF_STATUS, &status);
    bool ready = (ret == 0 && (status & 0x20) == 0); // Check TX_DS (data sent)

    LOG_DBG("NRF24L01: Ready check - status=0x%02X, ready=%d", status, ready);
    return ready;
}

bool nrf24l01_is_connected(void)
{
    // Consider connected if ACK received within last 1 second
    uint32_t now = k_uptime_get_32();
    return (now - last_ack_time < 1000U) && (ack_count > 0);
}

int nrf24l01_ano_set_error_callback(nrf24l01_ano_error_callback_t callback)
{
    LOG_INF("NRF24L01: Error callback registered");
    error_callback = callback;
    return 0;
}