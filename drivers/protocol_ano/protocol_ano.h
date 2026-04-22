/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * ANO Protocol Library for QuadDrone Remote Controller
 * Implements ANO frame format with CRC checksum for error detection
 */

#ifndef APP_DRIVERS_PROTOCOL_ANO_H_
#define APP_DRIVERS_PROTOCOL_ANO_H_

#include <stdint.h>
#include <stdbool.h>

/* ANO Protocol constants */
#define ANO_HEADER        0xAA
#define ANO_TAIL         0x55
#define ANO_FRAME_MAX    64
#define ANO_DATA_MAX     32

/* ANO Command Types */
typedef enum {
    ANO_CMD_RC_DATA    = 0x01,  /* RC channel data */
    ANO_CMD_HEARTBEAT  = 0x02,  /* Heartbeat packet */
    ANO_CMD_TELEMETRY  = 0x03,  /* Telemetry data */
} ano_cmd_type_t;

/* ANO Frame Structure */
struct ano_frame {
    uint8_t header;        // 0xAA
    uint16_t length;      // Little-endian length
    uint8_t type;         // Command type
    uint8_t data[ANO_DATA_MAX];  // Payload
    uint16_t crc;         // CRC-16 CCITT checksum
    uint8_t tail;         // 0x55
};

/**
 * @brief Initialize ANO protocol library
 */
void protocol_ano_init(void);

/**
 * @brief Build ANO frame from data
 *
 * @param frame Frame structure to fill
 * @param type Command type
 * @param data Payload data
 * @param len Length of payload data
 * @return int 0 on success, negative error code on failure
 */
int protocol_frame_build(struct ano_frame *frame,
                         ano_cmd_type_t type,
                         const uint8_t *data,
                         uint16_t len);

/**
 * @brief Parse ANO frame from buffer
 *
 * @param buf Input buffer containing frame data
 * @param frame Frame structure to fill
 * @param len Length of buffer data
 * @return int 0 on success, negative error code on failure
 */
int protocol_frame_parse(const uint8_t *buf, struct ano_frame *frame, uint16_t len);

/**
 * @brief Validate ANO frame checksum
 *
 * @param frame Frame to validate
 * @return bool True if checksum is valid
 */
bool protocol_frame_valid(const struct ano_frame *frame);

/**
 * @brief Get CRC-16 CCITT checksum
 *
 * @param data Data buffer
 * @param len Length of data
 * @return uint16_t CRC checksum
 */
uint16_t protocol_get_crc(const uint8_t *data, uint16_t len);

#endif /* APP_DRIVERS_PROTOCOL_ANO_H_ */