/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>
#include "protocol_ano.h"

LOG_MODULE_REGISTER(protocol_ano, CONFIG_LOG_DEFAULT_LEVEL);

#ifdef CONFIG_PROTOCOL_ANO_DEBUG
#define PROTO_LOG_DBG(...) LOG_DBG(__VA_ARGS__)
#else
#define PROTO_LOG_DBG(...)
#endif

uint16_t protocol_get_crc(const uint8_t *data, uint16_t len)
{
    uint16_t crc = crc_ccitt(0xFFFF, data, len);
    PROTO_LOG_DBG("Protocol: CRC calculated for %d bytes = 0x%04X", len, crc);
    return crc;
}

int protocol_frame_build(struct ano_frame *frame,
                         ano_cmd_type_t type,
                         const uint8_t *data,
                         uint16_t len)
{
    if (!frame || !data || len > ANO_DATA_MAX) {
        LOG_ERR("Protocol: Invalid parameters for frame build");
        return -EINVAL;
    }

    PROTO_LOG_DBG("Protocol: Building frame type=%d, length=%d", type, len);

    /* Build frame */
    frame->header = ANO_HEADER;
    frame->length = len;  // Little-endian
    frame->type = type;
    memcpy(frame->data, data, len);

    /* Calculate CRC for header+length+type+data */
    uint8_t crc_data[4 + ANO_DATA_MAX];
    crc_data[0] = ANO_HEADER;
    crc_data[1] = len & 0xFF;
    crc_data[2] = (len >> 8) & 0xFF;
    crc_data[3] = type;
    memcpy(&crc_data[4], data, len);

    frame->crc = protocol_get_crc(crc_data, 4 + len);
    frame->tail = ANO_TAIL;

    LOG_INF("Protocol: Frame built - type=%d, crc=0x%04X", type, frame->crc);
    return 0;
}

int protocol_frame_parse(const uint8_t *buf, struct ano_frame *frame, uint16_t len)
{
    if (!buf || !frame || len < sizeof(struct ano_frame)) {
        LOG_ERR("Protocol: Invalid parameters for frame parse");
        return -EINVAL;
    }

    memcpy(frame, buf, sizeof(struct ano_frame));

    PROTO_LOG_DBG("Protocol: Frame parsed - type=%d, length=%d, crc=0x%04X",
                  frame->type, frame->length, frame->crc);
    return 0;
}

bool protocol_frame_valid(const struct ano_frame *frame)
{
    if (!frame || frame->header != ANO_HEADER || frame->tail != ANO_TAIL) {
        LOG_WRN("Protocol: Frame validation failed - header/tail mismatch");
        return false;
    }

    /* Validate length */
    if (frame->length > ANO_DATA_MAX) {
        LOG_WRN("Protocol: Frame validation failed - length %d > max %d",
                frame->length, ANO_DATA_MAX);
        return false;
    }

    /* Recalculate CRC */
    uint8_t crc_data[4 + ANO_DATA_MAX];
    crc_data[0] = ANO_HEADER;
    crc_data[1] = frame->length & 0xFF;
    crc_data[2] = (frame->length >> 8) & 0xFF;
    crc_data[3] = frame->type;
    memcpy(&crc_data[4], frame->data, frame->length);

    uint16_t calculated_crc = protocol_get_crc(crc_data, 4 + frame->length);

    bool valid = calculated_crc == frame->crc;

    if (!valid) {
        LOG_WRN("Protocol: CRC validation failed - expected 0x%04X, got 0x%04X",
                calculated_crc, frame->crc);
    } else {
        LOG_INF("Protocol: Frame validation successful - type=%d", frame->type);
    }

    return valid;
}

void protocol_ano_init(void)
{
    LOG_INF("Protocol: ANO protocol library initialized");
}