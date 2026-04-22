/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <zephyr/sys/crc.h>
#include "protocol_ano.h"

LOG_MODULE_REGISTER(protocol_ano, CONFIG_LOG_DEFAULT_LEVEL);

uint16_t protocol_get_crc(const uint8_t *data, uint16_t len)
{
    return crc_ccitt(0xFFFF, data, len);
}

int protocol_frame_build(struct ano_frame *frame,
                         ano_cmd_type_t type,
                         const uint8_t *data,
                         uint16_t len)
{
    if (!frame || !data || len > ANO_DATA_MAX) {
        return -EINVAL;
    }

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

    return 0;
}

int protocol_frame_parse(const uint8_t *buf, struct ano_frame *frame, uint16_t len)
{
    if (!buf || !frame || len < sizeof(struct ano_frame)) {
        return -EINVAL;
    }

    memcpy(frame, buf, sizeof(struct ano_frame));

    return 0;
}

bool protocol_frame_valid(const struct ano_frame *frame)
{
    if (!frame || frame->header != ANO_HEADER || frame->tail != ANO_TAIL) {
        return false;
    }

    /* Validate length */
    if (frame->length > ANO_DATA_MAX) {
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

    return calculated_crc == frame->crc;
}

void protocol_ano_init(void)
{
    /* Protocol library initialization if needed */
}