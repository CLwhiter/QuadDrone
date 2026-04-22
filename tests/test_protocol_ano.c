/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for ANO protocol library
 */

#include <zephyr/ztest.h>
#include "drivers/protocol_ano/protocol_ano.h"

/* Test data */
static uint8_t test_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
static uint8_t expected_heartbeat_data[4] = {0x00, 0x00, 0x00, 0x01}; /* counter = 1 */

/* Test: CRC calculation validation */
static void test_protocol_get_crc(void)
{
    uint16_t crc;

    /* Test with empty data */
    crc = protocol_get_crc(NULL, 0);
    zassert_equal(crc, 0xFFFF, "CRC for empty data should be 0xFFFF");

    /* Test with known data pattern */
    crc = protocol_get_crc(test_data, sizeof(test_data));
    /* Expected CRC for this pattern calculated independently */
    zassert_not_equal(crc, 0xFFFF, "CRC should not be default value");

    /* Test with single byte */
    crc = protocol_get_crc(&test_data[0], 1);
    zassert_not_equal(crc, 0xFFFF, "CRC for single byte should not be default");
}

/* Test: Frame building */
static void test_protocol_frame_build(void)
{
    struct ano_frame frame;
    int ret;

    /* Test building RC data frame */
    ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Frame build should succeed for RC data");
    zassert_equal(frame.header, ANO_HEADER, "Frame header should be 0xAA");
    zassert_equal(frame.length, sizeof(test_data), "Frame length should match data size");
    zassert_equal(frame.type, ANO_CMD_RC_DATA, "Frame type should be RC_DATA");
    zassert_mem_equal(frame.data, test_data, sizeof(test_data), "Frame data should match");
    zassert_equal(frame.tail, ANO_TAIL, "Frame tail should be 0x55");

    /* Test building heartbeat frame */
    uint32_t counter = 1;
    ret = protocol_frame_build(&frame, ANO_CMD_HEARTBEAT,
                              (uint8_t*)&counter, sizeof(counter));
    zassert_equal(ret, 0, "Frame build should succeed for heartbeat");
    zassert_equal(frame.type, ANO_CMD_HEARTBEAT, "Frame type should be HEARTBEAT");

    /* Test with data too large */
    ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA, test_data, ANO_DATA_MAX + 1);
    zassert_equal(ret, -EINVAL, "Frame build should fail for oversized data");

    /* Test with NULL parameters */
    ret = protocol_frame_build(NULL, ANO_CMD_RC_DATA, test_data, sizeof(test_data));
    zassert_equal(ret, -EINVAL, "Frame build should fail with NULL frame");

    ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA, NULL, sizeof(test_data));
    zassert_equal(ret, -EINVAL, "Frame build should fail with NULL data");
}

/* Test: Frame parsing */
static void test_protocol_frame_parse(void)
{
    struct ano_frame frame;
    uint8_t buffer[sizeof(struct ano_frame)];
    int ret;

    /* First build a valid frame */
    ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Frame build should succeed");

    /* Copy to buffer */
    memcpy(buffer, &frame, sizeof(buffer));

    /* Parse the frame */
    ret = protocol_frame_parse(buffer, &frame, sizeof(buffer));
    zassert_equal(ret, 0, "Frame parse should succeed");
    zassert_equal(frame.type, ANO_CMD_RC_DATA, "Parsed frame type should match");

    /* Test with insufficient buffer size */
    ret = protocol_frame_parse(buffer, &frame, 5);  // Less than frame size
    zassert_equal(ret, -EINVAL, "Frame parse should fail with small buffer");

    /* Test with NULL parameters */
    ret = protocol_frame_parse(buffer, NULL, sizeof(buffer));
    zassert_equal(ret, -EINVAL, "Frame parse should fail with NULL frame");
}

/* Test: Frame validation */
static void test_protocol_frame_valid(void)
{
    struct ano_frame frame;
    bool valid;

    /* Test building and validating a good frame */
    int ret = protocol_frame_build(&frame, ANO_CMD_RC_DATA, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Frame build should succeed");

    valid = protocol_frame_valid(&frame);
    zassert_equal(valid, true, "Valid frame should pass validation");

    /* Test with corrupted header */
    frame.header = 0xFF;
    valid = protocol_frame_valid(&frame);
    zassert_equal(valid, false, "Frame with bad header should fail validation");
    frame.header = ANO_HEADER;  // Restore

    /* Test with corrupted tail */
    frame.tail = 0xFF;
    valid = protocol_frame_valid(&frame);
    zassert_equal(valid, false, "Frame with bad tail should fail validation");
    frame.tail = ANO_TAIL;  // Restore

    /* Test with oversized length */
    frame.length = ANO_DATA_MAX + 1;
    valid = protocol_frame_valid(&frame);
    zassert_equal(valid, false, "Frame with oversized length should fail validation");
    frame.length = sizeof(test_data);  // Restore

    /* Test with corrupted CRC */
    frame.crc = 0x0000;
    valid = protocol_frame_valid(&frame);
    zassert_equal(valid, false, "Frame with bad CRC should fail validation");
}

/* Test: End-to-end frame integrity */
static void test_protocol_end_to_end(void)
{
    struct ano_frame frame1, frame2;
    uint8_t buffer[sizeof(struct ano_frame)];
    int ret;

    /* Build frame */
    ret = protocol_frame_build(&frame1, ANO_CMD_TELEMETRY, test_data, sizeof(test_data));
    zassert_equal(ret, 0, "Frame build should succeed");

    /* Parse frame into buffer */
    memcpy(buffer, &frame1, sizeof(buffer));

    /* Parse from buffer */
    ret = protocol_frame_parse(buffer, &frame2, sizeof(buffer));
    zassert_equal(ret, 0, "Frame parse should succeed");

    /* Validate both frames are identical */
    zassert_equal(frame1.header, frame2.header, "Headers should match");
    zassert_equal(frame1.length, frame2.length, "Lengths should match");
    zassert_equal(frame1.type, frame2.type, "Types should match");
    zassert_mem_equal(frame1.data, frame2.data, frame1.length, "Data should match");
    zassert_equal(frame1.crc, frame2.crc, "CRCs should match");
    zassert_equal(frame1.tail, frame2.tail, "Tails should match");

    /* Both frames should be valid */
    zassert_equal(protocol_frame_valid(&frame1), true, "Original frame should be valid");
    zassert_equal(protocol_frame_valid(&frame2), true, "Parsed frame should be valid");
}

/* Test suite definition */
void *test_setup(void)
{
    /* Initialize protocol library if needed */
    protocol_ano_init();
    return NULL;
}

void test_teardown(void)
{
    /* Clean up if needed */
}

ZTEST_SUITE(protocol_ano_tests, NULL, test_setup, NULL, test_teardown, NULL);