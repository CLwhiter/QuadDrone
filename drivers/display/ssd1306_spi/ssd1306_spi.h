/*
 * Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0
 *
 * SSD1306 OLED display driver for QuadDrone Remote Controller
 * Implements 128x64 display with SPI bitbang interface
 */

#ifndef APP_DRIVERS_SSD1306_SPI_H_
#define APP_DRIVERS_SSD1306_SPI_H_

#include <stdint.h>
#include <zephyr/device.h>

/* Display dimensions */
#define SSD1306_WIDTH         128
#define SSD1306_HEIGHT        64
#define SSD1306_BUFFER_SIZE   (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

/* Commands */
#define SSD1306_COMMAND       0x00
#define SSD1306_DATA          0x40

/* Display control commands */
#define SSD1306_DISPLAYOFF    0xAE
#define SSD1306_DISPLAYON     0xAF
#define SSD1306_SETNORMAL     0xA6
#define SSD1306_SETINVERSE    0xA7

/* Addressing mode commands */
#define SSD1306_SETCOLUMNADDR 0x21
#define SSD1306_SETROWADDR    0x22

/* Display configuration commands */
#define SSD1306_SETDISPLAYCLOCKDIV  0xD5
#define SSD1306_SETMULTIPLEX        0xA8
#define SSD1306_SETDISPLAYOFFSET    0xD3
#define SSD1306_SETSTARTLINE        0x40
#define SSD1306_CHARGEPUMP         0x8D
#define SSD1306_MEMORYMODE         0x20
#define SSD1306_COMSCANDEC         0xC8
#define SSD1306_SEGREMAP           0xA1
#define SSD1306_COMPINCFG          0xDA
#define SSD1306_CONTRAST           0x81

/* Font definitions */
#define FONT_6X8_WIDTH        6
#define FONT_6X8_HEIGHT       8

#define FONT_8X16_WIDTH       8
#define FONT_8X16_HEIGHT      16

/**
 * @brief Initialize SSD1306 OLED display
 *
 * @param dev SPI device
 * @return int 0 on success, negative error code on failure
 */
int ssd1306_spi_init(const struct device *dev);

/**
 * @brief Clear the OLED display
 *
 * @return int 0 on success, negative error code on failure
 */
int ssd1306_spi_clear(void);

/**
 * @brief Write string to display at specified position
 *
 * @param x X position (0-127)
 * @param y Y position (0-7 for 6x8 font, 0-3 for 8x16 font)
 * @param str String to display
 * @param size Font size (6 or 8)
 * @return int 0 on success, negative error code on failure
 */
int ssd1306_spi_putstr(int x, int y, const char *str, int size);

/**
 * @brief Set display contrast
 *
 * @param contrast Contrast value (0-255)
 * @return int 0 on success, negative error code on failure
 */
int ssd1306_spi_set_contrast(uint8_t contrast);

#endif /* APP_DRIVERS_SSD1306_SPI_H_ */