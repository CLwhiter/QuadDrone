# Copyright (c) 2024 QuadDrone Project. SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_SSD1306_SPI ssd1306_spi.c)