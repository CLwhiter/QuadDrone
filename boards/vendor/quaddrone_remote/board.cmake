# Copyright (c) 2024 QuadDrone Project
# SPDX-License-Identifier: Apache-2.0

# Board-specific settings
set(BOARD_HAS_CUSTOMIZED 1)
set(BOARD_HAS_CUSTOMIZED_CONSOLE 1)
set(BOARD_HAS_CUSTOMIZED_LED 1)

# Enable flash for STM32F103C8T6
set(BOARD_HAS_FLASH ${CONFIG_HAS_FLASH})

# Enable Zephyr console support
if(CONFIG_UART_CONSOLE)
  set(BOARD_CONSOLE uart1)
endif()

# Enable GPIO support
if(CONFIG_GPIO)
  # LED configuration
  set(BOARD_LED0_PCIE 13)
  set(BOARD_LED0_PORT gpioa)
  set(BOARD_LED0_GPIO_ACTIVE_LOW 1)
endif()

# Board runner args for STM32
board_runner_args(jlink "--device=STM32F103C8" "--speed=4000")
board_runner_args(stlink "--flash=flash")

set(OPENOCD_TARGET "stm32f1x")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)