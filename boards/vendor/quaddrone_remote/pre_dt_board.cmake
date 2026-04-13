# Copyright (c) 2024 QuadDrone Project
# SPDX-License-Identifier: Apache-2.0

# Configure DT_CHOSEN_ZEPHYR_CONSOLE_LABEL for the board
set(DT_CHOSEN_ZEPHYR_CONSOLE_LABEL "uart1")

# Add board-specific preprocessor definitions
add_compile_definitions(
  CONFIG_SOC_STM32F103C8
  CONFIG_BOARD_VENDOR_QUADDRONE_REMOTE
)