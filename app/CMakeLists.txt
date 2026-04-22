#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

# Add QuadDrone ANO Remote Controller board support
set(BOARD_DIR ${APPLICATION_SOURCE}/boards)

# Zephyr board discovery
include_guard(GLOBAL)
if(NOT ZephyrBoard_FOUND)
  if(EXISTS ${BOARD_DIR}/vendor/${BOARD}/board.yml)
    # Board defined in QuadDrone/boards/vendor/
    set(ZephyrBoard_ROOT ${BOARD_DIR})
  elseif(EXISTS ${ZEPHYR_BASE}/boards/${BOARD}/board.yml)
    # Standard Zephyr board
    set(ZephyrBoard_ROOT ${ZEPHYR_BASE}/boards)
  endif()
  find_package(ZephyrBoard QUIET)
endif()

target_sources(app PRIVATE src/main.c)

# Include sensor drivers
zephyr_include_directories(../drivers/adc_multi)
zephyr_include_directories(../drivers/display/ssd1306_spi)

# Include ANO communication drivers
zephyr_include_directories(../drivers/uart_ano)
zephyr_include_directories(../drivers/nrf24l01_ano)
zephyr_include_directories(../drivers/protocol_ano)

# Link sensor drivers
target_sources(app PRIVATE ../drivers/adc_multi/adc_multi.c)
target_sources(app PRIVATE ../drivers/display/ssd1306_spi/ssd1306_spi.c)

# Link ANO communication drivers
target_sources(app PRIVATE ../drivers/uart_ano/uart_ano.c)
target_sources(app PRIVATE ../drivers/nrf24l01_ano/nrf24l01_ano.c)
target_sources(app PRIVATE ../drivers/protocol_ano/protocol_ano.c)

# Add OLED display driver
zephyr_library_named(ssd1306_spi)
target_sources(ssd1306_spi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/display/ssd1306_spi/ssd1306_spi.c
)

# The code below locates the git index file for this repository and adds it as a dependency for
# the application VERSION file so that if the repo has a new commit added, even if no files in
# the build have changed, the application version file will be regenerated with the new git commit
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --absolute-git-dir
    WORKING_DIRECTORY .
    OUTPUT_VARIABLE application_git_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
    ERROR_VARIABLE stderr
    RESULT_VARIABLE return_code
  )
  # If there is an error e.g. it is not a git repo, we just silently ignore it and continue
  # without a dependency, this will be the case with freestanding applications.
  if(NOT return_code)
    if(NOT "${stderr}" STREQUAL "")
      message(WARNING "Application build version git rev-parse warned: ${stderr}")
    endif()

    set_property(TARGET app_version_h PROPERTY APP_VERSION_DEPENDS ${application_git_dir}/index)
  endif()
endif()
