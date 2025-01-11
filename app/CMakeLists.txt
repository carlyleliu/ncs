#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

#set(BOARD nrf5x/nrf52840)

#list(APPEND ZEPHYR_EXTRA_MODULES
#    ${CMAKE_CURRENT_SOURCE_DIR}/drivers
#)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

project(app)

add_subdirectory(peripherals)
add_subdirectory(ble)

file(GLOB application_sources src/*.c)
target_sources(app PRIVATE ${application_sources})
