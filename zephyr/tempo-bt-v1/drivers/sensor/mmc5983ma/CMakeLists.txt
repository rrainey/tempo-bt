# Copyright (c) 2024
# SPDX-License-Identifier: Apache-2.0

zephyr_library_named(mmc5983ma)

zephyr_library_sources(
    mmc5983ma.c
    mmc5983ma_i2c.c
)

zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR})
