#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Add custom devicetree bindings
list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tempo-bt-v1)


# Generate version header
set(VERSION_HEADER ${CMAKE_CURRENT_BINARY_DIR}/include/generated/app_version.h)
add_custom_command(
    OUTPUT ${VERSION_HEADER}
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_version.py ${VERSION_HEADER}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/app.version
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating version header"
)
add_custom_target(version_header DEPENDS ${VERSION_HEADER})
add_dependencies(app version_header)

# NORDIC SDK APP START
# Add generated include directory
target_include_directories(app PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/include)

# Main application sources
target_sources(app PRIVATE 
    src/main.c
    src/app_init.c
    src/events.c
    src/mcumgr_custom.c
    src/ble_mcumgr.c
)

# Configuration sources
target_sources(app PRIVATE
    src/config/settings.c
)

# Service layer sources
target_sources(app PRIVATE
    src/services/timebase.c
    src/services/gnss_m10q.c
    src/services/imu_icm42688.c
    src/services/orientation.c
    src/services/baro_bmp390.c
    src/services/aggregator.c
    src/services/storage.c
    src/services/storage_fatfs.c
    src/services/storage_littlefs.c
    src/services/logger.c
    src/services/file_writer.c
    src/services/led.c
)

# Magnetometer driver and service (optional)
target_sources_ifdef(CONFIG_MMC5983MA app PRIVATE
    drivers/sensor/mmc5983ma/mmc5983ma.c
    drivers/sensor/mmc5983ma/mmc5983ma_i2c.c
    src/services/mag_mmc5983ma.c
)
target_include_directories_ifdef(CONFIG_MMC5983MA app PRIVATE
    drivers/sensor/mmc5983ma
)

# USB CDC-ACM TTY service (optional)
target_sources_ifdef(CONFIG_USB_TTY_OUTPUT app PRIVATE
    src/services/usb_tty.c
)

# Utility sources
target_sources(app PRIVATE
    src/util/nmea_checksum.c
    tests/test_imu.c
)

# Fusion AHRS library
target_sources(app PRIVATE
    Fusion/FusionAhrs.c
    Fusion/FusionBias.c
    Fusion/FusionCompass.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
target_include_directories(app PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/Fusion
)
