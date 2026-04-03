# Tempo-BT - A Skydiving Data Logger

A Zephyr RTOS-based flight data logging system for the nRF5340 platform, designed for capturing high-frequency sensor data during skydiving and other aerial activities. The system logs IMU, barometric, GPS, and optional magnetometer data to SD card or internal flash storage, with Bluetooth LE connectivity for wireless data transfer and configuration.

![Tempo-BT-V2](../../images/V2-render-02.png
)

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Software Prerequisites](#software-prerequisites)
- [Build Instructions](#build-instructions)
- [Configuration](#configuration)
- [Usage Examples](#usage-examples)
- [Data Format](#data-format)
- [LED Status Indicators](#led-status-indicators)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Support](#support)

## Features

- **Runs on the [Tempo-BT V1 Board](../../hardware/tempo-bt)**
  - open source hardware design
  - part of this GitHub repository
  - Design based on an FCC-certified u-blox NORA-B106 SOC

- **High-frequency sensor logging**
  - 50 Hz IMU data (TDK InvenSense ICM-42688-V)
  - 4 Hz barometric pressure/temperature (Bosch BMP390)
  - 2-10 Hz GPS (u-blox SAM-M10Q)
  - Optional magnetometer support (MMC5983MA) (not yet supported in firmware)
  
- **Flexible storage options**
  - Primary: SD card via SPI (includes exFAT support)
  - Fallback: Internal QSPI flash (littlefs)
  - Automatic storage detection at boot
  
- **Wireless connectivity**
  - Bluetooth LE with mcumgr protocol
  - File transfer over BLE (100-200 KB/s) (estimated)
  - Remote configuration and control
  - OTA firmware updates (not yet tested)
  
- **Intelligent logging**
  - Automatic logging based on environmental conditions - the device starts logging when the jumppland leaves the ground and closes the log after landing
  - State machine with flight phase detection

## Hardware Requirements

- **Development Board**: nRF5340 DK or custom Tempo-BT V2 board
- **Sensors** (connected as per hardware documentation):
  - ICM-42688-V IMU (SPI)
  - BMP390 Barometer (I²C)
  - SAM-M10Q GNSS module (115200 bps UART)
  - Optional: MMC5983MA Magnetometer (I²C)
- **Storage**: SD card (recommended) or internal 8MB QSPI flash

## Software Prerequisites

### Development Environment

1. **Visual Studio Code**
   - Download and install from [https://code.visualstudio.com/](https://code.visualstudio.com/)

2. **nRF Connect SDK v3.1.0**
   - Install the nRF Connect for VS Code extension
   - Use the extension to install nRF Connect SDK v3.1.0
   - Ensure toolchain v3.1.0 is selected

3. **Additional Tools**
   - Python 3.8 or higher
   - Git
   - Python `smpmgr` tool  ([link](https://pypi.org/project/smpmgr/)) (see below)

### VS Code Configuration

Configure your build in VS Code using these settings:

![VS Code Build Configuration](../../images/temp-bt-v1-build-configuration-page.png)

Key configuration points:
- **SDK**: nRF Connect SDK v3.1.0
- **Toolchain**: nRF Connect SDK Toolchain v3.1.0
- **Board target**: nrf5340dk/nrf5340/cpuapp
- **Base configuration**: prj.conf
- **Device tree overlay**: boards/tempo_v1.overlay

## Build Instructions

### 1. Clone the Repository

```bash
git clone https://github.com/rrainey/tempo-bt
```

### 2. Open in VS Code

```bash
code .
```

### 3. Build the Project

Using VS Code:
1. Open the nRF Connect extension sidebar
2. Click "Add Build Configuration"
3. Select the settings as shown in the configuration image above
4. Click "Generate and Build"

Using Command Line:
```bash
west build -b nrf5340dk_nrf5340_cpuapp -p auto
```

### 4. Flash the Firmware

Using VS Code:
- Click the "Flash" button in the nRF Connect extension

Using Command Line:
```bash
west flash
```
## Interacting with the device

The device is intended for use with the [tempo-insights](https://github.com/rrainey/tempo-insights) application.  That application is designed to run on a Raspberry Pi 5 (or later).  It leverages the Bluetooth hardware to communicate with a Tempo-BT device.

That application repository includes extensions to the Python-based `smpmgr` command line tool ([link](https://pypi.org/project/smpmgr/)).  You can see examples of and experiment with interacting with a Tempo-BT device using that tool.

#### Notes on UUIDs

- Both `user_uuid` and `device_uuid` are automatically generated on first boot
- UUIDs are version 4 (random) format
- The `user_uuid` is intended to identify the person using the device
- The `device_uuid` uniquely identifies this specific hardware unit
- UUIDs are displayed in standard format (e.g., `550e8400-e29b-41d4-a716-446655440000`)
- When setting UUIDs via mcumgr, provide them as 32 hex characters without dashes

#### Persistence

All settings are stored in non-volatile memory and persist across power cycles and firmware updates.

This revised section provides comprehensive documentation for all available settings, including examples of how to read and write each one using mcumgr commands.

### Manual Control

The device can also be controlled via the onboard BTN1 button (nearest the USB port):
- **BTN1 Long press (2s)**: Manually start logging

## Data Format

Log files are in extended NMEA sentence format with NMEA-style checksums. Key sentence types:

- `$PVER`: Version and metadata
- `$PIMU`: IMU data (50 Hz)
- `$PIM2`: Quaternion orientation
- `$PENV`: Environmental data (pressure, temperature)
- `$PFIX`: GPS fix information
- `$PST`: State changes
- `$PMAG`: Magnetometer data (optional)
- `$GNVTG` : track made good, from u-blox receiver
- `$GNGGA` : Fix, from u-blox receiver
- `$GNGSV` : Satellite data, from u-blox receiver
- `$GNRMC` : Fix, time, track, from u-blox receiver

Example log excerpt:
```
$PVER,1.0,V1,0.1.0,2025-01-15*AB
$PSFC,880*1C
$PST,1000,IDLE,ARMED,USER*5D
$PIMU,1001,9.81,0.02,-0.15,0.001,0.002,0.003*2F
$PIM2,1001,1.0000,0.0000,0.0000,0.0000*A1
```

## LED Status Indicators

The RGB LED provides visual system status:

| Color | Pattern | State |
|-------|---------|-------|
| Blue | Slow Pulse | IDLE - not used in the current implementation |
| Off | N/A | ARMED - Ready (green / red side LEDs indicate GNSS Fix State) |
| Green | Slow pulse | Logging - waiting for aircraft exit |
| Orange | Slow pulse | Logging - in freefall or under canopy |
| White | Solid | File transfer active (not yet implemented) |
| Red | Slow pulse | Fatal Error |

Additionally, there is a separate Red LED which will blink when there is no usable GPS signal.

A separate Amber LED is the USB charging indicator.

## Troubleshooting

### Common Issues

1. **SD card not detected**
   - Verify card is properly inserted
   - Validate that the SD card is readable on a PC
   - Check for supported card size (max 32GB recommended)

### Debug Output

Connect via RTT to see debug messages:
```bash
JLinkRTTViewer
```

## Acknowledgments

This project incorporates the following open-source libraries:

### Fusion AHRS Algorithm

The orientation tracking system uses the **Fusion** sensor fusion library by [xioTechnologies](https://x-io.co.uk/):

- **Project**: Fusion - Sensor Fusion Library for Inertial Measurement Units (IMUs)
- **Author**: Seb Madgwick
- **Repository**: https://github.com/xioTechnologies/Fusion
- **License**: MIT License
- **Usage**: Provides AHRS (Attitude and Heading Reference System) quaternion estimation from gyroscope and accelerometer data

The Fusion library implements a complementary filter with adaptive gain, gyroscope bias correction, and rejection of acceleration disturbances. It is specifically optimized for embedded systems.

**Citation**:
```
Madgwick, S. (2021). Fusion - Sensor Fusion Library for IMUs [Software].
Available from https://github.com/xioTechnologies/Fusion
```

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

```
Copyright 2025 Tempo-BT Contributors

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

## Support

For questions, bug reports, or feature requests:

- **Primary Contact**: Riley Rainey ([@rileyrainey](https://x.com/rileyrainey) on X.com)
- **Issues**: Please file issues on the project's GitHub repository
- **Documentation**: Additional documentation can be found in the `docs/` directory

### Contributing

Contributions are welcome!