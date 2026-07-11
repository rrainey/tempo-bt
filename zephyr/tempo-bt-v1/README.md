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
  - OTA firmware updates over BLE/SMP (MCUboot; validated 1.4.0 → 1.5.0 — see [DFU/OTA test plan](docs/dfu-ota-test-plan.md))

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
   - Ensure toolchain v3.1.0 or later is selected

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

The device is intended for use with the [tempo-insights](https://github.com/rrainey/tempo-testbed) application suite.  It leverages the Bluetooth hardware to communicate with a Tempo-BT device.

That application repository includes extensions to the Python-based `smpmgr` command line tool ([link](https://pypi.org/project/smpmgr/)).  You can see examples of and experiment with interacting with a Tempo-BT device using that tool.

#### Notes on UUIDs

- Both `user_uuid` and `device_uuid` are automatically generated on first boot
- UUIDs are version 4 (random) format
- The `user_uuid` is intended to identify the person using the device (although we don't use that today)
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

Jump logs are stored in a date-organized tree in `/logs/` on the device SD card.  The log file format is described in [LOG-FORMAT.md](docs/LOG-FORMAT.md)

## LED Status Indicators

A GREEN LED is used as a power indication.

The RGB LED provides visual system status:

| Color | Pattern | State |
|-------|---------|-------|
| Off | N/A | ARMED - Ready (green / red side LEDs indicate GNSS Fix State) |
| Green | Slow pulse | Logging - waiting for aircraft exit |
| Orange | Slow pulse | Logging - in freefall or under canopy |
| White | Solid | File transfer active (not yet implemented) |
| Red | Slow pulse | Fatal Error |
| Blue | Slow Pulse | IDLE - not used in the current implementation |

Additionally, there is a dedicated RED LED which will blink when there is no usable GPS signal; the GREEN LED nearest the user buttons will flash once per second when the GNSS receiver has a valid 3D Fix.

A separate AMBER LED is the USB charging indicator.

## Firmware Update over BLE (OTA DFU)

The device supports over-the-air firmware upgrades over Bluetooth using MCUboot and the
mcumgr/SMP protocol. There is **no serial/USB DFU** — updates arrive over the same BLE link
used for file transfer. This procedure is validated (see
[DFU/OTA test plan](docs/dfu-ota-test-plan.md)); the steps below are the reusable recipe.

### How it works

The nRF5340 is dual-core, so a full update ships **two images** — the application core and
the network (radio) core — packaged together in `build/dfu_application.zip`
(`tempo-bt-v1.signed.bin` = app, `ipc_radio.bin` = net core, plus a manifest). Both images
are uploaded to staging slots in the **external QSPI flash**, marked pending, and applied on
a single reboot: MCUboot overwrites the application slot and uses the PCD library to
reprogram the network core. Images are signed and validated by MCUboot before they are
accepted.

> ⚠️ **The build is overwrite-only: there is no automatic rollback.** A bad image — most
> dangerously a bad network-core image that breaks BLE — cannot self-recover. Recovery is a
> wired SWD/J-Link re-flash of `build/merged.hex` + `build/merged_CPUNET.hex` (see
> [Flash the Firmware](#4-flash-the-firmware)). For first-time or risky updates, use a
> device you can reach with a debugger.
>
> Production flight logs are **not** touched: the DFU staging slots live in a separate region
> of the external flash from the LittleFS log partition.

### Prerequisites

- `smpmgr` on PATH (`pip install smpmgr`). The DFU path uses only smpmgr's **native**
  `image`/`os`/`file` groups — the Tempo group-64 plugin is **not** required.
- A current DFU package at `build/dfu_application.zip` (produced by the normal sysbuild build).
- The target device powered on and advertising within BLE range.

### Scripted upgrade (recommended)

[`tools/dfu-upgrade.sh`](tools/dfu-upgrade.sh) performs the whole flow for one device and
verifies it — resolve by name, stage both images, mark both pending, reset (retrying until
the swap actually applies), re-discover by name, and confirm the running image. It is
version-agnostic (the upgrade target is whatever the package stages) and touches no flight
logs.

```bash
# uses build/dfu_application.zip by default; pass a path to use a different package
tools/dfu-upgrade.sh 0006
tools/dfu-upgrade.sh 0006 /path/to/dfu_application.zip
```

It prints `RESULT Tempo-BT-0006: PASS …` and exits 0 on success. A non-zero exit before the
reset leaves the device untouched (still on the old image); a non-zero exit after means the
swap did not take — recover/inspect via SWD. Requires `smpmgr`, `bluetoothctl`, `python3`,
`unzip`; the companion state parser is [`tools/dfu_state_parse.py`](tools/dfu_state_parse.py).
The manual steps below are exactly what the script automates.

### Procedure (manual)

```bash
# 0. Extract the two images from the DFU package
cd build
unzip -o dfu_application.zip     # → tempo-bt-v1.signed.bin, ipc_radio.bin, manifest.json

# 1. Find the device address. The BLE MAC is randomly assigned at each power-on, so always
#    resolve it by advertised name (Tempo-BT-nnnn) rather than a remembered address:
bluetoothctl --timeout 8 scan on | grep "Tempo-BT-0001"     # note the AA:BB:.. address
DEV=<address-from-scan>

# 2. Baseline — confirm SMP is up and record the running image hash:
smpmgr --ble $DEV os echo hello
smpmgr --ble $DEV image state-read      # note slot0/image0 hash = the OLD firmware

# 3. Stage both images (non-destructive — the device keeps running the old firmware).
#    NOTE: smpmgr's "--slot" is really the nRF5340 IMAGE NUMBER: 0 = app core, 1 = net core.
smpmgr --timeout 30 --ble $DEV image upload tempo-bt-v1.signed.bin --slot 0
smpmgr --timeout 30 --ble $DEV image upload ipc_radio.bin          --slot 1

# 4. Confirm both are staged, and copy the two staged hashes for the next step:
smpmgr --ble $DEV image state-read
#   expect: image0/slot1 = new app hash, image1/slot1 = new net hash (both pending=False here)

# 5. Mark both staged images pending (confirm=false). THIS IS THE POINT OF NO RETURN:
smpmgr --ble $DEV image state-write <NEW_APP_HASH> false
smpmgr --ble $DEV image state-write <NEW_NET_HASH> false
smpmgr --ble $DEV image state-read      # verify both staged images now show pending=True

# 6. Apply. The device reboots, overwrites the app slot, and reprograms the net core.
#    This swap takes ~30-45 s (much longer than a plain reboot). `os reset` can silently
#    fail to trigger — if step 7 still shows the OLD hash with the images still pending,
#    the device never rebooted; re-issue os reset.
smpmgr --ble $DEV os reset

# 7. Rediscover by name (the reboot is a new power-on → the MAC may change), then verify:
bluetoothctl --timeout 8 scan on | grep "Tempo-BT-0001"     # ← must still advertise its name
DEV=<new-address-from-scan>
smpmgr --ble $DEV os echo hello                              # BLE/net core OK
smpmgr --ble $DEV image state-read                          # running slot0 hash == NEW_APP_HASH,
                                                            #   active=True, confirmed=True,
                                                            #   and the pending images are gone
```

### Acceptance checks

- `image state-read` shows the running application image hash equal to the newly-uploaded
  app image, `active` and `confirmed`. (The MCUboot image *version* field reads `0.0.0` and
  is **not** a usable version signal — verify by hash.)
- The device still advertises its assigned `Tempo-BT-nnnn` name and answers `os echo` —
  this confirms the network core survived the update.
- `smpmgr --ble $DEV file read-size <path>` responds — file transfer intact.

### Notes

- **Single-image shortcut:** for an app-core-only update you can use smpmgr's one-shot
  `smpmgr --ble $DEV upgrade <image>` (upload + mark pending + reset). The dual-core case
  above deliberately stages both images before the single reset.
- **Typical timing** (single scan adapter): app-core upload (~337 KB) ≈ 23 s, net-core
  upload (~172 KB) ≈ 13 s, and the reboot+swap (both cores) ≈ 30–45 s before the device is
  reachable again → roughly 75–90 s end to end. (A ~11 s "reboot" means the swap did **not**
  happen — the reset didn't take; re-issue it.)
- **BLE reliability:** intermittent "device not found" / dropped requests are almost always
  **BT adapter contention**, not the device — running multiple controllers at once (e.g. the
  transfer-dongle pool alongside the onboard radio) makes BlueZ/bleak discovery flaky. Do
  DFU with a single active controller if you hit this. Direct `smpmgr` connects to a known
  address are far more reliable than `bluetoothctl scan`; since a device keeps its BLE MAC
  across a reboot, verify a post-reset swap by polling its address, not by re-scanning.

## Planned firmware work

Driven by the `tempo-tb-ingest` automated-harvest workstream:

- **`SESSION_LIST` pagination** — extend the custom group-64 `SESSION_LIST` command to
  return sessions in pages, so devices holding large numbers of sessions can be
  enumerated reliably over BLE without exceeding SMP payload limits.
- **Firmware version in `settings-get`** — add a firmware-version field to the
  `settings-get` response (to be implemented alongside the pagination change). This gives
  clients a first-class version indicator; today the only over-BLE version signal is the
  MCUboot image hash (the MCUboot image *version* is `0.0.0`, so it is not usable), which
  is why the [DFU/OTA test plan](docs/dfu-ota-test-plan.md) verifies upgrades by image-hash
  match rather than a version string.

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