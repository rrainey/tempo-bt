# Tempo-BT

Tempo-BT is the Bluetooth LE capable version of the Tempo skyding logger device.  It is based on the u-blox NORA-B106 SOC.

### Sensors and Capabilities

- GPS/GNSS using a u-blox SAM-M10Q
- 6-DOF Inertial measurement / gyro using a ICM-42688-V
- Barometric pressure / temperature using a BPM390
- 3-DOF Compass/Magnetic measurement using a MMC5983MA

## Directory Structure

| Folder      | Description |
| ----------- | ----------- |
| hardware    | KiCad PCB projects (using KiCad 7)    |
| hardware/tempo-bt |u-blox NORA-B106 Bluetooth SOC (Nordic nRF5340) + u-blox GNSS + IMU + Barometric sensor PCB |
| enclosure    | 3D-printable enclosures (Fusion360 format) |
| zephyr/tempo-bt-v1 | Nordic nRF Connect SDK (Zephyr) firmware |

## Enclosure

The enclosures are designed to be SLA printable. 

