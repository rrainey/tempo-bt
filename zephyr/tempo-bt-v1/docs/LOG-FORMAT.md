# Tempo-BT Log File Format

A Tempo-BT log (`flight.txt`, one per logging session, stored at
`/logs/<YYYYMMDD>/<SESSIONID>/` on the device's primary storage) is an extended
NMEA-sentence text file: [NMEA 0183](https://en.wikipedia.org/wiki/NMEA_0183)
standard GNSS sentences interspersed with the application-specific sentences
described below. Lines are CRLF-terminated.

A session begins when logging starts — by push-button (`manual_start`), by BLE
command, or automatically when the device detects the aircraft climbing
(`takeoff_detected`) — and ends on manual stop, BLE command, or automatically a
short time after the jumper reaches the ground.

This document describes the format emitted by **Tempo-BT firmware v1.x**
(`$PVER` format versions 110 and later). The format descends from the earlier
Dropkick logger; the legacy Dropkick format (versions 0xx) is documented in the
Dropkick repository. A version-history table appears at the end of this
document.

## Time bases

Some NMEA sentences include a UTC timestamp computed by the GNSS receiver. The
device's native clock is its uptime in milliseconds (an unsigned 32-bit count
since boot, written as "device-ms" below; the Dropkick heritage called this
`millis()`). Both time values appear in a log file, and the two timelines are
correlated using the `$PTH` sentence described below.

## NMEA Checksums

All sentences end with a three-character NMEA checksum sequence (`*HH`, where
`HH` is the hex representation of the checksum byte). Checksums are omitted
from most examples below for clarity.

## GNSS Sentences

The u-blox GNSS receiver contributes standard sentences with the `$GN` talker
prefix:

| Sentence | Rate | Notes |
|----------|------|-------|
| `$GNGGA` | 1 Hz normally, **10 Hz in jump mode** | position/fix; each GGA is immediately followed by a `$PTH` |
| `$GNVTG` | 1 Hz normally, **10 Hz in jump mode** | course/speed over ground |
| `$GNRMC` | 1 Hz (always; does **not** increase in jump mode) | carries the UTC **date** — the only sentence that does. Emitted since format version 112; **version-110 logs contain no RMC**, so tools must derive the date elsewhere (e.g. the session directory name). |
| `$GNGSA` | *planned — not currently emitted* | DOP/active satellites |

## $PVER Record

A single instance appears as the first sentence of each log, documenting the
firmware that created the file. The version number's "hundreds" digit
designates the board family: 1xx = Tempo, 0xx = the legacy Dropkick. (An
imperfect scheme, retained for compatibility.)

### Comma-separated Fields

| Description | |
|-------------|--|
| $PVER | Record identifier |
| id string | human-readable firmware identification |
| version number | integer format version; 1xx = Tempo |

### Example

`$PVER,"Tempo V2 1.4.0 (fa93d6d-dirty)",114*72`

## $PSFC Record

Follows the `$PVER` sentence. Records the estimated surface altitude of the
takeoff area, computed from static air pressure assuming a
[standard atmosphere lapse rate](https://en.wikipedia.org/wiki/Atmospheric_pressure).
Subtract this value from `$PENV` altitude reports to estimate height above
ground level (AGL).

### Comma-separated Fields

| Description | |
|-------------|--|
| $PSFC | Record identifier |
| estimated surface altitude | feet, MSL (reflects pressure sampled at the device) |

### Example

`$PSFC,650*19`

## $PST Record

Records logger state-machine transitions. Useful for isolating segments of the
jump during analysis — in particular, the transition **to `JUMPED` marks
freefall detection (exit)**.

The logger states are `IDLE`, `ARMED`, `LOGGING`, `JUMPED`, and `POSTFLIGHT`.
A session's first `$PST` is normally the transition into `LOGGING`; the reason
field is free text (observed values include `manual_start`,
`takeoff_detected`, `freefall_detected`, `manual_stop`).

> Heritage note: Dropkick used a 3-field form (`$PST,<ms>,<newstate>`) with
> states `WAIT/FLIGHT/JUMPING/LANDED1`. Tempo-BT has always used the 5-field
> transition form below.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PST | Record identifier |
| device-ms timestamp | time of state change in milliseconds |
| from state | state being left |
| to state | state being entered |
| reason | free-text trigger description |

### Examples

```
$PST,1037,ARMED,LOGGING,takeoff_detected*63
$PST,813987,LOGGING,JUMPED,freefall_detected*51
$PST,1096343,JUMPED,IDLE,manual_stop*37
```

## $PIMU Record

Logs inertial data from the ICM42688-V IMU, reported in Case Axes (body axes),
shown below.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PIMU | Record identifier |
| device-ms timestamp | time of sample in milliseconds |
| X-accel | meters per second squared |
| Y-accel | meters per second squared |
| Z-accel | meters per second squared |
| X-rate | rotation rate, radians per second |
| Y-rate | rotation rate, radians per second |
| Z-rate | rotation rate, radians per second |

![Tempo Frames](images/tempo-v1-frames.png)
Tempo board axes, including the Case (or Body) Axis definitions

### Example

`$PIMU,1396,-1.21,9.73,-3.75,-0.0701,-0.0653,-0.0075*07`

## $PIM2 Record

The real-time orientation of the jumper expressed as a
[quaternion](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation).
One `$PIM2` immediately follows each `$PIMU`.

The application maintains this orientation quaternion from a 200 Hz sample
stream inside the IMU pipeline (logged at the `$PIMU` rate). A startup value
of [1,0,0,0] is used, and the quaternion accumulates all body-axis rotation
from that original orientation. Transforming it into a world frame
(North-East-Down, for example) remains analysis-side work.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PIM2 | Record identifier |
| device-ms timestamp | time of sample in milliseconds |
| W | quaternion w component, non-dimensional |
| X | quaternion x component, non-dimensional |
| Y | quaternion y component, non-dimensional |
| Z | quaternion z component, non-dimensional |

### Example

`$PIM2,13925127,1.0000,0.0000,0.0000,0.0000`

## $PENV Record

Logs static pressure from the
[BMP390 sensor](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
and a derived standard-day altitude.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PENV | Record identifier |
| device-ms timestamp | time of sample in milliseconds |
| static air pressure | hPa |
| estimated altitude | from static pressure assuming a Standard Day; feet, MSL |
| VBATT voltage | **always -1.00 on Tempo-BT** (battery voltage is not monitored on V1 hardware; the field is retained for Dropkick compatibility) |

### Example

`$PENV,1286,987.85,701.05,-1.00*3A`

## $PTH Record

Correlates device-ms timestamps with the GNSS UTC clock in standard NMEA
sentences. **One `$PTH` immediately follows each `$GNGGA`**; its timestamp is
the device-ms at the arrival of the first character of that GGA sentence.
(`$PTH` follows the GGA only — not VTG or RMC.)

To convert a device-ms event time to UTC: take the GGA preceding the event,
its paired `$PTH`, and compute `event_utc = gga_utc + (event_ms - pth_ms)`.

### Example

```
$GNGGA,134316.00,3327.50347,N,09622.62352,W,1,12,0.59,258.0,M,-25.6,M,,*71
$PTH,1364*60
```

## $PMAG Record (devices with MMC5983MA magnetometer)

Logs calibrated magnetometer data. Only emitted when `mag_mode > 0` and
calibration is valid. Values are in the device body-axis frame after hard/soft
iron correction. Earth's magnetic field magnitude is typically 25–65 µT.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PMAG | Record identifier |
| device-ms timestamp | time of sample in milliseconds |
| X-mag | calibrated magnetic field X, µT |
| Y-mag | calibrated magnetic field Y, µT |
| Z-mag | calibrated magnetic field Z, µT |

### Example

`$PMAG,13925127,23.50,-12.30,45.60`

## $PRMG Record (USB calibration streaming)

Streams raw magnetometer readings during calibration mode (BTN2 short press
with USB connected). Values are SET/RESET corrected but NOT hard/soft-iron
calibrated. Used by the Python calibration tool (`mag_cal.py`) to collect
samples for ellipsoid fitting. Appears on the USB CDC-ACM stream, not in log
files.

### Comma-separated Fields

| Description | |
|-------------|--|
| $PRMG | Record identifier |
| device-ms timestamp | milliseconds since boot |
| raw X | raw X count, signed 18-bit (16384 counts/Gauss) |
| raw Y | raw Y count, signed 18-bit (16384 counts/Gauss) |
| raw Z | raw Z count, signed 18-bit (16384 counts/Gauss) |
| temperature | die temperature, °C |

### Example

`$PRMG,13925127,8192,-4096,12288,25.3`

## $PCMD / $PRSP Records (USB command protocol)

A bidirectional command/response protocol over the USB CDC-ACM interface.
Commands are only accepted during magnetometer calibration streaming mode.
These sentences do not appear in log files.

### Command Format (Host → Device)

`$PCMD,<verb>[,<arg1>,<arg2>,...]*HH\r\n`

### Response Format (Device → Host)

`$PRSP,<verb>,<result>[,<data>,...]*HH\r\n`

### Available Commands

| Command | Description |
|---------|-------------|
| CAL_GET | read current NVM calibration data |
| CAL_SET | write calibration data to NVM (6 fields: offset_x, offset_y, offset_z, scale_x, scale_y, scale_z) |
| MODE_GET | read current mag_mode setting |
| MODE_SET | set mag_mode (0=disabled, 1=factory, 2=NVM calibrated) |

### Examples

```
$PCMD,CAL_GET*27
$PRSP,CAL_GET,1,-234,567,-89,32100,33200,31800*4F

$PCMD,CAL_SET,-234,567,-89,32100,33200,31800*1A
$PRSP,CAL_SET,OK*3B

$PCMD,MODE_SET,2*5C
$PRSP,MODE_SET,OK*2E
```

## Sentence Reporting Rates

| Sentence Type | Reporting Rate |
|:-------------:|:---------------|
| PVER | first line of the log file |
| PSFC | follows the PVER sentence |
| GNGGA, GNVTG | 1 Hz normally; 10 Hz in jump mode |
| GNRMC | 1 Hz (unchanged in jump mode); since version 112 |
| GNGSA | planned — not currently emitted |
| PTH | one, immediately following each GGA |
| PIMU | 20 Hz |
| PIM2 | follows each $PIMU sentence |
| PENV | 4 Hz |
| PST | at each logger state change |
| PMAG | 20 Hz (when magnetometer enabled and logging) |
| PRMG | 20 Hz (USB calibration streaming mode only; not in logs) |
| PCMD / PRSP | on-demand (USB calibration streaming mode only; not in logs) |

## Format Version History

| $PVER version | Firmware | Changes |
|---------------|----------|---------|
| 110 | Tempo V2 1.0.0 | Tempo-BT baseline: GGA/VTG + PTH, PIMU/PIM2 (20 Hz), PENV (battery always -1.00), PSFC, 5-field $PST. **No $GNRMC** — logs carry no in-band UTC date. |
| 112 | Tempo V2 1.2.0 | Added `$GNRMC` (1 Hz), providing the UTC date in-band. |
| 114 | Tempo V2 1.4.0+ | `$PMAG` available when `mag_mode > 0`; settings gained `mag_mode`. |

*(Rates and behaviors above verified against real session logs from each
version era, 2026-07-08.)*
