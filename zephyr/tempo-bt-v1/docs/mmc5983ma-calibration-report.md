# MMC5983MA Magnetometer Calibration for Tempo-BT AHRS Integration

## Research Report — February 2026

---

## 1. Context and Objectives

The Tempo-BT skydiving logger includes a MEMSIC MMC5983MA 3-axis magnetic sensor (currently unused) alongside a TDK ICM-42688-V 6-DOF IMU. Integrating the magnetometer into a Madgwick Fusion AHRS algorithm would upgrade the system from 6-DOF (gyro + accelerometer) to 9-DOF (MARG: Magnetic, Angular Rate, Gravity), providing absolute heading reference — particularly valuable for reconstructing skydive trajectories and body orientation.

Achieving accurate heading from the magnetometer requires addressing multiple layers of calibration and compensation. This report identifies what's needed and assesses implementation options.

---

## 2. What the MMC5983MA Brings to the Table

Key sensor specifications relevant to AHRS integration:

| Parameter | Value |
|-----------|-------|
| Field Range | ±8 Gauss per axis |
| Resolution (18-bit) | 0.0625 mG/LSB (16384 counts/G) |
| Total RMS Noise (BW=00) | 0.4 mG |
| Max Output Rate (BW=11, CM) | 1000 Hz |
| Heading Accuracy (claimed) | ±1.0° typical |
| Null Field Output (18-bit) | 131072 counts (unsigned midpoint) |
| I²C Address | 0x30 (7-bit: 0110000) |
| Operating Temp | -40°C to +105°C |

The MMC5983MA uses unsigned output format — the midpoint (131072 at 18-bit) represents zero field. Earth's magnetic field is roughly 250–650 mG depending on location, so the sensor has substantial headroom.

**Notable hardware feature:** The built-in SET/RESET coil is critical for this sensor. It provides on-chip degaussing that eliminates thermal drift of the null-field offset — a significant advantage over competitors lacking this capability.

---

## 3. Calibration Layers Required

Integrating the MMC5983MA into the Madgwick AHRS requires addressing calibration at multiple levels. These are distinct concerns that build on each other:

### Layer 1: Sensor-Level — SET/RESET Bridge Offset Elimination

The MMC5983MA's AMR sensing elements have an inherent bridge offset that drifts with temperature. The datasheet prescribes a SET/RESET measurement protocol to eliminate this:

1. Perform **SET** (write 0x08 to Control Register 0) — magnetizes sensing film in one direction
2. Take measurement → `Output_SET = +H + Offset`
3. Perform **RESET** (write 0x10 to Control Register 0) — magnetizes film in opposite direction
4. Take measurement → `Output_RESET = -H + Offset`
5. Compute: `H = (Output_SET - Output_RESET) / 2` and `Offset = (Output_SET + Output_RESET) / 2`

This eliminates the temperature-dependent bridge offset entirely. The `Auto_SR_en` bit in Control Register 0 can automate this, but explicit control gives better results. The sensor can also perform periodic SET operations in continuous measurement mode via `En_prd_set` and `Prd_set[2:0]`.

**Important soldering caution:** The DRNadler/MMC5983MA_CompassTest project on GitHub documents that overheating during soldering can permanently damage the sensor's AMR film, causing the SET/RESET procedure to fail to properly auto-zero. Verify sensor health with a field-magnitude consistency test after board assembly.

**Firmware requirement:** The Zephyr driver or custom driver code must implement the SET/RESET measurement protocol. This is not optional — without it, the raw readings will have temperature-dependent offsets of up to ±0.5 G, which is comparable to or larger than Earth's field components.

### Layer 2: Hard and Soft Iron Calibration (HSI)

Even with perfect sensor-level operation, the magnetometer will measure a distorted version of Earth's field due to magnetic materials on or near the PCB:

- **Hard iron distortions** are constant magnetic fields from permanently magnetized materials (battery, speaker magnets, magnetized steel). They create a fixed bias — shifting the measurement sphere away from the origin.

- **Soft iron distortions** are caused by ferromagnetic materials (nickel, iron) that warp the existing field. They distort the measurement sphere into an ellipsoid.

As described by VectorNav's educational material, the compensation model is:

```
m_corrected = S_I × (m_raw - b_HI)
```

Where `S_I` is a 3×3 soft iron correction matrix and `b_HI` is a 3-element hard iron bias vector. Together these 12 parameters map the distorted ellipsoid back to a sphere centered at the origin.

**For the Tempo-BT specifically:** The PCB has a LiPo battery, a u-blox SAM-M10Q GNSS module, and other components that will create both hard and soft iron distortions. These must be characterized with the device fully assembled in its enclosure with battery connected.

### Layer 3: Sensor Alignment / Cross-Axis Calibration

The MMC5983MA has a specified alignment error of ±1.0° typical (±3.0° max) between its sensing axes and the package. For AHRS integration, the magnetometer's coordinate frame must be aligned with the ICM-42688-V accelerometer/gyro coordinate frame. Misalignment between the two sensors contributes directly to heading error.

This can be folded into the soft iron matrix or handled as a separate rotation matrix.

#### Tempo-BT sensor package mounting alignment

The sensor packages are placed on the PCB such that sensor axis coordinate frames are aligned in this manner:

| ICM-42688-V Axis | MMC5983MA Axis |
|:----------------:|:---------:|
|    +X            |   -Y      |
|    +Y            |   +X      |
|    +Z            |   -Z      |

### Layer 4: Madgwick Filter Integration

The Madgwick AHRS filter's `updateMARG()` function expects calibrated, normalized magnetometer data in the same coordinate frame as the accelerometer and gyroscope. Key requirements:

- Magnetometer readings must be calibrated (hard/soft iron corrected) before being passed to the filter
- The magnetometer coordinate frame must be aligned with the IMU frame
- The filter's `beta` parameter controls how much weight is given to the magnetometer/accelerometer correction vs. gyro integration — this needs tuning for the skydiving use case

---

## 4. Calibration Data Collection Procedure

The standard approach for HSI calibration requires collecting magnetometer samples while slowly rotating the device through all orientations:

**For 2D calibration** (heading only, device stays approximately level): Rotate the device in a few 360° circles about the gravity vector. Sufficient if pitch/roll stays within ±5-10°.

**For full 3D calibration** (recommended for skydiving AHRS): Slowly tumble the device through as many orientations as possible, aiming to uniformly cover the surface of a sphere. This typically requires 30-60 seconds of deliberate rotation, collecting hundreds to thousands of samples.

For the Tempo-BT, the recommended procedure is:

1. Power on the device with firmware in a "calibration data collection" mode
2. Stream raw (SET/RESET-corrected) magnetometer data via BLE or USB serial
3. Slowly tumble the device through all orientations outdoors, away from large metal structures
4. Capture the data on a desktop computer for offline processing

---

## 5. Calibration Algorithm Options

### 5A: Ellipsoid Fitting (Recommended)

The gold standard for magnetometer HSI calibration. The algorithm fits the collected 3D data points to an ellipsoid, then computes the transformation that maps the ellipsoid back to a unit sphere centered at the origin.

The mathematical foundation: raw magnetometer data in the absence of time-varying distortions traces an ellipsoid in 3D space. The ellipsoid's center gives the hard iron offset; its shape (axes lengths and rotation) encodes the soft iron distortion.

**Algorithmic approaches:**

| Method | Complexity | Quality | Notes |
|--------|-----------|---------|-------|
| Least-squares ellipsoid fit (Li et al.) | Low | Good | Single-step, easy to implement |
| Levenberg-Marquardt iterative fit | Medium | Better | Two-stage: sphere fit → ellipsoid refinement |
| Min/max per axis (simple) | Trivial | Poor | Only corrects hard iron, ignores soft iron |

The least-squares approach is well-suited for a desktop calibration tool. The key reference implementation is Yury Petrov's `ellipsoid_fit` (MATLAB), which has been ported to Python by several authors.

### 5B: Existing Open-Source Implementations

| Project | Language | Notes |
|---------|----------|-------|
| [nliaudat/magnetometer_calibration](https://github.com/nliaudat/magnetometer_calibration) | Python (numpy) | CSV/TXT input, 3D visualization, exports C header constants. Well-maintained. |
| [lundeen06/magnetometer-calibration-tool](https://github.com/lundeen06/magnetometer-calibration-tool) | Python (CLI) | Full CLI tool with serial port monitoring, real-time plotting, auto-generates C header. Originally for Stanford satellite. |
| [jremington/AltIMU-AHRS](https://github.com/jremington/AltIMU-AHRS) | Arduino + Python | Includes both Madgwick and Mahony AHRS implementations plus Python calibration script. Direct reference for your use case. |
| [kriswiner/MMC5983MA](https://github.com/kriswiner/MMC5983MA) | Arduino/C++ | Complete MMC5983MA driver with SET/RESET, self-test, and min/max calibration. 10-DOF AHRS sketch. |
| [DRNadler/MMC5983MA_CompassTest](https://github.com/DRNadler/MMC5983MA_CompassTest) | C++ / wxWidgets | Desktop test app specifically for MMC5983MA. Documents soldering damage issues. |
| [Teslabs magnetometer calibration](https://teslabs.com/articles/magnetometer-calibration/) | Python | Excellent tutorial with mathematical derivation and Raspberry Pi example. |
| [michal34512/Magnetometer-calibration](https://github.com/michal34512/Magnetometer-calibration) | C | Lightweight C library with built-in linear algebra — could run on-device. |
| ST DT0059 Design Tip | MATLAB/C | STMicroelectronics reference for ellipsoid fitting. Covers rotated ellipsoid case. |

---

## 6. Recommended Implementation Architecture

Given your preference for custom desktop-based tooling, here is a recommended architecture:

### Component 1: Firmware — MMC5983MA Driver + Calibration Data Streaming

**Platform:** Zephyr RTOS on nRF5340 (existing Tempo-BT firmware)

Responsibilities:
- Full MMC5983MA driver implementing SET/RESET measurement protocol
- Self-test verification (uses St_enp/St_enm bits to verify sensor health)
- Raw data streaming mode over BLE (or USB CDC if available) for calibration
- Runtime application of calibration parameters (stored in flash/NVS)
- Calibrated data output to Madgwick AHRS filter

Key driver considerations:
- Use 18-bit mode for maximum resolution during calibration
- Implement the full SET/RESET per-measurement cycle for offset elimination
- Use BW=00 (8ms measurement, 0.4mG noise) during calibration collection
- Store calibration parameters (3 hard iron offsets + 9 soft iron matrix elements = 12 floats) in non-volatile storage

### Component 2: Desktop Calibration Tool

**Recommended stack:** Python with numpy, scipy, matplotlib

This is where you'd develop custom tooling. A desktop application that:

1. **Connects** to the Tempo-BT via BLE or serial to receive raw magnetometer data
2. **Visualizes** incoming data in real-time as a 3D scatter plot, showing coverage of the measurement sphere
3. **Guides** the user through the calibration rotation procedure with visual feedback on coverage quality
4. **Computes** the ellipsoid fit once sufficient data is collected, producing hard iron (3-vector) and soft iron (3×3 matrix) parameters
5. **Validates** the calibration by displaying corrected data as a sphere, reporting field magnitude consistency
6. **Exports** calibration parameters in a format the firmware can consume (C header, JSON, binary blob for NVS)

**Coverage quality metric:** A good calibration requires data points distributed across the full sphere. The tool should assess angular coverage and warn if large gaps exist.

**Recommended Python libraries:**
- `numpy` / `scipy` — ellipsoid fitting via least-squares
- `matplotlib` — 3D scatter plots for visualization
- `bleak` — BLE communication with the Tempo-BT
- `pyserial` — alternative serial communication
- `pyqt6` or `tkinter` — GUI framework if desired (matplotlib alone can serve for a simpler tool)

### Component 3: Validation / Verification Harness

After calibration, verify the results:

- **Field magnitude consistency test:** Rotate the calibrated device; `sqrt(x² + y² + z²)` should remain approximately constant (matching the local geomagnetic field strength from the World Magnetic Model for your location, typically 450-600 mG in the US).
- **Heading comparison:** Compare computed heading against a known reference direction.
- **AHRS convergence test:** Feed calibrated magnetometer data plus IMU data through the Madgwick filter and verify heading stability and convergence time.

### Component 4: Madgwick AHRS Integration

Modify the existing Tempo-BT firmware to:

1. Initialize the Madgwick filter in MARG (9-DOF) mode instead of IMU-only (6-DOF) mode
2. On each sensor fusion cycle:
   - Read accelerometer + gyro from ICM-42688-V (existing)
   - Read magnetometer from MMC5983MA (new)
   - Apply SET/RESET correction to mag data
   - Apply HSI calibration: `m_cal = S_I × (m_raw - b_HI)`
   - Transform mag data to IMU coordinate frame
   - Call `MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz)`
3. Tune the `beta` parameter for the skydiving use case (higher beta = faster convergence but more noise sensitivity; lower beta = smoother but slower to track heading changes)

---

## 7. Implementation Approach Options

### Option A: Minimal / Fastest Path

Use the `kriswiner/MMC5983MA` Arduino driver as reference for the Zephyr driver, implement min/max calibration (hard iron only) on-device, and use the `jremington/AltIMU-AHRS` Python script for offline soft iron calibration.

**Pros:** Fastest to working prototype. Leverages battle-tested code.
**Cons:** Min/max calibration is limited; less accurate soft iron correction.

### Option B: Custom Desktop Tool (Recommended)

Develop a Python desktop tool that handles data collection, full 3D ellipsoid fitting, visualization, and parameter export. Use the `nliaudat/magnetometer_calibration` or `teslabs` implementations as starting points for the ellipsoid fitting math.

**Pros:** Full control over the calibration process, best accuracy, reusable for production calibration of multiple devices, visual feedback improves calibration quality.
**Cons:** More development effort up front.

### Option C: Full Integrated Solution

Option B plus: real-time 3D visualization during data collection, automated coverage analysis, on-device calibration verification mode, and a BLE-based calibration workflow (potentially via a mobile companion app).

**Pros:** Best user experience, suitable for end-user recalibration.
**Cons:** Significant development effort; may be overkill for initial integration.

---

## 8. Risk Areas and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Sensor damage from soldering | High — sensor may be permanently impaired | Verify with field magnitude test post-assembly; follow MEMSIC reflow profile strictly (260°C max, 10s) |
| LiPo battery magnetization | Medium — hard iron source that may vary with charge state | Characterize with battery at different charge levels; recalibrate if needed |
| Time-varying distortions during skydive | Medium — nearby magnets from other gear, aircraft | The Madgwick filter has built-in magnetic distortion rejection; consider weighting mag contribution lower during high-dynamic phases |
| Coordinate frame alignment between MMC5983MA and ICM-42688-V | Medium — directly affects heading accuracy | Measure or compute rotation matrix between sensor frames from PCB layout |
| Temperature effects during skydive (ground ~30°C to altitude ~-20°C) | Low-Medium — SET/RESET handles null offset drift | Use periodic SET/RESET; MMC5983MA spec shows ±3 mG drift with SET/RESET over full temp range |

---

## 9. Recommended Next Steps

1. **Enable the MMC5983MA driver in Zephyr firmware** — implement SET/RESET measurement protocol; verify sensor health with self-test
2. **Add a raw data streaming mode** — BLE characteristic or USB serial output of SET/RESET-corrected magnetometer data
3. **Build the desktop calibration tool** (Python) — start with the ellipsoid fitting math from existing open-source, add 3D visualization and parameter export
4. **Perform initial calibration** of a Tempo-BT unit in its enclosure with battery
5. **Integrate calibrated mag data into Madgwick AHRS** — upgrade from 6-DOF to 9-DOF
6. **Tune and validate** — compare AHRS heading output against known references, tune beta parameter

---

## 10. MCUmgr Integration — Calibration and Settings Protocol

The Tempo-BT firmware exposes magnetometer calibration and settings via custom mcumgr SMP commands over BLE. All commands use **group ID 64** (`MGMT_GROUP_ID_TEMPO`). The request/response payload is CBOR-encoded.

The mcumgr SMP protocol runs over BLE using the standard SMP service UUID. Clients can use the `mcumgr` CLI tool, the nRF Connect mobile app (with custom group support), or any SMP-capable library (e.g., `smpclient` for Python, `mcumgr-android`, `mcumgr-ios`).

### 10.1 Magnetometer Mode Setting

The `mag_mode` setting controls how the magnetometer is used in the AHRS pipeline. It is stored in NVM and persists across reboots.

| Mode | Behavior |
|:----:|----------|
| 0    | **Disabled** (default). Magnetometer is not used in AHRS. No `$PMAG` sentences in log output. AHRS runs as 6-DOF (gyro + accel only). |
| 1    | **Factory calibration**. Magnetometer enabled for 9-DOF AHRS using only the factory/devicetree calibration values. Any NVM calibration is cleared on boot. |
| 2    | **NVM calibration**. Magnetometer enabled for 9-DOF AHRS using calibration data stored in NVM (pushed via `MAG_CAL_SET`). |

The mode is read and written as part of the general settings commands (IDs 6 and 7). A device reboot is required for mode changes to take effect.

### 10.2 Settings Get — Command ID 6 (READ)

Returns all device settings including `mag_mode`.

**Request:** Empty (no payload required).

**Response (CBOR map):**
```json
{
    "ble_name": "Tempo-BT",
    "pps_enabled": false,
    "pcb_variant": 1,
    "log_backend": "littlefs",
    "mag_mode": 0
}
```

| Field | Type | Description |
|-------|------|-------------|
| `ble_name` | string | BLE advertising name (max 31 chars) |
| `pps_enabled` | bool | PPS sync enabled (always false on V1) |
| `pcb_variant` | uint | PCB revision (0x01 = V1) |
| `log_backend` | string | `"littlefs"` or `"fatfs"` |
| `mag_mode` | uint | Magnetometer mode (0/1/2) |

### 10.3 Settings Set — Command ID 7 (WRITE)

Sets one or more device settings. Only include the fields you want to change.

**Request (CBOR map) — any subset of fields:**
```json
{
    "mag_mode": 2
}
```

**Response:** Returns all current settings (same format as Settings Get) plus `"success": true`. If `ble_name` was changed, includes `"note": "BLE name changes require reboot"`.

### 10.4 Mag Calibration Get — Command ID 10 (READ)

Returns the current magnetometer calibration data from the running firmware. This reflects whatever calibration is currently active in memory (may differ from NVM if mode 1 cleared it on boot).

**Request:** Empty (no payload required).

**Response (CBOR map):**
```json
{
    "valid": true,
    "offset_x": 960,
    "offset_y": -6440,
    "offset_z": -1257,
    "scale_x": 36863,
    "scale_y": 35809,
    "scale_z": 27398
}
```

| Field | Type | Description |
|-------|------|-------------|
| `valid` | bool | `true` if calibration data has been loaded/set |
| `offset_x` | int32 | Hard-iron X offset in raw counts (16384 counts/Gauss) |
| `offset_y` | int32 | Hard-iron Y offset in raw counts |
| `offset_z` | int32 | Hard-iron Z offset in raw counts |
| `scale_x` | uint16 | Soft-iron X scale factor (Q1.15 fixed-point; 32768 = 1.0) |
| `scale_y` | uint16 | Soft-iron Y scale factor (Q1.15 fixed-point) |
| `scale_z` | uint16 | Soft-iron Z scale factor (Q1.15 fixed-point) |

**Interpreting offset values:** The offsets represent the center of the measurement ellipsoid in the magnetometer's native coordinate frame. These are subtracted from raw readings before scale correction.

**Interpreting scale values:** The scale factors are diagonal elements of the soft-iron correction matrix in Q1.15 fixed-point format. A value of 32768 means no correction (1.0x). Values above 32768 expand the axis; values below compress it. The correction normalizes the ellipsoid radii to a sphere.

To convert from the `mag_cal.py` JSON output:
- `offset_x/y/z`: Use directly (integer counts)
- `scale_x/y/z`: Use directly (already in Q1.15 format as computed by `mag_cal.py`)

### 10.5 Mag Calibration Set — Command ID 11 (WRITE)

Pushes calibration data to the device. The firmware applies it immediately to the running mag service and persists it to NVM. All six fields are required.

**Request (CBOR map):**
```json
{
    "offset_x": 960,
    "offset_y": -6440,
    "offset_z": -1257,
    "scale_x": 36863,
    "scale_y": 35809,
    "scale_z": 27398
}
```

**Response (CBOR map):**
```json
{
    "success": true,
    "offset_x": 960,
    "offset_y": -6440,
    "offset_z": -1257,
    "scale_x": 36863,
    "scale_y": 35809,
    "scale_z": 27398
}
```

**Error cases:**
- Missing any of the six required fields → `MGMT_ERR_EINVAL`
- `scale_x/y/z` exceeding `UINT16_MAX` (65535) → field ignored, missing field error
- `mag_cal_set()` failure (mag service not initialized) → `MGMT_ERR_EUNKNOWN`
- `mag_cal_save()` failure (NVM write error) → `MGMT_ERR_EUNKNOWN`

### 10.6 Calibration Application Model

The firmware applies calibration to each raw magnetometer reading as follows:

```
1. Read raw counts from sensor (SET/RESET corrected via auto-SR)
2. Subtract hard-iron offsets:
     x_corrected = x_raw - offset_x
     y_corrected = y_raw - offset_y
     z_corrected = z_raw - offset_z
3. Apply soft-iron scale (Q1.15 fixed-point multiply):
     x_scaled = (x_corrected * scale_x) >> 15
     y_scaled = (y_corrected * scale_y) >> 15
     z_scaled = (z_corrected * scale_z) >> 15
4. Convert to Gauss:
     x_gauss = x_scaled / 16384.0
```

The calibrated values are then rotated from the magnetometer's native coordinate frame to the IMU frame (see Section 3, Layer 3) before being passed to the Fusion AHRS algorithm.

### 10.7 NVM Storage Architecture

Calibration data is persisted using the Zephyr settings subsystem with NVS backend:

| Settings path | Contents | Size |
|---------------|----------|------|
| `app/mag_mode` | `uint8_t` mode value (0/1/2) | 1 byte |
| `mag/cal` | `mag_calibration_t` struct (offsets, scales, valid flag) | 19 bytes |

The `app/mag_mode` setting is managed by the app settings module (`src/config/settings.c`). The `mag/cal` calibration data is managed by the mag service module (`src/services/mag_mmc5983ma.c`) via a separate settings handler.

Both are loaded automatically during initialization via `settings_load()`.

### 10.8 End-to-End Calibration Workflow

1. **Collect raw data**: Press Button 1 (short press) to enter calibration streaming mode. The device outputs `$PRMG` NMEA sentences over USB CDC-ACM at 20 Hz. Tumble the device through all orientations.

2. **Compute calibration**: Run `tools/mag_calibration/mag_cal.py` to perform nonlinear least-squares ellipsoid fitting. The tool produces `mag_cal.json` containing offset and scale values.

3. **Push calibration to device**: Use MCUmgr `MAG_CAL_SET` (command ID 11) to send the six calibration parameters from `mag_cal.json`. The firmware applies them immediately and persists to NVM.

4. **Set magnetometer mode**: Use MCUmgr `SETTINGS_SET` (command ID 7) with `{"mag_mode": 2}` to enable the magnetometer with NVM calibration. Reboot the device.

5. **Verify**: After reboot, the AHRS runs in 9-DOF mode. Log output includes `$PMAG` sentences with calibrated magnetic field in microtesla. Heading (yaw) should stabilize and no longer drift.

---

## 11. Key References

- **MEMSIC MMC5983MA Datasheet Rev A** — SET/RESET protocol, register map, specifications
- **VectorNav INP §3.6** — HSI calibration mathematical framework: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-hsicalibration
- **Teslabs Magnetometer Calibration** — Detailed mathematical tutorial: https://teslabs.com/articles/magnetometer-calibration/
- **kriswiner/MMC5983MA** — Reference driver implementation: https://github.com/kriswiner/MMC5983MA
- **DRNadler/MMC5983MA_CompassTest** — Test harness and soldering damage documentation: https://github.com/DRNadler/MMC5983MA_CompassTest
- **jremington/AltIMU-AHRS** — Madgwick/Mahony + calibration: https://github.com/jremington/AltIMU-AHRS
- **nliaudat/magnetometer_calibration** — Python ellipsoid fitting tool: https://github.com/nliaudat/magnetometer_calibration
- **Madgwick AHRS filter** — Original implementation and documentation: https://ahrs.readthedocs.io/en/latest/filters/madgwick.html
- **ST DT0059** — Ellipsoid fitting design tip: https://www.st.com/resource/en/design_tip/dm00286302.pdf
- **Tempo-BT repository** — https://github.com/rrainey/tempo-bt
