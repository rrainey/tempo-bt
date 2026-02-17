# Tempo-BT Orientation Visualizer

This Python desktop application connects to a Tempo-BT device over USB and renders its real-time orientation in 3D. It parses the `$PIM2` (quaternion) and `$PIMU` (accelerometer/gyroscope) NMEA sentences from the Tempo-BT USB TTY log stream.

The application demonstrates the Tempo device's ability to compute a skydiver's orientation in real time using the on-board ICM-42688 IMU and Fusion AHRS algorithm. A telemetry overlay displays live accelerometer and gyroscope readings alongside the 3D view.

## Prerequisites

- Tempo-BT device with firmware v1.1.0 or later (USB CDC-ACM support)
- USB cable connecting Tempo-BT to your desktop machine
- Python 3.9+

## Setup

1. Open a terminal in the `tools/visualizer` folder.

2. Set up a Python virtual environment (recommended):

```
$ python -m venv .venv
$ .venv\Scripts\activate
```

or Powershell:

```
> .venv\\Scripts\\Activate.ps1 
```

On Windows, you may need to allow script execution first:
```
> Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

3. Install dependencies:

```
> pip install -r requirements.txt
> pip install PyOpenGL PyOpenGL_accelerate
```

*Installation of `PyOpenGL_accelerate` is optional.* It may require Microsoft Visual Studio or a prebuilt wheel. See [this guide](https://stackoverflow.com/questions/77346740/install-pyopengl-for-windows-under-python-3-12) for prebuilt options.

## Running

1. Connect Tempo-BT to your desktop via USB.

2. Start the visualizer:

```
> python visualizer.py
```

The application auto-detects the Tempo-BT device by its USB VID (`0x1209`) and PID (`0x2026`). If auto-detection fails (e.g., multiple devices or driver issues), specify the port manually:

```
> python visualizer.py --port COM8
```

3. On the Tempo-BT device, long-press **BTN2** (2 seconds) to start the USB log stream.

4. The 3D model will rotate to mirror the device's physical orientation. The top-right overlay shows live IMU telemetry (accelerometer in m/s², gyroscope in rad/s).

5. Long-press **BTN2** again to stop streaming, or simply disconnect the USB cable.

## Keyboard Controls

| Key | Action |
|-----|--------|
| **F** | Front view |
| **R** | Right view |
| **Q** | Quit |

## NMEA Sentences

The visualizer processes these sentences from the USB stream:

| Sentence | Rate | Usage |
|----------|------|-------|
| `$PIM2` | 20 Hz | Quaternion orientation (w, x, y, z) for 3D rendering |
| `$PIMU` | 20 Hz | Accelerometer + gyroscope values for telemetry overlay |

All other sentences (`$PENV`, `$PST`, `$PVER`, `$GxGGA`, `$GxVTG`, `$GxRMC`, etc.) are silently ignored.

## Troubleshooting

- **"No Tempo-BT device found"**: Ensure the device is connected via USB and appears as a COM port in Device Manager. Use `--port COMx` if auto-detection fails.
- **No data streaming**: Make sure you long-pressed Button 1 / BTN2 on the Tempo-BT to start the USB log stream. The green LED indicates the device is in LOGGING state.
- **3D model orientation seems wrong**: Set the device flat on your desk with the USB connector coming out the left side, then press F for front view to establish a reference.
