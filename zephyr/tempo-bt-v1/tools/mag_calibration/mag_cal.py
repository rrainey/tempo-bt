#!/usr/bin/env python3
"""
Tempo-BT Magnetometer Calibration Tool

Connects to a Tempo-BT device streaming $PRMG sentences over USB CDC-ACM,
collects raw magnetometer data, performs ellipsoid fitting to compute
hard-iron and soft-iron calibration parameters, and exports results.

Usage:
    python mag_cal.py [--port /dev/ttyACM0] [--baud 115200]
                      [--input file.csv] [--output cal_results.json]

Steps (serial mode):
    1. Connect the Tempo-BT via USB and run:  python mag_cal.py
    2. A single window opens with instructions on the left and a live 3D
       scatter plot on the right.
    3. Press BTN2 on the Tempo-BT to start calibration streaming.
    4. Slowly rotate the device through all orientations (30-60 seconds).
       The plot and octant coverage indicators update in real time.
    5. When the title turns green (8/8 octants), click the [Done] button.
    6. The window switches to a raw-vs-calibrated results view.
    7. Close the window to export calibration parameters.

Steps (file mode):
    1. Run:  python mag_cal.py --input data.csv
    2. Calibration is computed and results are shown.

The tool outputs:
    - 3D scatter plot of raw vs calibrated data
    - Hard-iron offsets (in raw counts)
    - Soft-iron diagonal scale factors (Q1.15 format for firmware)
    - Calibration quality metrics (field magnitude consistency)
    - JSON and C header format for firmware integration
"""

import argparse
import json
import queue
import struct
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from scipy.optimize import least_squares

# Tempo-BT USB identifiers (prj.conf)
TEMPO_VID = 0x1209
TEMPO_PID = 0x2026

# MMC5983MA constants
COUNTS_PER_GAUSS = 16384
GAUSS_TO_UT = 100.0  # 1 Gauss = 100 uT


def parse_prmg(line):
    """Parse a $PRMG NMEA sentence.

    Format: $PRMG,<timestamp_ms>,<raw_x>,<raw_y>,<raw_z>,<temp_c>*HH

    Returns (timestamp_ms, raw_x, raw_y, raw_z, temp_c) or None on error.
    """
    line = line.strip()
    if not line.startswith("$PRMG,"):
        return None

    # Strip checksum
    star = line.rfind("*")
    if star > 0:
        # Verify checksum
        payload = line[1:star]  # between $ and *
        expected = int(line[star + 1 : star + 3], 16)
        actual = 0
        for c in payload:
            actual ^= ord(c)
        if actual != expected:
            return None
        line = line[:star]

    parts = line.split(",")
    if len(parts) != 6:
        return None

    try:
        timestamp_ms = int(parts[1])
        raw_x = int(parts[2])
        raw_y = int(parts[3])
        raw_z = int(parts[4])
        temp_c = float(parts[5])
        return (timestamp_ms, raw_x, raw_y, raw_z, temp_c)
    except (ValueError, IndexError):
        return None


def find_tempo_port():
    """Find the first serial port matching Tempo-BT VID/PID."""
    for p in serial.tools.list_ports.comports():
        if p.vid == TEMPO_VID and p.pid == TEMPO_PID:
            return p.device
    return None


def serial_reader_thread(ser, samples, stop_event, response_queue):
    """Background thread: read lines, route $PRMG to samples and $PRSP to response_queue.

    Args:
        ser: open serial.Serial object (caller manages open/close)
        samples: list to append parsed $PRMG tuples
        stop_event: threading.Event to signal shutdown
        response_queue: queue.Queue for $PRSP response lines
    """
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except serial.SerialException:
            break
        if not line:
            continue
        if line.startswith("$PRSP"):
            response_queue.put(line)
        else:
            result = parse_prmg(line)
            if result is not None:
                samples.append(result)


def send_command(ser, response_queue, cmd_body, timeout=3.0):
    """Send a $PCMD sentence and wait for the $PRSP response.

    Args:
        ser: open serial.Serial object
        response_queue: queue.Queue receiving $PRSP lines from reader thread
        cmd_body: command content after "$PCMD," (e.g., "CAL_GET")
        timeout: seconds to wait for response

    Returns:
        Parsed response body (between "$PRSP," and "*") or None on timeout.
    """
    # Build sentence with NMEA checksum
    sentence = f"$PCMD,{cmd_body}"
    checksum = 0
    for c in sentence[1:]:  # skip '$'
        checksum ^= ord(c)
    full = f"{sentence}*{checksum:02X}\r\n"

    # Drain any stale responses
    while not response_queue.empty():
        try:
            response_queue.get_nowait()
        except queue.Empty:
            break

    # Send
    ser.write(full.encode("ascii"))

    # Wait for response
    try:
        resp = response_queue.get(timeout=timeout)
        if resp.startswith("$PRSP,"):
            star = resp.rfind("*")
            if star > 0:
                return resp[6:star]
            return resp[6:]
        return resp
    except queue.Empty:
        return None


def upload_calibration(ser, response_queue, cal):
    """Upload calibration data to device and set mag_mode=2.

    Returns (success: bool, messages: list[str]).
    """
    messages = []

    # Send CAL_SET
    cmd = (f"CAL_SET,{cal['offset_x']},{cal['offset_y']},{cal['offset_z']},"
           f"{cal['scale_x']},{cal['scale_y']},{cal['scale_z']}")
    resp = send_command(ser, response_queue, cmd)
    if resp is None:
        messages.append("ERROR: No response (timeout)")
        return False, messages
    if resp != "CAL_SET,OK":
        messages.append(f"ERROR: CAL_SET: {resp}")
        return False, messages
    messages.append("Calibration uploaded to NVM")

    # Send MODE_SET,2
    resp = send_command(ser, response_queue, "MODE_SET,2")
    if resp is None:
        messages.append("ERROR: No response (timeout)")
        return False, messages
    if resp != "MODE_SET,OK":
        messages.append(f"ERROR: MODE_SET: {resp}")
        return False, messages
    messages.append("Mag mode set to 2 (NVM cal)")

    # Verify with CAL_GET
    resp = send_command(ser, response_queue, "CAL_GET")
    if resp:
        messages.append(f"Verify: {resp}")

    return True, messages


def run_calibration_app(port, baud, min_samples):
    """Single-window calibration app: live collection then results view.

    Opens a persistent window with an instruction panel on the left and a
    live 3D scatter on the right.  Auto-detects when $PRMG data starts
    flowing.  The user clicks "Done" to stop collection, at which point the
    calibration is computed and the window transitions to a raw-vs-calibrated
    results view.

    Returns (cal_dict, samples) — cal_dict is None if insufficient data.
    """
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button

    # Open serial port (kept open for entire session including upload)
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"ERROR: Could not open {port}: {e}")
        return None, []

    samples = []
    stop_event = threading.Event()
    response_queue = queue.Queue()
    result = {}

    # ── main window ──────────────────────────────────────────────────
    fig = plt.figure(figsize=(15, 8))
    fig.canvas.manager.set_window_title("Tempo-BT Magnetometer Calibration")

    # Left info panel (frameless axes for text)
    ax_info = fig.add_axes([0.01, 0.12, 0.23, 0.85])
    ax_info.set_axis_off()

    instructions = (
        "Magnetometer Calibration\n"
        "========================\n\n"
        "1. Press BTN2 on the\n"
        "   Tempo-BT to start\n"
        "   calibration streaming.\n\n"
        "2. Slowly rotate the device\n"
        "   through all orientations:\n"
        "   - face up / face down\n"
        "   - each edge pointing up\n"
        "   - each corner pointing up\n"
        "   - smooth diagonal sweeps\n\n"
        "3. Watch the 3D scatter plot.\n"
        "   Aim for 8/8 octants.\n\n"
        "4. Click [Done] when the\n"
        "   plot title turns green."
    )
    ax_info.text(0.05, 0.98, instructions,
                 transform=ax_info.transAxes,
                 fontsize=9, fontfamily="monospace",
                 verticalalignment="top")

    status_text = ax_info.text(
        0.05, 0.28, "Waiting for data...",
        transform=ax_info.transAxes,
        fontsize=10, fontfamily="monospace",
        fontweight="bold", verticalalignment="top",
        color="gray")

    octant_text = ax_info.text(
        0.05, 0.01, "",
        transform=ax_info.transAxes,
        fontsize=8, fontfamily="monospace",
        verticalalignment="bottom")

    # Done button
    ax_btn = fig.add_axes([0.03, 0.02, 0.18, 0.06])
    btn = Button(ax_btn, "Done", color="#dddddd", hovercolor="#90EE90")

    # 3D live scatter (right portion of window)
    ax_live = fig.add_axes([0.30, 0.08, 0.65, 0.85], projection="3d")
    ax_live.set_xlabel("X (counts)")
    ax_live.set_ylabel("Y (counts)")
    ax_live.set_zlabel("Z (counts)")
    ax_live.set_title("Waiting for magnetometer data...", fontsize=12)

    phase = {"current": "waiting"}  # waiting -> collecting -> computing -> results

    # ── serial reader ────────────────────────────────────────────────
    print(f"Opened {port} at {baud} baud")
    reader = threading.Thread(
        target=serial_reader_thread,
        args=(ser, samples, stop_event, response_queue),
        daemon=True)
    reader.start()

    # ── button callback ──────────────────────────────────────────────
    def on_done(event):
        if phase["current"] in ("waiting", "collecting"):
            phase["current"] = "computing"

    btn.on_clicked(on_done)

    prev_len = 0

    # ── animation update (called at ~5 Hz) ───────────────────────────
    def update(_frame):
        nonlocal prev_len

        if phase["current"] == "computing":
            phase["current"] = "transitioning"
            anim.event_source.stop()
            # Keep reader thread alive — needed for upload commands
            _transition_to_results()
            return ()

        n = len(samples)

        # Auto-detect data start
        if n > 0 and phase["current"] == "waiting":
            phase["current"] = "collecting"

        if n == prev_len:
            return ()
        prev_len = n

        data = np.array([(s[1], s[2], s[3]) for s in samples[:n]],
                        dtype=np.float64)

        # Redraw scatter
        ax_live.cla()
        ax_live.scatter(data[:, 0], data[:, 1], data[:, 2],
                        c="b", s=2, alpha=0.4)
        ax_live.set_xlabel("X (counts)")
        ax_live.set_ylabel("Y (counts)")
        ax_live.set_zlabel("Z (counts)")

        # Equal-aspect bounding box
        mid = np.mean(data, axis=0)
        max_range = max(np.max(np.abs(data - mid)) * 1.1, 1)
        ax_live.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax_live.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax_live.set_zlim(mid[2] - max_range, mid[2] + max_range)

        # Octant coverage
        center_est = (data.min(axis=0) + data.max(axis=0)) / 2
        centered = data - center_est
        labels = ["+X+Y+Z", "+X+Y-Z", "+X-Y+Z", "+X-Y-Z",
                  "-X+Y+Z", "-X+Y-Z", "-X-Y+Z", "-X-Y-Z"]
        signs = [(1, 1, 1), (1, 1, -1), (1, -1, 1), (1, -1, -1),
                 (-1, 1, 1), (-1, 1, -1), (-1, -1, 1), (-1, -1, -1)]
        counts = []
        for sx, sy, sz in signs:
            mask = ((centered[:, 0] * sx > 0) &
                    (centered[:, 1] * sy > 0) &
                    (centered[:, 2] * sz > 0))
            counts.append(int(mask.sum()))
        filled = sum(1 for c in counts if c >= 5)

        # Update status text
        if n >= min_samples and filled == 8:
            status_text.set_text(
                f"Samples: {n}\nOctants: {filled}/8\n\n"
                "Ready -- click Done")
            status_text.set_color("green")
        elif n >= min_samples:
            status_text.set_text(
                f"Samples: {n}\nOctants: {filled}/8\n\n"
                "Fill more octants...")
            status_text.set_color("orange")
        else:
            status_text.set_text(
                f"Samples: {n}\nOctants: {filled}/8\n\n"
                "Collecting...")
            status_text.set_color("blue")

        # Octant detail
        olines = []
        for lbl, cnt in zip(labels, counts):
            tag = "OK" if cnt >= 5 else ".." if cnt > 0 else "  "
            olines.append(f"{lbl}:{cnt:3d} [{tag}]")
        octant_text.set_text("\n".join(olines))

        color = "green" if filled == 8 else "orange" if filled >= 6 else "red"
        ax_live.set_title(f"{n} samples -- {filled}/8 octants",
                          color=color, fontsize=12)
        return ()

    # ── transition to results view ───────────────────────────────────
    def _transition_to_results():
        n = len(samples)
        if n < min_samples:
            status_text.set_text(
                f"Only {n} samples\n(need {min_samples})\n\n"
                "Close window to exit.")
            status_text.set_color("red")
            fig.canvas.draw()
            return

        status_text.set_text("Computing\ncalibration...")
        status_text.set_color("blue")
        fig.canvas.draw()
        fig.canvas.flush_events()

        try:
            cal, raw_data, corrected_data = compute_calibration(samples)
        except Exception as e:
            status_text.set_text(
                f"Calibration failed:\n{e}\n\nClose window to exit.")
            status_text.set_color("red")
            fig.canvas.draw()
            return

        result["cal"] = cal
        result["raw_data"] = raw_data
        result["corrected_data"] = corrected_data

        # Remove collection axes and Done button
        ax_live.remove()
        ax_btn.remove()

        # Create two side-by-side 3D result axes
        ax_raw = fig.add_axes([0.27, 0.08, 0.33, 0.82], projection="3d")
        ax_cal = fig.add_axes([0.63, 0.08, 0.33, 0.82], projection="3d")

        # Raw data (ellipsoid)
        ax_raw.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2],
                       c="b", s=1, alpha=0.3)
        ax_raw.scatter([cal["offset_x"]], [cal["offset_y"]],
                       [cal["offset_z"]], c="r", s=100, marker="+")
        ax_raw.set_xlabel("X (counts)")
        ax_raw.set_ylabel("Y (counts)")
        ax_raw.set_zlabel("Z (counts)")
        ax_raw.set_title("Raw Data (ellipsoid)")

        # Calibrated data (sphere)
        ax_cal.scatter(corrected_data[:, 0], corrected_data[:, 1],
                       corrected_data[:, 2], c="g", s=1, alpha=0.3)
        ax_cal.set_xlabel("X (counts)")
        ax_cal.set_ylabel("Y (counts)")
        ax_cal.set_zlabel("Z (counts)")
        ax_cal.set_title("Calibrated Data (sphere)")

        # Equal-aspect bounding box for both
        for ax, d in [(ax_raw, raw_data), (ax_cal, corrected_data)]:
            mid = np.mean(d, axis=0)
            mr = max(np.max(np.abs(d - mid)) * 1.1, 1)
            ax.set_xlim(mid[0] - mr, mid[0] + mr)
            ax.set_ylim(mid[1] - mr, mid[1] + mr)
            ax.set_zlim(mid[2] - mr, mid[2] + mr)

        # Synchronize rotation between the two 3D axes
        def on_move(event):
            if event.inaxes is ax_raw:
                ax_cal.view_init(elev=ax_raw.elev, azim=ax_raw.azim)
            elif event.inaxes is ax_cal:
                ax_raw.view_init(elev=ax_cal.elev, azim=ax_cal.azim)
            fig.canvas.draw_idle()

        fig.canvas.mpl_connect("motion_notify_event", on_move)

        # Update left panel with results summary
        ax_info.clear()
        ax_info.set_axis_off()

        summary = (
            "Calibration Results\n"
            "====================\n\n"
            f"Samples: {cal['num_samples']}\n\n"
            "Hard-iron offsets:\n"
            f"  X: {cal['offset_x']:>7d}\n"
            f"  Y: {cal['offset_y']:>7d}\n"
            f"  Z: {cal['offset_z']:>7d}\n\n"
            "Soft-iron scale (Q1.15):\n"
            f"  X: {cal['scale_x']:>7d}\n"
            f"  Y: {cal['scale_y']:>7d}\n"
            f"  Z: {cal['scale_z']:>7d}\n\n"
            f"Field magnitude:\n"
            f"  {cal['mean_magnitude_ut']:.1f}"
            f" +/- {cal['std_magnitude_ut']:.2f} uT\n\n"
            "Click [Upload] to send\n"
            "calibration to device.\n"
            "Close window when done."
        )
        ax_info.text(0.05, 0.98, summary,
                     transform=ax_info.transAxes,
                     fontsize=9, fontfamily="monospace",
                     verticalalignment="top")

        # Upload status text (below summary)
        upload_text = ax_info.text(
            0.05, 0.05, "",
            transform=ax_info.transAxes,
            fontsize=8, fontfamily="monospace",
            verticalalignment="bottom")

        # Upload to Device button
        ax_upload = fig.add_axes([0.03, 0.02, 0.18, 0.06])
        btn_upload = Button(ax_upload, "Upload to Device",
                            color="#dddddd", hovercolor="#90EE90")

        def on_upload(event):
            btn_upload.label.set_text("Uploading...")
            fig.canvas.draw()
            fig.canvas.flush_events()

            success, msgs = upload_calibration(ser, response_queue, cal)

            upload_text.set_text("\n".join(msgs))
            upload_text.set_color("green" if success else "red")
            btn_upload.label.set_text("Done!" if success else "Failed")
            btn_upload.color = "#90EE90" if success else "#FFB0B0"
            fig.canvas.draw()

        btn_upload.on_clicked(on_upload)

        fig.suptitle(
            f"Calibration: {cal['num_samples']} samples, "
            f"field = {cal['mean_magnitude_ut']:.1f}"
            f" +/- {cal['std_magnitude_ut']:.2f} uT",
            fontsize=11)

        phase["current"] = "results"
        fig.canvas.draw()

    # ── run ───────────────────────────────────────────────────────────
    anim = FuncAnimation(fig, update, interval=200, blit=False,
                         cache_frame_data=False)
    try:
        plt.show()  # blocks until the window is closed
    except KeyboardInterrupt:
        pass

    # Shut down serial reader and close port
    stop_event.set()
    if reader.is_alive():
        reader.join(timeout=3)
    ser.close()

    print(f"\nCollection complete: {len(samples)} samples")
    return result.get("cal"), samples


def collect_from_file(filepath):
    """Load samples from a file (NMEA $PRMG sentences or CSV format).

    CSV format: timestamp_ms,raw_x,raw_y,raw_z,temp_c (with header line)
    """
    samples = []
    with open(filepath) as f:
        for lineno, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            # Try NMEA first
            result = parse_prmg(line)
            if result is not None:
                samples.append(result)
                continue
            # Try CSV (skip header)
            if lineno == 0 and line.startswith("timestamp"):
                continue
            parts = line.split(",")
            if len(parts) >= 5:
                try:
                    samples.append((
                        int(parts[0]), int(parts[1]), int(parts[2]),
                        int(parts[3]), float(parts[4]),
                    ))
                except (ValueError, IndexError):
                    pass
    print(f"Loaded {len(samples)} samples from {filepath}")
    return samples


def ellipsoid_fit(data):
    """Fit an axis-aligned ellipsoid using nonlinear least-squares.

    Finds center (cx, cy, cz) and radii (rx, ry, rz) that minimize the
    residual of each point's normalized distance from the ellipsoid surface:
        sqrt(((x-cx)/rx)^2 + ((y-cy)/ry)^2 + ((z-cz)/rz)^2) - 1

    Seeded from min/max midpoints, which is already a reasonable estimate
    for magnetometer data.  Much more robust than the unconstrained algebraic
    fit, which can converge on hyperboloids or degenerate quadrics when
    sphere coverage is incomplete.

    Returns:
        center: (3,) array of ellipsoid center (hard-iron offsets)
        radii: (3,) array of semi-axis lengths
    """
    x = data[:, 0]
    y = data[:, 1]
    z = data[:, 2]

    # Seed from min/max midpoints
    cx0 = (x.min() + x.max()) / 2
    cy0 = (y.min() + y.max()) / 2
    cz0 = (z.min() + z.max()) / 2
    rx0 = (x.max() - x.min()) / 2
    ry0 = (y.max() - y.min()) / 2
    rz0 = (z.max() - z.min()) / 2

    # Guard against zero range on any axis
    for r in [rx0, ry0, rz0]:
        if r < 1.0:
            raise ValueError("Data has zero range on one or more axes")

    p0 = np.array([cx0, cy0, cz0, rx0, ry0, rz0])

    def residuals(p):
        cx, cy, cz, rx, ry, rz = p
        dx = (x - cx) / rx
        dy = (y - cy) / ry
        dz = (z - cz) / rz
        return np.sqrt(dx**2 + dy**2 + dz**2) - 1.0

    result = least_squares(residuals, p0, method='lm')

    center = result.x[:3]
    radii = np.abs(result.x[3:])

    return center, radii


def assess_coverage(data, center, radii):
    """Assess how well the data covers the ellipsoid surface.

    Divides the sphere into octants and checks for data in each one.
    Returns (fraction_covered, details_string).
    """
    centered = data - center
    # Normalize to unit sphere
    normalized = centered / radii

    # Check 8 octants (sign combinations of x, y, z)
    octant_labels = [
        "+X+Y+Z", "+X+Y-Z", "+X-Y+Z", "+X-Y-Z",
        "-X+Y+Z", "-X+Y-Z", "-X-Y+Z", "-X-Y-Z",
    ]
    octant_counts = []
    for sx in [1, -1]:
        for sy in [1, -1]:
            for sz in [1, -1]:
                mask = ((normalized[:, 0] * sx > 0) &
                        (normalized[:, 1] * sy > 0) &
                        (normalized[:, 2] * sz > 0))
                octant_counts.append(mask.sum())

    filled = sum(1 for c in octant_counts if c >= 5)
    fraction = filled / 8.0

    # Per-axis range as fraction of expected diameter
    axis_coverage = []
    for i, label in enumerate(["X", "Y", "Z"]):
        span = data[:, i].max() - data[:, i].min()
        expected = 2 * radii[i]
        pct = span / expected * 100 if expected > 0 else 0
        axis_coverage.append((label, pct))

    return fraction, octant_counts, octant_labels, axis_coverage


def compute_calibration(samples):
    """Compute hard-iron and soft-iron calibration from raw samples.

    Args:
        samples: list of (timestamp, raw_x, raw_y, raw_z, temp) tuples

    Returns:
        dict with calibration parameters
    """
    # Extract raw XYZ data
    data = np.array([(s[1], s[2], s[3]) for s in samples], dtype=np.float64)

    print(f"\nComputing calibration from {len(data)} samples...")
    print(f"  Raw ranges: X=[{data[:,0].min():.0f}, {data[:,0].max():.0f}]  span={data[:,0].max()-data[:,0].min():.0f}")
    print(f"              Y=[{data[:,1].min():.0f}, {data[:,1].max():.0f}]  span={data[:,1].max()-data[:,1].min():.0f}")
    print(f"              Z=[{data[:,2].min():.0f}, {data[:,2].max():.0f}]  span={data[:,2].max()-data[:,2].min():.0f}")

    # Min/max seed estimate (for reference)
    mm_center = np.array([(data[:,i].min() + data[:,i].max()) / 2 for i in range(3)])
    print(f"\n  Min/max center estimate: [{mm_center[0]:.0f}, {mm_center[1]:.0f}, {mm_center[2]:.0f}]")

    # Fit ellipsoid
    center, radii = ellipsoid_fit(data)

    print(f"  Fitted center (hard-iron): [{center[0]:.1f}, {center[1]:.1f}, {center[2]:.1f}] counts")
    print(f"  Fitted radii:              [{radii[0]:.1f}, {radii[1]:.1f}, {radii[2]:.1f}] counts")

    # Sanity: check center is within data range
    for i, axis in enumerate(["X", "Y", "Z"]):
        if center[i] < data[:, i].min() or center[i] > data[:, i].max():
            print(f"  WARNING: Fitted {axis} center ({center[i]:.0f}) is outside data range "
                  f"[{data[:,i].min():.0f}, {data[:,i].max():.0f}]")

    # Radii ratio check
    max_ratio = radii.max() / radii.min()
    if max_ratio > 1.5:
        print(f"  WARNING: Radii ratio {max_ratio:.2f} is high (expect < 1.3 for typical soft-iron)")

    # Coverage assessment
    frac, octant_counts, octant_labels, axis_cov = assess_coverage(data, center, radii)
    print(f"\n  Sphere coverage: {frac*100:.0f}% of octants ({sum(1 for c in octant_counts if c >= 5)}/8)")
    for label, count in zip(octant_labels, octant_counts):
        marker = "ok" if count >= 5 else "SPARSE" if count > 0 else "EMPTY"
        print(f"    {label}: {count:4d} pts  [{marker}]")
    for label, pct in axis_cov:
        marker = "ok" if pct > 70 else "LOW"
        print(f"  {label}-axis range: {pct:.0f}% of expected diameter  [{marker}]")

    if frac < 0.75:
        print("\n  ** Incomplete sphere coverage will degrade calibration quality.")
        print("  ** Rotate the device through more orientations for better results.")

    # Compute scale factors (normalize to average radius)
    avg_radius = np.mean(radii)
    scale_factors = avg_radius / radii  # >1 means axis is compressed
    print(f"\n  Scale factors: [{scale_factors[0]:.4f}, {scale_factors[1]:.4f}, {scale_factors[2]:.4f}]")

    # Convert to Q1.15 fixed-point (32768 = 1.0)
    scale_q15 = np.round(scale_factors * 32768).astype(int)
    scale_q15 = np.clip(scale_q15, 0, 65535)

    # Hard-iron offsets (integer counts)
    offsets = np.round(center).astype(int)

    # Apply calibration and check quality
    corrected = data - center
    # Apply diagonal scale correction
    corrected[:, 0] *= scale_factors[0]
    corrected[:, 1] *= scale_factors[1]
    corrected[:, 2] *= scale_factors[2]

    # Convert to Gauss for magnitude check
    corrected_gauss = corrected / COUNTS_PER_GAUSS
    magnitudes = np.sqrt(np.sum(corrected_gauss**2, axis=1))

    # Also compute raw magnitudes for comparison
    raw_centered = data - center
    raw_gauss = raw_centered / COUNTS_PER_GAUSS
    raw_magnitudes = np.sqrt(np.sum(raw_gauss**2, axis=1))

    # Quality metrics
    mean_magnitude = np.mean(magnitudes) * GAUSS_TO_UT
    std_magnitude = np.std(magnitudes) * GAUSS_TO_UT
    raw_std = np.std(raw_magnitudes) * GAUSS_TO_UT

    print(f"\n  Calibrated field magnitude: {mean_magnitude:.2f} +/- {std_magnitude:.2f} uT")
    print(f"  Raw field magnitude std:    {raw_std:.2f} uT")
    if std_magnitude > 0:
        print(f"  Improvement:                {raw_std/std_magnitude:.1f}x")

    # Expected Earth field range: 25-65 uT
    if 20 < mean_magnitude < 70:
        print(f"  Field magnitude looks reasonable for Earth's field")
    else:
        print(f"  WARNING: Field magnitude {mean_magnitude:.1f} uT is outside expected range (25-65 uT)")

    cal_result = {
        "offset_x": int(offsets[0]),
        "offset_y": int(offsets[1]),
        "offset_z": int(offsets[2]),
        "scale_x": int(scale_q15[0]),
        "scale_y": int(scale_q15[1]),
        "scale_z": int(scale_q15[2]),
        "scale_float": scale_factors.tolist(),
        "avg_radius_counts": float(avg_radius),
        "mean_magnitude_ut": float(mean_magnitude),
        "std_magnitude_ut": float(std_magnitude),
        "num_samples": len(data),
        "ellipsoid_center": center.tolist(),
        "ellipsoid_radii": radii.tolist(),
    }

    return cal_result, data, corrected


def visualize(raw_data, corrected_data, cal):
    """Show 3D scatter plots of raw and calibrated data."""
    fig = plt.figure(figsize=(14, 6))

    # Raw data
    ax1 = fig.add_subplot(121, projection="3d")
    ax1.scatter(
        raw_data[:, 0],
        raw_data[:, 1],
        raw_data[:, 2],
        c="b",
        s=1,
        alpha=0.3,
    )
    ax1.set_xlabel("X (counts)")
    ax1.set_ylabel("Y (counts)")
    ax1.set_zlabel("Z (counts)")
    ax1.set_title("Raw Data (ellipsoid)")

    # Mark center
    ax1.scatter(
        [cal["offset_x"]],
        [cal["offset_y"]],
        [cal["offset_z"]],
        c="r",
        s=100,
        marker="+",
    )

    # Calibrated data
    ax2 = fig.add_subplot(122, projection="3d")
    ax2.scatter(
        corrected_data[:, 0],
        corrected_data[:, 1],
        corrected_data[:, 2],
        c="g",
        s=1,
        alpha=0.3,
    )
    ax2.set_xlabel("X (counts)")
    ax2.set_ylabel("Y (counts)")
    ax2.set_zlabel("Z (counts)")
    ax2.set_title("Calibrated Data (sphere)")

    # Make axes equal
    for ax in [ax1, ax2]:
        all_data = raw_data if ax == ax1 else corrected_data
        max_range = (
            np.max(np.abs(all_data - np.mean(all_data, axis=0))) * 1.1
        )
        mid = np.mean(all_data, axis=0)
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)

    plt.suptitle(
        f"Magnetometer Calibration: {cal['num_samples']} samples, "
        f"field = {cal['mean_magnitude_ut']:.1f} +/- {cal['std_magnitude_ut']:.2f} uT"
    )
    plt.tight_layout()

    # Synchronize rotation between the two 3D axes
    def on_move(event):
        if event.inaxes is ax1:
            ax2.view_init(elev=ax1.elev, azim=ax1.azim)
        elif event.inaxes is ax2:
            ax1.view_init(elev=ax2.elev, azim=ax2.azim)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_move)

    plt.show()


def export_results(cal, output_path):
    """Export calibration results in multiple formats."""
    # JSON
    json_path = output_path.with_suffix(".json")
    with open(json_path, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"\nJSON output: {json_path}")

    # C header / devicetree properties
    print("\n--- DeviceTree properties (add to mmc5983ma node) ---")
    print(f"    cal-offset-x = <{cal['offset_x']}>;")
    print(f"    cal-offset-y = <{cal['offset_y']}>;")
    print(f"    cal-offset-z = <{cal['offset_z']}>;")
    print(f"    cal-scale-x = <{cal['scale_x']}>;")
    print(f"    cal-scale-y = <{cal['scale_y']}>;")
    print(f"    cal-scale-z = <{cal['scale_z']}>;")

    # NVS binary blob (matches mag_calibration_t struct layout)
    # struct: int32 offset_x/y/z, uint16 scale_x/y/z, bool valid
    print("\n--- mag_calibration_t struct values ---")
    print(f"    .offset_x = {cal['offset_x']},")
    print(f"    .offset_y = {cal['offset_y']},")
    print(f"    .offset_z = {cal['offset_z']},")
    print(f"    .scale_x = {cal['scale_x']},")
    print(f"    .scale_y = {cal['scale_y']},")
    print(f"    .scale_z = {cal['scale_z']},")
    print(f"    .valid = true,")

    # Binary blob for NVS
    bin_path = output_path.with_suffix(".bin")
    with open(bin_path, "wb") as f:
        f.write(struct.pack("<iii", cal["offset_x"], cal["offset_y"], cal["offset_z"]))
        f.write(struct.pack("<HHH", cal["scale_x"], cal["scale_y"], cal["scale_z"]))
        f.write(struct.pack("<B", 1))  # valid = true
    print(f"\nBinary blob: {bin_path} ({bin_path.stat().st_size} bytes)")


def main():
    parser = argparse.ArgumentParser(
        description="Tempo-BT Magnetometer Calibration Tool"
    )
    parser.add_argument(
        "--port", "-p",
        type=str,
        default=None,
        help="Serial port (e.g., /dev/ttyACM0). Auto-detects Tempo-BT if omitted.",
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--input", type=str, help="Load data from file instead of serial port"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="mag_cal",
        help="Output file base name (default: mag_cal)",
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=200,
        help="Minimum samples required (default: 200)",
    )
    args = parser.parse_args()

    output_path = Path(args.output)

    if args.input:
        # File mode: load, compute, visualize (separate windows)
        samples = collect_from_file(args.input)

        if len(samples) < args.min_samples:
            print(f"\nError: Only {len(samples)} samples "
                  f"(minimum {args.min_samples} required)")
            sys.exit(1)

        cal, raw_data, corrected_data = compute_calibration(samples)
        export_results(cal, output_path)
        visualize(raw_data, corrected_data, cal)
    else:
        # Serial mode: single-window calibration app
        port = args.port
        if port is None:
            port = find_tempo_port()
        if port is None:
            print("ERROR: No Tempo-BT device found (VID=0x%04X, PID=0x%04X)." %
                  (TEMPO_VID, TEMPO_PID))
            print("       Connect Tempo-BT via USB or specify port with --port.")
            sys.exit(1)

        cal, samples = run_calibration_app(port, args.baud, args.min_samples)

        # Save raw data
        if samples:
            raw_path = output_path.with_suffix(".csv")
            with open(raw_path, "w") as f:
                f.write("timestamp_ms,raw_x,raw_y,raw_z,temp_c\n")
                for s in samples:
                    f.write(f"{s[0]},{s[1]},{s[2]},{s[3]},{s[4]:.1f}\n")
            print(f"Raw data saved: {raw_path}")

        if cal:
            # Calibration was computed in-window
            export_results(cal, output_path)
        elif len(samples) >= args.min_samples:
            # User closed window early; compute and export anyway
            cal, raw_data, corrected_data = compute_calibration(samples)
            export_results(cal, output_path)
        else:
            print(f"\nError: Only {len(samples)} samples collected "
                  f"(minimum {args.min_samples} required)")
            sys.exit(1)


if __name__ == "__main__":
    main()
