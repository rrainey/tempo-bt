# Tempo-BT V1 - Flight Detection and State Machine Algorithms

## Overview

This document describes the algorithms used in the Tempo-BT V1 Zephyr implementation for flight phase detection and automatic state management. The system uses a combination of barometric pressure sensing and accelerometer data to detect takeoff, freefall, and landing events.

## State Machine

### State Definitions

```c
typedef enum {
    LOGGER_STATE_IDLE,       // Device idle, not monitoring
    LOGGER_STATE_ARMED,      // Monitoring for takeoff
    LOGGER_STATE_LOGGING,    // In-flight, recording data
    LOGGER_STATE_JUMPED,     // Freefall/canopy descent detected
    LOGGER_STATE_POSTFLIGHT, // Post-flight processing
    LOGGER_STATE_ERROR       // Error condition
} logger_state_t;
```

### State Transition Diagram

```
                    ┌──────────────┐
                    │     IDLE     │
                    └──────┬───────┘
                           │ logger_arm()
                    ┌──────▼───────┐
              ┌────►│    ARMED     │◄────────────────────┐
              │     └──────┬───────┘                     │
              │            │ Climb rate > 200 ft/min     │
              │            │ for 3 consecutive seconds   │
              │     ┌──────▼───────┐                     │
              │     │   LOGGING    │                     │
              │     └──────┬───────┘                     │
              │            │ Freefall detected:          │
              │            │ • Baro: < -1000 ft/min 2s   │
              │            │ • Accel: < 0.6g for 500ms   │
              │     ┌──────▼───────┐                     │
              │     │    JUMPED    │                     │
              │     └──────┬───────┘                     │
              │            │ Low activity for 60s        │
              └────────────┴─────────────────────────────┘
                           │
              (6 min timeout in LOGGING also returns to ARMED)
```

### Transition Thresholds

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EXEC_TAKEOFF_CLIMB_RATE_MPS` | 1.016 m/s | 200 ft/min - triggers ARMED→LOGGING |
| `EXEC_FREEFALL_DESCENT_RATE_MPS` | -5.08 m/s | -1000 ft/min - triggers LOGGING→JUMPED (baro method) |
| `EXEC_FREEFALL_ACCEL_THRESHOLD_G` | 0.6g | Low-g threshold for freefall (accel method) |
| `EXEC_LOW_ACTIVITY_RATE_MPS` | 1.016 m/s | |200 ft/min| - low activity threshold |
| `EXEC_JUMPED_TIMEOUT_S` | 60s | Low activity duration to detect landing |
| `EXEC_LOGGING_ABORT_TIMEOUT_S` | 360s | 6 min low activity in LOGGING aborts session |

### Executive Function

The executive function (`logger_executive()`) runs every **250ms** and manages automatic state transitions. This faster interval (compared to the original 1s) enables more responsive freefall detection.

## Climb Rate Estimation

### Alpha-Beta Filter

The system uses an alpha-beta filter (also known as a g-h filter) for coupled altitude/velocity estimation from barometric pressure data. This approach provides better transient rejection than simple derivative-based methods.

**Filter Parameters:**
- `ALPHA_BETA_ALPHA = 0.15` - Position (altitude) correction weight
- `ALPHA_BETA_BETA = 0.005` - Velocity (climb rate) correction weight

These values provide approximately 1-2 second time constant for good transient rejection while maintaining reasonable responsiveness.

**Algorithm:**

```c
// Predict step: advance state using current velocity estimate
altitude_pred = altitude_est + climb_rate_est * dt;

// Update step: correct prediction using measurement residual
residual = measured_altitude - altitude_pred;
altitude_est = altitude_pred + ALPHA * residual;
climb_rate_est = climb_rate_est + (BETA / dt) * residual;
```

### Median Pre-Filter

Before the alpha-beta filter, a 3-sample median filter is applied to reject single-sample outliers in the barometric altitude measurements:

```c
filtered_altitude = median3(buffer[0], buffer[1], buffer[2]);
```

### Data Flow

```
Barometric       Median         Alpha-Beta        Executive
Altitude    ──►  Pre-filter ──► Filter       ──►  Function
(~50 Hz)         (3 samples)    (position +       (250ms interval)
                                velocity)
```

## Freefall Detection

The system implements two parallel methods for detecting freefall, with the accelerometer method providing faster response:

### Method 1: Barometric (Descent Rate)

- **Threshold:** Descent rate exceeds 1000 ft/min (5.08 m/s)
- **Confirmation:** 2 consecutive seconds (8 executive ticks)
- **Latency:** ~2 seconds from jump initiation

### Method 2: Accelerometer (Low-G)

- **Threshold:** Filtered acceleration magnitude < 0.6g
- **Confirmation:** 2 consecutive readings at 250ms intervals (500ms total)
- **Latency:** ~500ms from jump initiation

The accelerometer method triggers state transition to JUMPED significantly faster, allowing earlier capture of high-rate GNSS position data during freefall.

### Accelerometer Magnitude Filtering

The IMU service maintains an EMA-filtered acceleration magnitude when enabled:

**Filter Parameters:**
- Sample rate: 200 Hz (ICM42688 FIFO)
- `ACCEL_MAG_FILTER_ALPHA = 0.15` - ~33ms time constant

**Algorithm:**

```c
// Compute magnitude in g
magnitude_g = sqrt(ax_g² + ay_g² + az_g²);

// EMA filter update
filtered_magnitude = α * magnitude + (1-α) * filtered_magnitude_prev;
```

The filter is enabled when entering LOGGING state and disabled when logging stops, minimizing computational overhead during non-flight periods.

## Ground Altitude Tracking

### Continuous Sampling

During IDLE and ARMED states, the system periodically samples barometric altitude to maintain an accurate ground reference:

- **Sampling interval:** Every 5 minutes (300 seconds)
- **Storage:** Moving average maintained by baro service

### Initial Calibration

When arming, the current barometric altitude is captured as the initial ground reference if not already set.

## Takeoff Detection

Takeoff is detected by monitoring for sustained positive climb rate:

- **Threshold:** > 200 ft/min (1.016 m/s)
- **Confirmation:** 3 consecutive seconds (12 executive ticks at 250ms)
- **Result:** Automatic transition from ARMED to LOGGING

## Landing Detection

Landing is detected by monitoring for sustained low vertical activity:

- **Threshold:** |climb rate| < 200 ft/min (1.016 m/s)
- **Confirmation:** 60 seconds of continuous low activity
- **Result:** Automatic transition from JUMPED to ARMED

## GNSS Rate Management

The system dynamically adjusts GNSS update rate based on flight phase:

| State | GNSS Rate | Rationale |
|-------|-----------|-----------|
| IDLE/ARMED | 1 Hz | Power conservation |
| LOGGING | 1 Hz | Sufficient for aircraft climb |
| JUMPED | 10 Hz | High-rate position capture during freefall/canopy |

## Timing Summary

| Component | Update Rate | Purpose |
|-----------|-------------|---------|
| IMU FIFO | 200 Hz | Orientation tracking, accel filtering |
| Barometric | ~50 Hz | Altitude/pressure measurement |
| Alpha-beta filter | Per baro sample | Climb rate estimation |
| Accel magnitude filter | Per IMU sample | Freefall detection |
| Executive function | 4 Hz (250ms) | State transition decisions |
| Ground altitude sample | Every 5 min | Ground reference update |

## Implementation Files

| File | Purpose |
|------|---------|
| `src/services/logger.c` | State machine, executive function, baro handler |
| `src/services/imu_icm42688.c` | Acceleration magnitude filter |
| `src/services/baro.c` | Barometric sampling, ground altitude tracking |
| `src/services/orientation.c` | IMU FIFO processing, quaternion estimation |

## References

- Alpha-beta filter theory: [PMC4179067](https://pmc.ncbi.nlm.nih.gov/articles/PMC4179067/)
- ICM42688 datasheet for accelerometer specifications
