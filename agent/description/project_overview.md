# Robot Project Overview

## Goal
Omni-directional robot (3-wheel) running on NVIDIA Jetson. Supports:
- Autonomous navigation with goal-based path following
- Stereo vision 3D mapping (point cloud)
- Browser-based path planning simulation with real-time ZMQ bridge
- Voice / LLM control (in development)

## Hardware
| Component | Role |
|-----------|------|
| NVIDIA Jetson | Main compute, runs all modules |
| ESP32 (Unified Controller) | Motor driver + camera servo PWM over UART |
| 3× Omni-wheels | At 150°, 270°, 30° from center |
| 2× USB cameras (left=0, right=1) | Stereo depth |
| Laptop (WiFi) | Simulation UI + mission planning |

## Robot kinematics
Source of truth: ESP32 firmware (`motivation/esp32_unified_controller/PinConfig.h`
+ `OmniKinematics.h`). These values OVERRIDE any older chassis estimates.
- Wheel radius r: **5.5 cm** (`WHEEL_RADIUS_M = 0.055`, ⌀11 cm wheel)
- Robot radius L (center → wheel contact): **21 cm** (`ROBOT_RADIUS_M = 0.21`)
- Steps per wheel revolution: **12800** (`STEPS_PER_REV`, 200 × 64 microstep)
- Wheel angles α: W1=60°, W2=180°, W3=300°
- Inverse kinematics: `ω_i = (-sin(α_i)·vx + cos(α_i)·vy + L·ω_z) / r`
- Rotation is open-loop: `T <deg>` issues a fixed step count
  `steps/wheel = (L/r) · (deg/360) · STEPS_PER_REV ≈ 135.8 · deg`.
  Odometry is integrated from these commanded steps (no encoder/IMU) → it
  cannot detect physical slip; calibrate estimated→actual by measurement.

## ESP32 Unified Controller (`motivation/esp32_unified_controller/`)
Single firmware handles: motor PWM, encoder odometry, pan/tilt servo.

## Build (all C++ modules)
```bash
mkdir build && cd build && cmake .. && make -j$(nproc)
```

## Run modes
| Module | Command | Notes |
|--------|---------|-------|
| motivation | `./motivation --nav` | Navigation mode, waits for ZMQ goals |
| motivation | `./motivation --demo` | Run preset move sequence |
| stereo-camera | `./stereo_scan --output scan.ply` | Scan + save point cloud |
| simulation | `JETSON_IP=<ip> python app.py` | Browser at http://localhost:5000 |
