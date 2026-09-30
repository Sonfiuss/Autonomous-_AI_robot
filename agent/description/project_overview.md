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
| 3× Omni-wheels | W1 0° (front), W2 120°, W3 240° from center, CCW from +x |
| 2× USB cameras (left=0, right=1) | Stereo depth |
| Laptop (WiFi) | Simulation UI + mission planning |

## Robot kinematics
Source of truth: `project/config/constants.h` (RM), compiled into both the ESP32 firmware and the
Jetson side. These values OVERRIDE any older chassis estimates.
- Wheel radius r: **4.0 cm** (`WHEEL_RADIUS_M = 0.040`, ⌀8 cm wheel, measured by the user 2026-09-27).
  It was 0.055 until then, copied from the old firmware: every distance and turn came out × 0.727,
  which the June "r_eff 4.03 cm slip" measured. Runs record the r they were commanded with
  (`run.json` `plan.wheel_radius_m`; absent = 0.055).
- Robot radius L (center → wheel contact): **22.17 cm** (`ROBOT_RADIUS_M = 0.2217`): the three wheel
  contacts form an equilateral triangle of side 38.4 cm, L = 0.384 / √3 (user, 2026-09-29). It was 0.21
  until then (outer wheel face minus half the width, 2026-09-27): every turn came out × 0.947.
- Steps per wheel revolution: **12800** (`STEPS_PER_REV`, 200 × 64 microstep)
- Wheel angles α: W1=0° (STEP 12/DIR 13), W2=120° (4/5), W3=240° (26/27). +x (forward)
  points at W1, the front wheel. Fixed on the robot 2026-09-25 from two drive tests
  (60/180/300 and 120/240/0 were both wrong). Driving forward: W1 still, W2 negative,
  W3 positive. Firmware `DIR_INVERTED` = false on all three.
- Inverse kinematics: `ω_i = (-sin(α_i)·vx + cos(α_i)·vy + L·ω_z) / r`
- Rotation is open-loop: `T <deg>` issues a fixed step count
  `steps/wheel = (L/r) · (deg/360) · STEPS_PER_REV ≈ 197.1 · deg`.
  Odometry is integrated from these commanded steps (no encoder/IMU) → it
  cannot detect physical slip; calibrate estimated→actual by measurement.
  Measured 2026-09-27 on the gray tile (F ±0.4 m, tape): 93 % of the geometric distance
  (forward 91 %, backward 95 %) — the real slip + roller effective radius.
- Chassis ceilings at these constants (`mc_max_speed`): 0.45 m/s forward, 0.39 m/s sideways,
  1.77 rad/s spinning; accelerations 0.092 m/s², 0.080 m/s², 0.361 rad/s².
- The floor visual odometry reads turns ~10.8 % long (turned by hand one full turn onto tape marks:
  398.4° for 359.5°, 2026-09-29): `vision/vo_scale.json` `turns` → `frame_motion.vo_turn_scale` (0.902)
  corrects every VO turn, as `legs` → `vo_scale` (1.063) corrects its distances. Cause not found yet; the
  camera also has ~1.25° roll that no model includes.

## ESP32 Unified Controller (`motivation/esp32_unified_controller/`)
Single firmware handles: motor PWM, encoder odometry, pan/tilt servo.

## Build (all C++ modules)
```bash
mkdir build && cd build && cmake .. && make -j$(nproc)
```

## Run modes
| Module | Command | Notes |
|--------|---------|-------|
| communication | `python3 communication/demo_drive.py "<lệnh>"` | text / Gemini → steps or a place → robot; `--compensate` measures each leg by VO; "lập bản đồ" / `--map` builds + publishes the run's map |
| realroom | `python3 realroom/app.py [--dry-run]` | the real room's map (published by `vision/drive_map.py --publish`) + chat that drives the real robot (Run / Stop), port 5002 |
| motivation | `./motivation --nav` | Navigation mode, waits for ZMQ goals |
| motivation | `./motivation --demo` | Run preset move sequence |
| stereo-camera | `./stereo_scan --output scan.ply` | Scan + save point cloud |
| simulation | `JETSON_IP=<ip> python app.py` | Browser at http://localhost:5000 |
