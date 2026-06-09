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
- Outer radius: 19 cm, wheel distance L: 14.4 cm, wheel radius r: 4.1 cm
- Inverse kinematics: `V_i = Vx·cos(θ_i) + Vy·sin(θ_i) + ω·L`
- Wheel angles: V1=150°, V2=270°, V3=30°

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
