# Plan: stereo-camera

_Last updated: 2026-06-10_

---

## Quick-start reference

### 1 — Build (C++ host side)

```bash
cd /home/nvidia/Documents/stereo-camera
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

Targets produced in `build/`:

| Binary | Purpose |
|--------|---------|
| `stereo_scan` | Full 3-thread step-and-hold scanner |
| `servo_ctrl` | Debug: move servo to absolute angle |
| `servo_test` | Interactive velocity-mode servo test |

Run tests:
```bash
cd build && ctest --output-on-failure
```

---

### 2 — Flash ESP32 firmware

**Firmware location:** `motivation/esp32_unified_controller/esp32_unified_controller.ino`

Handles: motors + servos + odometry on one ESP32. Protocol:

| Command | Direction | Meaning |
|---------|-----------|---------|
| `M vx vy w\n` | Jetson→ESP32 | Continuous wheel velocity |
| `F dist spd\n` | Jetson→ESP32 | Move straight |
| `T angle w\n` | Jetson→ESP32 | Rotate |
| `V pan_v tilt_v\n` | Jetson→ESP32 | Servo velocity |
| `MOVE pan tilt\n` | Jetson→ESP32 | Absolute servo position (blocks until OK) |
| `RESET\n` | Jetson→ESP32 | Servo home (0°, 0°) |
| `S\n` | Jetson→ESP32 | Stop all |
| `R\n` | Jetson→ESP32 | Reset odometry |
| `OK\n` | ESP32→Jetson | Confirms MOVE/RESET done (after 500ms settle) |
| `P pan tilt\n` | ESP32→Jetson | Servo angles at 50 Hz |
| `O x y theta\n` | ESP32→Jetson | Odometry at 10 Hz |
| `READY\n` | ESP32→Jetson | Boot complete |

**Compile + upload (one command):**

```bash
# Compile
arduino-cli compile \
  --fqbn esp32:esp32:esp32 \
  /home/nvidia/Documents/motivation/esp32_unified_controller

# Upload
arduino-cli upload \
  --fqbn esp32:esp32:esp32 \
  --port /dev/ttyUSB0 \
  /home/nvidia/Documents/motivation/esp32_unified_controller
```

Or combined:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32 \
  /home/nvidia/Documents/motivation/esp32_unified_controller && \
arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 \
  /home/nvidia/Documents/motivation/esp32_unified_controller
```

**Verify ESP32 is running:**
```bash
stty -F /dev/ttyUSB0 115200 raw -echo && timeout 2 cat /dev/ttyUSB0
# Expected: stream of "P 0.00 0.00" and "O 0.000 0.000 0.00"
```

---

### 3 — Control servo

**Shell wrapper (simplest):**
```bash
cd /home/nvidia/Documents/stereo-camera

./scripts/servo.sh 0 0          # reset to origin
./scripts/servo.sh 45 -20       # pan=45° tilt=-20°
./scripts/servo.sh reset        # same as 0 0
./scripts/servo.sh 0 0 /dev/ttyUSB1   # override port
```

**Direct binary:**
```bash
cd build
./servo_ctrl D 0 0
./servo_ctrl D 45 -20
./servo_ctrl --port /dev/ttyUSB1 D 0 0
```

**Debug mode (see all serial traffic):**
```bash
SERVO_DEBUG=1 ./servo_ctrl D 0 0
```

Servo limits (firmware enforced): pan −80°…+80°, tilt −70°…+30°

---

### 4 — Run full scan

```bash
cd /home/nvidia/Documents/stereo-camera/build

./stereo_scan \
  --port /dev/ttyUSB0 \
  --left 0 --right 2 \
  --calib ../calib/calib.yml \
  --step-pan 30 --step-tilt 20 \
  --output scan.ply
```

Key flags:

| Flag | Default | Notes |
|------|---------|-------|
| `--left` | 0 | Left camera index |
| `--right` | 2 | Right camera (Jetson UVC: 0/2 not 0/1) |
| `--calib` | _(none)_ | Without calib: normalized disparity only, not metric |
| `--step-pan` | 30 | Degrees per pan step |
| `--step-tilt` | 20 | Degrees per tilt step |
| `--output` | scan.ply | `.ply` or `.npy` |
| `--config` | scan.config | Checkpoint file — delete to restart scan |

**Resume interrupted scan:** just re-run same command; `scan.config` auto-resumes.

**Start fresh:** `rm build/scan.config` then re-run.

---

### 5 — View point cloud

```bash
# Python viewer (Open3D)
cd /home/nvidia/Documents/stereo-camera
python3 tools/map3d_server.py --input build/scan.ply
```

---

## Task status

### Sub-module: servo
- [x] ServoClient UART velocity mode (old)
- [x] ServoController MOVE/OK step-and-hold
- [x] `servo_ctrl` debug binary
- [x] `servo.sh` shell wrapper
- [x] MOVE/OK added to unified ESP32 firmware

### Sub-module: cv — raw depth
- [x] StereoCamera: SGBM disparity → CV_32F depth map
- [x] MapBuilder: 3D point cloud accumulation (PLY/NPY)
- [ ] Stereo calibration tested end-to-end (calib.yml generated)
- [ ] map3d_server.py live viewer verified

### Sub-module: cv — AI detection + 2D map  ← NOT STARTED
- [ ] Choose AI model (YOLO/TensorRT on Jetson)
- [ ] Implement Detector wrapper (vision/Detector.hpp/.cpp)
- [ ] Implement ObjectMapper: bbox + depth → 3D instance position
- [ ] Implement Map2D: project instances to X-Z floor plane
- [ ] ZMQ publisher: send Map2D to simulation module
- [ ] Integration test: live detection + map overlay in browser

### Known open bugs (deferred)
- [ ] StereoCamera sequential `.read()` — up to 33ms stereo skew (fix: grab/retrieve)
- [ ] MapBuilder FOV default 70° treated as half-FOV — should be ~35° H
- [ ] No max-depth clamp — SGBM outliers enter point cloud

## Cross-module interface (pending)
- [ ] Define Map2D ZMQ message format (port TBD, must not conflict with 5555/5556)
- [ ] Update simulation/app.py to receive and display 2D map overlay
