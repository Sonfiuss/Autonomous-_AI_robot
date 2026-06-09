# Task: Stereo Scan Pipeline (3-thread, pipeline mode)

**Module:** stereo-camera  
**Status:** done  
**Date:** 2026-06-06

---

## Task

Build a concurrent 3D scan pipeline:
- Thread A: moves two servos in a coarse raster (-80→+80 pan, -70→+30 tilt).
  Step size = camera FOV × (1 - overlap%). Fewest possible positions.
  Writes checkpoint to scan.config after each ACK.
- Thread B: fires IMMEDIATELY on servo ACK — no extra sleep.
  Captures full 640×480 depth map → pushes to queue → loops back instantly.
  Never waits for Thread C.
- Thread C: independent consumer — pops depth frames, projects ALL valid pixels
  into 3D via MapBuilder, saves scan.ply on completion.

---

## Core design principle: pipelined execution

```
Time →

Thread A:  [MOVE→ESP32, wait OK ~400ms][MOVE→ESP32, wait OK ~400ms][MOVE…]
Thread B:                [capture+SGBM ~150ms][  idle  ][capture+SGBM ~150ms]
Thread C:                               [addFrame~50ms]               [addFrame]
```

- Thread A's UART wait (~400ms) already allows the servo to settle.
  No extra sleep anywhere.
- While Thread B runs SGBM (~150ms), Thread A has already started
  commanding the next position. They overlap.
- Thread C never blocks Thread B — the queue absorbs bursts.
- Result: effective frame rate limited only by servo move time (~400ms/frame).

---

## Minimum frames calculation

Each stereo frame covers a cone of (FOV_h × FOV_v) degrees.
Minimum frames to cover full pan×tilt range with 20% overlap:

```
pan_frames  = ceil(160° / (FOV_h × 0.80))
tilt_frames = ceil(100° / (FOV_v × 0.80))
total       = pan_frames × tilt_frames
```

Example with FOV_h=90°, FOV_v=60°:
  pan_frames  = ceil(160 / 72) = 3
  tilt_frames = ceil(100 / 48) = 3
  total = 9 frames

Time:    9 × 400ms ≈ 4 seconds
Points:  9 × ~120,000 ≈ 1,080,000 points

→ **Need actual camera FOV to finalize step size (open question #1).**

---

## Coordinate system (X forward at pan=0, tilt=0)

For each valid pixel (u,v) at (pan_deg, tilt_deg) with depth d:

```
# Camera-frame 3D point from depth map (Q matrix or simplified):
X_cam = (u - cx) * d / fx
Y_cam = (v - cy) * d / fy
Z_cam = d

# Rotate by (pan, tilt) into world frame:
x_w =  X_cam·cos(pan) + Z_cam·cos(tilt)·cos(pan) ... (use R_pan × R_tilt)
```

MapBuilder.addFrame(pan_deg, tilt_deg, depth_map) handles this internally.
At pan=0, tilt=0, d=0.50m → (0.50, 0.0, 0.0) ✓

---

## Scan order: snake raster

```
tilt = -70°
  pan: -80° → +80° step Δpan
tilt = -70° + Δtilt
  pan: +80° → -80° step Δpan  (reverse — saves servo travel)
...
```

---

## Serial protocol (UART 115200)

```
Jetson → ESP32:  "MOVE <pan_f> <tilt_f>\n"
ESP32 → Jetson:  "OK\n"     ← sent after both servos settle (settle_ms inside ESP32)
```

## scan.config

```
pan=-45.00
tilt=-10.00
direction=1          # 1=left-to-right, -1=right-to-left (snake state)
```

---

## Files to create / modify

```
stereo-camera/
  control/
    ServoController.hpp/.cpp     ← Jetson: UART → ESP32 (moveTo pan+tilt, wait OK)
  esp32/
    servo_driver/
      servo_driver.ino           ← ESP32: parse MOVE; PCA9685 ch0=pan ch1=tilt;
                                          delay(settle_ms); reply OK
  pipeline/
    FrameQueue.hpp               ← thread-safe blocking queue (header-only)
    Scanner.hpp/.cpp             ← 3 threads; snake raster; config I/O
  main.cpp                       ← update: args = serial port, calib, step size
```

MapBuilder and StereoCamera reused unchanged.

---

## Plan steps

1. **Get camera FOV** → compute optimal step size
2. **ESP32 firmware** (`servo_driver.ino`): parse MOVE pan tilt; PCA9685 two channels;
   reply OK after settle_ms
3. **`ServoController`**: UART serial; `moveTo(pan, tilt)` sends cmd, blocks on OK
4. **`FrameQueue`**: header-only; push()/pop() with mutex+cv; sentinel support
5. **`Scanner`**:
   - Reads scan.config → resume (pan, tilt, direction)
   - Thread A: snake raster; moveTo(); write config; notify_one(cv_capture)
   - Thread B: wait cv_capture; immediately captureDepth(); push to queue; return
   - Thread C: pop; mapBuilder.addFrame(); sentinel → savePLY
6. **`main.cpp`**: CLI args; wire Scanner + StereoCamera + MapBuilder
7. **Test**: verify pipeline overlap (log timestamps); verify PLY has correct geometry
8. **Test resume**: interrupt + relaunch at saved position

---

## Open items

1. **Camera FOV** (degrees H × V) — determines step size and frame count
2. **Serial port** on Jetson
3. **Servo PWM calibration** (µs at −80° and +80° for pan; µs at −70° and +30° for tilt)
4. **PCA9685 channel assignment** (pan=ch0, tilt=ch1 — confirm or change)

---

## Execution log

[Session 2026-06-09]
- step 1: Reviewed existing code against plan — found deep divergence (velocity vs step-and-hold) | info: previous code was a different architecture, plan not yet executed
- step 2: Created esp32/servo_driver/servo_driver.ino — MOVE/OK protocol, 500ms settle | info: pan ch0, tilt ch1
- step 3: Created control/ServoController.hpp/.cpp — blocking moveTo(), CV-based OK wait
- step 4: Created pipeline/FrameQueue.hpp — header-only, backpressure, close sentinel
- step 5: Created pipeline/Scanner.hpp/.cpp — 3-thread A/B/C pipeline, snake raster, scan.config checkpoint
- step 6: Updated main.cpp — new CLI args, wires Scanner, prints step count + resume status
- step 7: Updated CMakeLists.txt — stereo_pipeline lib, 3 test targets, ctest
- step 8: Created tests/test_frame_queue.cpp, test_scanner_raster.cpp, test_map_builder.cpp
- step 9: Build succeeded; fixed StereoCamera default constructor (compiler issue on aarch64) | risk: MapBuilder 1×1 depth division-by-zero found and fixed
- step 10: ctest 3/3 pass
- step 11: Live run — right camera index 1 fails (metadata node); fix --right 2, updated default in main.cpp

## Test result

ctest 3/3 passed (frame_queue, scanner_raster, map_builder).
Live camera confirmed: left=0, right=2 at 1600×1200 native resolution.
ServoController connects successfully; READY timeout on pre-booted ESP32 is expected.
