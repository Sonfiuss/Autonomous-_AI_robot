# Session: stereo-camera — 2026-06-06

## What was done

Design session only — no code written. Full architecture for the 3D scan pipeline
was designed, reviewed, and approved by the user.

## Confirmed decisions

### Hardware
- Jetson runs stereo capture and map building
- ESP32 controls 2 servos via PCA9685, connected to Jetson via UART serial
- Stereo camera connected directly to Jetson

### Servo sweep
- Pan servo : −80° → +80° (PCA9685 ch0)
- Tilt servo : −70° → +30° (PCA9685 ch1)
- Scan order : snake raster (pan sweeps left↔right, tilt steps down→up)
- Step size  : configurable CLI arg, default 10°

### Serial protocol (UART 115200)
```
Jetson → ESP32:  "MOVE <pan_float> <tilt_float>\n"
ESP32  → Jetson: "OK\n"    ← after both servos have settled (delay inside ESP32)
```

### 3D coordinate system (X-forward)
```
x = d · cos(tilt_rad) · cos(pan_rad)
y = d · sin(tilt_rad)
z = d · cos(tilt_rad) · sin(pan_rad)
```
Camera at origin (0,0,0). At pan=0°, tilt=0°, d=0.50m → (0.50, 0.0, 0.0).
MapBuilder.addFrame(pan_deg, tilt_deg, depth_map) handles projection internally.

### Thread architecture (zero artificial delays)
- Thread A (Servo)  : snake raster; MOVE+wait OK (~400ms); write scan.config; notify B
- Thread B (Capture): fires immediately on notify; captureDepth() → full 640×480 depth map;
                      push FramePacket{pan,tilt,depth_map} to FrameQueue; no sleep
- Thread C (Map)    : independent consumer; pop queue; mapBuilder.addFrame(); sentinel → savePLY

Thread A and B overlap: SGBM (~150ms) runs while A commands next MOVE (~400ms).
Thread C never blocks Thread B — queue absorbs bursts.

### Output
- `scan.ply`    — ASCII PLY, all valid depth pixels from all frames
- `scan.config` — plain text: pan, tilt, direction (for crash resume)

### Key insight confirmed
Full depth map per frame >> one center pixel per frame.
~120,000 valid 3D points per frame vs. 1 point per frame.
9 frames at 10° step → ~1M points in ~4 seconds.

## Files to create (next session)

```
stereo-camera/
  control/
    ServoController.hpp/.cpp     ← Jetson UART → ESP32
  esp32/
    servo_driver/
      servo_driver.ino           ← ESP32 firmware: parse MOVE, PCA9685, reply OK
  pipeline/
    FrameQueue.hpp               ← thread-safe blocking queue (header-only)
    Scanner.hpp/.cpp             ← 3-thread orchestrator + snake raster + config I/O
  main.cpp                       ← update: CLI args, wire all components
```

Reused unchanged: `vision/StereoCamera`, `vision/MapBuilder`

## Implementation plan (9 steps, approved)

1. ESP32 firmware — servo_driver.ino (independent)
2. ServoController — Jetson UART client (independent)
3. FrameQueue — header-only (independent)
4. Scanner — 3-thread orchestrator (depends on 2, 3)
5. main.cpp update (depends on 4)
6. Test: full sweep, verify PLY geometry
7. Test: crash resume from scan.config

Steps 1, 2, 3 can be implemented in parallel at session start.

## Placeholders to replace when known

| Item | Placeholder | Where |
|------|-------------|-------|
| Serial port | `/dev/ttyUSB0` | `main.cpp` CLI arg |
| Pan PWM (µs at ±80°) | 500–2400 µs linear | `servo_driver.ino` constants |
| Tilt PWM (µs at −70°/+30°) | 500–2400 µs linear | `servo_driver.ino` constants |
| Camera FOV | N/A (affects recommended step size only) | `main.cpp` CLI arg |

## Pending tasks

- See `agent/tasks/2026-06-06_stereo-scan-pipeline.md` (status: approved)

## Interface changes

None — existing StereoCamera and MapBuilder APIs unchanged.
New public API entry point: `Scanner::run(serial_port, calib_file, step_deg)`
