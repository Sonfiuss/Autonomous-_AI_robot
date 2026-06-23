# Session summary — 2026-06-15 — stereo-camera

Goal: get a point-cloud output from the stereo scanner and view it in a browser
on localhost.

## What was implemented / fixed

### 1. C++ build errors (blocking compile)
- `core/Geometry.hpp:50` — `apply(R, shifted)` resolved via ADL to `std::apply`
  (because `Vec3`/`Mat3` are `std::array`, pulling `namespace std` into lookup).
  Qualified it as `geom::apply(R, shifted)`.
- `vision/StereoCamera.hpp:76` + `vision/StereoCamera.cpp:127` — `applyIntrinsicsTo`
  took the nested `StereoCamera::Config` (no `fx/fy/cx/cy/baseline`). Changed the
  parameter type to the global `::Config` (core/Config.hpp), which has those fields.

### 2. ESP32 firmware ↔ host protocol mismatch (servo wouldn't move)
- The chip was flashed with the OLD `control/esp32_servo_controller/esp32_servo_controller.ino`
  (velocity protocol: `V`/`R`/`S` in, streams `P <pan> <tilt>` feedback) on
  PCA9685 **channels 12 / 14**.
- `stereo_scan` expects `esp32/servo_driver/servo_driver.ino` (step-and-hold:
  `MOVE`→`OK`, `READY` on boot). That firmware used channels **0 / 1** — wrong wiring.
- Fix: patched `servo_driver.ino` channels 0/1 → **12/14** (to match the real
  wiring proven by the old controller), compiled + flashed via arduino-cli
  (`esp32:esp32:esp32`, /dev/ttyUSB0). Verified `MOVE 0 0` → `OK`.

### 3. Scan produced 0 points without calibration
- `vision/StereoCamera.cpp` uncalibrated branch returned normalized disparity in
  [0,1], which the reconstruct validity clamp (z_min=200 … z_max=6000 mm) dropped
  entirely.
- Fix: uncalibrated fallback now computes a plausible (non-metric) depth
  `Z = fx·baseline/disp` with assumed `fx=600 px`, `baseline=60 mm` (matching the
  reconstruct defaults), so points land in [z_min, z_max]. After this, a full
  36-step sweep produced ~100k–150k points.

### 4. Web viewer — display a saved .ply (reused map3d_server viewer)
- `tools/map3d_server.py`: added a static `--ply FILE` mode (+ `--scale`) via new
  `_load_ply()`. It parses ASCII PLY, **auto-scales mm→m**, remaps body Z-up →
  viewer Y-up, and **recenters on the median centroid** (otherwise the cloud is
  all forward of the camera and invisible). When `--ply` is set, the camera/serial
  threads are skipped; the existing WebGL viewer serves the static cloud.
- Hardened the HTTP handler: `do_GET`/`do_POST` now wrap routing in try/except for
  `BrokenPipeError`/`ConnectionResetError` (browser cancelling polls mid-download).

### 5. Convenience
- New `run_all.sh` at stereo-camera root: build → flash → scan → view, with flags
  `--no-build/--no-flash/--no-scan/--view-only` and `--port/--left/--right/--http`.

## How to run
```
./run_all.sh                 # full pipeline
./run_all.sh --no-flash      # everyday scans (firmware already flashed)
./run_all.sh --view-only     # serve existing build/scan.ply
# viewer at http://localhost:8080  (or http://<jetson-ip>:8080)
```

## Interface changes
- `StereoCamera::applyIntrinsicsTo` now takes `::Config&` (global), not the nested
  `StereoCamera::Config`.
- `tools/map3d_server.py` gained `--ply` / `--scale` (static viewer mode).
- ESP32 `servo_driver.ino` now drives PCA9685 channels 12/14 (was 0/1).

## Known gotchas
- `pkill -f map3d_server.py` kills its own shell (command line contains the
  pattern). Stop the viewer by port instead: `fuser -k 8080/tcp`.
  run_all.sh still uses `pkill -f` — candidate to switch to `fuser -k`.
- `rm -f scan.config` before a fresh scan or it resumes the previous sweep.
- `stereo_scan` and `map3d_server` can't both run (camera contention).
- The `--ply` viewer loads the file once at launch — restart it to see a new scan.

## Pending / next steps
- **Calibration** is the main open item: cloud is non-metric (assumed
  fx=600/baseline=60). Use `tools/stereo_calibrate.py` to produce a `.yml` in
  `calib/`, then run with `--calib` for true-mm depth (rectification + Q).
- Consider switching run_all.sh viewer-kill to `fuser -k 8080/tcp`.
- Cameras: left=0, right=2 (Jetson UVC = 2 nodes per camera).
