# Task: Stereo Pan-Tilt Point-Cloud Fusion (C++ port of implement_plan_stereo.md)

- **Module:** stereo-camera
- **Status:** executing
- **Created:** 2026-06-14
- **Source plan:** `agent/plan/implement_plan_stereo.md`

## Task
Apply the new stereo point-cloud implementation plan to `stereo-camera/`. The plan
is written for Python (`stereo_cloud/`); per user decision we **keep C++** and port the
plan's ideas: the geometry contract, angle buffer + sync design, depth/validity, voxel
accumulation, scan controller, and calibration — fixing the existing bugs along the way.

## Decisions (from user)
- **Stay in C++**, port the plan's concepts onto the existing pipeline.
- **Implement all phases 0–8** in one pass, review whole diff at the end.
- **Coordinate convention = plan §3:** body X forward, Y right, Z up; units **millimetres**.
  This is a breaking change vs. the old MapBuilder (X right, Y up, Z fwd, metres).

## Input
- Existing C++: `StereoCamera`, `MapBuilder`, `Scanner`, `ServoController`.
- Known bugs to fix: spherical projection in `MapBuilder::addFrame` (plan §3 forbids it),
  FOV-as-half-FOV, no max-depth clamp, sequential `.read()` stereo skew.

## Expected output
- `core/` geometry + angle buffer + reconstruct + config (new).
- Refactored depth (validity mask + z clamp) and MapBuilder (proper unproject + rotation +
  voxel dedup).
- Mocks for offline pipeline; calibration tools; continuous-sweep mode.
- ctest suite encoding plan §8 validation tables. All green on Jetson.

## Plan (C++ mapping of phases 0–8)
0. `core/Config.hpp` — intrinsics, baseline, lever_arm, t_offset, thresholds, conventions.
1. `core/Geometry.*` — unproject / remapAxes / rotation(pan,tilt) / toWorld + `test_geometry`.
2. `core/AngleBuffer.*` — ring buffer + angleAt(t) interpolation + `test_angle_buffer`.
3. `core/Depth.*` — disparity→depth + validity mask; `StereoCamera` uses it + `test_depth`.
4. `tests/mocks/` — MockFirmware + MockStereo replaying fixtures.
5. `core/Reconstruct.*` — mask + decimation + unproject + toWorld + `test_reconstruct`.
6. `MapBuilder` voxel-grid rewrite (uses Reconstruct), Scanner 3-pan tiling −44/0/+44 +
   `test_accumulate` + `test_end_to_end`.
7. Quality: edge-column trim, voxel dedup verify, cloud-size guard.
8. `calibration/` lever_arm + hand_eye + t_offset; continuous-sweep mode gated on
   `shutter_synced` + `test_sync`.

## Execution log
[--:--] phase 0: core/Config.hpp — coordinate contract + all constants | info: units=mm, X-fwd/Y-right/Z-up
[--:--] phase 1: core/Geometry(.hpp/.cpp) + test_geometry | info: G1–G6 compiled & PASS locally (g++)
[--:--] phase 2: core/AngleBuffer(.hpp/.cpp) + test_angle_buffer | risk: MinGW.org lacks std::mutex, build on Jetson
[--:--] phase 3: core/Depth(.hpp/.cpp) + StereoCamera grab/retrieve + mm + validity mask + test_depth | info: fixed skew + z-clamp bugs
[--:--] phase 5: core/Reconstruct(.hpp/.cpp) + test_reconstruct | info: mask+edge-trim+decimation+unproject+toWorld
[--:--] phase 6: MapBuilder voxel rewrite (killed spherical math) + Scanner 3-pan tiling + test_accumulate/end_to_end + rewrote test_map_builder
[--:--] phase 4: tests/mocks/SyntheticScene.hpp (wall depth fixture + MockFirmware)
[--:--] phase 7: edge_trim_frac + voxel dedup + max_points guard (in Reconstruct/MapBuilder)
[--:--] phase 8: core/Sync + calibration/Calibration (lever-arm bow min) + Scanner::runContinuous (gated) + test_sync(PASS)/test_calibration | risk: continuous needs firmware P-feedback wiring (pending); hand_eye is scaffold
[--:--] wiring: CMakeLists (core sources + 8 test targets), main.cpp (--pan-tiling + intrinsics plumbing)

## Status: testing
Build + run on Jetson:
```
cd /home/nvidia/Documents/stereo-camera
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j$(nproc)
cd build && ctest --output-on-failure
```
Expect green: geometry, angle_buffer, depth, reconstruct, accumulate, end_to_end, sync,
calibration, map_builder, scanner_raster, frame_queue.

