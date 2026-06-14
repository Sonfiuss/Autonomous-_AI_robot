# Session: stereo-camera — 2026-06-14

## Task
Apply `agent/plan/implement_plan_stereo.md` to `stereo-camera/`. The plan is written for
Python; user chose to **keep C++ and port the plan's ideas**, adopt the plan's coordinate
convention (mm, X-fwd/Y-right/Z-up), implement **all phases 0–8**, review whole diff at end.

## What was implemented

### New `core/` fusion package (port of plan phases 0–5, 6, 8)
| File | Contents |
|------|----------|
| `core/Config.hpp` | Coordinate contract doc + all constants (intrinsics, baseline, lever_arm, t_offset, angle_gap_threshold, z_min/z_max, voxel_size, column_decimation, edge_trim_frac, max_points, shutter_synced). Units mm. |
| `core/Geometry.{hpp,cpp}` | `unproject`, `remapAxes`, `rotation(pan,tilt)`, `toWorld`, `pixelToWorld`. Rotation built by hand for the left-handed body frame. **Replaces the spherical projection.** |
| `core/AngleBuffer.{hpp,cpp}` | Fixed-size ring buffer, O(1) push, `angleAt(t)` linear interp + gap + extrapolation flag. |
| `core/Depth.{hpp,cpp}` | `disparityToDepth` (Z=fx·B/d, mm), `validityMask` (NaN/inf/≤0 + [z_min,z_max]), `applyMask`. |
| `core/Reconstruct.{hpp,cpp}` | depth+angle → world points: validity → edge-trim → column decimation → unproject → remap → +lever → rotate. |
| `core/Sync.{hpp,cpp}` | `interpAt`/`nearestAt`/`gapTo`, `estimateTOffset` (offset sweep), `countDiscarded`. |
| `calibration/Calibration.{hpp,cpp}` | `mergedPlanarityRMS`, `estimateLeverArm` (coordinate-descent bow minimization), `handEyeOffset` scaffold. |

### Refactors of existing C++
- `vision/StereoCamera`: grab()+retrieve() (fixes ≤33 ms stereo skew); capture timestamp
  overload; mm depth + validity clamp; `applyIntrinsicsTo(Config&)` exposes rectified
  fx/fy/cx/cy + baseline so Reconstruct uses the same K as the depth.
- `vision/MapBuilder`: **rewritten** — takes `Config`, calls `Reconstruct`, voxel-grid
  dedup (`unordered_set` of packed 21-bit voxel keys), `max_points` guard. Kept PLY/NPY.
- `pipeline/Scanner`: `pan_tiling` option → fixed centres {−44,0,44}°; new
  `runContinuous(core, AngleBuffer&, n, out)` gated on `shutter_synced`.
- `main.cpp`: `--pan-tiling` flag; builds `Config` from rectified intrinsics (warns if
  uncalibrated).

### Tests (CMake `foreach` adds 8 targets)
`test_geometry` (G1–G6), `test_angle_buffer` (A1–A4), `test_depth` (D1–D3),
`test_reconstruct` (R1–R2), `test_accumulate` (C1–C2 + overlap), `test_end_to_end` (E1),
`test_sync` (S1–S3), `test_calibration` (lever-arm convergence). Rewrote `test_map_builder`
to the new convention. Added `tests/mocks/SyntheticScene.hpp` (wall-depth fixture +
MockFirmware).

## Verification
- Built + ran `test_geometry` and `test_sync` locally with g++ 6.3 (-std=c++1z): **all pass**
  (G1–G6 = 15/15, sync 4/4). Geometry signs confirmed against the plan §8 table.
- Could **not** build OpenCV/threaded targets here: this Windows box has MinGW.org which
  lacks `std::mutex`. Full `cmake + ctest` must run on the Jetson.

## Decisions
| Decision | Reason |
|----------|--------|
| Stay C++, port plan concepts | User choice; existing pipeline works and is C++ |
| Adopt plan convention (mm, X-fwd/Y-right/Z-up) | Matches plan §8 test tables exactly; user choice |
| Build R(pan,tilt) by hand | Body frame is left-handed; stock right-handed libs would flip tilt |
| Voxel dedup via packed 21-bit keys | No Open3D in C++; unordered_set is enough for dedup |
| Continuous mode gated + scaffolded | ServoController is step-and-hold; no P-feedback stream yet |

## Interface / convention changes
- **Point cloud frame changed**: X forward, Y right, Z up, **millimetres** (was
  X right/Y up/Z forward, metres). `tools/map3d_server.py` and any consumer must adapt.
- `MapBuilder` API: now `MapBuilder(const Config&)`, `addFrame` returns added-point count,
  `setLensFOV` removed.
- New `stereo_scan --pan-tiling` flag.

## Pending / next session
- [ ] `cmake -S . -B build && cmake --build build && ctest` on the Jetson — confirm green.
- [ ] Generate `calib.yml`; confirm metric mm depth end-to-end.
- [ ] Update `map3d_server.py` for mm + new axes.
- [ ] Wire ESP32 `P pan tilt` feedback → AngleBuffer to enable real continuous-sweep.
- [ ] Real hand-eye calibration (checkerboard) to replace scaffold.
