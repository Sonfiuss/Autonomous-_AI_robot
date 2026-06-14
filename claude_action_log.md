# Claude Action Log

Actions taken beyond the literal request are recorded here for transparency.

## 2026-06-14 — Stereo point-cloud fusion (C++ port of implement_plan_stereo.md)

User asked to apply `agent/plan/implement_plan_stereo.md` in `stereo-camera/`, save a
session summary, and update `agent/plan/stereo_plan.md`. Decisions (confirmed by user):
keep C++ (not the plan's Python), adopt the plan's convention (mm, X-fwd/Y-right/Z-up),
implement all phases 0–8, review whole diff at the end.

Beyond-request / notable actions:
- **Changed coordinate convention** of MapBuilder from X-right/Y-up/Z-fwd (metres) to
  X-fwd/Y-right/Z-up (mm). Breaking change to cloud orientation — required by the plan.
- **Replaced** the spherical projection in `MapBuilder::addFrame` (the old code did
  `X = r·cosφ·sinθ`, which the plan §3 explicitly forbids) with proper pinhole
  unprojection + rotation via new `core/Geometry`.
- **Fixed three deferred open bugs** while porting: FOV-as-half-FOV (removed entirely),
  no max-depth clamp (added z_min/z_max validity mask), sequential `.read()` stereo skew
  (switched to grab()+retrieve()).
- **Rewrote** the existing `tests/test_map_builder.cpp` (it encoded the retired spherical
  convention and would have failed/compiled against the old API).
- Added `--pan-tiling` CLI flag and rectified-intrinsics plumbing
  (`StereoCamera::applyIntrinsicsTo`) so Reconstruct unprojects with the same K used for
  depth.
- Verified geometry (G1–G6) and sync (S1–S3) suites compile and pass locally with g++.
  Could not build OpenCV/threaded targets on this Windows box (MinGW.org lacks
  `std::mutex`); those must be built + `ctest`-run on the Jetson.

Pending (noted, not done): continuous-sweep firmware feedback wiring (ServoController is
step-and-hold only — does not stream P-feedback into an AngleBuffer); hand-eye calibration
is a documented scaffold (needs checkerboard detections).
