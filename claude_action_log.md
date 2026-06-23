# Claude Action Log

Actions taken beyond the literal request are recorded here for transparency.

## 2026-06-22 — Port AD-Census to `stereo-camera-AD-Census/`

User asked to vendor ethan-li-coding/AD-Census into a new folder, keep image-file
input, build for Jetson. Beyond-request / notable actions:
- Modified the vendored sources (user said "keep using the solution", but they did
  not build as-is on GCC): converted all `.cpp/.h` from GBK→UTF-8, and added missing
  standard headers (`<cstdio>`, `<cstring>`, `<cmath>`) that MSVC includes
  transitively but GCC does not. Algorithm logic untouched.
- Did NOT vendor upstream `main.cpp` (Windows-only imshow/system("pause")/fopen_s);
  wrote a headless replacement instead.
- Left a temp clone at `/tmp/adcensus_src` (outside repo, can be deleted).

## 2026-06-22 — Arm mask + free-space grid (task arm-mask-freespace)

User approved a 5-step plan to mask the robot arm and add a free-space grid. Beyond
the literal request:
- Installed a system package: `mingw-w64-x86_64-python-opencv` (OpenCV 4.13) into
  MSYS2 via pacman, because NO working Python+OpenCV existed on the box (the base
  `C:\Python\Python310` referenced by the `py` launcher and the `labelimg` venv had
  been removed; `Python313` has no interpreter exe). User chose this option.
  -> Run the wrapper with `C:\msys64\mingw64\bin\python`.
- Changed `run_adcensus()` to CAPTURE the binary's stdout (previously inherited) to
  parse the "Disparity range" line — needed because `_disp.png` is min-max
  normalized, so metric depth is otherwise unrecoverable.
- Verified algorithm sources via `g++ -fsyntax-only` (clean).
- Added one-shot build+run scripts (`run_ad-census.sh`, `build.sh`, `build_windows.bat`)
  and a per-module `captures/` copy of the test pair; default output now lands in
  `stereo-camera-AD-Census/captures/`.
- Removed a stray `-p` folder created by a bad Windows `mkdir -p build`.
- INSTALLED SYSTEM TOOLING (user asked to "install all tools needed"): MSYS2 via
  winget, plus pacman packages mingw-w64-x86_64-{gcc,opencv,pkgconf} + make. Built
  (g++ 16.1.0 + OpenCV 4.13) and ran successfully on Windows — output PNGs produced.
  Result is noisy because the capture pair is unrectified (expected).
- Added `tools/preprocess_run.py` (user-requested): rectify→denoise+CLAHE→adcensus
  →speckle+guided clean→depth-band zoning. Reuses depth_grid.py's uncalibrated
  rectification. Key finding: rectify MUST precede denoise (denoise blurs keypoints
  → residual 17.9px; reversed order → 0.34px). With correct order the depth map is
  coherent (near=red, far=blue) — fixes the "too much noise" complaint.

## 2026-06-21 — Apply depth-grid rectify-fix plan (tools/depth_grid.py)

User asked to apply the plan in `agent/tasks/2026-06-21_depth-grid-rectify-fix.md`.
Implemented all 6 steps. Beyond-request / notable actions:
- Created comparison output images while diagnosing: `captures/depth_grid_norect.jpg`
  (--no-rectify) and `captures/depth_grid_swap.jpg` (swapped L/R) to confirm image
  ordering and quantify the rectify-vs-coverage tradeoff. Can be deleted.
- Created temporary scratch script `/tmp/rtest.py` (outside repo) to tune the RANSAC
  threshold; found 1.0/conf 0.999 fixes the degenerate homography.
- Added a self-validation step NOT in the original plan: reject the rectification and
  fall back to the original pair if the warp does not lower the vertical residual.
- Step 6 target docs (`PROJECT_KNOWLEDGE.md`, `agent/plan/stereo-camera_plan.md`) do
  not exist; recorded the field note in `agent/plan/implement_plan_stereo.md` Phase 3
  instead, the relevant existing doc.

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
