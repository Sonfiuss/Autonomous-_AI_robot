# Session 2026-07-24..26 — Astra Open3D SLAM: live-harden, RGB, big cleanup

## Context
Continued `agent/tasks/2026-07-23_astra-open3d-slam.md` (Astra Pro depth +
Open3D pose-graph SLAM). Focus this session: make the live run actually work on
hardware, add features, then delete the whole legacy pipeline.

## What was implemented / fixed in astra_slam.py
- **Display freeze fixes:** register map geometry ONCE before the loop (was only
  at frame_i==1, which a degenerate first frame skipped → blank window forever);
  downsample map IN-PLACE (`map_pcd.points = ds.points`) instead of rebinding
  (rebind froze the Visualizer). `pump()` helper centralises update+render.
- **Hang fixes (root cause of repeated "treo"):** OpenNI2 `read_frame()` blocks
  forever when the Astra drops its USB stream. Added `wait_for_any_stream(...,
  timeout=3s)` guard in `_grab()`; warmup + first frame now time out with a clear
  error instead of hanging before the window opens. Live loop stops cleanly on a
  dead stream and still saves the map.
- **Degenerate-frame guard:** `<100` pts after downsample → skip (was crashing
  `orient_normals` "No normals"); `<2000` pts → "too close, back off >=1m" warn.
- **Speed:** `--stride 2` (default) subsamples the depth grid (~4x fewer pixels,
  intrinsics scaled to keep metric geometry). Map accumulates the DOWNSAMPLED
  cloud, not the 240k full frame. FPFH only at keyframes. Replay ~4→~7-9 FPS.
- **Debug log:** one line/frame — `fN STATUS pts= fit= rmse= | dist= map= kf= +KF`,
  status OK/LOST/LOST!/SKIP/INIT. Step-labelled try/except prints which step
  crashed (`read_frame/make_pcd/icp_odom/accumulate_map/keyframe/render/...`) and
  still saves partial map.
- **Snapshots:** `--save-every 100` dumps numbered PLY (`_000100.ply` ...).
- **RGB mode A (approximate):** `--rgb` opens Astra colour via cv2 (UVC, separate
  device — OpenNI2 can't), colours each depth pixel by the same colour pixel;
  `--rgb-du/dv` hand-nudge parallax; `--rgb-index` picks the cam. New
  `save_map_ply` writes xyz+rgb PLY. Verified on synthetic colour. Mode B
  (extrinsic calib) is the documented upgrade path.

## Hardware run outcome
- Repeated hangs were BOTH the display bug AND the Astra dropping its depth
  stream. After a **physical USB replug** (rear port, no hub) the stream returned
  and the pipeline ran live (intrinsics fx≈570, window updates, ~7-9 FPS).
- Not yet done: a full closed-loop walk with visual check of `astra_map.ply`
  (walls/floor not doubled) + locking FIT_MIN/RMSE_MAX from real ICP logs.

## Big cleanup (user-approved, informed of consequences)
Deleted the entire legacy DA-V2 / manual-stereo / deepmap line as dead code —
user explicitly confirmed even after being warned it kills the in-testing YOLO
task on stereo_cloud.py and uncommitted changes:
- `depth-anything/src/`: removed all except `astra_cloud.py`, `astra_slam.py`,
  `rec/` (deleted depth_to_3d*, drive_area, merge_360, object_detect,
  obstacle_grid, run_local, stereo_cloud, stereo_ruler, view_web,
  depth_anything_v2/, README/LICENSE/requirements, 100k).
- Removed whole `deepmap/` folder and `stereo-camera/tools/*` scripts (kept
  captures/checkerboard data).
- Plans: rewrote `agent/plan/deepmap_plan.md` clean (Astra pipeline only);
  deleted `stereo_plan.md`, `implement_plan_stereo.md`.
- NOT committed — deletions are staged in the working tree only.

## Note / stale references
Several memories now point at deleted files (stereo_cloud disp-offset,
stereo_alignment, callgraph tool over DBM, deepmap_mono_only). They describe the
old pipeline that no longer exists — treat as historical.

## Next session
- Live closed-loop walk → eyeball `astra_map.ply` → set task `done`.
- RGB mode B calibration; ESP32 odom into `read_wheel_odom`.
- Decide whether to `git commit` the deletion + astra_slam work.
