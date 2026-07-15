# deepmap module plan

## Direction (updated 2026-06-28): migrate to ROS 2 / RTAB-Map SLAM
The naive 360° angle-merge below is now treated as **legacy / seed-only**. The
mapping pipeline is moving to **graph SLAM (RTAB-Map on ROS 2 Foxy)** so the robot
can map a **wide area while driving**, with loop closure correcting odometry drift
and point clouds fused by pose-graph alignment instead of raw commanded angle.

- Full SLAM plan + resolved decisions (D1–D6): see
  `agent/tasks/2026-06-28_rtabmap-slam.md` — that task is the source of truth.
- New ROS 2 workspace lives under `slam/` (isolated from `deepmap/`).
- Why migrate: angle-merge assumes a perfect in-place pivot and dead-reckoned yaw.
  Any slip, eccentric-camera arc-smear (15.6 cm radius), or translation breaks it.
  RTAB-Map replaces "merge by commanded angle" with "merge by optimized pose".

### What carries over from deepmap → slam
| deepmap asset | Role in RTAB-Map plan |
|---------------|----------------------|
| Depth Anything V2 pipeline (`depth-anything/src`) | DA-V2 depth node (Step A4) — mono RGB-D source |
| `deepmap/scale_calib.py` (motion-parallax `k`) | metric-scale recovery (Step A4), saved to `slam/config/scale.yaml` |
| `drive_area.py` movable-space labels | optional obstacle/floor colouring of exported cloud |
| ESP32 UART protocol (`O`/`T`/`F`/`M`/`S`) | serial bridge node: odom IN + teleop OUT (Step A5) |

### Migration milestones (high level — detail in the slam task)
1. Scaffold `slam/ros2_ws` colcon workspace + `env.sh` (A1).
2. Install RTAB-Map for Foxy (apt, source fallback) (A2).
3. Mono camera bring-up + intrinsic calibration → `camera.yaml` (A3).
4. DA-V2 depth node + motion-parallax scale `k` → `scale.yaml` (A4).
5. Serial bridge node: `/odom` + `/cmd_teleop` (A5); TF tree + extrinsics (A6).
6. First RTAB-Map RGB-D launch = seed map (A7–A8).
7. Drive wide area; incremental mapping + nearest-wins voxel merge (D6) (Part B).
8. Export `map.ply` + occupancy grid `map.pgm/.yaml` for `simulation/` (B6).

---

## Purpose (legacy 360° builder — superseded by RTAB-Map)
360° depth map builder — rotates robot in-place, captures images at each angle,
then merges per-frame Depth Anything V2 point clouds into a single 360° PLY.
Kept for offline single-spot scans and for the DA-V2 / scale-calib code it shares
with the SLAM depth node.

## Files
| File | Role |
|------|------|
| `deepmap/rotate_scan.py` | Feature 1: UART rotation control + camera capture |
| `deepmap/build_map.py`   | Feature 2: depth pipeline + 360° merge |
| `deepmap/shots/`         | (runtime) captured images + angles.csv |
| `deepmap/output/`        | (runtime) final map_360.ply |

## Workflow
```
1. python deepmap/rotate_scan.py [--step-deg 30] [--omega 0.5] [--cam 0]
   → deepmap/shots/shot_NNN.jpg + angles.csv

2. python deepmap/build_map.py [--shots deepmap/shots] [--encoder vits] [--show]
   → deepmap/output/map_360.ply
```

## Dependencies
- pyserial (for rotate_scan.py UART)
- cv2, torch, numpy (for build_map.py — same as depth-anything)
- open3d (optional, for --icp / --voxel / --show)
- depth-anything/src on sys.path (imported directly, no copy)

## Key design decisions
- Capture image FIRST at current position, then command rotation → natural 0° start
- Uses cumulative commanded angle for CSV (not odometry theta) — simpler, repeatable
- build_map.py imports merge_360.py helpers directly (DRY, no code duplication)
- --dry-run flag on rotate_scan.py allows testing image capture without robot connected

## Completed
- [2026-06-28] Created rotate_scan.py and build_map.py (initial 360° implementation)
- [2026-06-28] Ported motion-parallax metric scale into `scale_calib.py` (reused by SLAM A4)
- [2026-06-28] Decided to migrate map fusion to RTAB-Map graph SLAM (see slam task)

## Pending / future
- [ ] **Migration to RTAB-Map SLAM** — primary direction, tracked in
      `agent/tasks/2026-06-28_rtabmap-slam.md` (Parts A & B)
- [ ] (legacy) --360-check: after scan, confirm final theta ≈ 360° from odometry
- [ ] (legacy) Web viewer integration (pipe PLY to view_web.py)
- [ ] (legacy) Auto-calibrate camera FOV from intrinsics file — superseded by real
      `camera_info` intrinsic calibration in SLAM Step A3

## Pipeline improvements (2026-06-29)
1. **Overlap merge (72° FOV vs 30° step):** `build_map.py` now defaults `--fov 72`
   and adds `--keep-deg` central-wedge crop (default=auto=scan step). Each frame
   keeps only its central ±step/2° via bearing=atan2(X,-Z), so overlapping frames
   tile instead of doubling geometry. `crop_central_wedge` + `auto_step_deg` helpers.
2. **Drive-area side rays:** `drive_area.compute_drive_polygon` casts rays from the
   lower half of the LEFT & RIGHT edges too (not just the bottom), converging to
   top-center, ordered by polar angle → tighter object extent. `--no-side-rays`,
   `--n-side` flags. cast_ray_to_top generalized to (ox,oy).
3. **Camera-yaw compensation:** `build_map.py --cam-yaw-deg` adds a constant yaw to
   every frame (camera mounted left+/right- of forward). Default 0. NEEDS the
   measured offset angle from user (or switch to a pan-servo centering command).

### Issue 3 resolved — firmware pan trim (2026-06-29)
Root cause: camera servo + steppers share ONE ESP32. Re-flashing the movement
firmware resets pan to mechanical 0° (PULSE_CENTER), which points LEFT on this
robot. manual-control can re-center but needs its own firmware (re-flash resets again).
Fix: `TILT_TRIM_DEG` in esp32_unified_controller.ino + `tiltAngleToPulse()` applied to
ALL TILT writes (runtime + boot) so logical tilt=0 -> physical home (misaligned axis
is TILT, not pan; pan left unchanged). Baked into
firmware => survives flashing. User sets PAN_TRIM_DEG to measured offset, flash once.
Software `--cam-yaw-deg` in build_map remains for map-side fine-tune (360° scan
coverage is unaffected by the offset regardless).

### Explore-and-map loop added (2026-07-01) — `deepmap/explore_map.py`
Autonomous explore: robot drives toward the LARGEST reachable free area, stops
every `--step-cm` (30cm default), captures image+depthmap, and merges each view
into one growing global PLY (`output/explore_map.ply`, saved incrementally).
- Reuses (no copied logic): `obstacle_grid` (BEV occupancy + flood reachability →
  pick area via `goal_from_bev`), `slam/robot_drive` (forward + spin), `rotate_scan`
  (camera open + fresh-frame), `depth_to_3d`/`merge_360` (3D + PLY/ICP).
- NEW geometry: `Pose` dead-reckons world (x,z,yaw) from commanded moves;
  `transform_to_world_pose` = merge_360.transform_to_world (yaw+cam_offset) PLUS
  robot translation, so moving captures land at the right world position.
- Stop rule (v1): `--max-steps` / `--max-dist-m` cap, plus halt if largest area
  < `--min-free-cells` or the cell straight ahead isn't confirmed free (safety).
- Validated offline (`--test-explore`): 4-step run accumulates pose
  (0,0,0°)→(0.42,1.11,33°), 1.20m, clouds fan out into world frame correctly.
- KNOWN LIMIT: no encoder/IMU → drift on long runs; `--voxel` enables ICP merge.
  Untested on hardware as of this entry.

### explore_map merge fix (2026-07-01) — duplicate-shell collapse
Symptom: explore map came out as ~3 separate concentric shells. Cause:
Depth-Anything depth is per-frame non-metric (each frame self-normalises scale),
so the same surface lands at a different radius each capture → overlapping shells;
dead-reckon translation spreads them. Raw vstack kept all → all visible.
Fix (user's rule "lấy point gần nhất"): `nearest_merge` — spherical z-buffer that
bins points by (azimuth, elevation) from a viewpoint and keeps only the NEAREST
per cell. New flags: `--merge nearest|all` (default nearest), `--ang-res 0.3`,
`--view-from mean|start|last`. Per-capture camera origins recorded as viewpoints.
Validated: 3-step straight run 691200 pts → 29430 pts (one surface).
LIMIT: single viewpoint — fine while the robot stays near one vantage; over a long
multi-room run a per-segment viewpoint (or true per-frame metric scale via
scale_calib) would be the next step.

### explore_map merge fix v2 (2026-07-01) — metric floor-anchor (the real cause)
v1 z-buffer was wrong tool: the "1 box → 3 copies" are SPATIALLY-OFFSET duplicates
(mis-registration), not concentric shells, so per-direction nearest can't merge them.
Also: open3d is NOT installed on this Jetson → merge_360.icp_merge / --voxel(open3d)
never ran. Root cause = per-frame non-metric depth (each frame own scale + own floor
tilt) → same object placed at different world spots.
Fix (no open3d): `level_and_scale` — per frame fit floor (depth_to_3d.fit_floor_plane),
rotate floor→horizontal (Rodrigues `_rot_to_up`), scale so camera = --camera-height
above floor. Every frame becomes metric + gravity-aligned → captures overlap. Default
ON via --metric-floor (--no-metric-floor reverts to fixed --tilt/--scale). Bonus:
obstacle_grid thresholds (h_obs etc.) now real metres. depth_to_points called with
tilt=0/height=0 (true tilt derived from floor fit). New numpy `voxel_dedup` (default
--voxel 0.02) replaces open3d downsample. --merge default now 'all' (frames already
aligned); 'nearest' kept as option. Residual error = dead-reckon pose drift only.
Verified offline: per-frame scale now consistent (1.40/1.38/1.40 vs wild before).

### stereo_walk_map added (2026-07-04) — stereo-metric walk, replaces floor-anchor scale
`deepmap/stereo_walk_map.py`: straight-line walk (default 3 stops × 30 cm); each
stop captures a STEREO pair and runs the stereo-ruler pipeline for METRIC depth
(f*B from the 5.4 cm baseline), so every frame shares the same real-metre scale
by construction — no more per-frame `level_and_scale` scale guessing.
- stereo_ruler.py refactored for import: `load_model()` (DA-V2 loaded once),
  `load_rectify()` → namespace incl. fx/fy/cx/cy from P2 (back-projection K),
  `compute_metric_depth(left, right, calib, model)` → (Z_m, valid, info).
- Floor plane still fitted per frame but ROTATION-ONLY (`level_to_floor`): levels
  the ~13° camera tilt + puts floor at Y=0; `--no-level` to disable.
- Merge chain reused from explore_map: incremental concat saves → final
  `icp_merge_clouds` (open3d) → `voxel_dedup` → `output/stereo_walk/walk_map.ply`.
- Calibration frame is 1280×720 (stereo_rectify.yml) — walk defaults match; a
  size mismatch between shot and calib raises immediately.
- Offline `--test` (same saved pair each stop) PASSED; hardware 3-stop run pending
  (task 2026-07-04_stereo-walk-map, status testing).

### stereo_walk_map pipeline rebuild (2026-07-06) — one thread per stage
Task 2026-07-04_pipeline-walk-speed (status testing — hardware step pending).
`deepmap/pipeline_bus.py` (NEW, stdlib-only): Msg / Bus (per-subscriber queue
fan-out, stop_event-aware publish) / Worker (catches BaseException so
stereo_ruler's sys.exit can't kill a thread silently) / ordered join_all
(pill re-offered while joining — one-shot pills get lost when a busy worker's
queue is full). Future VoiceWorker/DetectWorker plug into this bus.
`stereo_ruler.py`: compute_metric_depth split into `stereo_half` +
`depth_half` + `fuse_metric` (same pieces run sequentially OR on threads;
fuse_metric raises RuntimeError — incl. converted RANSAC sys.exit — as the
"expected per-frame failure" tier). `stereo_walk_map.py`: PersistentStereoCam
(open once, BUFFERSIZE=1, grab-flush `--discard-s` to drop frames buffered
during the move), Capture/Stereo/Depth/FusionMapper/Motion workers (one owner
per resource: cameras/model/UART+Pose), Coordinator issues "move" as soon as
the pair lands. `--sequential` A/B (same persistent cams), `--profile`,
`--save-every`, `--inject-crash/--inject-badframe`. Offline: regression
identical, crash → partial map + clean join, bad frame → skipped + walk
continues; wall 21.9s vs 26.6s sequential on CPU (depth-bound; Jetson GPU +
real drives overlap far more). Hardware A/B pending (plan step 8).

### Free-move merge design SHELVED — stereo accuracy first (2026-07-11)
Free-movement multi-frame merge (route F/S/T, turn-burst yaw, motion-type VO
gates, QC gate, pairwise ICP) was designed and approved but is ON HOLD — design
saved at `agent/description/deepmap_free_move_merge.md` for later. Reason: the
stereo ruler that everything depends on is untrustworthy at the root —
calibration from only 2 chessboard pairs @0.9m (laptop screen), stereo RMS
1.897px, distortion=0, fx ±15%, never tape-measure-validated. Decision: fix
stereo accuracy FIRST (recalibrate + measure), then revisit the merge design.
Active task: `agent/tasks/2026-07-11_stereo-accuracy.md` (stages A–E: eval
harness → printed-board recalib → empirical fb → matching experiments →
integration + deepmap replay A/B).

### explore_map merge v3 (2026-07-01) — open3d ICP enabled
User installed open3d 0.16.0. Added `--merge icp` (now default): point-to-plane ICP
(merge_360.icp_merge) refines the residual dead-reckon pose drift AFTER metric
floor-anchoring has fixed scale+level, then merges. Runs once on the FINAL save
(incremental saves stay cheap concat). Falls back to concat if open3d missing.
Verified offline: icp fitness 0.93→1.00, rmse ~0.012-0.018. numpy --voxel still
applied after. Modes: icp(default) | all | nearest.
