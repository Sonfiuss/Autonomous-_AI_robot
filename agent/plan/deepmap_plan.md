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
