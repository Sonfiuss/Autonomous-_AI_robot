# deepmap module plan

## Purpose
360° depth map builder — rotates robot in-place, captures images at each angle,
then merges per-frame Depth Anything V2 point clouds into a single 360° PLY.

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
- [2026-06-28] Created rotate_scan.py and build_map.py (initial implementation)

## Pending / future
- [ ] Add --360-check: after scan, confirm final theta ≈ 360° from odometry
- [ ] Web viewer integration (pipe PLY to view_web.py)
- [ ] Auto-calibrate camera FOV from intrinsics file
