---
id: 2026-06-30_explore-and-map
status: testing
module: stereo-camera
started: 2026-06-30
---

## Task
Autonomous explore-and-map loop: robot drives toward the largest available free
area, stops every 30 cm to capture an image + depthmap, and merges each capture
into one growing global ("origin") point cloud.

## Input
- Decision: pick area from the **forward BEV at each step** (reuse obstacle_grid.py).
- Decision: stop on a **max-steps / total-distance cap** (predictable v1).
- Reuse, do not duplicate:
  - `depth-anything/src/obstacle_grid.py` — BEV occupancy + flood-fill reachability (grid==3 drivable).
  - `slam/robot_drive.py` — `drive_forward(cm)`, `drive('spin left/right <deg>')`.
  - `deepmap/rotate_scan.py` — `capture_fresh(cap, flush_n)`, camera config.
  - `deepmap/build_map.py` / `depth-anything/src/merge_360.py` — depth→3D, `transform_to_world`, `write_ply`, `to_o3d`, `icp_merge`.
- Constraint: no encoder/IMU → pose is dead-reckoned from commanded moves (consistent with project odometry). stepper_ctrl owns /dev/ttyUSB0 per move; nothing else may hold the port.

## Expected output
`python deepmap/explore_map.py --max-steps 12 --step-cm 30` makes the robot:
head toward the largest reachable free region, advance in 30 cm steps, and after
each step append that view's 3D points (placed at the robot's accumulated pose)
into `output/explore_map.ply` (+ incremental BEV). Offline `--test-explore` runs
the full loop geometry with a fixed image, no robot/camera.

## Plan
- [x] 1. New `deepmap/explore_map.py` skeleton + argparse. Mirror scan360's import-and-call style; copy no logic.
- [x] 2. `Pose` tracker: accumulate world `(x, z, yaw)` from each commanded forward + turn. Dead-reckoned.
- [x] 3. `goal_from_bev(grid)`: largest reachable free region (`grid==3`) → centroid bearing → `(turn_deg, n_free, ahead_clear)`.
- [x] 4. `transform_to_world_pose(pts, pose, cam_offset)`: merge_360 rotation + translation by `(x, z)`.
- [x] 5. Per-step pipeline reusing depth_to_points / obstacle_grid / merge_360; incremental save + optional voxel.
- [x] 6. Main loop: capture → BEV → goal → turn → merge → forward → update Pose → stop on cap / no-area / unsafe.
- [x] 7. `--test-explore` offline path validated (pose accumulation + world placement, final bounds).
- [x] 8. Updated `deepmap/readme.txt` + `agent/plan/deepmap_plan.md`.

## Execution log
[--:--] step 1-6: wrote deepmap/explore_map.py (Pose, goal_from_bev, transform_to_world_pose, perceive, main loop) | info: reuses obstacle_grid+robot_drive+rotate_scan+depth_to_3d+merge_360, no copied logic
[--:--] step 4: transform_to_world only rotates → added translation by pose (x,z); local pts feed nav grid, world pts feed map | info: depth_to_points frame = X right/Y up/Z forward, floor Y~0
[--:--] step 7: offline test PASS | info: --test-explore shot_000 4 steps → pose (0,0,0°)→(0.42,1.11,33°), 1.20m, bounds fan out; PLY written. test.jpg scene is obstacle-heavy → stops step0 (correct)
[--:--] risk: no encoder/IMU → dead-reckon drift on long runs; --voxel ICP mitigates. Untested on real hardware.
[--:--] fix v1 (z-buffer nearest_merge) INSUFFICIENT — box still 3 copies. Copies are spatially-offset (mis-registration), not concentric shells. Also open3d NOT installed → icp_merge/--voxel never ran.
[--:--] fix v2: metric floor-anchor (level_and_scale) — per frame fit floor, level (Rodrigues) + scale to camera-height → all frames metric+gravity-aligned → captures overlap. Default --metric-floor. numpy voxel_dedup. info: per-frame scale now consistent (~1.40); obstacle thresholds now real metres.
[--:--] fix v3: user installed open3d 0.16.0 → added --merge icp (default): point-to-plane ICP refines residual dead-reckon drift after metric-anchor, merges on final save. info: offline fitness 0.93→1.00 rmse ~0.012-0.018. Modes icp|all|nearest. Needs hardware re-test.

## Test result
<!-- Filled when user confirms. -->
