---
id: 2026-07-04_stereo-walk-map
status: testing
module: deepmap
started: 2026-07-04
---

## Task
Drive the robot forward 3 steps × 0.3 m. At each stop capture a STEREO pair,
run the stereo-ruler scaled depth pipeline (DA-V2 + golden points + f*B) to get
a METRIC depth map, back-project to a 3D cloud, place it at the known odometry
pose (z = 0 / 0.3 / 0.6 m), and merge all 3 into one PLY. The stereo scale
(f*B from the 5.4 cm baseline) guarantees every frame shares the same real-metre
scale — replacing explore_map's floor-anchor heuristic (`level_and_scale`).

## Input
- `depth-anything/src/stereo_ruler.py` — per-frame metric depth (working:
  capture_pair, load_rectify, golden_points, infer_mono, fit_affine_ransac)
- `stereo-camera/calib/stereo_rectify.yml` — K, rectify maps, f*B
- `slam/robot_drive.py` — calibrated `drive_forward(cm)`
- `deepmap/explore_map.py` — Pose dead-reckon, voxel_dedup, icp_merge, save_cloud
- Cameras: left=/dev/video0, right=/dev/video2, 640×480
- open3d 0.16.0 installed (ICP refine available)

## Expected output
`python3 deepmap/stereo_walk_map.py --steps 3 --step-cm 30`
→ `deepmap/output/stereo_walk/` containing:
  - `shot_N_left.jpg` / `shot_N_right.jpg` (3 pairs)
  - `depth_N_m.npy` metric depth per stop
  - `walk_map.ply` single merged metric point cloud where the 3 clouds
    overlap consistently (same object at same world position, offsets ≈ 0.3 m)

## Plan
- [x] 1. Refactor stereo_ruler.py: extract a callable
      `compute_metric_depth(left_bgr, right_bgr, calib, model) -> (Z_m, valid)`
      so the walk script imports it (no subprocess, model loaded ONCE for all 3 shots).
- [x] 2. New `deepmap/stereo_walk_map.py`:
      loop { capture stereo pair → metric depth → back-project with rectified
      K (X=(u-cx)Z/fx, Y=(v-cy)Z/fy) → transform by Pose → accumulate;
      drive_forward(30) ; pose.advance(0.3) } × 3.
- [x] 3. Merge: concat → optional ICP refine (open3d, reuse icp_merge_clouds)
      → voxel_dedup → save walk_map.ply (+ per-shot artifacts).
- [x] 4. Offline test with saved pairs (--test mode, no robot / no cameras).
- [ ] 5. Hardware run: 3 real shots on the floor, verify a known object appears
      once (not 3 copies) and step offsets measure ≈ 0.3 m in the cloud.

## Execution log
[--:--] step 1: stereo_ruler.py refactor — load_model() (model loaded once, reusable), load_rectify() now returns namespace incl. fx/fy/cx/cy from P2 (back-projection K), new compute_metric_depth(left,right,calib,model)->(Z_m,valid,info) | risk: load_rectify return type changed (only caller was main(), updated) | info: calib + captures are 1280x720, NOT 640x480 as task Input said — walk script defaults to 1280x720
[23:20] step 2: deepmap/stereo_walk_map.py — capture→metric depth→back-project (Y up = -(v-cy)Z/fy)→level_to_floor (ROTATION-ONLY floor fit + floor→Y=0; stereo scale already metric, no scaling)→Pose→accumulate; drives between stops via robot_drive | risk: leveling assumes bottom 30% of image is floor; --no-level escape hatch | info: leveling needed because camera tilt (~13°) would otherwise break pose.advance along horizontal Z
[23:22] step 3: merge inside save_map — incremental cheap concat each stop, final icp_merge_clouds (open3d) + voxel_dedup 0.02 → walk_map.ply
[23:25] step 4: offline --test PASS — regression: stereo_ruler.py main unchanged output (41 golden, f*B=60.69); walk: 3 stops, 72695 pts/stop, icp fitness 1.00/0.54, 18029 pts final, all expected artifacts present | info: test pair has weak fit (8/41 inliers) → near-field depth biased → cam_h read 0.01-0.03 m; floor NORMAL (tilt ~13°) correct, so leveling rotation fine; absolute height quality depends on per-frame stereo fit

## Test result
Offline (`--test`): PASS — pipeline end-to-end, artifacts complete. Note: in test
mode the SAME pair is reused each stop, so the scene "moves with" the robot —
duplicate objects are expected there and are NOT the mis-registration bug.
Hardware (step 5): pending user run:
  `python3 deepmap/stereo_walk_map.py --steps 3 --step-cm 30`
Verify: known object appears ONCE in walk_map.ply and step offsets ≈ 0.3 m.
