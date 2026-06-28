# Task: Wide-area mapping with RTAB-Map (ROS 2)

| Field | Value |
|-------|-------|
| Date | 2026-06-28 |
| Module | slam (new) — RTAB-Map graph SLAM |
| Status | planning |

## Task
Replace the in-place 360° / naive odometry point-cloud merge with a real graph-SLAM
system (RTAB-Map) so the robot can map a **wide area** while driving, with loop
closure correcting drift over long paths.

## Environment (probed 2026-06-28)
- Ubuntu 20.04 (Focal), L4T R35.6.4 (JetPack 5.1.x), Tegra t186, ~7.3 GB RAM
- ROS 2 **Foxy** present at `/opt/ros/foxy` (not auto-sourced)
- RTAB-Map: **not installed**
- torch 2.1 + CUDA OK; OpenCV 4.2; camera `/dev/video0` (mono, left only)
- ESP32 odometry stream: `O <x> <y> <theta_deg>` @10Hz over `/dev/ttyUSB0` (115200)

## Input
- Single USB camera `/dev/video0` (mono RGB)
- ESP32 wheel odometry (`O x y theta`) — existing motivation/MotionClient protocol
- Depth Anything V2 `vits` checkpoint (relative inverse depth) → scaled to metres
  via motion-parallax calibration

## Expected output
- ROS 2 SLAM stack producing a globally consistent map with loop closure
- Persistent RTAB-Map database (`~/.ros/rtabmap.db`)
- Exported map: 3D point cloud (.ply) + 2D occupancy grid (.pgm/.yaml)
- Reusable launch files + odometry bridge node under `slam/`

---

## KEY DECISIONS (RESOLVED with user 2026-06-28)

### D1 — Depth source: **Mono RGB-D via Depth Anything V2 + motion-parallax scale**
- Single camera `/dev/video0`. DA-V2 `vits` checkpoint outputs **relative inverse
  depth** `d = k/Z` where `k` is unknown. Metric scale `k` is recovered once by
  driving forward a known distance `a` and comparing matched feature depths between
  the two frames (see Step A4). Saved to `slam/config/scale.yaml`, reloaded on start.
- `k` is treated as a session constant. If scene/lighting changes drastically,
  re-run the calibration drive before mapping.
- No stereo cameras. `/dev/video1` is unused. The hardware-sync risk is eliminated.

### D2 — Odometry: **wheel odometry (ESP32)** as `/odom`
- RTAB-Map consumes external wheel odom as the motion prior. DA-V2 depth drives
  map geometry + loop-closure ICP. DA-V2 runs at ~1–3 Hz; odom at 10 Hz.
- Future upgrade: fuse wheel + visual (+IMU) via `robot_localization` EKF.

### D3 — Serial `/dev/ttyUSB0`: **teleop + odom multiplexed in ONE node**
- The serial bridge is the **sole owner** of the port during mapping.
  It reads `O x y theta` AND forwards teleop commands (`M/F/T/V/S`).
- `motivation` and `stereo-camera` MUST NOT run concurrently (port conflict).

### D4 — Install: **apt first, source fallback** (Foxy is EOL).

### D5 — Workspace: new top-level `slam/` (ROS 2 packages isolated from `deepmap/`).

### D6 — Point cloud merge policy: **Nearest-Wins**
- Global map is divided into voxels of size `v` (default 5 cm).
- When a new scan arrives, each new point is transformed to the global frame.
  - Voxel **empty** → insert the new point directly (new geometry).
  - Voxel **occupied** → compare `new_depth` vs `existing_obs_depth`:
    - `new_depth < existing_obs_depth` → **replace** stored point with new point.
    - `new_depth >= existing_obs_depth` → **discard** new point, keep existing.
- Rationale: closer observations have lower DA-V2 relative error → more accurate
  3D position in the global frame. The nearer view always wins.
- Pre-filter: clamp depth at `D_max` (default 5 m) before voxel lookup.
  DA-V2 is unreliable beyond ~5 m for a standard indoor/corridor scene.
- Implementation: custom `cloud_merger` node in Part B (Step B3), or configured
  via RTAB-Map's `cloud_voxel_size` + a per-point depth metadata pass.

---

## PLAN

---

### PART A — Initial 3D map construction

Goal: build the first seed map from a fixed origin with metric-scaled depth,
before the robot begins wide-area exploration. The robot makes one short
forward-and-back calibration move to lock in the depth scale `k`.

---

#### A1 — Scaffold `slam/` ROS 2 workspace
**Do:** Create `slam/ros2_ws/src/` colcon workspace. Add `slam/env.sh`:
```bash
source /opt/ros/foxy/setup.bash
source ~/Documents/slam/ros2_ws/install/setup.bash
```
Add `slam/config/` for YAML params. Add `slam/README.md` with sourcing instructions.
- **Risk:** Foxy not sourced by default → "package not found" everywhere. Mitigate
  with `env.sh`; add a check that prints a warning if `ROS_DISTRO != foxy`.

#### A2 — Install RTAB-Map for ROS 2 Foxy
**Do:** `sudo apt install ros-foxy-rtabmap-ros ros-foxy-image-pipeline
ros-foxy-camera-calibration ros-foxy-image-transport`
- **Risk (HIGH):** Foxy is EOL — arm64 debs may be missing. Fallback: build
  `rtabmap` + `rtabmap_ros` from source (~1–2 h on Jetson, needs ~3 GB free).
- **Risk:** apt + source version skew. Keep both from the same origin.
- **Info needed:** ROS 2 apt source list on Jetson?
  (`ls /etc/apt/sources.list.d/ | grep ros`). Internet access available?

#### A3 — Mono camera bring-up + intrinsic calibration
**Do:** Launch `usb_cam` node on `/dev/video0`, publishing `/rgb/image_raw` +
`/rgb/camera_info`. Calibrate intrinsics (fx, fy, cx, cy, distortion) once using
`camera_calibration` with a checkerboard. Save to `slam/config/camera.yaml`.
- **Note:** No stereo, no sync problem. One camera only.
- **Risk (HIGH):** Intrinsics are required even for mono DA-V2 depth. Wrong fx/fy
  → 3D back-projection is warped → map distorts during translation. The old
  `build_map.py` used FOV approximations; RTAB-Map needs real `camera_info`.
- **Risk:** Resolution/USB bandwidth. Use 640×480 MJPG; fresh-frame grabbing to
  avoid V4L2 buffer accumulation lag.
- **Info needed:** Checkerboard available? Native resolution + FPS of the lens?

#### A4 — DA-V2 depth node + motion-parallax scale recovery
**Do (depth node):** Write `slam/ros2_ws/src/depth_anything_node` (rclpy):
- Subscribe `/rgb/image_raw`
- Run DA-V2 `vits` inference (fp16, `torch.inference_mode()`)
- If `slam/config/scale.yaml` exists: load `k`, publish `k / d_relative` as
  `32FC1` `/depth/image_raw`, clamp to `[0, D_max]`, copy RGB header exactly.
- If `scale.yaml` absent: publish raw relative inverse depth (calibration mode).

**Do (scale calibration procedure):**
1. Robot at origin P1. Node captures relative depth map D1; extract ORB keypoints
   + descriptors. Save D1 + keypoints to RAM.
2. Send ESP32 command: drive forward `a` cm (e.g. 30 cm). Wait for odom
   confirmation that `|Δx| ≈ a`.
3. Robot at P2. Capture D2 + keypoints. Match features D1↔D2 (BFMatcher).
4. For each inlier match (pixel `p1` ↔ `p2`): compute
   `k_i = a · d1[p1] · d2[p2] / (d2[p2] − d1[p1])`.
   Filter: discard `k_i < 0` or outliers (IQR). Take median `k`.
5. Save `k` and `D_max` to `slam/config/scale.yaml`. Print confirmation.
6. Robot drives back to origin (optional; odom tracks position).

- **Risk (HIGH — scale validity):** `k` assumes a static scene during the
  calibration move. Any moving object invalidates matched pairs → outlier filter
  (IQR/RANSAC on `k_i` distribution) must be robust. Discard pairs where
  `d2 ≤ d1` (robot moved toward point → denominator should be positive).
- **Risk (HIGH — GPU/RAM):** DA-V2 + RTAB-Map on 7.3 GB. Use `vits`, fp16,
  small input (518×518 or smaller), low publish rate (1–2 Hz). Do NOT run
  `deepmap` pipeline concurrently.
- **Risk:** DA-V2 artifacts at edges/reflective surfaces/textureless regions →
  add edge mask (Sobel gradient) and `D_max` clamp before publishing.
- **Info needed:** Indoor or outdoor? Max expected depth range (sets `D_max`).
  Minimum texture in calibration area for ORB matching.

**Depth filters applied before publishing (derived from A6 geometry):**

Filter 1 — Near-clip (floor contact):
```
Discard pixels where depth_metric < 0.26 m
```
Any pixel closer than 26 cm in camera frame is ground within the bottom-FOV
region. Keeping these points floods the map with floor geometry.

Filter 2 — Far-clip:
```
Discard pixels where depth_metric > D_max  (default 5.0 m)
```

Filter 3 — Ground plane (applied after back-projection to global frame):
```
Discard any 3D point where z_global < z_floor + 0.02 m
```
`z_floor` is 0 by definition (Oxy plane = ground). The 2 cm margin avoids
clipping points resting on the floor that are legitimate obstacles.

Filter 4 — Ceiling clamp (optional, for indoor scenes):
```
Discard any 3D point where z_global > ceiling_height - 0.05 m
```
The top-center ray (21° above horizontal) can see ceiling. Set
`ceiling_height` in `slam/config/scale.yaml` alongside `k` and `D_max`.

#### A5 — Serial bridge node (odom IN + teleop OUT)
**Do:** Write `slam/ros2_ws/src/serial_bridge` (rclpy), sole owner of
`/dev/ttyUSB0`:
- Reader thread: parse `O x y theta_deg` → publish `nav_msgs/Odometry` on `/odom`
  + broadcast TF `odom → base_link`. Convert theta degrees → radians → quaternion.
- Writer: subscribe `/cmd_teleop` (string) → write ESP32 `M/F/T/V/S` commands.
- Single serial handle; writer acquires a lock before each write.
- **Risk (HIGH):** Port conflict if `motivation` is also running. Assert sole
  ownership; print error and exit if port is busy.
- **Risk:** Fixed odometry covariance — set higher variance on theta (drifts most).
- **Risk:** ±180° theta wrap → unwrap or use quaternion directly.
- **Info needed:** Teleop input: keyboard node or direct string topic?

#### A6 — TF tree + camera extrinsics
**Do:** Define frames: `odom → base_link → camera_link → camera_optical_frame`.
Publish static TF for `base_link → camera_link` using the known hardware geometry.

**Known hardware values (robot frame: X=forward, Y=right, Z=up, origin=rotation center):**
- Camera position: `(0.10, 0.12, 0.32)` m from robot rotation center
- Camera tilt: **15° nose-down** (pitch = −15° around Y axis)
- Vertical FOV: **72°** → half-FOV = 36°

Static TF to publish:
```
base_link → camera_link
  translation: x=0.10 m, y=0.12 m, z=0.32 m
  rotation:    roll=0°, pitch=−15°, yaw=0°
```

**Frustum geometry (derived, for filter parameters):**
```
top-center ray:    15° − 36° = 21° above horizontal  → sees walls/ceiling
center ray:        15° below horizontal
bottom-center ray: 15° + 36° = 51° below horizontal

ground contact of bottom-center ray (z=0 plane):
  x_ground = 0.10 + 0.32 × tan(90°−15°−36°)
           = 0.10 + 0.32 × tan(39°)
           ≈ 0.10 + 0.259
           ≈ 0.359 m from robot center

near-clip distance in camera frame: 0.359 − 0.10 = 0.259 m ≈ 26 cm
```

**Eccentric rotation radius:**
```
r = sqrt(0.10² + 0.12²) = sqrt(0.0244) ≈ 0.156 m (15.6 cm)
```
During the initial 360° sweep (Step A8), the camera traces a circle of radius
15.6 cm around the robot center. RTAB-Map correctly handles this only if the
above TF is exact — otherwise clouds from different rotation angles will smear
by up to 31 cm (diameter). Validate by checking that static objects appear as
sharp points (not arcs) in the assembled cloud after the sweep.

- **Risk (HIGH):** Wrong extrinsics → systematic map tilt and arc-smear on every
  object. The 15.6 cm eccentric radius is large enough to cause visible artifacts.
- **Risk:** optical vs link frame axis convention (optical: z fwd, x right, y down)
  → common source of 90° map rotations. Use `camera_optical_frame` with the
  standard ROS optical-frame rotation applied on top of the above TF.
- **Risk:** pitch sign convention varies by URDF tool — confirm nose-down is
  negative pitch in the chosen convention before publishing.

#### A7 — Time synchronization
**Do:** Depth node copies the **original RGB header** (timestamp + frame_id) to
the published depth image — do NOT stamp with `now()`. RTAB-Map uses
`approx_sync=true` with adequate `queue_size` (20) to pair RGB + depth + odom.
- **Risk:** If depth node stamps with `now()`, RTAB-Map cannot pair it with the
  correct odom pose (inference latency ~0.3–1 s). Header copy is mandatory.
- **Risk:** USB camera timestamps at arrival, not capture → consistent but adds
  fixed latency offset. Acceptable as long as it is consistent across topics.

#### A8 — First RTAB-Map launch (seed map)
**Do:** Launch `rtabmap` in RGB-D mode with external odometry:
```
subscribe_depth=true, subscribe_rgb=true
odom_frame_id=odom, frame_id=base_link
approx_sync=true, queue_size=20
Rtabmap/DetectionRate=1.0
Mem/IncrementalMemory=true
Mem/STMSize=30
cloud_voxel_size=0.05   (5 cm, matches D6)
Proj/MaxDepth=5.0       (matches D_max)
```
Robot stays near origin (or does a slow 360°) to build the initial seed map.
Stop cleanly → `~/.ros/rtabmap.db` persists.
- **Risk:** Parameter sprawl → start minimal, one change at a time.
- **Risk (RAM):** Keep `Mem/STMSize` small; rely on on-disk db. Watch for OOM.
- **Risk:** GPU contention between DA-V2 and RTAB-Map feature extraction →
  offset their compute: DA-V2 at 1–2 Hz, RTAB-Map `DetectionRate` 1 Hz.

---

### PART B — Incremental mapping during movement

Goal: robot drives through the wide area. Each new RGB-D keyframe is localized
against the existing map, new geometry is added (occluded regions revealed), and
the nearest-wins voxel policy prevents duplicate cloud points.

---

#### B1 — Apply saved scale `k` to all new depth frames
**Do:** On node start, `depth_anything_node` reads `slam/config/scale.yaml`.
Every new frame: `depth_metric = k / d_relative`, clamp to `[0, D_max]`.
No change to the rest of the pipeline — metric depth flows into RTAB-Map as before.
- **Risk:** `k` may drift if robot moves to a very different scene (different
  average depth range). Mitigation: re-run calibration drive (A4) before entering
  a new environment type. Log a warning if the median of the current depth image
  deviates > 2× from the calibration session's median.

#### B2 — Robot drives; RTAB-Map extends map
**Do:** Continue the RTAB-Map session (load existing `rtabmap.db`).
As the robot moves:
- Each new keyframe is matched against the map via SURF/ORB loop detection.
- ICP + pose-graph optimisation corrects accumulated odom drift.
- New point clouds are added for regions not yet seen (formerly occluded).
- **Risk (HIGH — loop closure):** Blank walls, low light, repetitive texture →
  few visual features → missed loop closures. Mitigate: adequate lighting,
  drive slowly, tune `Kp/MaxFeatures`, `Vis/MinInliers`.
- **Risk:** Wheel-odom drift accumulates over long paths → robot revisit may
  fall outside proximity detection radius. Tune `RGBD/ProximityBySpace`.
- **Risk:** False loop closures warp the map. Validate visually each run;
  raise `RGBD/OptimizeMaxError` threshold if needed.

#### B3 — Nearest-wins voxel merge (cloud_merger node)
**Do:** Write or configure the map assembler to enforce D6 policy:

For each incoming point cloud (in global frame after pose transform):
1. Pre-filter: remove points with `depth > D_max` or `depth <= 0`.
2. For each point `P_new` at voxel `V`:
   - If `V` is empty → insert `P_new`, store `obs_depth = depth_of_P_new`.
   - If `V` occupied and `depth_of_P_new < stored_obs_depth`:
     → replace stored point with `P_new`, update `stored_obs_depth`.
   - Else → discard `P_new`.

Implementation options (choose one):
- **Option A (preferred):** Use RTAB-Map's `rtabmap_ros/point_cloud_assembler`
  with `voxel_size=0.05`. Then post-process the assembled cloud with a custom
  node that re-runs nearest-wins using the per-point `z` value in sensor frame
  (requires keeping the sensor-frame depth as a point field).
- **Option B:** Write a standalone `cloud_merger` rclpy node that subscribes
  `/rtabmap/cloud_map` + per-scan `/depth_cloud`, maintains a voxel hashmap
  (`{voxel_key: (point_xyz, obs_depth)}`), and republishes the filtered global
  cloud on `/map_cloud`.

- **Risk:** Voxel hashmap grows with map size → RAM pressure on Jetson.
  Prune voxels outside a max-radius if memory is tight. Use `numpy` structured
  arrays or `open3d.geometry.VoxelGrid` for efficiency, not a Python dict.
- **Risk:** obs_depth stored in sensor frame; after pose correction (loop
  closure), the stored depth may no longer match the actual global position.
  Re-evaluate affected voxels after each pose-graph optimisation.

#### B4 — Rosbag recording
**Do:** Record a bag early (after B2 first drive) covering all topics:
`/rgb/image_raw`, `/depth/image_raw`, `/odom`, `/tf`, `/tf_static`.
This enables offline RTAB-Map reprocessing and tuning without re-driving.
- **Risk:** Bag fills disk fast at 640×480 @ 1–2 Hz depth. Use compressed
  transport (`image_transport compressed`) for RGB. Depth as raw 32FC1
  (lossless). Budget ~200 MB/min.

#### B5 — Loop closure tuning
**Do:** Drive a closed loop; confirm RTAB-Map fires a loop closure at the revisit.
Tune iteratively:
- `Kp/MaxFeatures` (number of ORB features per frame)
- `Vis/MinInliers` (minimum feature matches to accept a loop)
- `RGBD/OptimizeMaxError` (max graph-edge error before rejection)
- `Rtabmap/LoopThr` (score threshold)
- **Risk (mono-specific):** Loop correction transform uses DA-V2 depth. Scale
  inconsistency → biased correction. If `k` is good, this is fine. If the map
  warps on accepted closures, improve `k` first before re-tuning.

#### B6 — Map export + project integration
**Do:** After clean stop: export via `rtabmap-export` / `rtabmap-databaseViewer`:
- `map.ply` — 3D point cloud (view with `depth-anything/src/view_web.py` or
  MeshLab)
- `map.pgm` + `map.yaml` — 2D occupancy grid for `simulation/` path planner
- **Risk:** Coordinate scale mismatch with `simulation/`'s cm-based grid
  (see `interfaces.md CM_TO_M`). Apply scale factor in the export step.
- **Risk:** Hard-kill corrupts `rtabmap.db`. Always `ros2 node kill` cleanly,
  not `Ctrl+C` on the launch terminal directly.

#### B7 — Validation
**Do:** Define success criteria:
- (a) Loop closure fires and visibly corrects a known 1–2 m drift.
- (b) Cabinet / wall dimensions in exported cloud within ±10% of tape-measured
  ground truth.
- (c) No duplicate wall surfaces visible in the point cloud after a revisit.
- (d) Nearest-wins: cabinet appears once in the cloud, positioned at the closer
  observation's estimated distance from the global origin.
- **Info needed:** Ground-truth room dimensions for validation.

---

## OPEN QUESTIONS FOR USER
1. **Calibration environment (A4):** Indoor or outdoor? Max expected depth range
   (sets `D_max`, default 5 m)? Minimum texture available for ORB matching during
   the calibration drive?
2. **Checkerboard (A3):** Available for one-time mono intrinsic calibration?
3. **Teleop (A5):** Keyboard node or direct string topic on `/cmd_teleop`?
   Confirm `motivation` will be stopped before mapping.
4. **Network/apt (A2):** ROS 2 apt source configured on Jetson?
   (`ls /etc/apt/sources.list.d/ | grep ros`). Internet available?
5. **Map area scope (B2):** Rough size of area to map (sets `Mem/STMSize`
   and rosbag disk budget).
6. **Cloud merger (B3):** Prefer Option A (RTAB-Map assembler + post-process)
   or Option B (standalone `cloud_merger` node)?

---

## Execution log
| Time | Step | Summary |
|------|------|---------|
<<<<<<< HEAD
| (planning) | — | Plan drafted, awaiting approval |
=======
| (planning) | — | Plan split into Part A / Part B; D1 updated to motion-parallax scale; D6 nearest-wins merge added |
| 2026-06-28 | A4 (deepmap port) | Ported step A4 (motion-parallax metric scale) into non-ROS deepmap pipeline as a Test_round360 harness. New `deepmap/scale_calib.py` (apply_scale, project_metric, recover_scale_motion_parallax, calibrate, confirm_output_type). `--test-round360` flag on rotate_scan.py (fixed `right_1.jpg` for all 12 frames, no camera/UART) + build_map.py (skips 30 cm move, placeholder k, confirms metric-depth TYPE float32 HxW, builds PLY). | info: type path verified offline (PASS); full build_map test needs DA-V2 checkpoint on Jetson |
>>>>>>> 48cc49e6c0d117616f3daf3bbadfd095866a76ba
