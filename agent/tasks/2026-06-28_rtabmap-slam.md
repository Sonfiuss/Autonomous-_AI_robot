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
- torch 2.1 + CUDA OK; OpenCV 4.2; cameras `/dev/video0`, `/dev/video1`
- ESP32 odometry stream: `O <x> <y> <theta_deg>` @10Hz over `/dev/ttyUSB0` (115200)

## Input
- Stereo USB cameras (video0=left, video1=right)
- ESP32 wheel odometry (`O x y theta`) — existing motivation/MotionClient protocol
- (Optional/experimental) Depth Anything V2 mono depth as RGB-D fallback

## Expected output
- ROS 2 SLAM stack producing a globally consistent map with loop closure
- Persistent RTAB-Map database (`~/.ros/rtabmap.db`)
- Exported map: 3D point cloud (.ply) + 2D occupancy grid (.pgm/.yaml)
- Reusable launch files + odometry bridge node under `slam/`

---

## KEY DECISIONS (RESOLVED with user 2026-06-28)

### D1 — Depth source: **MONO RGB-D via Depth Anything V2** (user choice)
- User commits to Depth Anything V2 for depth. We feed RTAB-Map in **RGB-D mode**
  (rgb image + a registered depth image + camera_info + external odometry).
- **Consequence (THE central problem): metric scale.** The base DA-V2 checkpoint
  outputs *relative inverse depth*, not metres. RTAB-Map needs metric, consistent
  depth. Two ways forward — pick in Step 0:
  - **(Recommended) Use the DA-V2 *Metric* checkpoint** (indoor=Hypersim,
    outdoor=VKITTI). Outputs absolute metres directly, roughly consistent across
    frames. We currently only have the *relative* `vits` checkpoint → must download
    a metric one.
  - **Scale-calibrate the relative depth** to metres via a fixed reference (known
    floor-plane height, or align scale to wheel-odom translation). Fragile, drifts.
- **Upside of mono (vs stereo):** depth is computed FROM the rgb image, so it is
  **pixel-aligned to rgb by construction** — no stereo registration, and the
  HIGH "two USB cameras not hardware-synced" risk disappears entirely. Only ONE
  camera (`/dev/video0`) is used.

### D2 — Odometry: **wheel odometry (ESP32)** as `/odom`
- RTAB-Map consumes external wheel odom as the motion guess; depth is used for
  the map + loop-closure geometry, RGB for loop *detection*. This avoids needing
  high-rate depth for visual odometry → DA-V2 can run at a low ~1–3 Hz.
- Future upgrade: fuse wheel + visual (+IMU) via `robot_localization` EKF.

### D3 — Serial `/dev/ttyUSB0`: **teleop + odom multiplexed in ONE node** (user choice)
- The robot is driven while mapping. The bridge node is the **sole owner** of the
  port: it reads `O x y theta` AND forwards teleop motion commands (`M/F/T/V/S`).
- `motivation` / `stereo-camera` must NOT run concurrently (port conflict).

### D4 — Install: **apt first, source fallback** (Foxy is EOL).

### D5 — Home directory: new top-level `slam/` (ROS 2 packages isolated from the
  Python depth tooling in `deepmap/`).

---

## PLAN (numbered, with deep analysis per part)

### Step 1 — Confirm decisions + scaffold `slam/` ROS 2 workspace
**Do:** Create `slam/ros2_ws/src/` colcon workspace; decide D1–D3; add a
`slam/README.md` recording the sourcing command (`source /opt/ros/foxy/setup.bash`).
- **Risk:** Foxy not sourced by default → every terminal must source it; easy to
  forget and get "package not found". Mitigate with a `slam/env.sh` helper.
- **Info needed:** Confirm D1 (stereo) and D3 (slam/) from user.

### Step 2 — Install RTAB-Map for ROS 2 Foxy
**Do:** `sudo apt install ros-foxy-rtabmap-ros` (+ `ros-foxy-stereo-image-proc`,
`ros-foxy-image-pipeline`, `ros-foxy-camera-calibration`).
- **Risk (HIGH):** Foxy is EOL — the apt repo key/sources may be stale or the
  arm64 debs pulled. If `apt` fails → fallback is **build rtabmap + rtabmap_ros
  from source** (long compile on Jetson, ~1–2h, needs ~3GB free + swap).
- **Risk:** Version skew between `rtabmap` (lib) and `rtabmap_ros` if mixing
  apt + source. Keep both from the same origin.
- **Info needed:** Is the ROS 2 apt source list still configured?
  (`ls /etc/apt/sources.list.d/ | grep ros`). Internet access on the Jetson?
- **Decision point:** apt-binary (fast) vs source-build (robust but slow).

### Step 3 — Mono camera bring-up + monocular intrinsic calibration
**Do:** Run a single USB camera node (`/dev/video0`) publishing `/rgb/image_raw`
+ `/rgb/camera_info`. Calibrate ONE camera's intrinsics (fx, fy, cx, cy, distortion)
with `camera_calibration` (checkerboard) → camera_info YAML.
- **Note:** NO stereo calibration, NO stereo sync problem (only one camera). The
  HIGH async-stereo risk from the original plan is GONE.
- **Risk (still HIGH):** Intrinsics are STILL required. DA-V2 gives depth per pixel
  but not camera geometry. Wrong fx/fy/cx/cy → depth-to-3D is warped → map warps
  as the robot translates. The earlier `build_map.py` used an FOV *approximation*;
  RTAB-Map needs a real `camera_info`. Budget time for one mono calibration.
  (User said "I use DA-V2 for depth" — clarify this calibration is still needed.)
- **Risk:** Resolution/rate vs USB + GPU budget. Use modest res (e.g. 640×480),
  MJPG, fresh-frame grabbing to avoid V4L2 buffer lag.
- **Info needed:** Confirm `/dev/video0` is the intended camera; its native
  resolution/FPS; checkerboard available for calibration? Lens FOV.

### Step 3b — Depth Anything V2 ROS 2 depth node (NEW, core of mono path)
**Do:** Write `slam/ros2_ws/src/depth_anything_node` (rclpy): subscribe `/rgb/image_raw`,
run DA-V2 inference, publish `/depth/image_raw` as `32FC1` **in metres**, copying the
RGB header (same timestamp + frame_id) so it stays pixel-aligned & time-aligned.
- **Risk (HIGH — METRIC SCALE, the crux):** must output METRES. Use the DA-V2
  *Metric* checkpoint, or apply a calibrated scale to the relative output. Inconsistent
  scale across frames → RTAB-Map map scale drifts, loop-closure geometry biased.
- **Risk (HIGH — GPU/RAM):** DA-V2 + RTAB-Map share one ~8GB Jetson + one GPU.
  Mitigate: smallest encoder (`vits`), small input-size, fp16 (`model.half()`),
  `torch.inference_mode()`, publish depth at low rate (~1–3 Hz, RTAB-Map
  `Rtabmap/DetectionRate` low). Do NOT also run the old `deepmap` pipeline.
- **Risk:** Mono-depth artifacts — flying pixels at object edges, wrong depth on
  reflective/transparent/textureless surfaces, sky/far regions. Add edge-aware
  masking + max-depth clamp before publishing, else the map fills with noise.
- **Risk:** Inference latency while moving → depth lags the pose. Wheel odom is the
  motion source so this is tolerable, but keep rate up / drive slowly.
- **Info needed:** Confirm metric-checkpoint download is OK (size ~hundreds MB,
  VRAM). Indoor (Hypersim) or outdoor (VKITTI) scene? Expected depth range (max m).

### Step 4 — Serial bridge node: odometry IN + teleop OUT (ESP32 ↔ ROS 2)
**Do:** Write `slam/ros2_ws/src/serial_bridge` (rclpy), the **sole owner** of
`/dev/ttyUSB0`: (a) read `O x y theta` → publish `nav_msgs/Odometry` on `/odom`
+ broadcast TF `odom -> base_link`; (b) subscribe a teleop topic (e.g.
`/cmd_vel` or keyboard) → forward `M/F/T/V/S` commands to the ESP32.
- **Risk (HIGH — RESOURCE CONFLICT):** one process must own the port. This node
  multiplexes read+write; `motivation`/`stereo-camera` MUST be stopped during mapping.
- **Risk:** Read/write contention on one serial handle from multiple threads →
  interleaved bytes. Use a single IO thread or a lock; keep command writes short.
- **Risk:** ESP32 odometry has no covariance → set sensible fixed covariance,
  higher variance on theta (drifts most).
- **Risk:** `O` theta is DEGREES → convert to radians + quaternion; handle ±180
  wrap. Match REP-103 (x fwd, y left, z up, CCW+).
- **Risk:** Timestamp = arrival vs measurement; serial latency jitter affects sync.
- **Info needed:** Teleop input device (keyboard/gamepad/`/cmd_vel`)? Mapping
  between `/cmd_vel` (vx,vy,ω) and ESP32 `M`/`T` commands.

### Step 5 — TF tree + extrinsics (URDF)
**Do:** Define `base_link`, `base_footprint`, `camera_link` (+ optical frames).
Measure/calibrate camera mounting offset & orientation vs robot center. Static
TF publisher or minimal URDF.
- **Risk (HIGH):** Wrong camera extrinsics (position/tilt vs base_link) →
  systematic map tilt/offset that grows with translation. Pure rotation hid this;
  driving exposes it. Measure carefully (mm + degrees) or hand-eye calibrate.
- **Risk:** optical_frame vs link_frame axis conventions (camera optical = z
  forward, x right, y down) — a classic source of 90° map rotations.
- **Info needed:** Physical camera mount geometry relative to wheel center.

### Step 6 — Time synchronization
**Do:** Sync `/rgb/image_raw` + `/depth/image_raw` + `/odom`. Because depth copies
the rgb header, rgb↔depth are exact; rgb↔odom need `approx_sync=true` + queue_size.
- **Risk:** DA-V2 inference latency means depth is published well after the rgb
  instant — but since it carries the ORIGINAL rgb timestamp, RTAB-Map still pairs
  it with the correct odom pose. Ensure the node preserves the header (do NOT
  stamp with "now").
- **Risk:** Async rgb + serial odom → `message_filters` may not match → RTAB-Map
  "did not receive data" silence. Mitigate `approx_sync=true`, adequate queue_size.
- **Info needed:** Is the camera frame timestamped at capture or arrival?

### Step 7 — RTAB-Map node configuration & first launch
**Do:** Launch `rtabmap` in **RGB-D mode**: `subscribe_depth=true`,
`subscribe_rgb=true`, `odom_frame_id=odom`, `frame_id=base_link`, external odom.
Start conservative, `Mem/IncrementalMemory=true`, loop closure on.
- **Risk:** Parameter sprawl — RTAB-Map has hundreds of params. Start minimal,
  change one thing at a time.
- **Risk (Jetson RAM):** Working memory + feature DB + point clouds in ~8GB RAM
  is tight. Use `Mem/STMSize`, `Rtabmap/DetectionRate` (~1Hz), map assembling
  off during run, and rely on the on-disk db. Watch for OOM/swap thrash.
- **Risk:** GPU contention if Depth Anything also runs — don't run both on GPU.
- **Info needed:** Target area size → sets memory management strategy
  (incremental vs memory-managed graph).

### Step 8 — Mapping run + loop closure tuning
**Do:** Drive a closed-loop trajectory; verify loop closures fire (revisits
correct the graph). Tune `Kp/MaxFeatures`, `Vis/MinInliers`,
`RGBD/OptimizeMaxError`, `Rtabmap/LoopThr`.
- **Risk (HIGH):** Loop closure failures — too few visual features (blank walls,
  low light), motion blur, repetitive texture (false positives). Mitigate:
  good lighting, drive slower, enough scene texture, raise `Vis/MinInliers`.
- **Risk (HIGH, mono-specific):** Loop *detection* uses RGB features (fine with
  mono), but the loop *correction transform* uses DA-V2 depth. If depth scale is
  inconsistent, accepted loop closures apply a biased correction → map can warp
  even when the loop is "correct". Watch `RGBD/OptimizeMaxError`; if scale is
  shaky, loosen it cautiously or improve the metric depth first.
- **Risk:** Wheel-odom drift so large the visual proximity detection can't link
  revisits → tune `RGBD/ProximityBySpace`, increase odom covariance.
- **Risk:** False loop closures warp the whole map — validate visually each run.
- **Info needed:** Floor surface (slip?), lighting conditions, scene texture.

### Step 9 — Map persistence, export, project integration
**Do:** Confirm `~/.ros/rtabmap.db` persists; export cloud `.ply` + 2D grid
`map.pgm/.yaml` via `rtabmap-export`/`rtabmap-databaseViewer`. Wire output into
existing project: reuse `depth-anything/src/view_web.py` for the PLY, or document
how `simulation/` could consume the occupancy grid.
- **Risk:** db corruption on hard kill — always stop the node cleanly.
- **Risk:** Coordinate/scale mismatch with simulation's cm-based grid (see
  interfaces.md CM_TO_M) when feeding the occupancy map to the planner.
- **Info needed:** Which downstream consumer matters first — 3D cloud viewer or
  2D grid for navigation/simulation?

### Step 10 — Validation & testing
**Do:** Define success: (a) loop closure corrects a known loop; (b) measured map
dimensions within X% of ground truth; (c) no duplicate walls after revisit.
Record a rosbag of a representative run for offline reprocessing/tuning so we can
iterate without re-driving.
- **Risk:** Without a rosbag, every tuning iteration needs a physical re-run.
  Record bags EARLY (after step 6) — biggest time-saver.
- **Info needed:** Ground-truth measurements of the test area for validation.

---

## OPEN QUESTIONS FOR USER (remaining, after decisions D1–D5)
1. **Metric scale (Step 3b, the crux):** OK to download a DA-V2 *Metric* checkpoint
   (indoor Hypersim / outdoor VKITTI)? Or must we stick to the relative `vits`
   checkpoint + a calibrated scale hack? Indoor or outdoor scene? Max depth range?
2. **Mono intrinsic calibration (Step 3):** confirm we can do a one-time checkerboard
   calibration of `/dev/video0` (still required even for mono depth). Checkerboard available?
3. **Teleop input (Step 4):** keyboard, gamepad, or `/cmd_vel`? Confirm `motivation`
   will be stopped so the bridge can own `/dev/ttyUSB0`.
4. **Network/apt (Step 2):** internet + ROS 2 apt source available on the Jetson?
   (`ls /etc/apt/sources.list.d/ | grep ros`)
5. **Scope:** rough size of the area to map (sets RTAB-Map memory strategy).

## Execution log
| Time | Step | Summary |
|------|------|---------|
| (planning) | — | Plan drafted, awaiting approval |
