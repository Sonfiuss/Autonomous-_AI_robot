# slam/ — ROS 2 / RTAB-Map mono RGB-D SLAM

Wide-area 3D mapping that replaces the deepmap naive 360° angle-merge with graph
SLAM: a single USB camera + Depth Anything V2 gives metric RGB-D, ESP32 wheel
odometry is the motion prior, and RTAB-Map fuses everything with loop closure.

Full plan + design decisions: `agent/tasks/2026-06-28_rtabmap-slam.md`.

## Layout
```
slam/
  env.sh                 source this first (ROS 2 Foxy is NOT auto-sourced)
  calibrate_scale.py     one-shot metric-scale (k) recovery -> config/scale.yaml
  config/
    extrinsics.yaml      base_link -> camera TF (REP-103; 15.6 cm eccentric)
    camera.yaml          mono intrinsics (PLACEHOLDER until you calibrate)
    scale.yaml.template  template; real scale.yaml is produced by calibrate_scale.py
    rtabmap.yaml         RTAB-Map parameters
  ros2_ws/src/
    serial_bridge/       /dev/ttyUSB0 owner: ESP32 odom -> /odom, /cmd_teleop -> ESP32
    depth_anything_node/ DA-V2 -> /depth/image_raw (32FC1 metres)
    slam_bringup/        static_extrinsics node + launch files
  maps/                  exported map.ply / map.pgm+yaml
```

## One-time setup
```bash
# 1. Install RTAB-Map + camera tooling (REQUIRES sudo — run manually):
sudo apt-get install -y ros-foxy-rtabmap-ros ros-foxy-image-pipeline \
     ros-foxy-camera-calibration ros-foxy-image-transport ros-foxy-usb-cam \
     ros-foxy-cv-bridge

# 2. Build the workspace:
source slam/env.sh
( cd slam/ros2_ws && colcon build --symlink-install )
source slam/env.sh        # re-source to overlay the new workspace

# 3. Intrinsics. Mono DA-V2 still needs fx/fy/cx/cy to back-project depth into 3D
#    (NOT a stereo thing). Two options:
#    (a) FOV estimate (no checkerboard) — generic webcam ~60 deg HFOV:
python slam/make_intrinsics.py --hfov 60 --width 640 --height 480
#    (b) checkerboard (accurate fx/fy + distortion), if the map bows/scales wrong:
ros2 launch slam_bringup camera.launch.py
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
     image:=/rgb/image_raw camera:=/rgb
```

## Each mapping session
```bash
# A. Recover metric scale (port-free; robot drives forward 30 cm). Writes scale.yaml.
python slam/calibrate_scale.py --cam 0 --port /dev/ttyUSB0 --a-cm 30

# B. Build the seed map (fresh db), then drive slowly through the area:
source slam/env.sh
ros2 launch slam_bringup bringup.launch.py rtabmap_args:=--delete_db_on_start
#   drive via:  ros2 topic pub --once /cmd_teleop std_msgs/String "{data: 'F 0.3 0.1'}"

# C. Export when done (clean stop, NOT a hard kill):
rtabmap-export --cloud --output slam/maps/map ~/.ros/rtabmap.db
```

## Rules
- **Single owner of `/dev/ttyUSB0`.** Never run `motivation`, `stereo-camera`, or
  `deepmap` while the SLAM stack (or calibrate_scale.py) holds the port.
- Run `calibrate_scale.py` whenever the scene/lighting changes a lot — `k` is a
  per-session constant.
- Always stop RTAB-Map cleanly; a hard kill can corrupt `~/.ros/rtabmap.db`.
