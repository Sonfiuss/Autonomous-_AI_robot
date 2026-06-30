==============================================================
 slam — SLAM mono RGB-D bằng ROS 2 / RTAB-Map
==============================================================

CHỨC NĂNG
  Dựng bản đồ 3D diện rộng VỪA CHẠY VỪA MAP, thay cho cách ghép-360°-theo-góc
  của deepmap. Một camera USB + Depth Anything V2 cho RGB-D metric, odometry
  bánh xe ESP32 làm motion prior, RTAB-Map ghép lại bằng loop closure.

  Kế hoạch + quyết định thiết kế đầy đủ:
    agent/tasks/2026-06-28_rtabmap-slam.md  (nguồn sự thật)

CÀI ĐẶT MỘT LẦN
  # 1) Cài RTAB-Map + tooling camera (CẦN sudo — chạy tay):
  sudo apt-get install -y ros-foxy-rtabmap-ros ros-foxy-image-pipeline \
       ros-foxy-camera-calibration ros-foxy-image-transport ros-foxy-usb-cam \
       ros-foxy-cv-bridge

  # 2) Build workspace:
  source slam/env.sh
  ( cd slam/ros2_ws && colcon build --symlink-install )
  source slam/env.sh        # re-source để overlay workspace mới

  # 3) Intrinsics + scale metric:
  python slam/make_intrinsics.py        # tạo camera.yaml
  python slam/calibrate_scale.py        # tạo config/scale.yaml (hệ số k)

CHẠY
  source slam/env.sh                    # ROS 2 Foxy KHÔNG tự source
  # khởi chạy bằng launch file trong slam/launch/ hoặc slam_bringup
  # (xem README.md để biết launch cụ thể)

  python slam/robot_drive.py            # lái robot để quét diện rộng

CẤU TRÚC
  env.sh                 source TRƯỚC TIÊN
  calibrate_scale.py     hồi phục tỉ lệ metric (k) → config/scale.yaml
  make_intrinsics.py     tạo intrinsics camera mono
  robot_drive.py         lái robot (core-control + stepper_ctrl)
  config/
    extrinsics.yaml      TF base_link → camera (REP-103; lệch tâm 15.6 cm)
    camera.yaml          intrinsics mono (PLACEHOLDER tới khi calib)
    scale.yaml.template  template; scale.yaml thật do calibrate_scale.py tạo
    rtabmap.yaml         tham số RTAB-Map
  ros2_ws/src/
    serial_bridge/       chủ /dev/ttyUSB0: ESP32 odom → /odom, /cmd_teleop → ESP32
    depth_anything_node/ DA-V2 → /depth/image_raw (32FC1 mét)
    slam_bringup/        node static_extrinsics + launch files
  maps/                  map.ply / map.pgm+yaml xuất ra
  launch/                các file launch

GHI CHÚ
  Chi tiết đầy đủ xem README.md cùng thư mục.
