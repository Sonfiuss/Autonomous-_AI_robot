==============================================================
 motivation — Bộ điều khiển chuyển động + cầu nối ESP32 (C++)
==============================================================

CHỨC NĂNG
  Điều khiển robot omni-wheel thực tế: gửi lệnh vận tốc/đường đi xuống
  ESP32 qua UART, nhận odometry trả về, và cầu nối ZMQ với module
  simulation để chạy navigation theo goal.

BUILD
  mkdir -p build && cd build
  cmake .. && make -j$(nproc)
  → tạo 2 file thực thi: motivation, stepper_ctrl

CHẠY
  ./motivation --port /dev/ttyUSB0 [options]

  Options:
    --port     cổng serial ESP32 (mặc định /dev/ttyUSB0)
    --teleop   điều khiển bàn phím real-time (WASD + QE; IJKL cho camera)
    --demo     chạy demo (đi 5m, xoay 90°, v=3m/s)
    --nav      chế độ navigation: nhận goal từ simulation qua ZMQ
    --hz       tần số vòng lặp điều khiển (mặc định 20)

GIAO THỨC UART (Jetson → ESP32)
  M <vx> <vy> <omega>   vận tốc (m/s, m/s, rad/s)
  F <dist_m> <spd_ms>   đi thẳng N mét
  T <angle_deg> <rads>  xoay N độ
  V <pan_v> <tilt_v>    vận tốc servo camera (deg/s)
  S                     dừng
  R                     reset odometry về 0

GIAO THỨC UART (ESP32 → Jetson)
  O <x> <y> <theta>     odometry 10 Hz
  P <pan> <tilt>        góc servo 50 Hz
  K                     ack hoàn thành chuyển động
  READY                 ack khởi động

ZMQ (WiFi với laptop simulation)
  5555  simulation → motivation   G <x_m> <y_m> <theta_deg>  (goal)
  5556  motivation → simulation   O <x_m> <y_m> <theta_deg>  (odom)
  Jetson BIND cả hai cổng; laptop CONNECT tới IP Jetson.

FILE CHÍNH
  main.cpp           entry point
  MotionClient.*     gửi/nhận UART với ESP32
  StepperClient.*    điều khiển stepper trực tiếp
  PoseController.hpp compute(current, target) → vx, vy, omega, reached
  SimBridge.*        cầu nối ZMQ với simulation
  Teleop.*           điều khiển bàn phím
  esp32_unified_controller/  firmware ESP32 (.ino)
  upload_esp32.sh    nạp firmware lên ESP32
  cam_pan.py         điều khiển pan camera (Python)

CẢNH BÁO
  motivation và stereo-camera DÙNG CHUNG 1 ESP32 trên /dev/ttyUSB0 —
  KHÔNG chạy đồng thời trên cùng cổng.
