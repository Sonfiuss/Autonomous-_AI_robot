==============================================================
 stereo-camera — Chụp độ sâu stereo + dựng point cloud 3D (C++)
==============================================================

CHỨC NĂNG
  Dùng cặp camera USB (trái=0, phải=1) để đo độ sâu stereo, dựng point
  cloud 3D, và điều khiển servo pan/tilt để quét. Có công cụ tính khoảng
  cách, free-space, và hiệu chỉnh (calib).

BUILD
  mkdir -p build && cd build
  cmake .. && make -j$(nproc)
  → file thực thi trong stereo-camera/build/

CHẠY (tham khảo project_overview)
  ./stereo_scan --output scan.ply        # quét + lưu point cloud

CẤU TRÚC
  vision/     đo độ sâu / khoảng cách stereo (testdistance)
  control/    điều khiển servo camera (testcontrol) — ServoClient
  tools/      tiện ích: freespace.py, captures/
  calib/      hiệu chỉnh camera stereo
  captures/   ảnh kết quả (freespace_*.jpg)
  agent/      log lịch sử riêng của module

GIAO THỨC SERVO (ServoClient ↔ ESP32, qua /dev/ttyUSB0)
  TX: V <pan_vel> <tilt_vel>   đặt vận tốc góc servo
  TX: S                         dừng servo
  TX: R                         reset servo về 0°
  RX: P <pan> <tilt>           phản hồi vị trí 50 Hz
  RX: READY                     ack khởi động

TIỆN ÍCH PYTHON
  python stereo-camera/tools/freespace.py   # phát hiện vùng trống

CẢNH BÁO
  stereo-camera và motivation DÙNG CHUNG 1 ESP32 trên /dev/ttyUSB0 —
  KHÔNG chạy đồng thời trên cùng cổng.
