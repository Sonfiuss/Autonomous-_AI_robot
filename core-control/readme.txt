==============================================================
 core-control — Bộ giải động học bánh xe omni (C++)
==============================================================

CHỨC NĂNG
  Nhận lệnh chuyển động dạng văn bản (move / spin / arc) và tính ra
  số bước cho từng bánh xe omni theo động học nghịch (inverse kinematics).
  Đây là "bộ não" tính toán chuyển động, KHÔNG trực tiếp nói chuyện UART.

THÔNG SỐ ROBOT (nguồn: ESP32 firmware)
  - Bán kính bánh r        : 5.5 cm
  - Bán kính robot L       : 21 cm
  - Steps / vòng bánh       : 12800 (200 x 64 microstep)
  - Góc bánh α              : W1=60°, W2=180°, W3=300°

BUILD
  mkdir -p build && cd build
  cmake .. && make -j$(nproc)
  → tạo file thực thi: core-control/build/core_control

CHẠY
  ./core_control                          # chế độ tương tác (gõ lệnh)
  ./core_control "move left 30"           # chế độ một lệnh (cm)
  ./core_control "spin right 45"          # xoay tại chỗ 45°
  ./core_control "arc forward 20 turn 30" # vừa đi vừa xoay

CÁC LỆNH
  move forward|back|left|right <cm>            di chuyển thẳng
  move forward-left|forward-right|... <cm>     di chuyển chéo 45°
  spin [left|right] <deg>                       xoay tại chỗ
  self-round [left|right] <deg>                 giống spin
  arc forward|back|left|right <cm> turn <deg>   đi + xoay
  help                                          trợ giúp
  quit / q                                      thoát

FILE CHÍNH
  main.cpp            entry point, parse lệnh
  CommandParser.*     phân tích chuỗi lệnh
  KinematicsCore.*    công thức động học nghịch

GHI CHÚ
  Xoay là open-loop (không encoder/IMU) → không phát hiện trượt.
  Hiệu chỉnh ước lượng→thực tế bằng cách đo trực tiếp.
