# Session History — motivation module
Date: 2026-06-27

## What was implemented

### 1. PinConfig.h — cập nhật toàn bộ
- Pin mapping đúng thực tế:
  - M1 (W1, 60°): STEP=GPIO12, DIR=GPIO13
  - M2 (W2, 180°): STEP=GPIO4, DIR=GPIO5
  - M3 (W3, 300°): STEP=GPIO26, DIR=GPIO27
- ENA bỏ hoàn toàn (không nối dây)
- Physical params:
  - `WHEEL_RADIUS_M = 0.055f` (⌀11cm)
  - `ROBOT_RADIUS_M = 0.21f` (21cm tâm→bánh)
  - `STEPS_PER_REV = 12800` (DM556/DM542 cài 12800 pulse/rev)
  - `MAX_STEP_FREQ = 60000` Hz (đủ cho 1.5 m/s)
  - `MAX_ACCEL_STEPS = 200000` steps/s²

### 2. OmniKinematics.h — góc bánh mới
- Thay đổi từ 90°/210°/330° → **60°/180°/300°**
- Inverse kinematics:
  - ω1 = (-√3/2·vx + 1/2·vy + L·ω) / r
  - ω2 = (−vy + L·ω) / r
  - ω3 = (+√3/2·vx + 1/2·vy + L·ω) / r
- Forward kinematics (pseudoinverse):
  - vx = r·(1/√3)·(−w1 + w3)
  - vy = r·(1/3·w1 − 2/3·w2 + 1/3·w3)
  - ω  = r/(3L)·(w1+w2+w3)

### 3. esp32_unified_controller.ino
- Bỏ `en_pins[]`, `setEnablePin()`, `setAutoEnable()` — ENA không nối
- Thêm lệnh `W <idx> <steps>\n` (CMD_WHEEL) để test từng bánh riêng lẻ
  - idx: 0=W1, 1=W2, 2=W3
  - steps: số bước (dương=thuận, âm=nghịch)
  - Tốc độ cố định 2000 Hz khi test

### 4. MotionClient.hpp / .cpp
- Thêm `testWheel(int idx, int revolutions = 1)`
- Gửi `W <idx> <steps>\n` với steps = revolutions × 12800

### 5. main.cpp
- Thêm `runWheelTest()` — test lần lượt W1→W2→W3, mỗi bánh 1 vòng thuận + 1 vòng nghịch
- Thêm flag `--test`: `./motivation --port /dev/ttyUSB0 --test`

## Decisions made
- Dùng lệnh W riêng thay vì dùng kinematics để isolate từng bánh trong test
- Tốc độ test cố định 2000 Hz (chậm, an toàn) thay vì dùng MAX_STEP_FREQ
- ENA bỏ hoàn toàn — driver DM556/DM542 auto-enable khi không nối ENA

## Hardware confirmed
- Drivers: M1=DM556 (Peak 2.1A), M2=M3=DM542 (Peak 2.37A)
- Wheel angles: 60°, 180°, 300°
- Wheel diameter: 11cm → r=0.055m
- Robot radius: 21cm

## Pending tasks
- Flash firmware ESP32 và build lại Jetson binary
- Chạy `--test` để xác nhận chiều quay từng bánh
- Nếu bánh quay ngược → đổi dây DIR+/DIR− trên driver tương ứng
- Sau test chiều quay: chạy `--demo` kiểm tra odometry và di chuyển thực tế

## Interface changes
- Thêm UART command: `W <idx> <steps>\n` (ESP32 nhận)
- Thêm CLI flag: `--test` (Jetson motivation binary)
