==============================================================
 manual-control — Điều khiển servo camera bằng bàn phím (C++)
==============================================================

CHỨC NĂNG
  Điều khiển thủ công 2 servo (pan + tilt) của camera bằng phím bấm.
  Chuỗi điều khiển:
    Jetson ─USB serial→ ESP32 ─I²C→ PCA9685 ─PWM→ Servo

CẤU HÌNH PHẦN CỨNG
  Cổng serial        : /dev/ttyUSB0   (mặc định)
  Baud               : 115200
  PCA9685 I2C addr   : 0x40
  SDA / SCL          : GPIO 21 / GPIO 17
  Kênh pan / tilt    : 12 / 14
  Tầm pan            : -80° … +80°
  Tầm tilt           : -70° … +30°
  PWM                : 50 Hz

BUILD & CHẠY
  cd manual-control/camera-control
  bash run.sh                 # build + nạp firmware + chạy (xem run.sh)
  # hoặc build tay:
  mkdir -p build && cd build && cmake .. && make -j$(nproc)
  ./camera_control

FILE CHÍNH
  camera-control/main.cpp        vòng lặp đọc phím → gửi lệnh serial
  camera-control/serial_link.*   lớp giao tiếp serial
  camera-control/esp32_firmware/ firmware ESP32 (.ino)
  camera-control/run.sh          script build + upload + run
  camera-control/GUIDELINE.md    hướng dẫn chi tiết cho developer

GHI CHÚ
  Nếu board ESP32 không phải generic (esp32:esp32:esp32) thì sửa FQBN
  trong run.sh. Chi tiết đầy đủ xem GUIDELINE.md.
