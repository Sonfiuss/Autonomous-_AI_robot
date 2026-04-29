# Ultrasonic Radar Firmware

HC-SR04 gắn trên servo thông qua PCA9685, quét góc ±30° và truyền dữ liệu qua Serial.

---

## Sơ đồ nối dây

### Tổng quan

```
                    ┌────────────────────────────┐
                    │        Arduino Uno          │
                    │                             │
     ┌──────────┐   │  Pin 9  ──────── TRIG       │
     │  HC-SR04 │   │  Pin 10 ──────── ECHO       │
     │          │◄──┤  5V     ──────── VCC        │
     │  TRIG    │   │  GND    ──────── GND        │
     │  ECHO    │   │                             │
     │  VCC     │   │  A4 (SDA) ──────────────────┼──┐
     │  GND     │   │  A5 (SCL) ──────────────────┼──┼──┐
     └──────────┘   │                             │  │  │
                    │  5V  ───────────────────────┼──┼──┼──┐
                    │  GND ───────────────────────┼──┼──┼──┼──┐
                    └────────────────────────────┘  │  │  │  │
                                                    │  │  │  │
                    ┌───────────────────────────────┘  │  │  │
                    │   ┌───────────────────────────────┘  │  │
                    │   │   ┌───────────────────────────────┘  │
                    │   │   │   ┌───────────────────────────────┘
                    ▼   ▼   ▼   ▼
                    ┌─────────────────┐
                    │    PCA9685      │
                    │                 │
                    │  SDA ◄──────────┤
                    │  SCL ◄──────────┤
                    │  VCC ◄──────────┤
                    │  GND ◄──────────┤
                    │                 │
                    │  CH15 ──────────┼───► Servo Signal (Orange)
                    │  V+   ──────────┼───► Servo VCC   (Red)
                    │  GND  ──────────┼───► Servo GND   (Brown/Black)
                    └─────────────────┘
```

---

### Bảng kết nối chi tiết

#### HC-SR04 → Arduino Uno

| HC-SR04 | Arduino Uno | Ghi chú         |
|---------|-------------|-----------------|
| VCC     | 5V          | Nguồn 5V        |
| GND     | GND         | Mass            |
| TRIG    | Pin 9       | Trigger pulse   |
| ECHO    | Pin 10      | Echo pulse      |

#### PCA9685 → Arduino Uno (I2C)

| PCA9685 | Arduino Uno | Ghi chú                  |
|---------|-------------|--------------------------|
| VCC     | 5V          | Nguồn logic 3.3–5V       |
| GND     | GND         | Mass                     |
| SDA     | A4          | I2C Data                 |
| SCL     | A5          | I2C Clock                |
| V+      | 5V (riêng)  | Nguồn cho servo (5–6V)   |

> ⚠️ **Lưu ý:** Nên cấp nguồn `V+` của PCA9685 từ nguồn ngoài (5V/2A) thay vì lấy từ Arduino để tránh quá tải.

#### Servo → PCA9685

| Servo Wire          | PCA9685    | Ghi chú                |
|---------------------|------------|------------------------|
| Signal (Cam/Vàng)   | CH15       | PWM control            |
| VCC    (Đỏ)         | V+         | Nguồn servo            |
| GND    (Nâu/Đen)    | GND        | Mass                   |

---

## Cấu hình (connect.config)

| Thông số              | Giá trị    | Mô tả                         |
|-----------------------|------------|-------------------------------|
| `kTriggerPin`         | 9          | Chân TRIG của HC-SR04         |
| `kEchoPin`            | 10         | Chân ECHO của HC-SR04         |
| `kEchoTimeoutUs`      | 30000 µs   | Timeout = ~515 cm max range   |
| `kPCA9685Address`     | 0x40       | I2C address mặc định          |
| `kServoChannel`       | 15         | Kênh servo trên PCA9685       |
| `kPWMFrequency`       | 50 Hz      | Tần số PWM servo              |
| `kServoMinPulseUs`    | 500 µs     | Pulse nhỏ nhất (0°)           |
| `kServoMaxPulseUs`    | 2500 µs    | Pulse lớn nhất (180°)         |
| `kMinAngle`           | -30°       | Góc quét trái                 |
| `kMaxAngle`           | +30°       | Góc quét phải                 |
| `kSweepStepDeg`       | 1°         | Bước quét mỗi lần đọc         |
| `kStepDelayMs`        | 35 ms      | Delay giữa các bước           |
| `kMaxRangeCm`         | 400 cm     | Khoảng cách tối đa hợp lệ     |

---

## Serial Output

Baud rate: **115200**

| Message                          | Ý nghĩa                        |
|----------------------------------|--------------------------------|
| `RADAR_READY`                    | Khởi động xong                 |
| `RADAR_CONFIG:-30,30,400`        | Thông số góc và range          |
| `RADAR:15,23.45`                 | Góc 15°, khoảng cách 23.45 cm  |
| `RADAR:-5,-1`                    | Góc -5°, không có vật thể      |

---

## Build & Upload

### 1. Upload firmware lên Arduino

```bat
# Chỉ upload firmware (thay COM7 bằng cổng thực tế)
upload_firmware.bat COM7

# Upload + mở Serial Monitor luôn
upload_and_monitor.bat COM7
```

### 2. Chỉ mở Serial Monitor (không upload)

```bat
open_serial_monitor.bat COM7
```

### 3. Chạy web visualization (Radar UI)

```bat
# Bước 1 - Mở terminal tại thư mục web radar, chạy server
cd simulation\web\radar
run_server.bat

# Bước 2 - Mở trình duyệt
http://localhost:5000
```

Hoặc chạy thủ công:

```bash
# Với COM port cụ thể
python simulation/web/radar/server.py --port COM7

# Chế độ test (không cần Arduino)
python simulation/web/radar/server.py --test
```

### Kiểm tra COM port

```powershell
# Windows PowerShell - liệt kê tất cả cổng Serial
[System.IO.Ports.SerialPort]::getportnames()

# Hoặc dùng PlatformIO
python -m platformio device list
```

---

## Yêu cầu thư viện

- `Adafruit PWM Servo Driver Library ^3.0.0`
- `Wire` (built-in Arduino)
