# Hand Angle Detector + Servo Control

Tích hợp điều khiển servo realtime dựa trên góc bàn tay.

## 🔧 Kiến trúc

```
┌─────────────────┐    USB/Serial   ┌──────────────┐
│  DetectFinger   │ ──────────────> │  Arduino     │
│  (Python + CV)  │   <angle cmd>   │  (ATmega328) │
└─────────────────┘                 └──────────────┘
        ↓                                   ↓
   MediaPipe                          PCA9685 (I2C)
   Hand Tracking                           ↓
                                    ┌──────────────┐
                                    │  6x Servo    │
                                    └──────────────┘
```

## 📦 Yêu cầu

### Python
```bash
pip install opencv-python mediapipe pyserial
```

### Arduino (PlatformIO)
```bash
cd ../omni_wheel_module/ARD_DRVDM556
pio run --target upload
```

## 🚀 Cách sử dụng

### 1. Chế độ bình thường (Không điều khiển servo)
```bash
python DetectFinger.py
```

### 2. Điều khiển servo đơn (Servo index 0)
```bash
python DetectFinger.py --servo --port COM3 --servo-index 0
```

### 3. Điều khiển servo khác
```bash
# Điều khiển servo 1
python DetectFinger.py --servo --port COM3 --servo-index 1

# Linux/Mac
python DetectFinger.py --servo --port /dev/ttyUSB0 --servo-index 0
```

## ⚙️ Tùy chỉnh ánh xạ góc

Trong `DetectFinger.py`, dòng 113-117:

```python
self.servo_controller.control_single_servo(
    servo_index=self.servo_index,
    hand_angle=angle,        # Góc đầu vào (0-180°)
    min_angle=30,            # Góc servo khi hand = 0°
    max_angle=150,           # Góc servo khi hand = 180°
    invert=False             # True: đảo chiều
)
```

### Ví dụ ánh xạ:
- **Normal**: `hand 0° → servo 30°`, `hand 180° → servo 150°`
- **Inverted**: `hand 0° → servo 150°`, `hand 180° → servo 30°`
- **Full range**: `min_angle=0, max_angle=180`

## 🎮 Phím điều khiển

| Phím | Chức năng |
|------|-----------|
| `q` | Thoát chương trình |
| `s` | Chụp và lưu ảnh |
| `h` | Move servo về home (90°) |

## 📊 Protocol Serial

**Format**: `<angle0,angle1,angle2,angle3,angle4,angle5>\n`

**Example**: `<90.0,45.0,135.0,90.0,60.0,120.0>\n`

**Update rate**: 20-50Hz (configurable trong Arduino)

## 🔍 Troubleshooting

### Lỗi kết nối Serial
```
✗ Failed to connect to COM3: [WinError 2] The system cannot find the file specified.
```
**Giải pháp**:
1. Kiểm tra Arduino đã cắm vào chưa
2. Xác định đúng cổng COM: Device Manager → Ports (COM & LPT)
3. Thử đổi port: `--port COM4`, `--port COM5`...

### Servo không động
1. Kiểm tra nguồn PCA9685 (5V/6V riêng)
2. Kiểm tra I2C: địa chỉ 0x40
3. Xem log Arduino trong `test_robot_arm.py`

### Góc không chính xác
Điều chỉnh `min_angle`, `max_angle` trong code để calibrate.

## 🛠️ Advanced: Điều khiển nhiều servo

Sửa `DetectFinger.py` để map góc → nhiều servo:

```python
# Trong process_frame(), sau khi tính angle:
if self.enable_servo and self.servo_controller:
    # Servo 0: điều khiển bởi góc chính
    angles = list(self.servo_controller.current_angles)
    angles[0] = self.servo_controller.map_hand_angle_to_servo(
        angle, 0, min_angle=30, max_angle=150
    )
    # Servo 1: mirror của servo 0
    angles[1] = 180 - angles[0]
    
    self.servo_controller.send_angles(angles)
```

## 📝 Files

- `DetectFinger.py` - Main detection + servo integration
- `ServoController.py` - Serial communication module
- `../ARD_DRVDM556/src/main_realtime.cpp` - Arduino firmware
- `test_robot_arm.py` - Arduino test script (standalone)

## 🎯 Performance

- **Latency**: ~30-50ms (detection + serial)
- **Update rate**: 20-50Hz
- **Smoothing**: Có sẵn trong Arduino (MAX_SPEED limiting)

## 📖 Tham khảo

- [ARD_DRVDM556 Documentation](../omni_wheel_module/ARD_DRVDM556/docs/)
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands.html)
- [PCA9685 Datasheet](https://www.adafruit.com/product/815)
