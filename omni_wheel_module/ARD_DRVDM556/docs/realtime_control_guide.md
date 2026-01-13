# Realtime Robot Arm Controller - Chi tiết kỹ thuật

## 1. TẦN SUẤT CẬP NHẬT TỐI ƯU

### Tần suất PWM Servo:
- **50Hz (20ms/chu kỳ)**: Chuẩn cho servo analog
- Không cần cập nhật nhanh hơn vì servo không phản ứng được

### Tần suất cập nhật vị trí:
```
20-50ms (20-50Hz) - TỐI ƯU
├─ 20ms (50Hz): Mượt nhất, tốn CPU
├─ 33ms (30Hz): Cân bằng tốt
└─ 50ms (20Hz): Vẫn mượt, tiết kiệm CPU
```

### Tần suất nhận lệnh:
- **Càng nhanh càng tốt** (không giới hạn)
- Lưu vào buffer, xử lý theo UPDATE_INTERVAL

## 2. CẤU TRÚC DỮ LIỆU

### ✅ NÊN: Truyền tất cả 6 servo cùng lúc (ATOMIC UPDATE)

**Lý do:**
```
1. Đảm bảo đồng bộ: Tất cả servo nhận lệnh cùng thời điểm
2. Tránh pose trung gian lỗi: Không bị "xé" giữa 2 lệnh
3. Đơn giản hóa logic: 1 lệnh = 1 trạng thái hoàn chỉnh
4. Dễ debug: Biết chính xác pose tại mỗi thời điểm
```

### ❌ KHÔNG NÊN: Truyền từng servo riêng lẻ

**Vấn đề:**
```
1. Mất đồng bộ: Servo 0 đã đến vị trí mới, servo 5 còn ở vị trí cũ
2. Pose trung gian nguy hiểm: Có thể va chạm hoặc mất cân bằng
3. Phức tạp hơn: Phải quản lý timing của 6 servo riêng lẻ
4. Khó predict: Không biết robot ở trạng thái nào giữa các lệnh
```

## 3. ĐỊNH DẠNG DỮ LIỆU

### Format chuẩn (Text - dễ debug):
```
<90,45,135,90,60,120>\n

Ưu điểm:
- Human readable
- Dễ debug qua Serial Monitor
- Dễ parse

Nhược điểm:
- Tốn bandwidth (20-30 bytes)
- Parse chậm hơn binary
```

### Format Binary (tối ưu tốc độ):
```
[START_BYTE][S0_LSB][S0_MSB][S1_LSB][S1_MSB]...[CHECKSUM][END_BYTE]

Ưu điểm:
- Nhỏ gọn: 15 bytes (6 servo x 2 byte + overhead)
- Parse nhanh
- Có checksum để validate

Nhược điểm:
- Khó debug
- Phức tạp hơn
```

## 4. TRÁNH OVERLOAD

### 4.1 Input Buffer
```cpp
String inputBuffer = "";  // Tối đa 100 chars
if (inputBuffer.length() > 100) {
  inputBuffer = "";  // Clear nếu overflow
}
```

### 4.2 Speed Limiting (QUAN TRỌNG NHẤT)
```cpp
#define MAX_SPEED 3.0  // degrees per 20ms

// Giới hạn tốc độ di chuyển
float delta = targetAngle - currentAngle;
if (abs(delta) > MAX_SPEED) {
  delta = (delta > 0) ? MAX_SPEED : -MAX_SPEED;
}
```

**Lợi ích:**
- Servo không giật cục
- Tiết kiệm năng lượng
- An toàn cho cơ khí

### 4.3 Fixed Update Rate
```cpp
// Chỉ update servo mỗi 20ms, bỏ qua requests thừa
if (now - lastUpdateTime >= UPDATE_INTERVAL) {
  updateServos();
  lastUpdateTime = now;
}
```

### 4.4 Command Timeout
```cpp
// Nếu 2s không có lệnh -> về vị trí an toàn
if (now - lastCommandTime > 2000) {
  moveToSafePosition();
}
```

## 5. SO SÁNH PHƯƠNG PHÁP

### Method 1: Atomic Update (KHUYÊN DÙNG)
```
Timeline:
t=0ms:   Nhận lệnh [90,45,135,90,60,120]
t=0ms:   Set target cho cả 6 servo
t=20ms:  Servo 0-5 di chuyển 1 bước nhỏ
t=40ms:  Servo 0-5 di chuyển tiếp
...
t=600ms: Tất cả đến đích

✅ Đồng bộ hoàn hảo
✅ Pose luôn hợp lệ
✅ Dễ điều khiển
```

### Method 2: Sequential Update (KHÔNG KHUYÊN DÙNG)
```
Timeline:
t=0ms:   Nhận lệnh Servo 0 = 90
t=0ms:   Set target servo 0
t=100ms: Servo 0 đến đích
t=100ms: Nhận lệnh Servo 1 = 45
t=100ms: Set target servo 1
t=200ms: Servo 1 đến đích
...

❌ Mất 600ms mới đầy đủ
❌ Pose trung gian không hợp lệ
❌ Phức tạp
```

## 6. KIẾN TRÚC TỔNG THỂ

```
PC/Controller
    ↓ Serial (115200 baud)
    ↓ <90,45,135,90,60,120>
    ↓
Arduino
    ├─ Input Buffer (non-blocking)
    ├─ Parse & Validate
    ├─ Update target[] array (atomic)
    ↓
Update Loop (50Hz)
    ├─ Speed limiting
    ├─ Interpolation
    └─ Set all 6 servos
    ↓
PCA9685 (I2C)
    └─ PWM signals → 6 servos
```

## 7. THÔNG SỐ KHUYẾN NGHỊ

```cpp
#define NUM_SERVOS 6
#define UPDATE_INTERVAL 20      // 20ms = 50Hz
#define MAX_SPEED 3.0           // 3°/update = 150°/s
#define SERIAL_BAUD 115200      // Đủ nhanh cho realtime
#define COMMAND_TIMEOUT 2000    // 2s timeout
#define BUFFER_SIZE 100         // Đủ cho 1 command
```

## 8. TEST PROTOCOL

### Test 1: Latency
```python
# PC gửi lệnh và đo thời gian phản hồi
import serial, time

ser = serial.Serial('COM3', 115200)
start = time.time()
ser.write(b'<90,90,90,90,90,90>\n')
response = ser.readline()
latency = (time.time() - start) * 1000
print(f"Latency: {latency}ms")  # Kỳ vọng: <5ms
```

### Test 2: Throughput
```python
# Gửi liên tục với tần suất khác nhau
for freq in [10, 20, 30, 50, 100]:  # Hz
    interval = 1.0 / freq
    for i in range(100):
        ser.write(b'<90,90,90,90,90,90>\n')
        time.sleep(interval)
    # Kiểm tra số lệnh bị drop
```

### Test 3: Smooth Motion
```python
# Gửi chuỗi lệnh để tạo chuyển động mượt
import math
for t in range(0, 1000, 20):  # Mỗi 20ms
    angle = 90 + 30 * math.sin(t / 200.0)
    cmd = f'<{angle},{angle},{angle},{angle},{angle},{angle}>\n'
    ser.write(cmd.encode())
    time.sleep(0.02)
```

## 9. TROUBLESHOOTING

### Vấn đề: Servo giật cục
```
Nguyên nhân: MAX_SPEED quá lớn hoặc UPDATE_INTERVAL quá dài
Giải pháp: Giảm MAX_SPEED xuống 2.0 hoặc giảm UPDATE_INTERVAL xuống 10ms
```

### Vấn đề: Lệnh bị drop
```
Nguyên nhân: Serial buffer overflow
Giải pháp: Tăng baud rate lên 115200 hoặc gửi lệnh chậm hơn
```

### Vấn đề: Không đồng bộ
```
Nguyên nhân: Gửi từng servo riêng lẻ
Giải pháp: Luôn gửi tất cả 6 góc trong 1 lệnh
```

## 10. KẾT LUẬN

**KHUYẾN NGHỊ:**
1. ✅ Truyền tất cả 6 servo cùng lúc (atomic update)
2. ✅ Update rate: 20-50ms (20-50Hz)
3. ✅ Speed limiting: 2-3 degrees per update
4. ✅ Format: `<a0,a1,a2,a3,a4,a5>\n` (text) hoặc binary
5. ✅ Serial: 115200 baud minimum
6. ✅ Buffer overflow protection
7. ✅ Command timeout protection
