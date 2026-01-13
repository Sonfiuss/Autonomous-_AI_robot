# Hand Angle Detection with Servo Control

Professional modular architecture for real-time hand tracking and servo control.

## 📁 Project Structure

```
Vision/
├── main.py                         # Application entry point
├── config.py                       # Configuration settings
│
├── controllers/                    # High-level controllers
│   ├── __init__.py
│   ├── hand_detector.py           # Hand tracking & angle detection
│   └── servo_controller.py        # Servo control interface
│
├── communication/                  # Low-level communication
│   ├── __init__.py
│   └── serial_comm.py             # Arduino serial communication
│
├── utils/                         # Utility functions
│   ├── __init__.py
│   └── angle_mapper.py            # Angle mapping utilities
│
└── [Legacy files]
    ├── DetectFinger.py            # Old monolithic version
    └── ServoController.py         # Old servo controller
```

## 🎯 Features

- ✅ **Modular Architecture**: Separation of concerns (MVC pattern)
- ✅ **Thread-Safe Communication**: Non-blocking serial I/O
- ✅ **Configurable**: Centralized configuration in `config.py`
- ✅ **Professional Logging**: Structured logging with levels
- ✅ **Type Hints**: Full type annotations for IDE support
- ✅ **Error Handling**: Robust error handling and recovery
- ✅ **Real-time Performance**: Optimized for 30+ FPS

## 🚀 Quick Start

### 1. Install Dependencies

```bash
pip install opencv-python mediapipe pyserial
```

### 2. Upload Arduino Firmware

```bash
cd ../omni_wheel_module/ARD_DRVDM556
pio run --target upload
```

### 3. Run Application

```bash
# Detection only (no servo)
python main.py

# With servo control
python main.py --servo --port COM3 --servo-index 0

# Debug mode
python main.py --servo --port COM3 --debug
```

## ⚙️ Configuration

Edit [`config.py`](config.py) to customize:

### Camera Settings
```python
CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
```

### Servo Mapping
```python
SERVO_MIN_ANGLE = 30    # Servo angle when hand = 0°
SERVO_MAX_ANGLE = 150   # Servo angle when hand = 180°
SERVO_INVERT = False    # Reverse mapping
```

### Detection Threshold
```python
ANGLE_PINCH_THRESHOLD = 30  # degrees
```

## 📖 Module Documentation

### `controllers/hand_detector.py`
**HandAngleDetector** - Detects hand landmarks and calculates angles
- `process_frame(img)` - Process frame and return angle
- `calculate_angle(wrist, thumb, index)` - Calculate angle between fingers
- `draw_annotations(img, ...)` - Draw visual feedback

### `controllers/servo_controller.py`
**ServoController** - High-level servo control interface
- `send_angles(angles)` - Send angles to all servos
- `control_single_servo(index, angle, ...)` - Control one servo
- `move_to_home()` - Move to home position

### `communication/serial_comm.py`
**ArduinoSerialComm** - Low-level serial communication
- `send_command(cmd)` - Non-blocking command send
- `read_line()` - Read response from Arduino
- `get_statistics()` - Get communication stats

### `utils/angle_mapper.py`
**AngleMapper** - Angle conversion utilities
- `map_angle(hand_angle, min, max, invert)` - Map hand→servo angle
- `clamp_angles(angles)` - Clamp to valid range
- `interpolate(current, target, alpha)` - Smooth transition

## 🎮 Keyboard Controls

| Key | Function |
|-----|----------|
| `q` | Quit application |
| `s` | Save screenshot |
| `h` | Move servo to home (90°) |
| `c` | Show servo statistics |

## 🔧 Advanced Usage

### Custom Angle Mapping

```python
from controllers import ServoController

servo = ServoController(port='COM3')

# Custom mapping for specific servo
servo.control_single_servo(
    servo_index=0,
    hand_angle=45,
    min_angle=0,      # Full range
    max_angle=180,
    invert=True       # Reversed
)
```

### Multi-Servo Control

```python
# Control multiple servos simultaneously
angles = [90, 45, 135, 90, 60, 120]  # All 6 servos
servo.send_angles(angles)
```

### Direct Serial Communication

```python
from communication import ArduinoSerialComm

comm = ArduinoSerialComm(port='COM3', baudrate=115200)
comm.send_command("<90,90,90,90,90,90>\n")
response = comm.read_line()
```

## 📊 Performance

- **Detection Rate**: 30-60 FPS (depends on camera & CPU)
- **Servo Update**: 20-50 Hz (configurable in Arduino)
- **Latency**: ~30-50ms (detection + serial + servo)
- **Thread Overhead**: <5% CPU (background serial thread)

## 🐛 Troubleshooting

### Import Errors
```
ModuleNotFoundError: No module named 'controllers'
```
**Solution**: Run from Vision directory: `cd Vision && python main.py`

### Serial Connection Failed
```
✗ Failed to connect to COM3
```
**Solution**:
1. Check Device Manager for correct port
2. Close other programs using serial port
3. Try different port: `python main.py --servo --port COM4`

### Servo Not Moving
1. Check power supply (PCA9685 needs 5-6V separate)
2. Verify I2C address (default: 0x40)
3. Check Arduino logs: `pio device monitor`

## 📝 Protocol Reference

**Serial Format**: `<angle0,angle1,...,angle5>\n`

**Example**: `<90.0,45.0,135.0,90.0,60.0,120.0>\n`

- Angles: 0-180 degrees (float)
- Update rate: Configurable (default 50Hz)
- Terminator: `\n` (newline)

## 🔗 Related Files

- Arduino Firmware: [`../omni_wheel_module/ARD_DRVDM556/src/main_realtime.cpp`](../omni_wheel_module/ARD_DRVDM556/src/main_realtime.cpp)
- Test Script: [`../omni_wheel_module/ARD_DRVDM556/test_robot_arm.py`](../omni_wheel_module/ARD_DRVDM556/test_robot_arm.py)
- Documentation: [`../omni_wheel_module/ARD_DRVDM556/docs/`](../omni_wheel_module/ARD_DRVDM556/docs/)

## 📄 License

Part of Autonomous AI Robot project.
