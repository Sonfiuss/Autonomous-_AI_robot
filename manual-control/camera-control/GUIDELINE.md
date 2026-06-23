# Camera Servo Control — Developer Guide

## Overview

Manual keyboard control of two servos (pan + tilt) via:

```
Jetson  ──USB serial──►  ESP32  ──I²C──►  PCA9685  ──PWM──►  Servos
(keypress / main.cpp)  (firmware.ino)    (addr 0x40)
```

---

## Hardware Setup

| Item | Value |
|------|-------|
| Serial port | `/dev/ttyUSB0` (default) |
| Baud rate | 115200 |
| PCA9685 I2C addr | 0x40 |
| PCA9685 SDA / SCL | GPIO 21 / GPIO 17 |
| Pan servo channel | 12 |
| Tilt servo channel | 14 |
| Pan range | -80° … +80° |
| Tilt range | -70° … +30° |
| PWM frequency | 50 Hz |

If the ESP32 board variant is not generic (`esp32:esp32:esp32`), change `FQBN` in `run.sh`.

---

## Quick Start (one command)

```bash
cd manual-control/camera-control

./run.sh                    # flash ESP32 + build + run
./run.sh --no-flash         # skip flash if firmware already loaded
./run.sh --port /dev/ttyUSB1
./run.sh clean              # wipe build dir first
```

Manual build if needed:
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
./camera_control --port /dev/ttyUSB0
```

---

## Controls

| Key | Action |
|-----|--------|
| `A` | Pan left  (−1°) |
| `D` | Pan right (+1°) |
| `W` | Tilt up   (+1°) |
| `S` | Tilt down (−1°) |
| `R` | Reset both axes to 0° |
| `Q` or Ctrl+C | Quit |

**Each key press = 1° step.** Holding a key repeats via OS auto-repeat (~30 steps/sec).
Current position is printed live: `Pan: -12.00°   Tilt:   5.00°`

---

## Serial Protocol

All messages are newline-terminated (`\n`), ASCII, 115200 baud.

| Direction | Message | Meaning |
|-----------|---------|---------|
| Jetson → ESP32 | `M <dpan> <dtilt>` | Relative step (degrees). e.g. `M 1.000 0.000` |
| Jetson → ESP32 | `R` | Reset both axes to 0° |
| ESP32 → Jetson | `P <pan> <tilt>` | Absolute position after each move |
| ESP32 → Jetson | `READY` | Sent once on boot |

The ESP32 clamps angles silently — commands past the limit park the servo at the boundary.

---

## File Map

```
manual-control/camera-control/
├── esp32_firmware/
│   └── esp32_firmware.ino   ESP32 firmware (flash once, stays in flash)
├── serial_link.h / .cpp     Jetson UART client (RX thread + step/reset API)
├── main.cpp                 Keyboard input loop, drives SerialLink
├── CMakeLists.txt           Host build
└── run.sh                   One-command flash → build → run
```

---

## Tuning

All tunable constants live at the top of their respective files:

### `main.cpp`
```cpp
static constexpr float STEP_DEG    = 1.0f;        // degrees per key press
static constexpr int   LOOP_HZ     = 60;           // input poll rate
static const char*     DEFAULT_PORT = "/dev/ttyUSB0";
```

### `esp32_firmware.ino`
```cpp
#define PAN_CHANNEL     12      // PCA9685 channel for pan servo
#define TILT_CHANNEL    14      // PCA9685 channel for tilt servo
#define PAN_MIN        -80.0f   // software angle limits
#define PAN_MAX         80.0f
#define TILT_MIN       -70.0f
#define TILT_MAX        30.0f
#define SERVO_FREQ_HZ   50      // PWM frequency
#define PULSE_MIN       102     // PWM counts at -90°
#define PULSE_MAX       512     // PWM counts at +90°
#define PULSE_CENTER    307     // PWM counts at 0°
```

If the servo neutral position is off, adjust `PULSE_CENTER`.
If endpoints are off, adjust `PULSE_MIN` / `PULSE_MAX`.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Cannot open serial port /dev/ttyUSB0` | Check cable; try `--port /dev/ttyUSB1`. Add user to `dialout`: `sudo usermod -aG dialout $USER` |
| `Warning: no READY from ESP32` | Firmware not flashed or wrong port. Run `./run.sh` (with flash) |
| Servo doesn't reach expected angle | Calibrate `PULSE_MIN`/`PULSE_MAX`/`PULSE_CENTER` in firmware |
| Servo channel wrong | Change `PAN_CHANNEL` / `TILT_CHANNEL` in firmware |
| Steps feel too large / small | Change `STEP_DEG` in `main.cpp`; rebuild with `./run.sh --no-flash` |
| Hold key doesn't repeat | OS keyboard repeat disabled. Enable in system settings |
| Want single-press only (no repeat) | Switch input to evdev (`/dev/input/eventX`) for real key-up events |
