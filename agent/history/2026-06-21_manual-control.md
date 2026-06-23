# Session: manual-control — 2026-06-21

## Task
Create a standalone manual camera-servo control module (`manual-control/camera-control/`) that
lets the user drive two servos in real-time via keyboard (W/A/S/D). Separate from the main
project modules. Servos are controlled through an ESP32 which drives a PCA9685 over I2C.

## What was implemented

### New module: `manual-control/camera-control/`

| File | Contents |
|------|----------|
| `esp32_firmware/esp32_firmware.ino` | ESP32 firmware. Receives `M <dpan> <dtilt>` steps over UART, applies to PCA9685, replies `P <pan> <tilt>`. Clamps at limits. |
| `serial_link.h / .cpp` | Jetson UART client. RX thread parses `P`/`READY`. `step(dpan, dtilt)` sends `M`, `reset()` sends `R`. |
| `main.cpp` | Raw-mode keypress loop (60 Hz). Each char = one press = 1° step per axis. |
| `CMakeLists.txt` | Builds `camera_control` binary; links `Threads`. |
| `run.sh` | One-command pipeline: flash ESP32 via `arduino-cli` → build Jetson app → run. |

### Hardware wiring (matches stereo-camera project conventions)
- PCA9685 I2C: SDA=GPIO21, SCL=GPIO17, addr=0x40
- Pan servo: channel 12, range -80° … +80°
- Tilt servo: channel 14, range -70° … +30°
- Serial: 115200 baud, `/dev/ttyUSB0`

### Protocol (final iteration)
```
Jetson → ESP32 : "M <dpan> <dtilt>\n"   relative step (degrees)
Jetson → ESP32 : "R\n"                   reset to 0°
ESP32  → Jetson: "P <pan> <tilt>\n"     position feedback after each move
ESP32  → Jetson: "READY\n"              on boot
```

## Design decisions

| Decision | Reason |
|----------|--------|
| Jetson sends UART → ESP32 drives PCA9685 | Matches stereo-camera project architecture; ESP32 already has servo firmware there |
| Step protocol (`M`), not velocity (`V`) | 1° per key press is discrete; velocity model was designed for continuous sweep |
| Clamp (not bounce) at limits | Manual control should stop at boundary, not reverse direction |
| Send `M` only when keys are pressed | Avoids serial spam; no periodic heartbeat needed |
| Key press = one character event | OS auto-repeat handles "hold to keep stepping" naturally; no special hold detection |
| `run.sh` as single entry point | Encapsulates flash + build + run so user types one command |

## Iterations during session
1. **First version**: direct I2C from Jetson (PCA9685 driver in C++).
2. **Revised**: user clarified ESP32 sits between Jetson and PCA9685 (matches stereo-camera arch). Rewrote: dropped local I2C, added UART serial link, adapted ESP32 firmware from `stereo-camera/control/esp32_servo_controller/`.
3. **Final tweak**: changed from hold-to-move at 1°/s (velocity) to 1° per key press (step). Simplified ESP32 loop (no 50 Hz integrator needed).

## Pending / next session
- [ ] Test on real hardware: flash firmware, run `./run.sh`, verify servo movement.
- [ ] Tune `STEP_DEG` in `main.cpp` if 1° steps feel too coarse or too fine.
- [ ] If "hold key = no repeat after first press" is desired, switch input to evdev (`/dev/input`) for true key-up/down events.
- [ ] Consider adding a speed mode: short tap = 1°, long hold = larger step (configurable).
