# Session: stereo-camera — 2026-06-10

## What was done

### 1 — Bug diagnosis (`stereo_scan` crash on first run)

Identified 3 issues from the failed run output:

| # | Severity | Root cause | Fix |
|---|----------|-----------|-----|
| 1 | Critical | ESP32 running old unified firmware — no MOVE/OK protocol | Added MOVE/OK to unified controller (see §2) |
| 2 | Minor | `\n` in cmd string broke timeout error log formatting | Fixed in `ServoController.cpp:sendAndWaitOK` |
| 3 | Non-bug | `calib.yml` missing at `../calib.yml` | User must generate or omit `--calib` |

---

### 2 — ESP32 unified controller: MOVE/OK added

**File:** `motivation/esp32_unified_controller/esp32_unified_controller.ino`

**Problem:** Only one ESP32 handles motors + servos. Cannot flash standalone `servo_driver.ino` without breaking motor control.

**Solution:** Added step-and-hold absolute-position protocol to the unified controller.

**New primitives:**
```cpp
g_servo_move_queue   // QueueHandle_t, capacity 1, holds ServoMoveCmd{pan, tilt}
g_move_done_sem      // binary semaphore: ServoTask → CommTask when settled
```

**CommTask changes:**
- Parses `MOVE <pan> <tilt>\n` → sends to `g_servo_move_queue`, blocks on semaphore, sends `OK\n`
- Parses `RESET\n` (uppercase) → same as MOVE 0 0

**ServoTask changes:**
- Checks `g_servo_move_queue` at top of each 20ms tick
- On receive: stops velocity, clamps to limits, snaps to target angle, `vTaskDelay(500ms)`, gives semaphore
- Resets `vTaskDelayUntil` baseline after the long settle delay

**Why CommTask blocking 500ms is safe:** MotionTask on Core 1 is unaffected; only new UART commands are held during settle, acceptable for scan mode.

**Also fixed:** `TASK_STACK_SIZE` was missing from `PinConfig.h` — caused compile error. Added `#define TASK_STACK_SIZE 4096`.

---

### 3 — ServoController: debug logging added

**File:** `stereo-camera/control/ServoController.cpp`

Controlled by `SERVO_DEBUG=1` env var. Logs:
- Port open: fd number + baud
- Every TX: byte count + content (newline stripped)
- Every RX line received
- `parseLine` outcome: READY / OK / ignored

Usage:
```bash
SERVO_DEBUG=1 ./servo_ctrl D 0 0
```

---

### 4 — `servo_ctrl` debug tool created

**File:** `stereo-camera/tools/servo_ctrl.cpp`

CLI for direct absolute servo moves using the MOVE/OK protocol.

```bash
./servo_ctrl D <pan> <tilt>
./servo_ctrl --port /dev/ttyUSB1 D 0 0
```

Added to `CMakeLists.txt` as `servo_ctrl` target (links `ServoController.cpp` only, no OpenCV).

---

### 5 — `servo.sh` shell wrapper created

**File:** `stereo-camera/scripts/servo.sh`

```bash
./scripts/servo.sh 0 0          # reset to origin
./scripts/servo.sh 45 -20       # pan=45° tilt=-20°
./scripts/servo.sh reset        # shortcut for 0 0
./scripts/servo.sh 0 0 /dev/ttyUSB1   # override port
```

Auto-builds `servo_ctrl` if binary is missing.

---

### 6 — Firmware compiled and flashed via `arduino-cli`

```bash
arduino-cli lib install "FastAccelStepper"   # was missing

arduino-cli compile --fqbn esp32:esp32:esp32 \
  /home/nvidia/Documents/motivation/esp32_unified_controller

arduino-cli upload --fqbn esp32:esp32:esp32 --port /dev/ttyUSB0 \
  /home/nvidia/Documents/motivation/esp32_unified_controller
```

Flash verified: `P 0.00 0.00` + `O 0.000 0.000 0.00` streaming after boot.

MOVE command verified:
```
[DBG] TX (17 bytes): "MOVE 0.000 0.000"
[DBG] RX line: "OK"
[servo_ctrl] OK  (0.52 s)
```

---

### 7 — `stereo_plan.md` updated

Added full quick-start reference covering build, flash, verify, servo control, and full scan commands. All completed tasks marked done. Open bugs listed.

---

## Decisions made

| Decision | Reason |
|----------|--------|
| Add MOVE/OK to unified controller (not separate firmware) | Single ESP32 — cannot lose motor control |
| CommTask blocks during 500ms settle | Simpler than async state machine; motors unaffected (Core 1) |
| `SERVO_DEBUG` via env var (not CLI flag) | No API change to `ServoController`; works for all callers |
| `servo.sh` auto-builds if binary missing | Removes need to remember cmake/make before first use |

---

## Pending / next session

- [ ] Generate stereo calibration (`calib.yml`) — required for metric depth
- [ ] Run full `stereo_scan` end-to-end with calibration
- [ ] Verify `map3d_server.py` point cloud viewer
- [ ] Fix sequential `.read()` stereo skew (grab/retrieve pattern)
- [ ] Clamp MapBuilder FOV default (70° half-FOV bug)
- [ ] Add max-depth clamp for SGBM outliers

## Interface changes

### New protocol (ESP32 unified controller)
```
Jetson → ESP32 : "MOVE <pan_f> <tilt_f>\n"   (absolute position)
Jetson → ESP32 : "RESET\n"                    (servo home 0,0)
ESP32  → Jetson: "OK\n"                       (after SERVO_SETTLE_MS=500)
```
All previous commands (`M`, `F`, `T`, `V`, `S`, `R`) unchanged.

### New files
```
stereo-camera/tools/servo_ctrl.cpp
stereo-camera/scripts/servo.sh
```

### Modified files
```
motivation/esp32_unified_controller/esp32_unified_controller.ino  — MOVE/OK added
motivation/esp32_unified_controller/PinConfig.h                   — TASK_STACK_SIZE added
stereo-camera/control/ServoController.cpp                         — debug logging + log fix
stereo-camera/CMakeLists.txt                                      — servo_ctrl target added
agent/plan/stereo_plan.md                                         — full quickstart added
```
