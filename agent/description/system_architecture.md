# System Architecture — as it actually is

> Snapshot: 2026-09-24. Describes what is **in the repository**, not what is planned.
> The roadmap lives in `robot_process_llm.md`; the frozen wire formats live in `interfaces.md`.

Status legend used throughout:

| Mark | Meaning |
|------|---------|
| **[OK]** | implemented and tested |
| **[UNBUILT]** | implemented, never compiled/run on its real target |
| **[MISSING]** | documented elsewhere but no code exists |

---

## 1. The three machines

```
┌──────────────────────────────┐   ┌────────────────────────────────────────────┐   ┌────────────────────────┐
│ LAPTOP                       │   │ JETSON  (Ubuntu, Python + C++)             │   │ ESP32                  │
│                              │   │                                            │   │ (FreeRTOS, bare metal) │
│ simulation/room  [OK]        │   │  CM  Flask :5001          [OK]             │   │                        │
│  Flask :5000                 │   │   scene, LLM grounding, /api/plan,         │   │ 4 tasks:               │
│  room generator + viewer     │   │   /api/trajectory                          │   │  Comm / Motion /       │
│                              │   │            │ ctypes                        │   │  Peripheral / Status   │
│                              │   │            v                               │   │      [UNBUILT]         │
│                              │   │  MV .so  A* planner        [OK]            │   │                        │
│                              │   │       │         └──▶ MC .so  preview  [OK] │   │  drives 3 steppers     │
│                              │   │       │              ends at the CM page   │   │  + pan/tilt servo      │
│                              │   │       │  SEQ sequencer            [OK]      │   │                        │
│                              │   │       v                                    │   │                        │
│                              │   │  motivation/jetson  RobotLink  [UNBUILT]   │   │                        │
└──────────────────────────────┘   └────────────────────────────────────────────┘   └────────────────────────┘
         ^                                            │                                        ^
         │                                            │  USB serial /dev/ttyUSB0               │
         └────── x ZMQ 5555/5556  [MISSING] ──────────┘  115200 8N1, ASCII lines ──────────────┘
```

One break left in the chain, and one closed:

1. ~~Nobody sequences MV's primitives onto the wire.~~ **Closed 2026-09-24** by `project/src/SEQ`
   and `motivation/jetson/MissionRunner`: the loop that sends a leg, waits for `K` and sends the
   next now exists, is unit-tested against a fake firmware, and is reachable from the CLI as
   `robot_link --run-plan`. What is still untested is the *hardware* below it — `RobotLink` and the
   firmware are both still `[UNBUILT]`, so the chain is complete in code and unproven on a robot.
   Note what this was never about: MC is **not** meant to feed `RobotLink`. Under option A the ESP32
   recomputes the ticks itself, so MC's trajectory ending at the browser is its correct destination,
   not a gap.
2. **The ZMQ bridge described in `interfaces.md` does not exist.** A grep of the whole repo finds no
   `zmq` import, no `simulation/movement/app.py`, and none of `/api/robot/goal`, `/api/robot/pose`,
   `/api/pathfind`, `/api/config`. `simulation/room/app.py` serves only `/api/room/new`,
   `/api/room/latest`, `/api/room/<seed>`.

---

## 2. The pipeline, stage by stage

Each stage lists what goes **in**, what it **does**, and exactly what comes **out**.

```
 [1] scene            [2] candidates        [3] LLM grounding      [4] goal
 room JSON      ──▶   spots + summary  ──▶  slots / question  ──▶  MvPose (x,y,theta)
 (objects, walls)     (id,x,y,theta)        (target/side/spot)     world frame, radians
                                                                        │
        ┌───────────────────────────────────────────────────────────────┘
        v
 [5] MV plan                  [6] SEQ                  [7] RobotLink
 cells ▶ waypoints ▶     ──▶  sequencer:          ──▶  one leg on the wire
 primitives                   send a leg, wait K,       "F 1.500 0.150\n"
 ROTATE/FORWARD/MOVE/STOP     send the next                  │
        │                                                    │ UART 115200
        └──▶ MC preview ──▶ wheel-speed chart                │
             per-tick rows   on the CM page                  │
             (ends here, by design)                          │
        ┌────────────────────────────────────────────────────┘
        v
 [8] Comm task        [9] Motion task            [10] drivers      [11] Status task
 bytes ▶ line ▶   ──▶ RM chain @50 Hz       ──▶  LEDC pulses  ──▶  O / P / K / E
 Command struct       BodyVel ▶ wheels ▶ steps   + DIR pins        back to the Jetson
                      ▶ odometry
```

### Stage 1 — Scene
| | |
|---|---|
| **Where** | `project/src/CM/app.py` `_start()`, scenes from `simulation/room/room_generator.py` |
| **In** | `GET /api/scene/new?seed=N` |
| **Does** | loads or generates a room: walls, polygon objects, a door |
| **Out** | scene JSON: `{room:{w,l}, objects:[{id, class, polygon:[[x,y]...], ...}], door:{side}}`, metres |

### Stage 2 — Candidates
| | |
|---|---|
| **Where** | `project/src/CM/candidates.py` — `CandidateBuilder`, `build_candidates()` |
| **In** | the scene JSON |
| **Does** | for each object and each free side, places a standing spot at a clearance; labels it with a compass word (`side_normal`, `compass_word`, `DOOR_INWARD`) |
| **Out** | `spots: [{id, x, y, theta, description}]` — the ONLY places the robot may be asked to stand. Metres and radians, world frame |

### Stage 3 — LLM grounding
| | |
|---|---|
| **Where** | `project/src/CM/dialog.py` `GroundingSession`, prompt in `prompts/grounding.json` |
| **In** | `POST /api/ground/say {text}` + scene summary + history |
| **Does** | fills three slots in order — `SLOT_ORDER = (target, side, spot)`. Asks a question when a slot is ambiguous. Every reply is validated against real ids; an invalid one is re-asked with `RETRY_HINT` |
| **Out** | `{type: ask\|goal\|none\|error, text, slots, matches:[{id,x,y,theta,description}], resolved}` |
| **Rule** | the LLM never emits a distance or an angle — only ids. Geometry stays in code |

### Stage 4 — Goal
| | |
|---|---|
| **In** | the resolved spot, or an explicit `goal` in the request body |
| **Out** | `MvPose {x, y, theta}` — metres, radians, world frame, θ CCW from +x |

### Stage 5 — MV, path planning **[OK]**
| | |
|---|---|
| **Where** | `project/src/MV/*` via `mv_client.py` (ctypes → `mv.dll` / `libmv.so`) |
| **In** | `MvRequest {room_w, room_l, robot_radius, vertices[], poly_sizes[], n_polys, start, goal, holonomic}` |
| **Does** | rasterises obstacles onto a grid (`GRID_RES_M` 0.05 m), inflates by `robot_radius + SAFETY_MARGIN_M`, snaps start/goal to a free cell within `SNAP_RADIUS_M`, runs 8-connected A*, compresses the cell path into a waypoint polyline, emits primitives |
| **Out** | `MvResult`: `path[]` cells, `waypoints[]` `MvPoint`, `prims[] MvPrimitive {type, a, b}`, `length_m`, snapped start/goal |
| **Primitive codes** | `0 ROTATE(rad)`, `1 FORWARD(m)`, `2 MOVE(dx, dy world)`, `3 STOP` — pinned by `static_assert` in `MC/types.h` because MV and MC exchange raw ints |
| **Failure** | status ≠ 0 means no path at all, never a partial one |

### Side branch — MC, tick executor **[OK]** — *not a stage of the live path*
| | |
|---|---|
| **Where** | `project/src/MC/*` via `mc_client.py` |
| **In** | `McRequest {prims, n_prims, start_theta, cruise_speed, yaw_rate, dt}` |
| **Does** | per leg: plans an `rm::TrapezoidalProfile`, samples it at the MIDDLE of each tick (second-order accurate), rotates world→body for MOVE legs, applies the per-wheel slew guard, converts to step/dir, integrates `rm::Odometry` |
| **Out** | `McStep[]`, one per tick: `{t, u, v, r, w[3], hz[3], dir[3], x, y, theta}`. **The pose is a displacement from the trajectory start, not a world position** |
| **Also** | `McResult {duration_s, end_x, end_y, end_theta}` |
| **Role under option A** | a **preview and feasibility check** — it draws the trajectory and the wheel-speed curves in the CM page, and answers "how long, within limits, ending where" before anyone commits. Nothing it produces is ever sent to the robot: the ESP32 runs the same RM chain live from `F`/`T`/`M`. Both call RM, so the two cannot disagree |
| **Why it exists** | written 2026-09-13, before option A was chosen, when the Jetson was still expected to drive the wheels directly |

### Stage 6 — SEQ, the sequencer **[OK]**
| | |
|---|---|
| **Where** | `project/src/SEQ/sequencer.cpp`; the loop and the port live in `motivation/jetson/mission_runner.cpp` |
| **In** | `mc::Primitive[]` from MV, `startTheta`, `seq::Config {cruiseSpeed, yawRate}` |
| **Does** | expands one primitive at a time into wire legs and sends exactly one, then waits for its `K`. `MOVE` becomes a `T` onto its bearing plus an `F` along it; `STOP` becomes `S`. Drops any leg the firmware would silently refuse, and derives each leg's ack timeout from `rm::limitsFor` + `rm::TrapezoidalProfile` — the same call the firmware plans the leg with |
| **Out** | at most one `link::Command` per `update()`, and a `seq::Status` when it ends |
| **Pure** | no port, no clock, no heap. The caller passes a `Feedback {nowMs, acks, readies, faults[], linkOk}` and sends back what comes out, which is what lets `test_seq` drive the whole state machine against a fake firmware on a PC |
| **Fails fast** | `E 1`/`E 2` → FIRMWARE_REJECT, `E 4` → QUEUE_FULL, a new `READY` → REBOOTED, a dead port → LINK_ERROR. Each emits `S` rather than waiting out the ack timeout |
| **Why one leg at a time** | the Motion task pops a one-shot only while idle, so back-to-back legs pile into an 8-deep FIFO and the ninth is dropped as `E 4` |
| **Cost of the MOVE decomposition** | the robot turns onto each bearing instead of crabbing. It reaches the same poses: a holonomic plan produces the same wire lines as its non-holonomic twin |

### Stage 7 — RobotLink (Jetson) — built on the Jetson, not yet run against the real firmware
| | |
|---|---|
| **Where** | `motivation/jetson/` — `RobotLink`, `SerialPort` |
| **Verified** | 2026-09-25: built on the Jetson (clean), and `--run-plan` driven end to end against a fake ESP32 on a pty: one leg per `K`, a mid-plan `READY` → REBOOTED, Ctrl-C → `S` |
| **In** | `sendVelocity(vx,vy,omega)` / `sendForward(d,v)` / `sendTurn(rad,rate)` / `sendStop()` / `sendReset()` / `sendServo(pan,tilt)` |
| **Does** | formats through the shared LINK library, writes to `/dev/ttyUSB0`, and **repeats the last velocity every 50 ms** so the firmware watchdog stays fed. Any one-shot command cancels the stream |
| **Out (wire)** | `"M 0.150 -0.020 0.800\n"` etc. |
| **In (wire)** | `O` / `P` / `K` / `READY` / `E` → `RobotState {pose, panDeg, tiltDeg, ready, acks, errorCounts[]}` |

### Stage 8 — Comm task (ESP32) **[UNBUILT]**
| | |
|---|---|
| **In** | raw bytes from UART RX (the task's only blocking call) |
| **Does** | `LineAssembler` swallows `\r`, cuts at `\n`, DROPS an over-long line whole; `parseCommand()` checks the letter, the argument count and the value ranges; routes by kind |
| **Out** | `S` → estop flag **immediately, through no queue**; `M` → length-1 mailbox (overwrite); `F`/`T`/`R` → FIFO; `V` → servo mailbox. Each carries `stampMs` |
| **On failure** | bumps `ErrorCounters` — 1 malformed, 2 unknown, 3 overflow, 4 queue full |

### Stage 9 — Motion task (ESP32) **[UNBUILT]** — the only caller of RM
| | |
|---|---|
| **Cadence** | `vTaskDelayUntil` at 20 ms (50 Hz), pinned alone on core 1 |
| **Order inside a tick** | 1. e-stop flag → `requestStop()` (latched). 2. if idle, take one `F`/`T`/`R`. 3. resolve the velocity: stale (>200 ms) or older than the stop ⇒ **zero**. 4. RM chain. 5. push pulses. 6. odometry. 7. snapshot. 8. `K` notification if a leg ended |
| **RM chain** | `OmniKinematics::inverse` (body frame, no rotation) → `VelocityProfile::step` (slew guard) → `DriverStepDir::toStepCommand` → `StepAccumulator` → `Odometry::update` |
| **Out** | `MotionTick {StepCommand steps[3], Pose pose, bool legFinished}` |

### Stage 10 — Drivers **[UNBUILT]**
| | |
|---|---|
| **Steppers** | three LEDC **high-speed** timers, one per wheel (each needs its own frequency); 50 % duty; DIR on a GPIO; a rate under `STEP_MIN_HZ` switches the channel off. Pulses come from the peripheral, never the CPU |
| **Servo** | LEDC **low-speed** timer, 50 Hz, 16-bit, 500–2500 µs. A different timer group, so the two cannot contend |
| **Servo angle** | integrated from the `V` rate, clamped to ±90°, watchdog zeroes it after 300 ms of silence |

### Stage 11 — Status task (ESP32) **[UNBUILT]** — sole owner of UART TX
| | |
|---|---|
| **In** | `poseSnapshot`, `servoSnapshot`, the `K` notification, `ErrorCounters` |
| **Out** | `P` every 20 ms, `O` every 100 ms, `K` immediately on notification, `READY` once at boot, `E` at most 1 Hz and only for a counter that moved |
| **Back-pressure** | if the TX buffer cannot take a WHOLE line it drops the line and counts `E 5`. A half-written line would desynchronise the receiver, which is worse than a missing sample |

---

## 3. Module clusters

### 3.1 `project/` — the shared C++ libraries

| Module | Language | Depends on | Built as | Status |
|--------|----------|-----------|----------|--------|
| **RM** | C++14, no heap/OS/exceptions | — | static lib; also an ESP-IDF component | **[OK]** |
| **MV** | C++14 | constants only | static + shared (ctypes) | **[OK]** |
| **MC** | C++14 | RM | static + shared (ctypes) | **[OK]** — a tool, not a live stage |
| **LINK** | C++14 | constants only | static; compiled into BOTH sides | **[OK]** |
| **SEQ** | C++14, no heap/OS/exceptions | RM, LINK (+ MC/types.h, header-only) | static; Jetson only, NOT an ESP-IDF component | **[OK]** |
| **CM** | Python / Flask | MV, MC via ctypes | app on :5001 | **[OK]** |

**RM** is the single source of every physical number:

| Class | Input | Output |
|-------|-------|--------|
| `OmniKinematics::inverse` | `BodyVel {u,v,r}` | `WheelSpeeds {w[3]}` rad/s |
| `OmniKinematics::forward` | `WheelSpeeds` | `BodyVel` |
| `OmniKinematics::globalToBody` | `GlobalVel`, θ | `BodyVel` |
| `VelocityProfile::step` | target `WheelSpeeds`, dt | ramped `WheelSpeeds` — all three scaled by a COMMON factor so direction is preserved |
| `limitsFor` | `OmniKinematics`, direction, requested speed | `AxisLimits{vMax, accel, decel}` feasible for that direction. Moved out of MC on 2026-09-16: it is chassis maths, and both MC and the firmware need it |
| `TrapezoidalProfile::plan` | distance, vMax, accel, decel | profile; `velocityAt(t)`, `positionAt(t)`, `duration()` |
| `DriverStepDir::toStepCommand` | ω rad/s | `StepCommand {freqHz ≥ 0, forward}` |
| `StepAccumulator::accumulate` | `StepCommand`, dt | whole steps this tick, fraction carried |
| `Odometry::update` | `int32 counts[3]`, dt | `Pose {x, y, theta}`, mid-point integrated |

Chassis constants (`project/config/constants.h`): wheel radius 0.055 m, robot radius 0.21 m, wheels at
0°/120°/240° (W1 at the front, since 2026-09-25), 12800 steps/rev, `MAX_PULSE_HZ` 20000 (**a placeholder**).

Measured ceilings: **0.62 m/s forward, 0.54 m/s sideways, 2.57 rad/s spinning.** The chassis is not
equally fast in every direction, so an over-fast request is scaled **as a whole vector**, never
clipped per wheel — clipping one wheel bends the robot off the planned line.

### 3.2 `motivation/esp32_unified_controller/` — the firmware **[UNBUILT]**

```
include/fw/
  config.h        pins (ALL PLACEHOLDERS), rates, priorities, stacks, watchdogs
  context.h       RobotContext — every shared object, exactly one writer each
  motion_state.h  IDLE / RUNNING_LEG / ESTOP        <- host-testable, no RTOS
  rt_port.h       Mailbox / Queue / Flag / Snapshot (FreeRTOS + host shim)
  tasks.h         the four entry points
src/
  main.cpp  context.cpp  motion_state.cpp  drivers/  tasks/
tests/test_motion.cpp     PASSES on a PC
```

| Task | Core | Prio | Blocks on | Writes |
|------|------|------|-----------|--------|
| Motion | 1 (alone) | 6 | the 20 ms tick | wheels, `poseSnapshot` |
| Comm | 0 | 5 | UART RX | the three queues, the e-stop flag |
| Status | 0 | 4 | notification w/ timeout | **UART TX** |
| Peripheral | 0 | 3 | its 20 ms tick | servos, `servoSnapshot` |

Shared objects and their single writer:

| Object | Writer | Reader | Kind |
|--------|--------|--------|------|
| `estopFlag` + `estopStampMs` | Comm | Motion | atomic |
| `motionMailbox` | Comm | Motion | length-1, overwrite |
| `oneShotQueue` | Comm | Motion | FIFO, 8 deep |
| `servoMailbox` | Comm | Peripheral | length-1, overwrite |
| `poseSnapshot` | Motion | Status | struct + short mutex |
| `servoSnapshot` | Peripheral | Status | struct + short mutex |
| `ErrorCounters` | anyone | Status | atomic uint32 per code |

Three properties worth keeping in mind:

- **A stale velocity is zero.** Older than 200 ms ⇒ the robot coasts to a stop. Pulling the cable
  stops the robot; it does not leave it running.
- **The e-stop latches.** An earlier version released it as soon as the wheels stopped, which let a
  streamed `M` restart the robot immediately after a stop. `test_motion` catches that regression.
- **One TX writer.** Two writers would interleave mid-character.

### 3.3 `motivation/jetson/` — the Jetson application **[UNBUILT]**

`SerialPort` (termios, raw 8N1, `select()` for the read timeout, non-copyable) +
`RobotLink` (commands, telemetry, keep-alive) + `MissionRunner` (the sequencer's loop: poll, sample
the link, send what SEQ asks for) + a CLI (`--monitor`, `--vel`, `--forward`, `--turn`, `--stop`,
`--reset`, `--servo`, `--run-plan`).

The split with SEQ is deliberate. Everything that can be got wrong about the handshake lives in
`project/src/SEQ`, where it is tested on a PC with no robot; what is left here is the part that
cannot be tested without hardware — poll the port, read the clock, print progress.

### 3.4 `simulation/room/` — room generator **[OK]**

Flask on :5000, `/api/room/new`, `/api/room/latest`, `/api/room/<seed>`, plus a viewer. It generates
scenes; it does **not** plan paths and does **not** talk to the robot.

---

## 4. The transfer layer in detail

### 4.1 Physical
`/dev/ttyUSB0`, **115200 8N1**, no flow control, full duplex. ASCII lines terminated by `\n`.
`interfaces.md:15` still warns that `motivation` and `stereo-camera` share this one ESP32 and must
not run at the same time — nothing arbitrates that yet.

### 4.2 One implementation, both sides
`project/src/LINK/protocol.cpp` is compiled into the firmware **and** the Jetson application.

**Unit rule:** every struct is **SI and radians**; the wire carries **degrees** where `interfaces.md`
says so (`T`'s angle, `O`'s theta). Conversion happens inside the parsers and formatters and nowhere
else, so no caller has to think about it.

Floats are formatted **by hand**, not with `%f`: ESP-IDF's newlib-nano option drops `%f` from
`printf`, which would silently turn every telemetry line into garbage.

### 4.3 Wire format

Jetson → ESP32:

| Line | Fields | Units (wire) | Accepted range | Lifetime |
|------|--------|--------------|----------------|----------|
| `M vx vy omega` | body velocity | m/s, m/s, rad/s | ±2, ±2, ±10 | **expires in 200 ms** |
| `F dist speed` | distance move | m, m/s | ±100, 0.001…2 | runs to completion |
| `T angle rate` | turn in place | **deg**, rad/s | ±3600, 0.001…10 | runs to completion |
| `V pan tilt` | servo rate | deg/s | ±180 | **expires in 300 ms** |
| `S` | e-stop | — | — | latches |
| `R` | reset odometry + servos | — | — | once |

ESP32 → Jetson:

| Line | Fields | Rate |
|------|--------|------|
| `O x y theta` | pose, m/m/**deg** | 10 Hz |
| `P pan tilt` | servo angles, deg | 50 Hz |
| `K` | one `F`/`T` leg finished | on the event |
| `READY` | boot complete | once |
| `E code count` | cumulative error tally | ≤ 1 Hz, only when it moves |

Error codes: `1` malformed · `2` unknown command · `3` line overflow · `4` queue full ·
`5` TX dropped · `6` servo clamped.

Counts are cumulative and never reset, so a lost `E` costs no information — the receiver diffs
consecutive values to recover a rate. A healthy robot emits no `E` at all.

### 4.4 Bandwidth

11520 bytes/s each way; TX and RX do not contend.

| Direction | Traffic | Load |
|-----------|---------|------|
| ESP32 → Jetson | 50 Hz × ~14 B (`P`) + 10 Hz × ~20 B (`O`) ≈ 900 B/s | **~8 %** |
| Jetson → ESP32 | 50 Hz × ~21 B (`M`) ≈ 1050 B/s | **~9 %** |

Bandwidth is not the bottleneck, even streaming `M` every tick.

### 4.5 Failure behaviour

| Failure | What happens |
|---------|--------------|
| a corrupt byte | that line is dropped; the assembler resynchronises at the next `\n`; `E 1`/`E 2` |
| a line too long | dropped whole, never truncated into the next; `E 3` |
| the cable is pulled | `M` goes stale after 200 ms → the robot ramps to a stop |
| the queue fills | the `F`/`T`/`R` is dropped and counted; the UART reader never blocks |
| the TX buffer fills | that periodic line is dropped whole and counted; nothing blocks |
| the ESP32 reboots | it sends `READY`; the Jetson clears its streaming state |

**There is no CRC and no sequence number.** Deliberately deferred: adding either changes all eleven
lines and both sides at once. Today a corrupt *value* inside a well-formed line is only caught by the
range check.

---

## 5. What is missing

| Gap | Consequence |
|-----|-------------|
| ~~the sequencer~~ | **closed 2026-09-24** — `project/src/SEQ` + `MissionRunner` + `robot_link --run-plan` |
| ZMQ 5555/5556 **[MISSING]** | `interfaces.md` documents a bridge that no file implements |
| `simulation/movement/app.py`, `/api/pathfind`, `/api/robot/*` **[MISSING]** | documented in `interfaces.md`, not in the repo |
| ~~real pin map~~ | **filled 2026-09-25**: W1 STEP 12/DIR 13, W2 4/5, W3 26/27 — wheel order taken from the previous Arduino firmware's `PinConfig.h` (git `a96ff99^`), all DIR inverted (`DIR_INVERTED`). To confirm on the robot with `--forward 0.1`. Servos are on a PCA9685 (I2C 21/17) that `servo_driver.cpp` does not drive |
| firmware never flashed | the board still runs the **old Arduino firmware** (`O x y θ` with 2 decimals, `READY\r\n`); its `F`/`T` give every wheel the same step count and drop a negative distance, so `robot_link --run-plan` against it will not drive straight. ESP-IDF not installed on the Jetson yet (needs `sudo apt` for gperf, python3-venv, ninja-build, ccache, dfu-util) |
| `MAX_PULSE_HZ` unmeasured | 20 kHz is a guess; every speed ceiling derives from it |
| no encoder | odometry integrates *commanded* steps and cannot see wheel slip |
| perception | `vision/` is L1 (YOLO + Astra depth + map builder); not wired into CM yet |
| `communication/` | **drive demo, 2026-09-25**: text → LLM/rules → validated steps → plan.txt → `robot_link --run-plan`, with `vision/record.py` filming to `vision/output/<run>/`. No map; see `communication/README.md` |

Known defect, pre-existing and unrelated to the firmware work: `tests/test_rm.cpp`
"accumulator carries fraction" fails under `-O2` with MinGW's x87 maths and passes without `-O2` or
with `-mfpmath=sse`. `StepAccumulator` feeds odometry and is more rounding-sensitive than it looks.

---

## 6. How to verify each piece today

```bash
# shared protocol            — runs on any PC
cd project && bash tools/build_link.sh && ./build/link/test_link

# firmware logic             — runs on any PC, no ESP-IDF
cd motivation/esp32_unified_controller && bash tools/build_test_motion.sh && ./build_host/test_motion

# sequencer                  — runs on any PC, no robot
cd project && bash tools/build_seq.sh && ./build/seq/test_seq

# planner + executor         — runs on any PC
cd project && bash tools/build_mv.sh && ./build/mv/test_mv
cd project && bash tools/build_mc.sh && ./build/mc/test_mc

# firmware on hardware       — needs ESP-IDF
cd motivation/esp32_unified_controller && idf.py set-target esp32 && idf.py build flash monitor

# Jetson side                — needs Linux
cd motivation/jetson && bash tools/build_jetson.sh && ./build/robot_link --monitor
# a whole plan, leg by leg  — needs Linux AND a robot
cd project && ./build/mv/mv_cli < scene.txt > plan.txt
cd motivation/jetson && ./build/robot_link --run-plan ../../project/plan.txt --start-theta 0
```
