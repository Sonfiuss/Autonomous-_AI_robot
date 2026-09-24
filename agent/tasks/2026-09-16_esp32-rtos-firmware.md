---
id: 2026-09-16_esp32-rtos-firmware
status: testing
module: motivation/ (ESP32 firmware + Jetson app) + project/LINK (shared protocol) — all new
started: 2026-09-16
---

## Task
The two programs that make the robot actually move, plus the protocol library they share:

1. `project/LINK` — the wire protocol, compiled into BOTH sides so they can never disagree.
2. `motivation/esp32_unified_controller/` — ESP32 firmware, four FreeRTOS tasks
   (Comm / Motion / Peripheral / Status), calls RM to turn commands into step/dir pulses.
3. `motivation/jetson/` — the Jetson-side application: opens `/dev/ttyUSB0`, sends commands,
   reads telemetry back.

Nothing here existed before: `motivation/` and `communication/` are empty directories, and a grep of
the whole repo finds no serial/termios/pyserial code anywhere. `project_overview.md` and
`interfaces.md` describe `motivation/esp32_unified_controller/` as if it shipped; it never did.

## Input
- UART protocol fixed in `agent/description/interfaces.md` (115200 baud, `/dev/ttyUSB0`):
  - RX: `M <vx> <vy> <omega>`, `F <dist_m> <spd_ms>`, `T <angle_deg> <rads>`,
    `V <pan_v> <tilt_v>`, `S`, `R`
  - TX: `O <x> <y> <theta_deg>` (10 Hz), `P <pan> <tilt>` (50 Hz), `K`, `READY`,
    plus the new `E <code> <count>` added by decision 7 below
- RM library `project/include/RM/` as the ONLY source of maths: `OmniKinematics`, `VelocityProfile`,
  `TrapezoidalProfile`, `Odometry`, `DriverStepDir`, `StepAccumulator`; limits from
  `project/config/constants.h`. RM is embeddable by design: float only, no heap, no OS calls.
- User decisions (2026-09-15/16, this conversation):
  1. Four FreeRTOS tasks, named by the user: Communication, Motion, Peripheral, Status.
  2. Continuous `M` goes through a length-1 mailbox (overwrite), NOT a FIFO — a stale velocity is
     worthless, so the executor must always act on the newest value.
  3. `F`/`T`/`R` go through a real FIFO, because each must run to completion.
  4. `S` bypasses every queue: Comm sets a flag Motion checks first thing each tick.
  5. Motion is pinned to its own core; Comm/Status/Peripheral share the other.
  6. Status is the only task allowed to write UART TX.
  7. One new wire line, `E <code> <count>`. It is the ONLY protocol change in this task: without it
     the error counters are write-only and the Jetson can never see a dropped command. CRC and
     sequence numbers were considered and deliberately deferred — both would change all ten existing
     lines and force a matching change on the Jetson side.
  8. **Option A** (chosen 2026-09-16 over option B): the Jetson sends `M`/`F`/`T` and the ESP32 owns
     the kinematics. The alternative — a new per-wheel `W w1 w2 w3` line with the Jetson doing the
     maths — was rejected because it leaves the robot driving blind whenever the link stalls.
     MC on the Jetson keeps its role as trajectory preview / feasibility check. MC and the ESP32
     Motion task therefore repeat the same *computation*, but not the same *code*: both call RM,
     so the two can never drift apart.

## Decisions taken while executing (open points now closed)
- **Frame of `M vx vy w`: BODY frame.** `interfaces.md` was silent and
  `agent/history/2026-09-07_rm.md` recorded it as open. Body frame means Motion never calls
  `globalToBody`, and the Jetson — which already knows the heading — does any rotation it wants.
- **Framework: ESP-IDF**, for `xTaskCreatePinnedToCore` and direct LEDC control.
- **Tick: 50 Hz** (`rm`-side `mc::cfg::TICK_S`), matching MC. `project/README.md` sketched 100 Hz and
  `interfaces.md` records 20 Hz for the old PoseController; 50 Hz is now the single number.
- **Pin map: placeholders** in `config.h`, clearly marked — the real `PinConfig.h` is not in this repo.
- **Floats are formatted by hand, not `%f`.** ESP-IDF's newlib-nano option drops `%f` from `printf`,
  which would silently turn every telemetry line into garbage. A small fixed-decimal formatter in
  `protocol.cpp` removes that trap and makes the output byte-identical on both sides.
- **NEW: motion command watchdog.** Not in the original design, added during step 7 — without it two
  real failure modes exist: (a) the USB cable is pulled mid-drive and the robot keeps going forever
  on the last `M`; (b) an `F` leg finishes and the robot instantly resumes a stale `M` that has been
  sitting in the mailbox since before the leg. Motion now treats a velocity older than
  `MOTION_CMD_TIMEOUT_S` as zero, so the Jetson must refresh `M` or the robot coasts to a stop.

## Expected output
- `project/` gains `LINK`; `test_link` builds and passes with plain g++ ON THIS PC.
- `test_motion` (the ESP32 state machine, host build) builds and passes ON THIS PC.
- `motivation/esp32_unified_controller/` builds with ESP-IDF on a machine that has the toolchain.
- `motivation/jetson/` builds with g++ on the Jetson (POSIX termios; it does not build on Windows).
- `S` is acted on within one control tick (20 ms) even while an `F`/`T` leg is mid-flight.
- `M` always uses the newest value, and a velocity older than the timeout is treated as zero.
- No task except Status writes UART TX (no interleaved output lines).
- A malformed line, a full queue or a dropped TX message surfaces as `E <code> <count>` within one
  second — and a healthy robot sends no `E` at all.
- RM (`project/src/RM`, `project/include/RM`) is not modified by a single line.

## Plan

### Part 0 — shared wire protocol (`project/LINK`, compiled into BOTH sides)
- [x] 1. This task file + `agent/plan/firmware_plan.md`.
- [x] 2. `project/include/LINK/protocol.h`: `Command`/`Telemetry` structs, `CmdKind`, `TeleKind`,
        `ErrCode`, `ErrorCounters`, `LineAssembler`, parse/format declarations, wire limits.
- [x] 3. `project/src/LINK/protocol.cpp`: `LineAssembler` (swallow `\r`, cut at `\n`, drop and flag
        an over-long line), `parseCommand`/`parseTelemetry` (exact argument count, range and NaN
        checks), formatters for M/F/T/V/S/R and O/P/K/READY/E with the hand-rolled float writer.
- [x] 4. `project/tests/test_link.cpp` + `project/tools/build_link.sh` — runs on this PC.

### Part 1 — ESP32 firmware (`motivation/esp32_unified_controller/`)
- [x] 5. `include/fw/config.h`: pin placeholders, baud, tick, task priorities/cores/stacks, queue
        depths, watchdog timeouts, servo limits, LEDC timer assignment.
- [x] 6. `include/fw/rt_port.h`: `Mailbox<T>`, `Queue<T>`, `Mutex`, `Flag`, `nowMs()` — FreeRTOS on
        target, a single-threaded shim under `FW_HOST_BUILD` so the pure parts test on a PC.
- [x] 7. `include/fw/motion_state.h` + `src/motion_state.cpp`: the IDLE / RUNNING_LEG / ESTOP machine
        owning every RM object. Pure — no FreeRTOS, no GPIO — which is why it is host-testable.
- [x] 8. `src/drivers/`: `step_dir_driver` (three LEDC high-speed timers, one per wheel — hardware
        pulse generation, never bit-banged), `servo_driver` (one LEDC low-speed timer at 50 Hz),
        `uart_link` (driver install, RX read, non-blocking TX).
- [x] 9. `src/tasks/`: the four tasks, detailed as 9.1-9.4 below.
- [x] 10. `src/main.cpp` (init order, then create the four tasks pinned) + `CMakeLists.txt` +
        `README.md` (task diagram, pin map, build/flash, the newlib `%f` note).
- [x] 11. `tests/test_motion.cpp` + `tools/build_test_motion.sh` — host build of step 7.

### Part 2 — Jetson application (`motivation/jetson/`)
- [x] 12. `serial_port.h/.cpp`: termios wrapper — raw 8N1 115200, non-blocking read, full write,
        every return value checked, port closed on every error path.
- [x] 13. `robot_link.h/.cpp`: `RobotLink` — send `M`/`F`/`T`/`V`/`S`/`R`, feed bytes through the
        SAME `LineAssembler`, decode `O`/`P`/`K`/`READY`/`E`, expose pose, servo, acks and the
        ESP32's error counters. Auto-repeats the last velocity so the firmware watchdog stays fed.
- [x] 14. `main.cpp`: CLI — `--monitor`, `--vel vx vy w`, `--forward d s`, `--turn deg rate`,
        `--stop`, `--reset`.
- [x] 15. `tools/build_jetson.sh` + `README.md`.

### Part 3 — close out
- [x] 16. `agent/description/interfaces.md`: `E <code> <count>` + its code table, the body-frame
        decision, the shared LINK implementation and the streaming watchdog.
- [x] 16b. `agent/plan/firmware_plan.md`.
- [ ] 17. `/code-standards-review`, `/code-logic-review`, then
        `agent/history/2026-09-16_firmware.md` — the gate before this moves to `done`.

## Task architecture (detail of step 9)

### 9.1 Comm task — the only ingress from the Jetson
Subtasks:
1. Block waiting on the UART RX event queue. No polling, no busy-wait.
2. `LineAssembler`: bytes -> line. Swallow `\r`, cut at `\n`, drop a line that overruns the buffer.
3. `parseCommand()`: check the command character, the argument count, and the value ranges.
4. Route by kind:
   - `S` -> set `estopFlag` immediately, here, through no queue at all
   - `M` -> `xQueueOverwrite(motionMailbox)`, stamped with `nowMs()`
   - `F`/`T`/`R` -> `xQueueSend(oneShotQueue)`; if full, DROP and count — never block
   - `V` -> `xQueueOverwrite(servoMailbox)`, stamped
   - unrecognised -> count and ignore
5. Bump `ErrorCounters` on: malformed line, unknown command, over-long line, one-shot queue full.
   Status turns these into `E` lines, so they are no longer write-only.

Duty: turn raw bytes into structured commands and hand them to the right consumer. Nothing else.

Guarantees:
- Blocks nowhere except on UART RX, so no byte is lost at 115200 baud.
- Runs no kinematics, touches no GPIO, and NEVER writes UART TX.
- A malformed line costs exactly that line: the parser resynchronises at the next `\n` instead of
  sliding into the following line.
- `S` reaches the flag inside the same pass that recognised the line.

### 9.2 Motion task — the control tick, the only caller of RM
Subtasks:
1. `vTaskDelayUntil` at a fixed `TICK_S` (20 ms) — even cadence, no accumulated drift.
2. Check `estopFlag` FIRST, every tick: cancel the running leg, target zero, enter `ESTOP`;
   leave `ESTOP` only once the wheels are genuinely at rest.
3. Pick the command source by state:
   - `IDLE`: take one `F`/`T`/`R` from `oneShotQueue` if present (build a `TrapezoidalProfile`,
     enter `RUNNING_LEG`); otherwise peek the `M` mailbox for the newest velocity.
   - `RUNNING_LEG`: ignore the `M` mailbox entirely; sample the profile by leg time.
4. Velocity watchdog: an `M` older than `MOTION_CMD_TIMEOUT_S` counts as zero.
5. RM chain: `OmniKinematics::inverse` (body frame — no rotation, per the decision above) ->
   `VelocityProfile::step` (slew guard) -> `DriverStepDir::toStepCommand`.
6. Push the three `StepCommand`s to `step_dir_driver` (frequency + DIR per channel).
7. `StepAccumulator` -> steps emitted this tick -> `Odometry::update` -> new pose.
8. Copy the pose into `PoseSnapshot` (critical section = one struct copy, nothing more).
9. On leg completion: notify Status to emit `K`, return to `IDLE`.
10. `R` -> reset `Odometry` to the origin.

Duty: the only place RM is called and the only place the wheels are driven.

Guarantees:
- Tick cadence is independent of how fast the Jetson sends (mailbox reads are non-blocking).
- A stale `M` is never executed — always the newest value, and never one older than the timeout.
- `F`/`T` run to completion before the next command is taken, unless e-stop cuts in.
- E-stop is acted on within ONE tick (20 ms) of Comm setting the flag, whatever the queue backlog.
- Wheel speed and accel limits come from `VelocityProfile` + `constants.h`; the firmware invents no
  numbers of its own.
- Writes no UART, so it can never be delayed by TX.

### 9.3 Peripheral task — pan/tilt servo
Subtasks:
1. Its own 50 Hz cadence via `vTaskDelayUntil`.
2. Peek `servoMailbox` for the newest pan/tilt velocity (command `V`).
3. Integrate `angle += vel * dt`, clamped to the mechanical limits (a clamp bumps a counter).
4. Watchdog: no fresh `V` within `SERVO_CMD_TIMEOUT_S` -> velocity 0, so the servo cannot drift on
   after a dropped link.
5. Drive PWM through a LEDC low-speed timer — a different timer group from the step driver.
6. Write `ServoSnapshot` for Status.
7. `S` -> velocity 0; `R` -> return to 0 deg.

Duty: keep every servo concern out of the Motion task.

Guarantees:
- Shares no timer with the step driver, so there is no hardware contention.
- The servo stops by itself when commands stop arriving.
- The angle stays inside the mechanical limits even if the Jetson sends a bad velocity.
- Lower priority than Motion: when the CPU is tight the servo stutters, the wheels do not.

### 9.4 Status task — sole owner of UART TX
Subtasks:
1. 50 Hz base cadence, waiting on a notification with a timeout so `K` can cut in early.
2. Every tick: read `ServoSnapshot` -> send `P <pan> <tilt>`.
3. Every 5th tick (10 Hz): read `PoseSnapshot` -> send `O <x> <y> <theta_deg>`
   (radians -> degrees via `cfg::RAD_TO_DEG`).
4. On the `K` notification from Motion -> send immediately, without waiting for the cadence.
5. Send `READY` exactly once, after init completes.
6. Format every line through `protocol.cpp` — no `sprintf` scattered around.
7. TX buffer full -> DROP that periodic message rather than block; the next one is 20 ms away, and
   bump the TX-dropped counter so the loss itself is reported.
8. Every 50th tick (1 Hz): for each `ErrCode` whose counter moved since the previous pass, send
   `E <code> <count>`. Nothing moved -> nothing sent.

Duty: collect internal state and be the single point that writes to the wire.

Guarantees:
- Two tasks can never write at once, so no output line is ever interleaved.
- Reads snapshots only, never calls back into Motion/Peripheral, so it slows neither.
- A congested TX never stalls the system, and the messages it drops are counted rather than hidden.
- `K` has low latency because it travels as an event, not on the periodic schedule.
- A healthy robot emits no `E` traffic at all: silence means no errors, and the Jetson recovers the
  per-second rate by diffing consecutive cumulative counts.

### Shared resources
| Resource | Written by | Read by | Kind |
|---|---|---|---|
| `estopFlag` | Comm | Motion (first thing each tick), Peripheral | atomic flag |
| `motionMailbox` | Comm | Motion | length-1 queue, overwrite |
| `oneShotQueue` | Comm | Motion | FIFO |
| `servoMailbox` | Comm | Peripheral | length-1 queue, overwrite |
| `PoseSnapshot` | Motion | Status | struct + short mutex |
| `ServoSnapshot` | Peripheral | Status | struct + short mutex |
| `K` event | Motion | Status | task notification |
| `ErrorCounters` | Comm, Peripheral, Status | Status | one atomic uint32 per code |
| UART TX | Status ONLY | — | — |

### Error codes carried by `E`
| Code | Meaning | Bumped by |
|---|---|---|
| 1 | malformed line (bad argument count or value out of range) | Comm |
| 2 | unknown command character | Comm |
| 3 | RX line longer than the buffer, dropped | Comm |
| 4 | one-shot queue full, `F`/`T`/`R` dropped | Comm |
| 5 | TX buffer full, periodic message dropped | Status |
| 6 | servo target clamped to a mechanical limit | Peripheral |

Counts are cumulative and never reset, so a lost `E` line costs no information: the next one still
carries the true total. Sent at most once per second and only for codes whose count moved.

## Risks
- **Neither program can be built for its real target on this PC**: only MinGW g++ 6.3 is installed —
  no ESP-IDF, no cmake, and the Jetson app needs POSIX termios, which MinGW does not provide. What
  IS verified here: `test_link` (the shared protocol) and `test_motion` (the state machine). The
  drivers, the four tasks, `main.cpp` and the whole Jetson side ship UNBUILT — same situation as MC's
  unverified CMake targets, and the single largest risk in this task.
- No encoder: odometry dead-reckons from commanded steps and cannot see wheel slip.
- `MAX_PULSE_HZ = 20000` is still a placeholder and must be measured on hardware.
- The pin map is invented. Nothing will move until the real GPIO numbers replace it.
- `interfaces.md:15` says `motivation` and `stereo-camera` share this one ESP32 and must not run at
  the same time. This work does not solve that; it is an architectural issue one level up.
- **Pre-existing, found while regression-testing, NOT caused by this task**: `tests/test_rm.cpp`
  "accumulator carries fraction" fails when built `-O2` with MinGW's default x87 maths, and passes
  without `-O2` or with `-mfpmath=sse`. `StepAccumulator` sums `150 Hz x 0.01 s` and x87's 80-bit
  intermediates move where the truncation lands. The ESP32 FPU carries no excess precision, so it
  behaves like the SSE case — but this accumulator feeds odometry and is more rounding-sensitive
  than it looks. Worth its own task.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[--:--] step 1: task file rewritten for the 3-part split (LINK / ESP32 / Jetson); open points closed as body frame + ESP-IDF + 50 Hz | info: option A means MC and the firmware repeat the same computation but share RM, so they cannot drift
[--:--] steps 2-4: LINK protocol + test_link, 7 groups, ALL PASS on this PC | info: structs are SI+radians and the wire is degrees, converted only inside parse/format; floats written by hand because ESP-IDF newlib-nano drops %f; E count parsed as an integer so counts above 2^24 keep every digit
[--:--] steps 5-7,11: config, rt_port, MotionState + test_motion, ALL PASS | risk: FIRST run failed two checks, one of them a real safety hole — ESTOP left itself as soon as the wheels stopped, so a streamed M restarted the robot immediately after a stop | info: the stop now LATCHES and only clearStop() or a new explicit move releases it; the second failure was my test asserting the pose moves after one tick, which it cannot, since the ramp starts below the driver's minimum pulse rate
[--:--] steps 8-10: three drivers, four tasks, main.cpp, CMake, README | info: MOTION_CMD_TIMEOUT_MS/SERVO_CMD_TIMEOUT_MS/KEEPALIVE_MS moved into link::cfg because they bind BOTH sides; S also posts a zero servo command so the camera stops without the servo task knowing about e-stop; SERVO_CLAMPED counts on the edge, not per tick, so parking at a limit does not spam E
[--:--] steps 12-15: Jetson SerialPort + RobotLink + CLI + README | info: RobotLink repeats the last velocity every 50 ms because silence means stop; every one-shot command cancels the stream so a keep-alive cannot restart the robot when a leg ends
[--:--] verification: whole tree syntax-checked with hand-written ESP-IDF + POSIX stubs in the scratchpad, zero warnings | risk: this proves it PARSES and type-checks, NOT that it works on hardware | info: stubs are scratch only, nothing was added to the repo
[--:--] regression: test_link PASS, test_motion PASS, test_mc PASS, test_mv PASS, test_rm 1 FAILURE | info: the RM failure is PRE-EXISTING and unrelated — "accumulator carries fraction" fails only under -O2 with MinGW's x87 maths and passes without -O2 or with -mfpmath=sse; confirmed by rebuilding against a constants.h with my link block stripped out

## Test result
