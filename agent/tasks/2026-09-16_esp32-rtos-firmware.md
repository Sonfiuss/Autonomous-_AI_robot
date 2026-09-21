---
id: 2026-09-16_esp32-rtos-firmware
status: planning
module: motivation/esp32_unified_controller (ESP32 firmware, C++) — new
started: 2026-09-16
---

## Task
FreeRTOS firmware for the ESP32 unified controller: four tasks (Comm / Motion / Peripheral / Status)
that receive UART commands from the Jetson, call the RM library (`project/`) to generate step/dir
pulses for the three omni wheels and PWM for the pan/tilt servo, and publish O/P/K/READY back.

Note: `motivation/esp32_unified_controller/` does NOT exist in this repo, although
`project_overview.md` and `interfaces.md` describe it as if it did. `motivation/` and
`communication/` are empty. This is a from-scratch implementation, not an edit of existing firmware.

## Input
- UART protocol, already fixed in `agent/description/interfaces.md` (115200 baud, `/dev/ttyUSB0`):
  - RX: `M <vx> <vy> <omega>`, `F <dist_m> <spd_ms>`, `T <angle_deg> <rads>`,
    `V <pan_v> <tilt_v>`, `S`, `R`
  - TX: `O <x> <y> <theta_deg>` (10 Hz), `P <pan> <tilt>` (50 Hz), `K` (motion-complete ack), `READY`,
    plus the new `E <code> <count>` added by decision 7 below
- RM library `project/include/RM/` as the ONLY source of maths: `OmniKinematics`, `VelocityProfile`,
  `TrapezoidalProfile`, `Odometry`, `DriverStepDir`, `StepAccumulator`. Limits from
  `project/config/constants.h` (`TICK_S` 0.02, `MAX_PULSE_HZ` 20000, a 0.055 m, L 0.21 m,
  12800 steps/rev). RM is embeddable by design: float only, no heap, no OS calls, caller passes `dt`.
- User architecture decisions (2026-09-15/16, this conversation):
  1. Four FreeRTOS tasks, named by the user: Communication, Motion, Peripheral, Status.
  2. Continuous `M` commands go through a length-1 mailbox (overwrite), NOT a FIFO — a stale velocity
     command is worthless, so the executor must always act on the newest value.
  3. `F`/`T`/`R` go through a real FIFO queue, because each must run to completion.
  4. `S` (e-stop) bypasses every queue: Comm sets a flag that Motion checks first thing each tick.
  5. Motion is pinned to its own core; Comm/Status/Peripheral share the other.
  6. Status is the only task allowed to write to UART TX.
  7. One new wire line, `E <code> <count>` (approved 2026-09-16). It is the ONLY protocol change in
     this task: without it the error counters in 8.1 are write-only and the Jetson can never see a
     dropped command. CRC and sequence numbers were considered and deliberately deferred — both would
     change all ten existing lines and force a matching change on the Jetson side.

## Expected output
- `motivation/esp32_unified_controller/` builds on a machine that has the ESP32 toolchain.
- `tests/test_fw` compiles with plain g++ on a PC and passes: line parser, command routing,
  motion state machine.
- `S` is acted on within one control tick (20 ms) even while an `F`/`T` leg is mid-flight.
- `M` always uses the newest value; a backlog of stale velocity commands can never build up.
- No task except Status writes to UART TX (no interleaved/corrupted output lines).
- Errors are observable from the Jetson: a malformed line, a full queue or a dropped TX message each
  surface as an `E <code> <count>` line within one second — and a healthy robot sends no `E` at all.
- RM (`project/`) is not modified by a single line.

## Plan
- [ ] 1. This task file + `agent/plan/firmware_plan.md`.
- [ ] 2. `include/fw/config.h`: pin map (placeholders until the real GPIO numbers arrive), baud,
        tick 50 Hz, O at 10 Hz / P at 50 Hz, per-task priority + core, stack sizes, queue depths.
- [ ] 3. `include/fw/cmd_types.h`: `CmdKind` enum (M/F/T/V/S/R), `MotionCmd` (mailbox payload),
        `OneShotCmd` (F/T/R, FIFO payload), `ServoCmd` (V). Three separate types because the three
        have different lifetimes — that distinction is the whole point of the mailbox/queue split.
        Also `ErrCode` + `ErrorCounters`: one cumulative uint32 per code, bumped from any task,
        read by Status (code table under "Error codes carried by `E`" below).
- [ ] 4. `protocol.h` + `protocol.cpp`: `LineAssembler` (bytes -> line: swallow `\r`, cut at `\n`,
        drop over-long lines) + `parseLine()` -> tagged command + formatters for O/P/K/READY/E.
        Pure: no OS, no GPIO. Every wire format lives in this one file, and it is host-testable.
- [ ] 5. `motion_state.h` + `.cpp`: the IDLE / RUNNING_LEG / ESTOP state machine, owning the RM
        objects. `tick(dt, latestM, oneShot, estop) -> {StepCommand[3], pose, legFinished}`.
        Pure: no FreeRTOS, no GPIO. This is the part that actually gets tested on the PC.
- [ ] 6. `rt_port.h`: thin wrappers `Mailbox<T>` (xQueueOverwrite / xQueuePeek), `Queue<T>`, `Mutex`,
        `Flag`. A trivial single-threaded host implementation so the tests build without FreeRTOS.
- [ ] 7. Drivers: `uart_link` (UART init, RX, TX), `step_dir_driver` (pulse train from a HARDWARE
        peripheral — MCPWM/RMT — never bit-banged), `servo_driver` (LEDC 50 Hz, velocity -> angle,
        clamped).
- [ ] 8. The four tasks — detailed as 8.1-8.4 below.
- [ ] 8.1 `comm_task`
- [ ] 8.2 `motion_task`
- [ ] 8.3 `peripheral_task`
- [ ] 8.4 `status_task`
- [ ] 9. `main.cpp`: init order (UART -> drivers -> queues/snapshots -> create the four tasks pinned:
        Motion on core 1, Comm/Status/Peripheral on core 0) -> emit `READY`.
- [ ] 10. `tests/test_fw.cpp` (g++ on PC): valid lines, a line split across several reads, buffer
        overflow, junk bytes, CRLF; routing to the right queue; `M` always newest; `F` runs to
        completion then `K`; `S` interrupts a leg and ramps to zero; `R` resets odometry;
        no tick exceeds the accel limit; `E` is emitted only for counters that moved and stays
        silent when none did.
- [ ] 11. `tools/build_test_fw.sh` + firmware `README.md` (task/queue diagram, pin map, build+flash)
        + add `E <code> <count>` and its code table to `agent/description/interfaces.md`. This is a
        cross-module contract change: the Jetson side must learn to parse (or at least ignore) it.
- [ ] 12. Run /code-standards-review + /code-logic-review, update `agent/plan/firmware_plan.md`,
        write `agent/history/2026-09-16_firmware.md`.

## Task architecture (detail of step 8)

### 8.1 Comm task — the only ingress from the Jetson
Subtasks:
1. Block waiting on the UART RX event queue. No polling, no busy-wait.
2. `LineAssembler`: bytes -> line. Swallow `\r`, cut at `\n`, drop a line that overruns the buffer.
3. `parseLine()`: check the command character, the parameter count, and the value ranges.
4. Route by kind:
   - `S` -> set `estopFlag` immediately, here, through no queue at all
   - `M` -> `xQueueOverwrite(motionMailbox)`
   - `F`/`T`/`R` -> `xQueueSend(oneShotQueue)`; if full, DROP and count — never block
   - `V` -> `xQueueOverwrite(servoMailbox)`
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

### 8.2 Motion task — the control tick, the only caller of RM
Subtasks:
1. `vTaskDelayUntil` at a fixed `TICK_S` (20 ms) — even cadence, no accumulated drift.
2. Check `estopFlag` FIRST, every tick: cancel the running leg, target zero, enter `ESTOP`;
   clear the flag only once the wheels are genuinely at rest.
3. Pick the command source by state:
   - `IDLE`: read `oneShotQueue` first (F/T/R -> build a `TrapezoidalProfile`, enter `RUNNING_LEG`);
     if empty, `xQueuePeek` the `M` mailbox for the newest velocity.
   - `RUNNING_LEG`: ignore the `M` mailbox entirely; sample the profile by leg time.
4. RM chain: (`globalToBody` if the world frame is chosen) -> `OmniKinematics::inverse` ->
   `VelocityProfile::step` (slew guard) -> `DriverStepDir::toStepCommand`.
5. Push the three `StepCommand`s to `step_dir_driver` (frequency + DIR per channel).
6. `StepAccumulator` -> steps emitted this tick -> `Odometry::update` -> new pose.
7. Copy the pose into `PoseSnapshot` (critical section = one struct copy, nothing more).
8. On `profile.finished(t)`: notify Status to emit `K`, return to `IDLE`.
9. `R` -> reset `Odometry` to zero.

Duty: the only place RM is called and the only place the wheels are driven.

Guarantees:
- Tick cadence is independent of how fast the Jetson sends (mailbox reads are non-blocking).
- A stale `M` is never executed — always the newest value.
- `F`/`T` run to completion before the next command is taken, unless e-stop cuts in.
- E-stop is acted on within ONE tick (20 ms) of Comm setting the flag, regardless of queue backlog.
- Wheel speed and accel limits are enforced by `VelocityProfile` + `constants.h`; the firmware
  invents no numbers of its own.
- Writes no UART, so it can never be delayed by TX.

### 8.3 Peripheral task — pan/tilt servo
Subtasks:
1. Its own 50 Hz cadence via `vTaskDelayUntil`.
2. `xQueuePeek(servoMailbox)` for the newest pan/tilt velocity (command `V`).
3. Integrate `angle += vel * dt`, clamped to the mechanical limits.
4. Watchdog: no new `V` within N ms -> velocity 0, so the servo cannot drift on after a dropped link.
5. Drive PWM through LEDC, on a channel that does not share a peripheral with the step driver.
6. Write `ServoSnapshot` for Status.
7. `S` -> velocity 0; `R` -> return to 0 deg.

Duty: keep every servo concern out of the Motion task.

Guarantees:
- Shares no timer/peripheral with the step driver, so there is no hardware contention.
- The servo stops by itself when commands stop arriving.
- The angle stays inside the mechanical limits even if the Jetson sends a bad velocity.
- Lower priority than Motion: when the CPU is tight the servo stutters, the wheels do not.

### 8.4 Status task — sole owner of UART TX
Subtasks:
1. 50 Hz base cadence.
2. Every tick: read `ServoSnapshot` -> send `P <pan> <tilt>`.
3. Every 5th tick (10 Hz): read `PoseSnapshot` -> send `O <x> <y> <theta_deg>`
   (radians -> degrees via `cfg::RAD_TO_DEG`).
4. Wait on the `K` notification from Motion -> send immediately, without waiting for the cadence.
5. Send `READY` exactly once, after init completes.
6. Format every line through the formatters in `protocol.cpp` — no `sprintf` scattered around.
7. TX buffer full -> DROP that periodic message rather than block; the next one is 20 ms away,
   and bump the TX-dropped counter so the loss itself is reported.
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
| 1 | malformed line (bad parameter count or value out of range) | Comm |
| 2 | unknown command character | Comm |
| 3 | RX line longer than the buffer, dropped | Comm |
| 4 | one-shot queue full, `F`/`T`/`R` dropped | Comm |
| 5 | TX buffer full, periodic message dropped | Status |
| 6 | servo target clamped to a mechanical limit | Peripheral |

Counts are cumulative and never reset, so a lost `E` line costs no information: the next one still
carries the true total. Sent at most once per second and only for codes whose count moved.

## Open points for you
1. **Frame of `M vx vy w`: world or body?** `interfaces.md` is silent and
   `agent/history/2026-09-07_rm.md` records this as still open (the old firmware applied it as body
   frame, the RM spec assumes world). It decides whether Motion calls `globalToBody` first.
   Recommendation: **body frame** — the Jetson/MC side already knows the heading.
2. **Real pin map.** `PinConfig.h` is referenced in `project_overview.md` but is not in this repo.
   Needed: GPIO numbers for 3x step/dir, 2x servo, and the UART pins. Otherwise: placeholders.
3. **Framework: ESP-IDF or PlatformIO/Arduino?** Recommendation: **ESP-IDF** (better control over
   core pinning and the MCPWM API). This choice decides which build files get written.
4. **Tick rate.** 50 Hz from `cfg::TICK_S`, matching MC. Note `project/README.md` sketches 100 Hz and
   `interfaces.md` records 20 Hz for the older PoseController — three different numbers, not yet
   reconciled. Proposal: 50 Hz.

## Risks
- **Cannot be built or flashed on this PC**: only MinGW g++ is installed — no PlatformIO, no ESP-IDF,
  no cmake, no arduino-cli. Steps 2-9 will ship UNVERIFIED on hardware; only step 10 (parser + state
  machine on the host) actually runs. Same pattern as MC's unverified CMake targets.
- No encoder, so odometry stays dead-reckoning from commanded steps and cannot see wheel slip.
- `MAX_PULSE_HZ = 20000` is still a placeholder and must be measured on real hardware.
- `interfaces.md:15` says `motivation` and `stereo-camera` share this one ESP32 and must not run at
  the same time. This firmware does not solve that; it is an architectural issue one level up.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->

## Test result
