# Firmware plan (`motivation/` + `project/LINK`)

The two programs that move the robot, and the protocol library they share.
Task: `agent/tasks/2026-09-16_esp32-rtos-firmware.md`.

## Architecture (option A, chosen 2026-09-16)
The ESP32 owns the kinematics. The Jetson sends intent (`M` body-frame velocity, `F`, `T`) and never
per-wheel speeds. Rejected alternative: a `W w1 w2 w3` line with the Jetson doing the maths — it
leaves the robot driving blind whenever the link stalls.

MC (Jetson) and the firmware's Motion task therefore repeat the same *computation*. They do not
duplicate *code*: both call RM, so they cannot drift apart.

## Done
- [x] 2026-09-16 `project/LINK` — one protocol implementation for both sides. Structs are SI +
      radians, the wire is degrees, conversion lives only in parse/format. Floats formatted by hand
      (ESP-IDF newlib-nano drops `%f`). `test_link` passes on the PC.
- [x] 2026-09-16 `E <code> <count>` added to the protocol and to `interfaces.md`; six error codes.
- [x] 2026-09-16 ESP32 firmware: config, rt_port (Mailbox/Queue/Flag/Snapshot + host shim),
      MotionState (IDLE/RUNNING_LEG/ESTOP), three drivers, four tasks, `main.cpp`, CMake, README.
- [x] 2026-09-16 `test_motion` — the state machine and the queue semantics, passing on the PC.
- [x] 2026-09-16 Jetson app: `SerialPort` (termios), `RobotLink` (commands, telemetry, keep-alive),
      a CLI, build script, README.
- [x] 2026-09-16 Whole tree syntax-checked against hand-written ESP-IDF/POSIX stubs: zero warnings.

## Design decisions worth remembering
- **E-stop latches.** `S` sets a flag Comm raises and Motion reads first thing each tick, and the
  stop then holds. An earlier version released it as soon as the wheels stopped, which let a stale
  streamed `M` restart the robot the instant it came to rest — caught by `test_motion`.
- **Streaming watchdog.** An `M` older than 200 ms counts as zero, so a pulled cable stops the robot
  and a finished `F` leg cannot resume a long-dead velocity.
- **One TX writer.** Only the Status task writes the port; two writers would interleave mid-line.
- **Timers.** Three LEDC high-speed timers for the wheels (each needs its own frequency), the
  low-speed group for the servo, so the two cannot contend.

## Pending
- [ ] Real pin map — every pin in `include/fw/config.h` is a placeholder.
- [ ] Build with ESP-IDF and flash. Nothing here has ever run on hardware.
- [ ] Build the Jetson side on the Jetson (needs POSIX termios; it does not build on Windows).
- [ ] Measure `MAX_PULSE_HZ` and the accel limits; tune `constants.h`.
- [ ] Wire the plan runner to CM (an `/api/run` over `/api/plan` output) and to the ZMQ bridge on
      5555/5556. A plan now reaches the robot from the CLI (`robot_link --run-plan`, 2026-09-24,
      see `agent/plan/seq_plan.md`); CM still cannot start a run by itself.
- [ ] `/code-standards-review` + `/code-logic-review` before the task moves to `done`.
- [ ] LEDC high-speed mode does not exist on ESP32-S3/C3; revisit if the board changes.

## Known pre-existing defect (not from this work)
`tests/test_rm.cpp` "accumulator carries fraction" FAILS when built `-O2` with MinGW's default x87
maths, and passes without `-O2` or with `-mfpmath=sse`. `StepAccumulator` accumulates
`150 Hz x 0.01 s` and x87's 80-bit intermediates change where the truncation lands. The ESP32's FPU
has no excess precision, so it behaves like the SSE case — but the accumulator is more sensitive to
rounding than it looks, and it feeds odometry.
