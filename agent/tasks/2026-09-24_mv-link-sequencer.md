---
id: 2026-09-24_mv-link-sequencer
status: testing
module: motivation
started: 2026-09-24
---

## Task
Write the sequencer that walks an MV primitive list onto the ESP32 wire: send one leg,
wait for `K`, send the next. Closes the `[MISSING]` stage 6 of `system_architecture.md`.

## Input
- MV output: `MvPrimitive{type, a, b}` — 0 ROTATE(rad), 1 FORWARD(m), 2 MOVE(dx,dy world), 3 STOP.
- The frozen wire protocol (`project/include/LINK/protocol.h`) and `motivation/jetson/RobotLink`.
- Firmware facts that constrain the design (read from the code, 2026-09-24):
  - `motion_task.cpp` pops a one-shot only when `!motion.busy()` -> the `K` handshake is mandatory.
  - `applyOneShot` IGNORES the bool from `beginForward`/`beginTurn`: a leg under
    `mc::cfg::MIN_LEG_LENGTH_M` / `MIN_LEG_ANGLE_RAD` is dropped with no `K` and no error
    counter, so the sender must filter those itself or it deadlocks.
  - Legs are timed, not pose-driven -> the ack timeout can be derived from RM exactly.
  - `S` LATCHES the e-stop; the next `F`/`T` releases it.
  - `RobotState.ready` is sticky, so an ESP32 reboot mid-run is currently undetectable.

## Expected output
`robot_link --run-plan plan.txt` drives the robot through a whole MV plan leg by leg and exits
0 when the last leg is acknowledged; `plan.txt` may be `mv_cli`'s own output verbatim.
`test_seq` passes on the PC, as do the four existing suites.

## Decisions (user approved 2026-09-24)
- **A1** MOVE is decomposed into ROTATE+FORWARD with heading-bias compensation, so every leg is
  confirmed by a firmware `K`. Rejected: streaming `M` (no ack, drives on Jetson timing) and
  refusing MOVE outright.
- **B** The state machine lives in `project/src/SEQ/` — pure, no OS/heap/exceptions, testable on a
  PC like LINK. Only the pump lives in `motivation/jetson/`.
- **C** `RobotState` gains `readies`, a count of `READY` lines, so a reboot mid-run is detectable.

## Plan
- [x] 1. `constants.h`: `seq::cfg` block (ack timeout margin/floor, ready wait, max primitives)
- [x] 2. `include/SEQ/sequencer.h` + `seq_debug.h`: State, Status, Config, Feedback, Action, Sequencer
- [x] 3. `src/SEQ/sequencer.cpp`: leg mapping, short-leg filter, heading bias, RM-derived timeout,
       fault/reboot abort, trailing `S`
- [x] 4. `tests/test_seq.cpp`: handshake, 1:1 non-holonomic map, MOVE decomposition + final heading,
       short-leg skip, timeout, abort on E4, abort on reboot, trailing `S`
- [x] 5. `tools/build_seq.sh` + `seq` target in `CMakeLists.txt`
- [x] 6. `RobotLink`: `RobotState.readies` + count it in `consume()`
- [x] 7. `motivation/jetson/mission_runner.{h,cpp}`: pump poll(), build Feedback, send Action
- [x] 8. `main.cpp`: `--run-plan FILE [--start-theta DEG] [--speed] [--yaw-rate]`, lenient parser
- [x] 9. `build_jetson.sh`: add SEQ + the three RM sources
- [x] 10. Re-run all five suites; syntax-check the Jetson side against POSIX stubs in the scratchpad
- [x] 11. `/code-standards-review` + `/code-logic-review`
- [x] 12. Docs: `interfaces.md`, `system_architecture.md`, `agent/plan/seq_plan.md`, history, action log

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->

[18:20] steps 1-10: SEQ module, MissionRunner, --run-plan; all 6 suites green | info: mv_cli output is a valid plan file as-is, the parser skips every non-primitive line
[18:52] standards-review: 4 violations fixed, 0 deferred | info: FAULT_SLOTS cannot live in constants.h (it derives from link::ErrCode, which includes constants.h) - same reason jetson::cfg::ERR_SLOTS sits in its module header
[19:05] logic-review: 1 mismatch fixed, 0 flagged | info: --speed/--yaw-rate were unchecked against link::cfg, so an out-of-range value failed per-leg seconds into the run instead of at load()
[19:20] step 12: docs done (interfaces, architecture, seq_plan, both READMEs, history) | info: project/README.md still drew MC->ESP32, contradicting option A - pre-existing, fixed here since SEQ owns that arrow

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
