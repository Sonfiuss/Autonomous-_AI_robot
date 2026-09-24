---
id: 2026-09-21_esp32-motion-firmware
status: planning
module: firmware (ESP32, C++/ESP-IDF) + simulation/esp32 (virtual ESP32, host C++)
started: 2026-09-21
---

## Task
ESP32 firmware whose only job at this stage is the motion data exchange with the Jetson: receive the
movement MV planned (executed by MC/RM on the ESP32), drive the three step/dir wheels, report pose/ack.
Plus a virtual ESP32 (same code, no hardware) that exposes a pseudo-serial port and writes what the
real board would output to `simulation/esp32/output/`.
Supersedes the motion part of `2026-09-16_esp32-rtos-firmware` (that task is marked `blocked`).

## Input
- User (2026-09-21): firmware in `firmware/`, professional folder layout, follows the earlier plan,
  scope = MV data exchange only, ESP32 decoupled from hardware -> virtual version writes its output to
  `simulation/esp32/`.
- Answers (2026-09-21): MV/MC stay on the Jetson and "just send the planned movement to the ESP32";
  ESP-IDF; `M vx vy w` is BODY frame; virtual ESP32 = PTY serial port + log files.
- Kept from the 2026-09-16 plan: 115200 baud, 50 Hz tick, `M` = length-1 mailbox (newest wins),
  `S` bypasses every queue and is acted on within one tick, only Status writes UART TX,
  Motion pinned to its own core, cumulative `E <code> <count>` error lines, RM is not modified.
- Found while reading: `mc::Executor` (project/include/MC/executor.h) is heap-free, tick-based and
  takes a `Primitive` list (ROTATE/FORWARD/MOVE/STOP) — exactly what MV emits. The firmware can run it
  as-is, so the "planned movement" is sent as that primitive list and no maths is duplicated.

## Expected output
- `firmware/` ESP-IDF project (layout below). Real-target build is UNVERIFIED here (no idf.py).
- `simulation/esp32/` host build of the same core: `./tools/run_vesp32.sh` prints the PTY path;
  `python3 -c "serial.Serial(<path>,115200).write(b'M 0.1 0 0\n')"` moves the virtual robot.
- Files written under `simulation/esp32/output/`:
  `state.json` (latest snapshot, atomically replaced), `wheels.csv` (per tick: t, state, hz[3], dir[3],
  pose), `uart_rx.log`, `uart_tx.log`.
- `firmware/tests/test_fw` passes with plain g++: line parser, plan staging, routing, motion state
  machine, `S` within one tick, `M` newest wins.
- `project/` (RM/MV/MC) untouched.

## Wire protocol (existing lines kept + the smallest addition that carries a plan)
RX (Jetson -> ESP32), unchanged: `M vx vy w`, `F dist spd`, `T deg rate`, `S`, `R`.
`F`/`T` are run as a one-primitive plan (FORWARD / ROTATE) through the same executor.
NEW RX (needs approval — `interfaces.md` currently has no plan line):
- `Q <type> <a> <b>`   append one primitive to the staging buffer (type codes = `MvPrimitive.type`:
                       0 ROTATE rad, 1 FORWARD m, 2 MOVE dx dy m world frame, 3 STOP)
- `X <n> <theta0> <cruise> <yaw>`  run the staged plan. `n` = how many `Q` lines the sender believes it
                       sent; if it differs from the staged count the plan is DISCARDED and reported, so a
                       lost line can never make the robot drive a truncated path. `theta0` = start heading
                       (rad, from the last `O`), cruise/yaw 0 = compiled default.
TX (ESP32 -> Jetson): `READY`, `O x y theta_deg` (10 Hz), `K` (plan / F / T finished), and
`E <code> <count>` (cumulative). `V`/`P` (servo) are NOT implemented: `V` is answered as an unknown
command (E code 2) until the Peripheral task exists.
Extra error codes: 7 = plan discarded (count mismatch), 8 = staging buffer full (cap 256 primitives).

## Folder layout
```
firmware/
  CMakeLists.txt              ESP-IDF project root
  sdkconfig.defaults          UART, FreeRTOS tick 1000 Hz, core pinning
  README.md                   layout, task/queue diagram, pin map, build/flash, virtual ESP32
  main/                       app_main only: init order, create tasks, emit READY
  components/
    fw_core/                  PURE (no OS, no GPIO) — host-testable
      include/fw/  config.h  cmd_types.h  protocol.h  motion_state.h  plan_buffer.h
      src/         protocol.cpp  motion_state.cpp  plan_buffer.cpp
    fw_rt/                    RTOS wrappers Mailbox<T> Queue<T> Flag; FreeRTOS impl (+ host impl)
    fw_tasks/                 comm_task  motion_task  status_task   (Peripheral slot left empty)
    fw_drivers/               uart_link  step_dir_driver (MCPWM/RMT, never bit-banged)  — ESP-only
    rm_mc/                    build glue: compiles ../../project/src/{RM,MC} untouched
  tests/  test_fw.cpp
  tools/  build_test_fw.sh
simulation/esp32/
  CMakeLists.txt  vesp32.cpp  pty_link.*  host_rt.*  file_sink.*   virtual ESP32 (links fw_core + fw_tasks)
  tools/run_vesp32.sh
  output/                     state.json wheels.csv uart_rx.log uart_tx.log (git-ignored)
```
`fw_core` and `fw_tasks` are shared: the virtual ESP32 differs from the real one only in
`fw_rt` (std::thread instead of FreeRTOS) and the drivers (`pty_link` + `file_sink` instead of
UART + MCPWM).

## Plan
- [ ] 1. Folder skeleton, `config.h` (baud, tick, rates, queue depths, cap 256, placeholder pin map),
        `cmd_types.h` (Cmd kinds, ErrCode incl. 7/8, ErrorCounters).
- [ ] 2. `protocol.*`: LineAssembler + parseLine + formatters (O/K/READY/E). Pure, tested.
- [ ] 3. `plan_buffer.*`: staging of `Q` lines, `X` count check, discard on mismatch / overflow.
- [ ] 4. `motion_state.*`: IDLE / RUNNING / ESTOP around `mc::Executor` + RM (`M` via
        OmniKinematics::inverse + VelocityProfile + DriverStepDir, StepAccumulator + Odometry). Pure.
- [ ] 5. `fw_rt` (FreeRTOS + host implementations) and the Comm / Motion / Status tasks.
- [ ] 6. ESP-only drivers (`uart_link`, `step_dir_driver`) + `main/`. Unverified on hardware.
- [ ] 7. Virtual ESP32: PTY link, host runtime, file sinks, `run_vesp32.sh`.
- [ ] 8. `tests/test_fw.cpp` + `tools/build_test_fw.sh`; end-to-end check through the PTY with pyserial
        (send a real MV->MC primitive list, compare `wheels.csv` with `mc_run` output).
- [ ] 9. READMEs; add `Q`/`X`/`E` + codes to `agent/description/interfaces.md`; run
        /code-standards-review + /code-logic-review; write `agent/plan/firmware_plan.md`
        and `agent/history/2026-09-21_firmware.md`.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->

## Risks
- ESP-IDF is not installed here (cmake 3.16, g++ 9.4 only): `main/`, `fw_drivers/`, FreeRTOS `fw_rt`
  ship unverified. Everything under `fw_core`, `fw_tasks` logic and the virtual ESP32 actually runs.
- Real GPIO pin numbers are unknown -> placeholders in `config.h`. `MAX_PULSE_HZ` still a placeholder.
- Open-loop odometry (no encoder): `O` cannot see slip.
- 256-primitive cap is a guess; MV can emit up to 8200 in the worst case. Longer plans must be sent
  in chunks or the cap raised (RAM: 12 B per primitive).
- `interfaces.md` says stereo-camera also talks to this ESP32 (`V`/`P`); this firmware will not
  serve those lines until the Peripheral task exists.
