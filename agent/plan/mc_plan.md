# MC module plan (`project/src/MC`, C++)

## Purpose
Roadmap L5: take the way MV planned (primitive list) plus a commanded speed and produce the angular
speed of each wheel for every control tick. MC sequences the legs; RM does every calculation.

## Active
- [ ] User acceptance (task `agent/tasks/2026-09-13_mc-executor.md`): restart `python
      project/src/CM/app.py`, resolve a goal and press Play. The CM server must be restarted because
      `/api/trajectory` did not exist in the process the user had running.
- [ ] The page JS has never been run in a browser on this PC (no browser, no Node): the chart and
      player were bracket-checked and reviewed, not executed. First real run may need touch-ups.

## Next (not started)
- [ ] Feed a trajectory to the ESP32 for real: either stream `M vx vy w` per tick, or add a firmware
      path that takes wheel speeds directly. The serial protocol in `interfaces.md` has no per-wheel
      command today.
- [ ] Closed loop: re-plan from odometry instead of replaying an open-loop trajectory. Needed before
      any long route, because the robot has no encoder and cannot see slip.
- [ ] Verify the CMake targets (`mc`, `mc_shared`, `mc_cli`, `test_mc`). They are written but no CMake
      is installed on the Windows PC, so only `tools/build_mc.sh` has actually been run.
- [ ] Tune `MAX_PULSE_HZ`, `WHEEL_ACCEL_RAD_S2` and `WHEEL_DECEL_RAD_S2` on hardware: every MC ceiling
      is derived from them, so the 0.62 m/s and 2.57 rad/s figures move with them.

## Done
- [x] `speed_limit`: feasible chassis speed and ramp per direction of travel, scaling the whole
      velocity vector instead of clipping a wheel.
- [x] `Executor`: trapezoid per leg, mid-tick sampling, world-to-body rotation for MOVE legs, slew
      guard, step/dir conversion; STOP holds until the wheels are at rest.
- [x] C API (`mc_run`, `mc_max_speed`), `tools/mc_cli.cpp`, `tools/build_mc.sh`.
- [x] `tests/test_mc.cpp`: 8 cases including a real MV plan executed end to end.
- [x] README MC section and the MC C API entry in `agent/description/interfaces.md`.
- [x] Per-tick pose in `McStep` (mc_version 2), `project/src/CM/mc_client.py`, `POST /api/trajectory`,
      and the CM page's wheel-speed chart + trajectory player.

## Measured ceilings (from `mc_max_speed`, at the current constants)
| Direction | Max speed | Max acceleration |
|-----------|-----------|------------------|
| Forward | 0.62 m/s | 0.127 m/s² |
| Sideways | 0.54 m/s | 0.110 m/s² |
| Spin | 2.57 rad/s | 0.524 rad/s² |
