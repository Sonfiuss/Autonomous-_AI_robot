---
id: 2026-09-13_mc-executor
status: testing
module: MC (project/src/MC, C++) — new
started: 2026-09-13
---

## Task
Turn a planned way (MV primitive list) plus a commanded speed into the angular speed of each of the
three wheels, tick by tick, by calling RM. New C++ module `MC` (roadmap L5, "Frame & Kinematics"),
same shape as MV and RM: C API, unit tests, builds on PC and ESP32.

## Input
- MV output: `primitives[]` of `{type, a, b}` with type 0 ROTATE(rad), 1 FORWARD(m), 2 MOVE(dx, dy in
  the WORLD frame), 3 STOP — from `mv_plan` (`include/MV/mv_api.h`), plus the robot's start pose.
- Commanded speed: cruise linear speed (m/s) and yaw rate (rad/s); tick period dt (s).
- RM library as the only math source: `OmniKinematics`, `TrapezoidalProfile`, `VelocityProfile`,
  `DriverStepDir`, `Odometry` (`project/include/RM/`), limits from `config/constants.h`.
- User decisions (2026-09-13): new C++ MC module (not inside RM, not Python); output is a per-tick
  trajectory, not per-primitive constants and not ESP32 serial commands.

## Expected output
- `project/` builds `libmc` (static + shared), `mc_cli`, `test_mc`; `test_mc` passes.
- One row per control tick: `{t, BodyVel{u, v, r}, WheelSpeeds{w1, w2, w3}, StepCommand[3]{Hz, dir}}`.
- Replaying the produced body velocities through `rm::Odometry` lands on MV's goal pose within
  2 cm / 1 deg, and no wheel ever exceeds `cfg::MAX_WHEEL_OMEGA_RAD_S` or the accel limits.
- A commanded speed that is too high for the requested direction is scaled down, not clipped per wheel
  (clipping per wheel would bend the path).

## Plan
- [x] 1. Task file + `agent/plan/mc_plan.md`.
- [x] 2. `config/constants.h`: new `namespace mc` — `TICK_S` (0.02 = 50 Hz), `CRUISE_SPEED_M_S`,
        `YAW_RATE_RAD_S`, `MAX_TRAJECTORY_STEPS`, `POSE_TOLERANCE_M`, `ANGLE_TOLERANCE_RAD`.
- [x] 3. `include/MC/types.h`: `MotionStep {t, BodyVel, WheelSpeeds, StepCommand[3]}`, `MotionLimits`
        (cruise, yaw rate, accel), `PrimitiveType` mirroring MV's 0..3 with a static assert on the order.
- [x] 4. `include/MC/speed_limit.h` + `.cpp`: `maxBodySpeed(direction)` — run a unit BodyVel through
        `OmniKinematics::inverse`, take the largest |ω|, scale so it equals `MAX_WHEEL_OMEGA_RAD_S`.
        This is why a diagonal leg is slower than a straight one; no per-wheel clipping anywhere.
- [x] 5. `include/MC/executor.h` + `.cpp`: `Executor` walks the primitive list one tick at a time.
        Per primitive: `TrapezoidalProfile` over |a| (rad for ROTATE, m for FORWARD, hypot(a, b) for
        MOVE) -> `velocityAt(t)` -> `BodyVel`. MOVE is a WORLD-frame delta, so the executor tracks the
        heading (start theta + integrated rotation) and calls `OmniKinematics::globalToBody` before
        inverse kinematics. Then `VelocityProfile::step` as the per-wheel slew guard, then
        `DriverStepDir::toStepCommand`. STOP ramps to zero and holds.
- [x] 6. `include/MC/mc_api.h` + `src/MC/mc_api.cpp`: C API `mc_run` (caller-owned MotionStep buffer),
        `mc_max_speed`, `mc_status_text`, `mc_version`; CMake targets mc, mc_shared, mc_cli, test_mc;
        `tools/build_mc.sh`.
- [x] 7. `tools/mc_cli.cpp`: read a primitive list on stdin, print the trajectory as a table (manual check).
- [x] 8. `tests/test_mc.cpp`: straight 1 m, pure 90 deg rotation, MOVE leg from a non-zero heading,
        speed cap (request 5 m/s -> no wheel over the limit), accel limit never exceeded between ticks,
        odometry replay of a real MV primitive list lands on the goal, undersized buffer returns an error.
- [x] 9. README MC section + `agent/description/interfaces.md` MC entry, review skills, plan + history.

### Step 10 — requested by the user on 2026-09-13 ("show me output of speed of each wheel
### and motion of robot in UI"), so no longer optional
- [x] 10. CM integration: `POST /api/trajectory` and a wheel-speed chart under the map, so the drawn
        path and the three wheel curves can be read side by side.

## Open points for you
1. **Speed value.** Default cruise 0.15 m/s and yaw 0.8 rad/s, both well under the 0.54 m/s rim limit.
   Say if you want different defaults, or the speed passed per call only.
2. **Tick rate.** 50 Hz assumed, matching the PoseController note in `interfaces.md` (20 Hz default there).
   A 1 m move at 0.15 m/s is ~340 rows at 50 Hz, which the default buffer must hold.
3. **Slip.** Odometry has no encoder, so the replay test proves the maths, not the physical result.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[00:43] step 1: task intake, design decisions captured (MC module, per-tick trajectory) | info: RM already has every math piece; MC is the tick loop that calls it
[00:45] steps 2-5: mc constants, types, speed_limit (scale-not-clip), executor (trapezoid per leg, mid-tick sampling, world->body for MOVE) | info: RM does every calculation; MC only sequences the legs
[00:48] steps 6-7: C API mc_run/mc_max_speed, tools/build_mc.sh, tools/mc_cli.cpp | risk: CMake targets added but NOT verified, no cmake on this PC | info: shell build is the tested path
[00:50] step 8: tests/test_mc.cpp, 8 cases | risk: first run failed "empty list is not an error" -> Executor::load now succeeds with nothing to execute | info: measured ceilings 0.62 m/s forward, 0.54 sideways, 2.57 rad/s spin
[00:52] step 9: README MC section + interfaces.md MC C API entry
[00:53] standards-review: 5 violations fixed, 0 deferred | info: named EPS_REST/EPS_OMEGA/TICK_MIDPOINT, dropped an unused include, and MAX_PRIMITIVES replaced MAX_TRAJECTORY_STEPS as the primitive-list bound (4096 was under MV's 8198 worst case)
[01:02] logic-review: 3 mismatches fixed, 2 deviations recorded | info: (a) agent/plan/mc_plan.md was missing from step 1, now written; (b) step 3's static assert on the MV primitive codes was missing, now in MC/types.h; (c) STOP held for a fixed 0.1 s, now also waits until the wheels are actually at rest. Deviations: speed_limit exposes peakWheelOmega+limitsFor instead of the planned maxBodySpeed (superset, also returns the ramp), and POSE_TOLERANCE_M/ANGLE_TOLERANCE_RAD stayed in the test file rather than becoming unused library constants
[01:03] verification: test_mc 8/8, test_mv and test_rm recompiled from current sources and pass (constants.h is shared, so both were re-checked)
[01:06] step 10: McStep gained the per-tick pose (x, y, theta) -> mc_version 2 | info: the executor already ran an rm::Odometry per tick, so the UI needs no integration of its own
[01:08] step 10: mc_client.py ctypes bridge + POST /api/trajectory (MV then MC) | info: long routes thinned by an integer stride to MC_UI_MAX_ROWS rows; the full trajectory is still executed
[01:09] step 10: UI player + wheel-speed chart in static/index.html | info: orange robot walks the path, three wheel curves with a tick marker, readout of wheel rad/s, pulse Hz, body velocity and pose
[01:10] logic-review: 4 mismatches fixed | info: (a) an MC failure threw away MV's path, now the path still draws with trajectory.ok=false; (b) Play at the end did nothing, now replays from 0; (c) the chart recomputed its y scale every frame, now hoisted into setTrajectory; (d) interfaces.md and the CM README did not document the new pose fields or the endpoint
[01:11] verification: test_mc 8/8 (mc_version 2), test_mv, test_grounding all pass; MC-unavailable path checked with MC_LIB pointed at a missing file (200 + path kept). No browser on this PC, so the page JS was bracket-checked and reviewed, NOT executed

## Test result
