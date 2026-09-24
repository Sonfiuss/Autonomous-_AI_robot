# MC — Motion executor

Turns the primitive list produced by MV into the angular speed of each wheel for every
control tick. It is the bridge between planning (MV) and the stepper drivers (RM / firmware).

- Namespace `mc`. Body frame, metres, radians. All maths is delegated to RM.
- Heap-free, no exceptions. MC's own constants live in `config/constants.h` (`mc::cfg`).
  Every physical limit comes from `rm::cfg`.
- Not re-entrant: one executor instance per process, like MV.
- CM (Python) calls MC through the C API using `project/src/CM/mc_client.py`.

```
MV primitives ──► Executor ──► per tick: trapezoid velocity → body velocity
                                       → rm::OmniKinematics::inverse
                                       → rm::VelocityProfile (slew guard)
                                       → rm::DriverStepDir (Hz + DIR)      ──► ESP32
```

## Headers

| Header | Provides |
|---|---|
| [types.h](types.h) | `PrimitiveType`, `Primitive`, `MotionLimits`, `AxisLimits`, `MotionStep` |
| [speed_limit.h](speed_limit.h) | `peakWheelOmega`, `limitsFor`: feasible speed per direction |
| [executor.h](executor.h) | `Executor`: walks the primitive list one tick at a time |
| [mc_api.h](mc_api.h) | The plain C API for foreign callers: `mc_run`, `mc_max_speed` |
| [mc_debug.h](mc_debug.h) | `MC_DLOG(...)` debug gate |

## Features

### types.h
- **`PrimitiveType`** is `ROTATE` 0, `FORWARD` 1, `MOVE` 2, `STOP` 3. The values must equal `MvPrimitive.type`
  in `MV/mv_api.h` because the two modules exchange raw ints. `static_assert`s pin them at compile time.
  - `ROTATE`: `a` = signed angle (rad).
  - `FORWARD`: `a` = signed distance along body +x (m).
  - `MOVE`: `a` = dx, `b` = dy, a straight leg in the world frame (m).
  - `STOP`: no arguments.
- `Primitive { type, a, b }`.
- `MotionLimits { cruiseSpeed, yawRate }`: the speeds the caller asks for. Defaults are
  `cfg::CRUISE_SPEED_M_S` = 0.15 m/s and `cfg::YAW_RATE_RAD_S` = 0.8 rad/s. The executor may
  lower them per leg but never raises them.
- `AxisLimits { vMax, accel, decel }`: the feasible limits for one direction, in that direction's units.
- `MotionStep`: one control tick, containing `t`, `body` (the velocity the wheels actually
  produce, from forward kinematics of `wheels`), `wheels` (rad/s), and `steps[3]` (pulse rate
  and DIR per wheel). Integrating these rows reproduces the motion the robot really performs.

### speed_limit.h — how fast in a given direction
A 3-omni base is not equally fast in every direction. For a unit body direction `d`, the
fastest wheel fixes the ceiling:

```
peak(d) = max_i |ω_i(d)|
v_max   = MAX_WHEEL_OMEGA_RAD_S / peak(d)
a_max   = WHEEL_ACCEL_RAD_S2   / peak(d)
```

- `peakWheelOmega(kin, direction)`: the largest wheel |ω| for that direction.
- `limitsFor(kin, direction, requested)`: the feasible `AxisLimits`. Only the direction matters,
  not its magnitude. The result is never faster than `requested`. A direction that moves no wheel
  (below `cfg::MIN_WHEEL_COEFF`) returns all zeros.
- The whole vector is scaled, never one wheel clipped, so the robot stays on the planned line.
- Resulting ceilings: about 0.62 m/s forward, 0.54 m/s sideways, 2.57 rad/s spinning.

### executor.h — `Executor`
- `load(primitives, count, limits, startTheta)` loads a list. `startTheta` is the heading at the
  first tick and is needed to place `MOVE` legs. It returns false for a null list, a negative
  count, an unknown primitive type, or a singular wheel layout.
- `step(dt, &out)` advances one tick and fills a `MotionStep`. It returns false when the whole
  list has been executed (nothing written) or `dt` is too small.
- `finished()`, `heading()` (integrated from the rotation actually commanded), `elapsed()`.
- Per leg it plans an `rm::TrapezoidalProfile` over the leg length using the direction's feasible
  speed and ramp. The velocity is sampled at the middle of each tick, which integrates to second
  order and keeps distance accurate at 50 Hz.
- The trapezoid already respects the wheel acceleration limit, so the `rm::VelocityProfile` slew
  guard only acts at leg boundaries (a safety net, not the main ramp).
- `MOVE` legs carry a world-frame delta. The executor rotates it into the body frame using the
  tracked heading before inverse kinematics.
- Legs shorter than `cfg::MIN_LEG_LENGTH_M` (0.1 mm) or `cfg::MIN_LEG_ANGLE_RAD` (1e-4) are skipped.
  `STOP` emits zero-speed ticks for `cfg::STOP_HOLD_S` (0.1 s).

### mc_api.h — C API
The only header a foreign caller (Python ctypes, C, firmware) needs. Plain C structs, and the caller owns every buffer.

- `mc_run(const McRequest*, McResult*) -> int` expands the primitive list into per-tick wheel
  speeds. Outputs are valid only on `MC_OK`.
  - `McRequest`: `prims`, `n_prims`, `start_theta`, `cruise_speed`, `yaw_rate`, `dt`. A value
    of 0 or less for the last three selects the compiled default (0.15 m/s, 0.8 rad/s, 0.02 s).
  - `McStep` (per tick): `t`, achieved body velocity `u v r`, wheel speed `w[3]` (rad/s, signed),
    pulse rate `hz[3]` (never negative), `dir[3]` (+1 / -1) ready for the step/dir driver, and
    the pose `x y theta` reached at the end of the tick. The pose is a displacement from the
    trajectory's start, not a world position: add the start cell to get world coordinates.
  - `McResult`: caller buffer `steps` with `step_cap` and `step_len`, plus `duration_s` and the
    pose reached (`end_x`, `end_y`, `end_theta`), integrated through `rm::Odometry`, so a caller can
    check where the trajectory lands without replaying it.
- `mc_max_speed(u, v, r, requested, &v_max, &accel)` gives the feasible chassis speed and ramp for a
  direction (magnitude ignored, `requested` caps the result). It returns `MC_OK` or `MC_ERR_ARGS`.
- `mc_status_text(status)` and `mc_version()`.

| `McStatus` | Code | Meaning |
|---|---|---|
| `MC_OK` | 0 | success |
| `MC_ERR_ARGS` | 1 | null pointer, negative count, capacity ≤ 0 |
| `MC_ERR_PRIMITIVE` | 2 | primitive type outside 0..3 |
| `MC_ERR_KINEMATICS` | 3 | wheel layout in `constants.h` is singular |
| `MC_ERR_BUFFER` | 4 | the trajectory needs more steps than `step_cap` |

### mc_debug.h
`MC_DLOG(...)` maps to `fprintf(stderr, ...)` when compiled with `-DMC_DEBUG` and to nothing otherwise.

## Limits (`config/constants.h`)
| Constant | Value |
|---|---|
| Control tick `TICK_S` | 0.02 s (50 Hz) |
| `MAX_TRAJECTORY_STEPS` | 4096 ticks (81.9 s at 50 Hz) |
| `MAX_PRIMITIVES` | 8200 (covers MV's worst case: 2 per path cell plus a final rotate and stop) |

## Example
```c
McPrimitive prims[] = {{2, 1.0f, 0.5f}, {0, 1.57f, 0.0f}, {3, 0.0f, 0.0f}};  /* MOVE, ROTATE, STOP */
McRequest req = {prims, 3, /*start_theta=*/0.0f, 0, 0, 0};                    /* zeros -> defaults */
static McStep steps[4096];
McResult res = {steps, 4096, 0, 0, 0, 0, 0};
if (mc_run(&req, &res) == MC_OK) {
    /* per tick i: steps[i].hz[k] and steps[i].dir[k] go to wheel k's driver at 50 Hz */
}
```
