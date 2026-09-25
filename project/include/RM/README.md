# RM — Robot Movement (kinematics layer)

Pure-math C++14 layer for the 3-wheel omni base. It turns body velocities into wheel
speeds, ramps them so the steppers never skip, converts rad/s into step/direction pulse
rates, and dead-reckons the pose from step counts.

- No GPIO, timers, UART or OS calls. The same sources build on a PC (tests) and on ESP32.
- `float` only, no heap, no exceptions. The caller passes `dt`; RM never reads a clock.
- Namespace `rm`. Every constant lives in `config/constants.h` (`rm::cfg`).
- RM depends on nothing. MC calls RM for all of its maths.

```
body [u v r] ──OmniKinematics──► wheel ω ──VelocityProfile──► ramped ω ──DriverStepDir──► Hz + DIR
                                                                                            │
       pose (x y θ) ◄──Odometry◄── step counts ◄──StepAccumulator◄──────────────────────────┘
```

## Headers

| Header | Provides |
|---|---|
| [types.h](types.h) | `BodyVel`, `GlobalVel`, `WheelSpeeds`, `Pose` |
| [omni_kinematics.h](omni_kinematics.h) | `OmniKinematics`: body ↔ wheel velocities, world ↔ body frame |
| [velocity_profile.h](velocity_profile.h) | `VelocityProfile` (slew limiter), `TrapezoidalProfile` (distance moves) |
| [odometry.h](odometry.h) | `Odometry`: step counts → pose and velocity estimate |
| [driver_stepdir.h](driver_stepdir.h) | `DriverStepDir` conversions, `StepCommand`, `StepAccumulator` |
| [rm_debug.h](rm_debug.h) | `RM_DLOG(...)` debug gate |

## Features

### types.h — shared data types
| Type | Fields | Meaning |
|---|---|---|
| `BodyVel` | `u` forward, `v` left, `r` CCW | Velocity in the robot body frame (m/s, m/s, rad/s). Also used for a displacement (m, m, rad). |
| `GlobalVel` | `vx`, `vy`, `wz` | Velocity in the world frame. |
| `WheelSpeeds` | `w[3]` | Angular speed of each wheel, rad/s (or angle, rad). |
| `Pose` | `x`, `y`, `theta` | World pose in m and rad. `theta` is wrapped to (-π, π]. |

### omni_kinematics.h — `OmniKinematics`
- Builds the inverse-kinematics matrix from `cfg::WHEEL_ANGLE_RAD` (wheels at 0°, 120°, 240°; W1 at the front)
  and keeps its exact 3×3 inverse for forward kinematics.
- `inverse(BodyVel)` → `WheelSpeeds`. The map is linear, so it also converts a body
  displacement into wheel angles.
- `forward(WheelSpeeds)` → `BodyVel`. This uses the true matrix inverse (the older paper
  Eq. 17 is wrong and is not used).
- `globalToBody` / `bodyToGlobal` (static): rotate a velocity between the world and body
  frames using the heading θ.
- `valid()` is false when the wheel layout gives a singular matrix (determinant below
  `cfg::MIN_DETERMINANT`). Check it once after construction.

### velocity_profile.h — smooth ramps
- **`VelocityProfile`** limits the wheel-speed change for streaming velocity commands (`M vx vy ω`).
  - `step(target, dt)` moves the current wheel speeds toward the target without exceeding
    accel/decel limits (default `cfg::WHEEL_ACCEL_RAD_S2` = `WHEEL_DECEL_RAD_S2` = 2 rad/s²).
  - All three wheels are scaled by one common factor, so the direction of motion is kept
    while the fastest-changing wheel obeys its limit.
  - A target above `cfg::MAX_WHEEL_OMEGA_RAD_S` (≈ 9.8 rad/s) is scaled down the same way.
  - `reset()`, `current()`, `atTarget()` (tolerance `cfg::OMEGA_TOLERANCE_RAD_S`).
- **`TrapezoidalProfile`** plans distance moves (`F`, `T`).
  - `plan(distance, vMax, accel, decel)` builds accel / hold / decel phases. If the distance
    is too short to reach cruise speed it falls back to a triangle profile. It returns false
    on invalid input.
  - `velocityAt(t)`, `positionAt(t)`, `duration()`, `peakVelocity()`, `finished(t)`. Distances
    are signed.

### odometry.h — `Odometry`
- Dead-reckoning pose from per-wheel count deltas. Counts are the steps actually commanded
  (no encoder fitted, `cfg::ODOM_COUNTS_PER_REV` = 12800). A future encoder only changes that constant.
- Each update converts counts → wheel angle delta → body displacement (forward kinematics)
  → world pose. It integrates with the mid-point heading, so arcs are tracked to second order.
- `update(counts[3], dt)` and `updateWheelAngles(rad[3], dt)`. The pose always advances. The
  velocity estimate is kept from the previous call when `dt < cfg::MIN_DT_S`.
- `reset(pose)`, `pose()`, `bodyVelocity()`, `globalVelocity()`, `valid()`,
  `wrapAngle()` (static, wraps to (-π, π]).
- Limitation: commanded steps cannot show physical wheel slip.

### driver_stepdir.h — step/direction conversion
No pulse generation here: the firmware takes a `StepCommand` and drives the pins.
- `StepCommand { freqHz, forward }`: pulse rate (always ≥ 0, 0 means hold) and DIR level.
- `DriverStepDir::toStepCommand(ω)`:
  - `|ω|` below `cfg::MIN_WHEEL_OMEGA_RAD_S` (0.01) gives 0 Hz.
  - The rate is clamped to `cfg::MAX_PULSE_HZ` (20 kHz placeholder, to be tuned on hardware).
- `toOmega(cmd)`, the inverse of the above.
- `angleToSteps` / `stepsToAngle` and `distanceToSteps` / `stepsToDistance` (rim distance),
  rounded to the nearest microstep. 12800 steps per revolution (200 full steps × 64 microsteps).
- **`StepAccumulator`**: `accumulate(cmd, dt)` returns the whole signed steps emitted in a tick
  and carries the fractional remainder to the next one. `total()` and `reset()`. The firmware
  feeds these counts to `Odometry` when there is no encoder.

### rm_debug.h
`RM_DLOG(...)` maps to `fprintf(stderr, ...)` when compiled with `-DRM_DEBUG` and to nothing
otherwise. On ESP32 stderr reaches the USB serial console.

## Key chassis constants (`config/constants.h`)
| Constant | Value |
|---|---|
| Wheel radius | 0.055 m |
| Robot radius (centre → wheel contact) | 0.21 m |
| Wheel angles | W1 0° (front), W2 120°, W3 240° |
| Steps per revolution | 12800 |
| Max pulse rate / max wheel ω | 20 kHz / ≈ 9.8 rad/s (≈ 0.54 m/s at the rim) |
| Wheel accel / decel | 2 rad/s² |

## Example
```cpp
rm::OmniKinematics kin;               // check kin.valid()
rm::VelocityProfile ramp;
rm::Odometry odo;
rm::StepAccumulator acc[3];

rm::WheelSpeeds target = kin.inverse({0.2f, 0.0f, 0.0f});   // 0.2 m/s forward
rm::WheelSpeeds now    = ramp.step(target, dt);
int32_t counts[3];
for (int i = 0; i < 3; ++i) {
    rm::StepCommand cmd = rm::DriverStepDir::toStepCommand(now.w[i]);
    counts[i] = acc[i].accumulate(cmd, dt);   // firmware emits these pulses
}
odo.update(counts, dt);                       // odo.pose() is the estimated pose
```

Build and tests: see `project/README.md`.
