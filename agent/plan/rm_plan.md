# RM module plan (`project/`)

Kinematics logic layer for the 3-wheel omni base, written as a standalone
C++14 library embeddable on ESP32. Spec: `documents/software/RM/omni3wheel.md`
(note: its Eq. 17 FK is wrong; code uses the true matrix inverse).

## Done
- [x] 2026-09-07 constants.h with old chassis params (a 0.055, L 0.21, 60/180/300°, 12800 steps/rev)
- [x] 2026-09-07 OmniKinematics: general IK matrix + 3x3 inverse FK, world<->body transforms
- [x] 2026-09-07 VelocityProfile (slew limiter, common scaling keeps direction) + TrapezoidalProfile
- [x] 2026-09-07 Odometry from commanded steps, mid-point heading integration, θ wrap
- [x] 2026-09-07 DriverStepDir conversions + StepAccumulator (steps emitted per tick)
- [x] 2026-09-07 CMake (PC + ESP-IDF component), library.json, README embed guide, unit tests pass (MinGW g++ 6.3)

## Pending
- [ ] ESP32 firmware (user): UART parser, pulse generator (LEDC/RMT), 100 Hz control loop, `O` publish at 10 Hz
- [ ] Measure MAX_PULSE_HZ and accel limits on hardware; tune constants.h
- [ ] Encoder (future): set ODOM_COUNTS_PER_REV = ENCODER_PPR * ENCODER_QUADRATURE_MULT, add slip detection
- [ ] Decide whether `motivation/esp32_unified_controller` firmware is replaced by RM-based firmware
- [ ] Fix omni3wheel.md Eq. 17 + "Wheel 3 Trái-sau" typo (doc only)

## Interfaces exposed
See `project/README.md` (API summary + control-loop sketch).
