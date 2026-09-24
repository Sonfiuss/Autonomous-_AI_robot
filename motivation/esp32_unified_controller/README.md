# ESP32 unified controller — firmware

Four FreeRTOS tasks turn the Jetson's commands into wheel pulses and camera
servo PWM, and report the robot's state back. Every piece of maths comes from
`project/` (RM for kinematics, MC for the per-direction speed limit); this
firmware only sequences it and drives the hardware.

**Status: never built or flashed.** The machine this was written on has no
ESP-IDF toolchain. What IS verified is `tests/test_motion` — the state machine
and the RTOS port semantics, both built with a plain host compiler.

## Tasks

```
             UART RX                                        UART TX
                |                                              ^
                v                                              |
        +---------------+                              +---------------+
        |   Comm task   |                              |  Status task  |
        | parse + route |                              | the ONLY task |
        +---------------+                              | that writes   |
           |    |     |                                +---------------+
   estop   |    |     | servoMailbox                      ^    ^    ^
   flag    |    |     +----------------> +-------------+  |    |    |
           |    | oneShotQueue           | Peripheral  |--+    |    |
           |    | motionMailbox          | task: servo | servoSnapshot
           v    v                        +-------------+       |
        +---------------+                                      |
        |  Motion task  |----- poseSnapshot -------------------+
        | 50 Hz control |----- K notification ------------------
        | the ONLY user |
        |    of RM      |---> step/dir pulses (LEDC, 3 timers)
        +---------------+
```

| Task | Core | Priority | Cadence | Owns |
|------|------|----------|---------|------|
| Motion | 1 (alone) | 6 | 50 Hz | all RM objects, the wheels |
| Comm | 0 | 5 | blocks on RX | the line parser |
| Status | 0 | 4 | 50 Hz | **UART TX** |
| Peripheral | 0 | 3 | 50 Hz | the pan/tilt servos |

Why it is split this way:

- **The mailbox / queue split.** `M` goes into a length-1 mailbox that overwrites,
  because a velocity command from 300 ms ago is worthless. `F`/`T`/`R` go into a
  real FIFO, because each has to run to completion. Using one queue for both
  would either stall the stream or drop the moves.
- **`S` bypasses both.** Comm sets a flag that Motion reads first thing each
  tick, so a stop is acted on within 20 ms no matter how much is queued. The
  stop then **latches**: streaming `M` commands that were already in flight
  cannot restart the robot. Only a command issued *after* the stop releases it.
- **One writer on TX.** If Motion and Status both wrote, two lines would
  interleave mid-character and the Jetson would resynchronise on the wrong
  boundary.
- **Motion alone on core 1.** Step timing is the one thing in this firmware with
  a hard real-time requirement, and there is no encoder to notice when it slips.

## Velocity watchdog

Motion treats an `M` older than `MOTION_CMD_TIMEOUT_MS` (200 ms) as zero, so the
Jetson must resend it faster than that. This is deliberate: without it, pulling
the USB cable mid-drive leaves the robot running on its last command. The servo
has the same protection on `V`.

## Protocol

Exactly the lines in `agent/description/interfaces.md`, plus `E <code> <count>`
for error counters. The implementation is shared with the Jetson side —
`project/src/LINK/protocol.cpp` — so the two cannot drift apart.

Commands are **body frame**: `M vx vy omega` is forward / left / CCW as the
robot sees it. The firmware never rotates them.

## Build

```bash
idf.py set-target esp32
idf.py build flash monitor
```

`project/` is pulled in as an IDF component by `EXTRA_COMPONENT_DIRS`, so RM, MC
and LINK compile straight from their source of truth.

Host tests, no toolchain needed:

```bash
tools/build_test_motion.sh && ./build/test_motion
```

## Before this runs on real hardware

1. **Replace the pin map** in `include/fw/config.h`. Every pin there is a
   placeholder; the real `PinConfig.h` is not in this repository.
2. **Measure `MAX_PULSE_HZ`** (`project/config/constants.h`). 20 kHz is a guess.
   Raise `WHEEL_ACCEL_RAD_S2` only after the motors prove they can follow.
3. **Check the LEDC speed mode.** `step_dir_driver.cpp` uses
   `LEDC_HIGH_SPEED_MODE`, which the original ESP32 has and the S3/C3 do not. On
   those parts both drivers have to share the low-speed group, and the timer
   split needs rethinking.
4. **IDF version.** `uart_get_tx_buffer_free_size()` is used to guarantee a
   telemetry line is written whole or not at all. It exists in IDF v4.2+.
5. Floats are formatted by hand in `protocol.cpp`, so enabling newlib-nano
   (which drops `%f` from `printf`) does not corrupt telemetry.

## Files

```
include/fw/
  config.h              pins, rates, priorities, stacks, watchdogs
  context.h             RobotContext: every shared queue/snapshot, one writer each
  motion_state.h        IDLE / RUNNING_LEG / ESTOP machine  (host-testable)
  rt_port.h             Mailbox / Queue / Flag / Snapshot, FreeRTOS + host
  tasks.h               the four entry points
  fw_debug.h            FW_DLOG gate (-DFW_DEBUG)
  drivers/              uart_link, step_dir_driver, servo_driver
src/
  main.cpp              app_main: init, then create the four tasks pinned
  context.cpp           create(): queues then drivers, stops the wheels on failure
  motion_state.cpp      the state machine and the RM call chain
  drivers/  tasks/
tests/test_motion.cpp   host build: the state machine + queue semantics
tools/build_test_motion.sh
```
