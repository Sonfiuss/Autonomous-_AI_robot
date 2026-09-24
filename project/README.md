# `project/` — RM (chassis maths), MV (path planning), MC (trajectory preview), LINK (wire protocol) and SEQ (plan sequencer)

Heap-free C++ libraries that build both on a PC and on ESP32.
**RM** turns velocity commands into wheel steps, **MV** plans a route on an occupancy
grid ([MV section](#mv--path-planning)), **SEQ** walks that route onto the wire one leg
at a time, and **LINK** is the wire format itself. **MC** expands the same route into
per-wheel speeds ([MC section](#mc--motion-executor)) as a *preview*, not as the thing
that drives the motors. RM, MV and LINK depend on nothing; MC and SEQ depend on RM.
CM (Python) calls MV through its C API.

```
CM goal --> MV: A* --> primitives --> SEQ: one leg per K --> LINK --> ESP32 (RM) --> wheels
                            |
                            +-------> MC: tick loop --> wheel-speed chart on the CM page
                                      (a preview: under option A the ESP32 recomputes the
                                       ticks itself, so nothing here reaches a motor)
```

# RM — Robot Movement kinematics layer

Pure-math C++ library for the 3-wheel omni robot. Converts Jetson velocity
commands into wheel speeds, ramps them safely for stepper motors, converts
rad/s to step/dir pulse rates, and dead-reckons the pose from step counts.
No GPIO, timers, UART or OS calls: the same sources build on a PC (tests)
and on ESP32 (firmware).

```
project/
  config/constants.h      every physical + tuning constant (constexpr)
  include/RM/
    types.h               BodyVel, GlobalVel, WheelSpeeds, Pose
    omni_kinematics.h     body <-> wheel velocities, world <-> body frame
    velocity_profile.h    VelocityProfile (slew limiter), TrapezoidalProfile
    speed_limit.h         AxisLimits: feasible speed + ramp for a direction of travel
    odometry.h            counts -> pose (x, y, θ) + velocity estimate
    driver_stepdir.h      ω <-> (Hz, DIR), angle/distance <-> steps, StepAccumulator
    rm_debug.h            RM_DLOG(...) gate, enabled by -DRM_DEBUG
  src/RM/*.cpp
  tests/test_rm.cpp       unit tests (plain asserts)
  CMakeLists.txt          PC build; also works as an ESP-IDF component
  library.json            PlatformIO library manifest
```

## Build and test on PC

With CMake:
```bash
cd project && mkdir build && cd build
cmake .. && cmake --build . && ctest --output-on-failure
```

Without CMake (any g++/clang++):
```bash
cd project
g++ -std=c++14 -Wall -Wextra -Iinclude -Iconfig src/RM/*.cpp tests/test_rm.cpp -o test_rm && ./test_rm
```

## Embedding on ESP32

The library follows embedded constraints: `float` only, no heap, no
exceptions/RTTI, no iostream or STL containers, no static initialisers with
side effects. Time is never read internally; the caller passes `dt`.

**PlatformIO (Arduino or ESP-IDF framework)** — in `platformio.ini`:
```ini
lib_extra_dirs = ../          ; directory that contains project/
```
or copy `project/` into `lib/rm/`. Include with `#include "RM/odometry.h"`.

**ESP-IDF** — copy or symlink `project/` to `<firmware>/components/rm/`.
The `CMakeLists.txt` detects the IDF build and registers the component.
Add `REQUIRES rm` to the main component.

## Control-loop sketch (firmware side, to be written)

```cpp
#include "RM/driver_stepdir.h"
#include "RM/odometry.h"
#include "RM/omni_kinematics.h"
#include "RM/velocity_profile.h"

using namespace rm;

static OmniKinematics  kin;
static VelocityProfile ramp;          // defaults from constants.h
static Odometry        odom;
static StepAccumulator emitted[cfg::NUM_WHEELS];
static GlobalVel       cmdFromJetson; // filled by the UART parser ("M vx vy w")

void controlTick(float dt) {          // call at a fixed rate, e.g. 100 Hz
    // 1. world -> body frame using current heading, then body -> wheel speeds
    const BodyVel     body   = OmniKinematics::globalToBody(cmdFromJetson, odom.pose().theta);
    const WheelSpeeds target = kin.inverse(body);

    // 2. ramp toward target without exceeding accel/decel limits
    const WheelSpeeds smooth = ramp.step(target, dt);

    // 3. convert to pulse rate + DIR, hand to the pulse generator (LEDC/RMT/timer)
    int32_t counts[cfg::NUM_WHEELS];
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        const StepCommand sc = DriverStepDir::toStepCommand(smooth.w[i]);
        setPulseOutput(i, sc.freqHz, sc.forward);           // firmware-specific
        counts[i] = emitted[i].accumulate(sc, dt);           // steps commanded this tick
    }

    // 4. dead-reckon pose from the commanded steps (no encoder yet)
    odom.update(counts, dt);
    // 5. publish "O x y theta_deg" at 10 Hz: theta_deg = odom.pose().theta * cfg::RAD_TO_DEG
}
```

Frame of `M vx vy w`: `interfaces.md` does not say. The old ESP32 firmware
applied it directly as body-frame velocity; the RM spec assumes world frame
and rotates by θ (step 1 above). If the Jetson sends body-frame velocities,
skip `globalToBody` and build `BodyVel{vx, vy, w}` directly.

Distance moves (`F <dist> <spd>`, `T <deg> <rads>`): plan a `TrapezoidalProfile`
on the chassis distance, sample `velocityAt(t)` each tick into a `BodyVel`,
and feed it through the same steps 1–4. Send `K` when `finished(t)`.

## Conventions

- Body frame: `u` forward (+x), `v` left (+y), `r` CCW (+z). θ = 0 means the
  robot faces world +x.
- Wheel i sits at angle `WHEEL_ANGLE_DEG[i]` from +x; positive ω_i pushes the
  robot CCW around its centre.
- IK: `ω_i = (-sin α_i · u + cos α_i · v + L · r) / a`. FK is the exact 3x3 inverse.
- `constants.h` chassis values come from `agent/description/project_overview.md`
  (a = 0.055 m, L = 0.21 m, 12800 steps/rev). Tune `MAX_PULSE_HZ`,
  `WHEEL_ACCEL_RAD_S2`, `WHEEL_DECEL_RAD_S2` on hardware.

# MV — path planning

A* on an occupancy grid, from the robot pose to a goal pose resolved by CM
(roadmap L4). Same embedded constraints as RM: `float` only, no heap, no
exceptions/RTTI, every buffer owned by the caller.

```
project/
  include/MV/
    types.h               Point, Pose, Cell
    occupancy_grid.h      OccupancyGrid: room + polygons rasterised and inflated
    astar.h               8-connected A*, octile heuristic, no corner cutting
    path.h                line-of-sight smoothing, waypoints -> primitives
    mv_api.h              C API (MvRequest / MvResult / mv_plan / mv_grid)
    mv_debug.h            MV_DLOG(...) gate, enabled by -DMV_DEBUG
  src/MV/*.cpp
  tools/mv_cli.cpp        text-script driver for manual tests
  tools/build_mv.sh       shared library + CLI + tests without CMake
  tests/test_mv.cpp       unit tests (plain asserts)
```

The robot is planned as a point: walls and object polygons are inflated by
`robot_radius + mv::SAFETY_MARGIN_M`, so a returned path always clears
obstacles by the hull radius. Start and goal inside an inflated cell are snapped
to the nearest free cell within `mv::SNAP_RADIUS_M`, otherwise `mv_plan` returns
`MV_ERR_START_BLOCKED` / `MV_ERR_GOAL_BLOCKED`. Cell size and grid limits live in
`config/constants.h` (`namespace mv`).

Primitives replay the smoothed waypoints: `ROTATE θ`, `FORWARD d`, a final
`ROTATE` onto the goal heading and `STOP`. With `holonomic = 1` each leg becomes
a single `MOVE dx dy` instead, which the omni chassis can execute directly.

## Build and test

```bash
cd project && mkdir -p build && cd build
cmake .. && cmake --build . && ctest --output-on-failure    # test_mv, mv_cli, mv.dll / mv.so
```

Without CMake (the shared library must match the Python interpreter's bitness):
```bash
cd project && tools/build_mv.sh                     # -> build/mv/{mv.dll|libmv.so, mv_cli, test_mv}
CXX=/c/msys64/mingw64/bin/g++ tools/build_mv.sh     # 64-bit MinGW on this PC
```

Manual run, grid and plan printed as text:
```bash
printf 'room 4 3\nrobot 0.22\npoly 4 1 1 2 1 2 2 1 2\nstart 0.5 0.5 0\ngoal 3.5 2.5 0\ngrid\nplan\n' \
  | build/mv/mv_cli
```

## Calling MV from Python

`project/src/CM/mv_client.py` wraps the C API with ctypes and is what the CM
server uses for `POST /api/plan`. It searches `build/mv/` then `build/`, or the
path in the `MV_LIB` environment variable.

```python
import mv_client
result = mv_client.plan_path(scene, {"x": 1.2, "y": 0.8, "theta": 1.57})
# {"ok": True, "path": [...], "waypoints": [...], "primitives": [...], "length_m": 3.14}
```


# MC — trajectory preview

Takes the primitive list MV produced plus a commanded speed, and reports the angular
speed of each wheel for every control tick. It contains no maths of its own: every
value comes from an RM call.

> **MC is a tool, not a stage of the live pipeline.** Under the option-A architecture
> (decided 2026-09-16) the ESP32 owns the kinematics: the Jetson sends `F`/`T`/`M`
> and the firmware's Motion task runs this same RM chain live, one tick at a time.
> Nothing MC produces is ever sent to the robot.
>
> What MC is for: **drawing** the trajectory and the three wheel-speed curves in the CM
> page, and **checking feasibility** before committing — how long a plan takes, whether
> it exceeds any limit, and where it ends up. `test_mc` also replays a real MV plan
> through `rm::Odometry` end to end, which is the strongest check the RM chain has.
>
> MC predates that decision; it was written on 2026-09-13, when the Jetson was still
> expected to drive the wheels directly.

```
project/
  include/MC/
    types.h           Primitive, MotionLimits, MotionStep
    executor.h        Executor: walks the primitive list one tick at a time
    mc_api.h          C API (McRequest / McResult / mc_run / mc_max_speed)
    mc_debug.h        MC_DLOG(...) gate, enabled by -DMC_DEBUG
  src/MC/*.cpp
  tools/mc_cli.cpp    text-script driver, prints the trajectory as a table
  tools/build_mc.sh   shared library + CLI + tests without CMake
  tests/test_mc.cpp   unit tests (plain asserts)
```

Per leg the executor plans an `rm::TrapezoidalProfile` over the leg length, then each
tick samples it at the middle of the tick and pushes the result through
`rm::OmniKinematics::inverse`, `rm::VelocityProfile` as a slew guard, and
`rm::DriverStepDir`. Mid-tick sampling integrates to second order, so the travelled
distance stays accurate at 50 Hz. The reported body velocity is the forward kinematics
of the wheel speeds actually commanded, so replaying the rows reproduces the real motion.

**Speed is scaled, never clipped.** A 3-omni base is not equally fast in every
direction, so `rm::limitsFor` computes the ceiling from the fastest wheel and scales the
whole velocity vector. Clipping one wheel at its limit would bend the robot off the
planned line. That rule lives in RM, so the firmware applies exactly the same one.
At the compiled limits:

| Direction | Max speed | Max acceleration |
|-----------|-----------|------------------|
| Forward (body +x) | 0.62 m/s | 0.127 m/s² |
| Sideways (body +y) | 0.54 m/s | 0.110 m/s² |
| Spin | 2.57 rad/s | 0.524 rad/s² |

`MOVE` legs carry a world-frame delta, so the executor tracks the heading it has
commanded and rotates the leg into the body frame before inverse kinematics.

## Build and test

```bash
cd project && tools/build_mc.sh          # -> build/mc/{mc.dll|libmc.so, mc_cli, test_mc}
CXX=/c/msys64/mingw64/bin/g++ tools/build_mc.sh    # 64-bit MinGW on this PC
```
CMake targets `mc`, `mc_shared`, `mc_cli` and `test_mc` exist too, on the same
`cmake .. && cmake --build . && ctest` flow as the other modules.

Manual run: drive half a metre, turn 90°, stop, printing every 20th tick.
```bash
printf 'speed 0.15 0.8
forward 0.5
rotate 1.5708
stop
every 20
run
' | build/mc/mc_cli
```
`limit <u> <v> <r>` prints the feasible speed and ramp for any direction.

# SEQ — plan sequencer

The loop between a plan and a moving robot. `seq::Sequencer` walks an MV primitive
list onto the wire one leg at a time: send a leg, wait for the firmware's `K`, send
the next. Jetson-side only — it is deliberately NOT registered as an ESP-IDF
component, because the firmware executes legs and does not sequence them.

```
project/
  include/SEQ/
    sequencer.h           State, Status, Config, Feedback, Action, Sequencer
    seq_debug.h           SEQ_DLOG(...) gate, enabled by -DSEQ_DEBUG
  src/SEQ/sequencer.cpp
  tools/build_seq.sh
  tests/test_seq.cpp      15 groups, all of them on a PC with no robot
```

**The handshake is the protocol.** The firmware's Motion task pops a one-shot only
while it is idle, so legs sent back to back pile into an 8-deep FIFO and the ninth
is dropped as `E 4`. Exactly one leg is ever in flight.

**Pure, and that is the point.** No port, no clock, no heap, no exceptions. The
caller polls `update(Feedback) -> Action` with what it knows — the millisecond
clock, the ack and `READY` counts, the firmware's error tallies — and sends back
the one command that comes out. `test_seq` therefore drives every failure path
(ack timeout, dropped one-shot, mid-plan reboot, abort, a refused leg) against a
fake firmware, with no ESP32 and no serial port. The platform half is
`motivation/jetson/MissionRunner`, which owns only the loop and the port.

| MV primitive | Wire | Acked |
|---|---|---|
| `ROTATE(rad)` | `T <deg> <rate>` | yes |
| `FORWARD(m)` | `F <dist> <speed>` | yes |
| `MOVE(dx, dy)` | `T` onto the bearing, then `F` along it | yes, both |
| `STOP` | `S` | no |

`MOVE` is decomposed rather than streamed as `M`: the protocol has no world-frame
straight-line command, and a streamed leg has no acknowledgement to wait for. Since
`mv::toPrimitives` emits its trailing `ROTATE` relative to the *planned* heading,
the sequencer tracks the planned and the commanded heading separately and turns
each `ROTATE` into the absolute heading the plan meant. A non-holonomic plan has
zero bias and maps 1:1; a holonomic plan of the same route produces the same wire
lines as its non-holonomic twin.

Two firmware facts it exists to respect:

- **A leg the firmware refuses vanishes silently** — `applyOneShot` discards the
  bool from `beginForward`/`beginTurn`, so a leg under `mc::cfg::MIN_LEG_LENGTH_M`
  or `MIN_LEG_ANGLE_RAD` produces no `K` and bumps no counter. SEQ filters those
  out itself, with the same constants.
- **Legs are timed, not pose-driven**, so each leg's duration is computed with the
  same `rm::limitsFor` + `rm::TrapezoidalProfile` call the firmware plans it with.
  The ack timeout is that duration times `seq::cfg::ACK_TIMEOUT_MARGIN` plus
  `ACK_TIMEOUT_FLOOR_MS`, not a fixed guess.

## Build and test

```bash
cd project && tools/build_seq.sh && ./build/seq/test_seq
```
CMake target `seq` and test `test_seq` exist too, on the same
`cmake .. && cmake --build . && ctest` flow as the other modules.

Run a plan against a real robot (needs Linux and the ESP32):
```bash
cd project && printf 'room 6 6\nrobot 0.25\nstart 0.5 0.5 0\ngoal 5 5 1.57\nplan\n' \
  | build/mv/mv_cli > plan.txt
cd ../motivation/jetson && ./build/robot_link --run-plan ../../project/plan.txt --start-theta 0
```
`mv_cli` output is a valid plan file as it stands: the reader takes the lines that
start with `ROTATE`/`FORWARD`/`MOVE`/`STOP` and ignores everything else.
