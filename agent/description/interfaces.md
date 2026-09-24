# Module Interfaces

## ZMQ (WiFi — Laptop ↔ Jetson)
| Direction | Port | Socket type | Message format |
|-----------|------|-------------|----------------|
| Simulation → motivation | **5555** | PUB → SUB | `G <x_m> <y_m> <theta_deg>` |
| motivation → Simulation | **5556** | PUB → SUB | `O <x_m> <y_m> <theta_deg>` |

- Jetson **binds** both ports; Laptop **connects** to Jetson IP
- `JETSON_IP=192.168.x.x python simulation/movement/app.py`
- Scale note: simulation uses cm internally, converts to meters before ZMQ send (`CM_TO_M = 0.01`)

## UART (USB serial — Jetson ↔ ESP32)
Both `motivation` and `stereo-camera` use ESP32 over `/dev/ttyUSB0` at 115200 baud.
**They share the same ESP32 unified controller** — do not run both simultaneously on the same port.

Both directions are implemented once, in `project/src/LINK/protocol.cpp`, and compiled into the
ESP32 firmware (`motivation/esp32_unified_controller/`) AND the Jetson application
(`motivation/jetson/`). Neither side has its own copy of the line format.

`M` is in the **body frame** — forward, left, CCW as the robot sees it. Decided 2026-09-16; the
question was previously open in `agent/history/2026-09-07_rm.md`. The firmware never rotates it.

### motivation TX → ESP32
```
M <vx> <vy> <omega>\n    velocity (m/s, m/s, rad/s), BODY frame
F <dist_m> <spd_ms>\n    move forward N metres
T <angle_deg> <rads>\n   turn N degrees
V <pan_v> <tilt_v>\n     camera servo velocity (deg/s)
S\n                       stop all motion
R\n                       reset odometry to 0
```

### motivation RX ← ESP32
```
O <x> <y> <theta_deg>\n  odometry at 10 Hz
P <pan> <tilt>\n          servo angles at 50 Hz
K\n                        motion-complete ack
READY\n                   boot acknowledgement
E <code> <count>\n        error tally, at most 1 Hz (added 2026-09-16)
```

`E` carries a cumulative count per code, sent only when that count moves — a healthy robot emits
none at all. Counts never reset, so a lost `E` costs no information; the receiver diffs consecutive
values to get a rate.

| Code | Meaning |
|------|---------|
| 1 | malformed line (bad argument count or a value out of range) |
| 2 | unknown command character |
| 3 | line longer than the receive buffer, dropped |
| 4 | one-shot queue full, an `F`/`T`/`R` was dropped |
| 5 | TX buffer full, a periodic message was dropped |
| 6 | servo target clamped to a mechanical limit |

### Streaming watchdog
`M` and `V` are **streamed** commands and expire: the firmware treats an `M` older than
`link::cfg::MOTION_CMD_TIMEOUT_MS` (200 ms) and a `V` older than 300 ms as zero. Going quiet means
**stop**, never "hold course" — that is what halts the robot when the USB cable is pulled. A sender
must refresh inside `link::cfg::KEEPALIVE_MS` (50 ms). All three constants live in
`project/config/constants.h` because they bind both sides.

### stereo-camera ServoClient TX/RX (subset)
```
TX: V <pan_vel> <tilt_vel>\n   set servo angular velocity
TX: S\n                         stop servos
TX: R\n                         reset servos to 0°
RX: P <pan> <tilt>\n            position feedback at 50 Hz
RX: READY\n                     boot ack
```

## Simulation Flask API (port 5000)
| Endpoint | Method | Body / Response |
|----------|--------|-----------------|
| `/api/robot/goal` | POST | `{x: cm, y: cm}` → sends ZMQ goal in meters |
| `/api/robot/pose` | GET | `{x_cm, y_cm, theta_deg, connected, ts}` |
| `/api/robot/stop` | POST | Sends `S` over ZMQ |
| `/api/pathfind` | POST | A* path `{start, goal, obstacles}` |
| `/api/config` | GET/POST | Robot/grid/simulation config |

## PoseController (motivation internal)
`compute(current_pose, target_pose)` → `{vx, vy, omega, reached}`
Drives motion loop at configurable Hz (default 20 Hz).

## CM Flask API (port 5001) — LLM grounding + path planning
| Endpoint | Method | Body / Response |
|----------|--------|-----------------|
| `/api/scene/new?seed=N`, `/api/scene/latest` | GET | `{scene, candidates, slots, provider}` |
| `/api/ground/say` | POST | `{text}` → `{type: ask\|goal\|none\|error, text, slots, matches[{id,x,y,theta,description}], resolved}` |
| `/api/ground/reset` | POST | clears slots, history and the resolved goal |
| `/api/plan` | POST | `{goal?, start?, holonomic?}` → `{ok, path[], waypoints[], primitives[], length_m, snapped_start, snapped_goal, goal}`; `goal` defaults to the spot the dialog last resolved |
| `/api/trajectory` | POST | `{goal?, start?, cruise_speed?, yaw_rate?, dt?}` → the `/api/plan` reply plus `trajectory` (MC per-tick rows) and `origin` (the pose the rows are relative to) |

Units everywhere: metres and radians, world frame, θ CCW from +x. Distinct from the simulation
app's `/api/pathfind` (port 5000, centimetres).

## MV C API (`project/include/MV/mv_api.h`) — CM → MV
`mv_plan(const MvRequest*, MvResult*) -> MvStatus` and
`mv_grid(const MvRequest*, unsigned char* out, int cap, int* cols, int* rows, float* res_m)`.
- `MvRequest`: room w/l, `robot_radius` (disc; MV adds `mv::SAFETY_MARGIN_M`), object polygons as a
  flat vertex array + per-polygon sizes, start/goal `MvPose`, `holonomic` flag.
- `MvResult`: caller-owned `path` / `waypoints` / `prims` buffers with capacities, plus `length_m` and
  the snapped start/goal actually searched from.
- `MvPrimitive.type`: 0 ROTATE(rad), 1 FORWARD(m), 2 MOVE(dx, dy), 3 STOP. MOVE replaces the roadmap's
  ARC: the omni chassis drives a leg directly. Status ≠ 0 means no path was produced (never a partial one).
- Python side: `project/src/CM/mv_client.py` (ctypes); the shared library must match the interpreter's
  bitness.


## SEQ C++ API (`project/include/SEQ/sequencer.h`) — MV → LINK

The piece that puts a plan on the wire. `seq::Sequencer` walks an MV primitive list one leg at a
time: send a leg, wait for the firmware's `K`, send the next. Added 2026-09-24; until then nothing
connected the planner to the robot.

- **The handshake is the protocol, not a precaution.** The firmware's Motion task pops a one-shot
  only while it is idle, so legs sent back to back pile into an 8-deep FIFO and the ninth is dropped
  as `E 4`. Exactly one leg is ever in flight.
- `load(primitives, count, startTheta, config)` → false on a null list, a count outside
  1..`seq::cfg::MAX_PRIMITIVES`, an unknown primitive type, a speed outside the `link::cfg` ranges,
  or a singular wheel layout. `Config{cruiseSpeed, yawRate}`; either at 0 means the compiled default.
- `update(Feedback) -> Action` is polled by the caller and returns **at most one** `link::Command`.
  `Feedback{nowMs, acks, readies, faults[], linkOk}` is everything the caller knows about the link;
  the three counters are compared for INEQUALITY, so they survive the uint32 wrap.
- Pure: no port, no clock, no heap, no exceptions. The platform half is
  `motivation/jetson/MissionRunner`, which owns the loop and the port and nothing else.

### Primitive → wire mapping

| MV primitive | Wire | Acked |
|---|---|---|
| `ROTATE(rad)` | `T <deg> <rate>` | yes, by `K` |
| `FORWARD(m)` | `F <dist> <speed>` | yes, by `K` |
| `MOVE(dx, dy)` | `T` onto the bearing, then `F` along it | yes, both |
| `STOP` | `S` | no — the firmware never acks a stop |

**`MOVE` is decomposed, not streamed** (decision A1, 2026-09-24). The protocol has no world-frame
straight-line command, and streaming `M` would leave the leg with no acknowledgement to wait for —
the robot would be driving on the Jetson's timing, which is what option A exists to avoid. The cost
is that the robot turns onto each bearing instead of crabbing; it reaches the same poses.

Decomposition changes the heading mid-plan, and `mv::toPrimitives` emits its trailing `ROTATE`
relative to `startTheta` (its holonomic branch never advances the planned heading). The sequencer
therefore tracks two headings — the one the plan assumes and the one actually commanded — and turns
each `ROTATE` into the **absolute** heading the plan meant. A non-holonomic plan has zero bias, so it
maps 1:1; a holonomic plan of the same route produces the same wire lines as its non-holonomic twin.

### Two things that will hang a sequencer that does not know them

- **A leg the firmware refuses vanishes silently.** `applyOneShot` discards the bool from
  `beginForward`/`beginTurn`, so a leg under `mc::cfg::MIN_LEG_LENGTH_M` / `MIN_LEG_ANGLE_RAD`
  produces no `K` and bumps no counter. The sequencer drops those itself, using the same constants.
- **Legs are timed, not pose-driven**, so a leg's duration is computable exactly: `rm::limitsFor` +
  `rm::TrapezoidalProfile`, the same call the firmware makes. The ack timeout is that duration times
  `seq::cfg::ACK_TIMEOUT_MARGIN` plus `ACK_TIMEOUT_FLOOR_MS`, not a fixed guess.

### Failing early instead of waiting out the timeout

| Feedback | Status | Why |
|---|---|---|
| `E 1` / `E 2` moved | `FIRMWARE_REJECT` | a line of ours was refused |
| `E 4` moved | `QUEUE_FULL` | a one-shot was dropped; its `K` is never coming |
| `readies` moved | `REBOOTED` | the odometry is back at zero, so the plan is void |
| `linkOk` false | `LINK_ERROR` | the port failed |
| no `K` in time | `ACK_TIMEOUT` | |
| `abort()` | `ABORTED` | |

Every one of these emits `S`. `RobotState` gained `readies` for this: `ready` latches true on the
first `READY` and so cannot tell a boot from a reboot mid-plan.

`READY` is a grace period, not a gate — the firmware may have booted long before the port was
opened, in which case its `READY` is gone for good. After `seq::cfg::READY_WAIT_MS` the plan starts
anyway. Every counter is re-baselined at the moment the plan goes live, so a fault raised before the
first leg is not blamed on it.

### Jetson CLI

```bash
robot_link --run-plan plan.txt [--start-theta DEG] [--speed M_S] [--yaw-rate RAD_S]
```

`plan.txt` holds one primitive per line (`ROTATE rad` | `FORWARD m` | `MOVE dx dy` | `STOP`); every
other line is ignored, so **`mv_cli plan ... > plan.txt` output is a valid plan file as it stands**.

## MC C API (`project/include/MC/mc_api.h`) — MV → MC → wheels
`mc_run(const McRequest*, McResult*) -> McStatus` expands a primitive list into per-tick wheel speeds.
- `McRequest`: `prims` (same int codes as `MvPrimitive.type`), `n_prims`, `start_theta` (rad, needed to
  place MOVE legs), `cruise_speed` (m/s), `yaw_rate` (rad/s), `dt` (s). Any of the last three at 0 means
  the compiled default (0.15 m/s, 0.8 rad/s, 0.02 s).
- `McStep` per tick: `t`, achieved body velocity `u, v, r`, `w[3]` wheel speed (rad/s, signed),
  `hz[3]` pulse rate (never negative), `dir[3]` (+1 / -1) ready for the step/dir driver, and the pose
  `x, y, theta` reached at the end of that tick. The pose is a DISPLACEMENT from the trajectory's
  start, not a world position: add the start cell to get world coordinates (mc_version 2 added it).
- `McResult` also reports `duration_s` and the pose reached (`end_x`, `end_y`, `end_theta`), integrated
  through `rm::Odometry`, so a caller can check where the trajectory lands without replaying it.
- `mc_max_speed(u, v, r, requested, &v_max, &accel)` gives the feasible chassis speed and ramp for a
  direction. The chassis is not equally fast in every direction: ~0.62 m/s forward, ~0.54 m/s sideways,
  ~2.57 rad/s spinning. An over-fast request is scaled as a whole vector, never clipped per wheel.
- Not re-entrant: one executor instance per process, like MV.
