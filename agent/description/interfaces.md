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

### motivation TX → ESP32
```
M <vx> <vy> <omega>\n    velocity (m/s, m/s, rad/s)
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
```

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
