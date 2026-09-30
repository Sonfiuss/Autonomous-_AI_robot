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
  direction. The chassis is not equally fast in every direction: ~0.45 m/s forward, ~0.39 m/s sideways,
  ~1.77 rad/s spinning (r 0.040 m since 2026-09-27, L 0.2217 m since 2026-09-29). An over-fast request is scaled as a whole vector,
  never clipped per wheel.
- Python: `mc_client.chassis()` → `{wheel_radius_m, robot_radius_m, steps_per_rev}` read from
  `project/config/constants.h` (no C API exposes them); `communication/test_drive` checks libmc's pulses
  agree, so a libmc not rebuilt after the header changed fails there. demo_drive records the radius as
  `run.json` `plan.wheel_radius_m`.
- Not re-entrant: one executor instance per process, like MV.

## Drive command JSON (LLM / rules → validator) — `communication/prompts/drive_command.json` v1.2
`{"steps": [{"action", "value", "unit"}], "question": str, "goal": str}` — at most one of steps / question / goal.
- action: `forward | backward | turn_left | turn_right | turn_around | turn` (`turn` = no side said: the validator
  makes a 180 / 360° one `turn_left`, asks for any other angle); unit `mm | cm | m` or `deg | rev` (rev × 360 in code).
- `turn_around` takes a stated angle (default 180, left). Every stated value must be a number the user wrote, in order.
- `goal`: the user's own words for a place in the room (every word must appear in the command, articles aside);
  resolved by CM grounding on the real map (`communication/goto.py`), never by the parser.

## realroom (the real room's map) — `realroom/`
Scene JSON v2.0 (schema: `simulation/room/README.md`) + a known grid, published from `vision/drive_map.py --publish`.
- `realroom/maps/map_<run>.json` (as built), `map_<run>_grid.npz` (`cells` int8 [iy, ix]: 1 occupied, 0 free —
  seen or under the robot's path, −1 unseen; `res_m`; `origin_m` = scene-frame corner of cell (0, 0)),
  `latest.json` (the map commands use: `robot` = CURRENT pose, `pose_history` [{t, source, predicted, x, y, theta}]),
  `map_<run>.png` / `latest.png` (`map_image.py`, redrawn on every publish / pose change).
- Scene additions (2026-09-29): `objects[].color` = one of black white gray red orange yellow green blue purple pink
  brown or "unknown" (`vision/object_color.py`), `objects[].color_rgb` "#rrggbb" | null, `objects[].color_share`
  (0..1, the name's share of the pixel votes) | null; `robot.chassis / wheels
  [{angle_deg, polygon}] / footprint` (body frame) = the simulator's outline turned 180° → wheels 0/120/240, W1 in
  front (`scene_export.robot_shape`). Maps published before lack them (drawn as a disc).
- Object classes (2026-09-30): a region's class counts the detector votes on its cells and within
  `LABEL_REACH_CELLS` (2) of them when nearer to it than to any other object (a vote counts once); a YOLO box whose floor point lies behind an obstacle votes on that obstacle (`drive_map.support_points`: a
  person on a bed); a ≥ 1 m line holding ≥ `WALL_OBJECT_MIN_VOTES` (10) of one class is that object's side — an
  object of the line's own cells, not a wall.
- drive_map `RUN/map/objects.json` (landmarks, map frame) `[{class, x, y, range_m, sightings, mean_conf, first_seq,
  last_seq, extent_m [x min, y min, x max, y max], parts}]`: clusters of one class that one YOLO box held together
  (no farther apart than the box is wide + 0.15 m) are one object of `parts` clusters; a box ending ≤ 8 px above
  the frame's bottom gives none.
- Frame: the building run's start frame (x forward, y left, θ CCW) shifted by `map_offset` (scene = map − offset). m, rad.
- `map_store`: `publish_run(run_dir, reset_pose=False)` (the current map published again in the same frame keeps
  the robot's pose + history), `load_latest()`, `load_map(id)`, `load_grid(scene) -> KnownGrid(cells, res, origin)`,
  `robot_pose(scene)`, `save_pose(pose, source, predicted)`, `move_robot(motion, source, predicted)` (motion in the
  robot's body frame), `compose(pose, motion)`.
- `planner.plan(scene, grid, goal) -> RoomPlan(ok, reason, legs [(ROTATE rad | FORWARD m)], path, length_m,
  unseen_share, occupied_cells, goal)`: MV non-holonomic from the stored pose, refused when the robot's swept disc
  touches an occupied cell, crosses > `MAX_UNSEEN_SHARE` (0.35) unseen cells, or MV moved the start > 5 cm.
- Flask port **5002**: `GET /api/map/latest`, `GET /api/map/<id>` (scene + grid cropped to the room), `GET /api/spots`
  (CM candidates), `POST /api/plan {spot | goal}` → RoomPlan (view only), `GET /api/map/latest.png[?spot=<id>]` and
  `GET /api/map/<id>.png` (image/png), `GET /api/map/version` → `{version}` (latest.json's mtime_ns as a string, null
  before any map; `map_store.map_version()`).
- Chat + run control (2026-09-30, `realroom/drive_chat.py`; the page drives the REAL robot):
  - `POST /api/chat {text}` → `{type: ask | plan | none | busy | error, text, plan}`; after `ask` the next message is the
    answer. `plan` = `preview()`: `{command, provider, goal, target, steps [words], legs [{primitive, wire, duration_s,
    label}], duration_s, length_m, map_id, path [[x, y]] (map frame, the pose composed leg by leg), occupied_cells,
    unseen_share}`; `target` = `Prepared.target` (below) or null for steps. `busy` while a run goes. A map request
    ("lập bản đồ", `cmd_parser.split_map_request`) → `none` with `drive_chat.NO_MAPPING` (2026-09-30 chat-ui: the
    chat moves on the map it has; maps come from the terminal). `POST /api/chat/reset` drops the question and the plan.
    A whole message in `drive_chat.RUN_WORDS` ("chạy", "ok", "run"…; `ChatSession.intent`, not while a question
    waits) runs the plan → `{type: run, text, plan: null, run: /api/run's body}` (none + `NO_PLAN_TO_RUN` without
    one); in `GO_ON_WORDS` ("đi tiếp", "tiếp tục") → `ChatSession.go_on(RunManager.unreached())`. `POST
    /api/chat/go_on` = the same. `go_on(target)`: a plan (provider `go on`) from the pose now to `target.spot_id`
    (then the object's other spots), `none` with `NOTHING_TO_GO_ON` / `GO_ON_MAP_CHANGED`.
  - `POST /api/run {compensate}` → `{ok, run_id, compensate, dry_run}`; 400 no plan, 409 a run is going. The plan is spent.
    Runs `demo_drive --yes --plan-json <run>/approved_plan.json --run-dir <run> [--compensate]` (`--dry-run --no-vision`
    when the app has `--dry-run`) in a new session; `POST /api/run/stop` = SIGINT to that process group, once.
  - `GET /api/run/status` → `{state: idle | running | stopping | finished, run_id, line (status.txt), log (console.log
    tail), elapsed_s, target (the plan's), exit_code, result (run.json; cancelled when stopped before run.json existed),
    end_pose, map (last map_* event), video, message, go_on, dry_run}`. `go_on`: the run has a target, ended > 0.15 m
    from it on the same map, the pose known, not a dry run / cancelled. `message` = `drive_chat.said()` (+ where the
    robot is for a run that stopped) + `leg_report(legs, run.json)` lines when not arrived: "Đã tới <target
    name>…" when end_pose (same map_id) is within `ARRIVE_TOL_M` 0.15 m of the target, else how far off / stopped /
    failed / cancelled / not run / a dry run.
- `REALROOM_MAPS_DIR` (env) replaces `realroom/maps` for map_store in every process that inherits it (tests).

## demo_drive run.json additions (2026-09-28)
- `start_pose` / `end_pose` `{map_id, x, y, theta[, predicted]}`: the robot on the real map before / after the run.
- `--compensate`: `executed_legs` [{primitive, wire, duration_s, waits_ready, role main|compensation, step_index, k,
  outcome, cut_short, predicted, firmware_s, measured}] (vision/drive_timeline reads it instead of `plan.legs`),
  `requests` [{kind, target, measured, error, motion, legs, predicted, outcome}], `measured_pose`, `vo_scale`, `vo_turn_scale`;
  events `leg_acked {leg}` (the K of executed leg N), `leg_measured {leg, dx, dy, dtheta, pairs, weak, frames}`.

## demo_drive options and run.json additions (2026-09-30, task realroom-chat-drive)
- `--plan-json FILE` = `demo_drive.plan_json(Prepared)`: `{command, provider, raw, goal, goal_text, map, from_pose,
  target, steps [{action, amount, stated}] | legs [[ROTATE | FORWARD, value]]}`; no parser, no question. Steps must be inside
  the validator's limits; a route (legs) needs `from_pose {map_id, x, y, theta}` and is refused (exit 2) when
  latest.json's map id or pose differs by > 1e-3.
- `--map`, or "lập bản đồ / quét phòng / quét xung quanh / scan the room" in the command (`cmd_parser.split_map_request`
  → (bool, rest)): alone = `cmd_parser.MAP_SCAN` (left 90, right 180, left 90; provider `map scan`); after the run
  (complete, or `--compensate` with `measured_pose`; never `--dry-run`): `vision/drive_map.py --run <run> --publish`
  (log `map_build.log`, timeout 600 s). run.json: `build_map` (bool); events `map_published {map_id}`,
  `map_failed {exit_code, reason}`, `map_skipped {reason}`; status lines `building map`, `map published`, `map failed`.
  Exit 1 when the map failed.
- `--run-dir DIR` (default `vision/output/<YYYYmmdd_HHMMSS>`).
- A real run holds `fcntl.flock` on `vision/output/.robot.lock`; a second demo_drive exits 2 without sending anything.
  The route's `from_pose` is checked while holding it.
- SIGINT: a KeyboardInterrupt (abort / cancel) until the drive returns; from then until run.json is saved it is only
  noted (`run.json stop_requested: true`, result unchanged, no map built - `map_skipped {reason: "stopped by the
  user"}`); during the map build it stops drive_map (`map_failed {reason: "stopped"}`).
- `demo_drive.prepare(parser, text, speed, yaw_rate, wants_map, ask, say) -> Prepared(result, plan, text, goal,
  wants_map, from_pose, target) | None`: the terminal's and the page's shared path (ask(question) -> answer, '' = gave
  up). `target` (2026-09-30 chat-ui; default None, steps) = `{map_id, spot_id, object_id, name (goto.describe:
  "cạnh sau của ghế xanh lá (obj_16)"), x, y, theta}` where a route ends.
- LLM (2026-09-30 chat-ui): CM `config.GEMINI_MODEL` default `gemini-3.8-flash` (2.5 models answer 404 to new API
  keys). `cmd_parser` wraps a Gemini client in `QuotaFallbackClient`: HTTP 429 / 503 → `CM_GEMINI_FALLBACK_MODEL`
  (default `drive_config.GEMINI_FALLBACK_MODEL` `gemini-3.5-flash-lite`, sent `GEMINI_FALLBACK_EXTRA_BODY` {} - it
  refuses thinking_budget) for `QUOTA_COOLDOWN_S` 600 s per process; primary `max_retries` 0. `CommandParser.model`
  is now a property (the model answering now); `CommandParser.llm_failure` = why the rules read the last command.
- `goto.ground(scene, words, llm, ask, say=None) -> (spots best first, None) | (None, why)` (was one spot): the LLM
  (CM GroundingSession on `goto.realroom_prompt()` - CM's prompt with the side / corner rules swapped and Vietnamese
  examples; CM's file unchanged) only settles which object; once `slots.target` is set the robot picks
  (`order_spots`: center first, nearest first). `goto.route_first(scene, spots) -> (spot, RoomPlan, tried)`;
  `demo_drive.plan_to_spots(scene, spots, speed, yaw_rate, say)` -> (plan, what, from_pose, target) | None.
  `demo_drive._map_update` (--compensate): end_pose.predicted only when a request was predicted or ended in
  `leg_executor.UNMEASURED_ENDS` (robot_link failed / aborted) - no longer for every run that stopped early.
- `goto` (cont.): say(note) when the LLM's grounding failed and the rules picked; the
  rules filter by a color after the class ("ghế xanh", "ghế màu xanh dương", "green chair"; `COLOR_WORDS`, "xanh" =
  green | blue). `goto.describe(scene, spot)`, `goto.object_name(obj)`, `goto.short_error(text)`. Rule goals also read
  "di chuyển tới …", "tới cạnh …" (sentence start), "đến bên trái / phải / trước / sau <object>".

## Slip and VO calibration files
- `communication/slip_profiles.json`: `{floor: {forward|backward|left|right: [sample]}}`, sample `{t, commanded,
  measured, ratio, learned, why, run, leg, pairs, speed}`; k = Σ|measured| / Σ|commanded| of the last 10 learned.
- `vision/vo_scale.json`: `{mount, legs [{run, command_m, tape_m, vo_m}], turns [{run, how, truth_deg, vo_deg}]}` →
  `frame_motion.vo_scale(mount)` = Σ tape / Σ VO over `legs`, `frame_motion.vo_turn_scale(mount)` = Σ truth / Σ VO
  over `turns` (added 2026-09-29: the floor VO reads turns ~10.8 % long → 0.902), for that mount only (1.0 after a
  recalibration, or with no `turns`). Applied by drive_map (poses, leg measures; `--vo-turn-scale`), leg_executor
  (`measure_leg(..., turn_scale)`) and map_check; run.json and map_meta.json record `vo_turn_scale`.
