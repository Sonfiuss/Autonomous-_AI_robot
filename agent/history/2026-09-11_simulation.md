# 2026-09-11 — simulation module (room simulator)

## Implemented
- New `simulation/room/`: `room_generator.py` (seeded random room, SAT overlap rejection, derived
  `free_sides` / `near`, CLI), `app.py` (Flask, /api/room/new|latest|<seed>, writes `scenes/`),
  `static/index.html` (Canvas renderer, click-to-inspect JSON, download), `test_room_generator.py`
  (invariants over N seeds, 500/500 pass), `README.md` (schema v2.0 + field → real-world source table).

## Decisions (with user)
- JSON must contain only what a depth camera + detector can produce: anonymous `obj_N` ids, `class`,
  `confidence`, `color`, `center`, `yaw`, `polygon`; walls as line segments, doors as wall gaps.
  Rejected the first draft (hand-authored w/l + `on: table_1`) and the 3D bbox draft.
- 2D only. Every footprint is a convex polygon with 3..6 vertices (rect = 4, round = hexagon).
- Units m, world frame, origin bottom-left inner corner, yaw rad CCW. Side names front/left/back/right =
  +x/+y/-x/-y in the object frame, consistent with roadmap L3 ("right" = -y offset).
- Robot shape from user drawing: hexagon chassis (0.145 front / -0.195 rear along x, ±0.173 max width),
  3 wheels 8×3 cm at r=0.21, angles 60/180/300; collision = convex hull of chassis+wheels (r 0.225).
  JSON `robot.chassis/wheels/footprint` are body-frame constants, `robot.x/y/theta` world.
- Bed and table always keep >= 1 free side (protected-object guard during placement).
- Python on this PC: `D:\University\Python\Python310\python.exe`; bare `python` launcher is broken.

## Pending
- User to run `python simulation/room/app.py` and confirm (task status `testing`).
- `room.width/length` kept for the renderer although not camera-observable — decide later.
- Follow-ups listed in `agent/plan/simulation_plan.md` (noise mode, occupancy grid, L2 hookup).

## Interface changes
- New scene JSON v2.0 documented in `simulation/room/README.md`. Supersedes the roadmap §3.1 example
  shape (`w`/`l` fields) for simulator output; roadmap doc not yet updated.
