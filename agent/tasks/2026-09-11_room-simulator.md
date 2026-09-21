---
id: 2026-09-11_room-simulator
status: testing
module: simulation
started: 2026-09-11
---

## Task
2D room simulator (Flask + Canvas) that randomly generates a room (walls, door, table, bed, chair, fan,
mini objects), renders it in the browser, and saves the scene as JSON in the roadmap scene-graph style.

## Input
- Folder: `simulation/room/` (simulation/ is empty locally).
- Roadmap `agent/description/robot_process_llm.md` §3.1: scene graph is the L1 → L2/L3 interface;
  Phase 1 L1 = 2D simulator output.
- User constraints (2026-09-11):
  - 2D only, no 3D / height fields for now.
  - Every object footprint = convex polygon, 3..6 vertices (rect = 4-gon, round = hexagon).
  - JSON must contain only what a depth camera + detector pipeline can produce later:
    anonymous `id` (obj_N), `class`, `confidence`, `color`, `center`, `yaw`, `polygon`,
    derived `free_sides`, `near`. Walls = line segments, doors = gaps in walls.
  - Units meters, world frame, origin bottom-left inner corner, x right, y up, yaw rad CCW.
- Robot shape (user drawing 2026-09-11): hexagon chassis 34 cm long, 24 cm front / 10 cm rear width,
  3 wheels 8x3 cm at 21 cm radius, angles 60/180/300 (body +x = forward = wide edge). Hull radius 0.225 m.
- Python: `D:\University\Python\Python310\python.exe`, Flask 3.1.3 installed.

## Expected output
- `python simulation/room/app.py` → http://localhost:5000 shows a random room; "New room" regenerates,
  "Download JSON" gives the scene; `simulation/room/scenes/latest.json` + `room_<seed>.json` written.
- `python simulation/room/room_generator.py --seed N` prints the same JSON (CLI, no Flask).
- 50 random seeds: all polygons inside walls, no object overlap, robot spawn free, door on a wall.

## Plan
- [x] 1. Create task file + `agent/plan/simulation_plan.md`.
- [x] 2. `room_generator.py`: room 4–7 m, 4 wall segments, 1 door gap (0.8–1.0 m) on random wall,
        bed against a wall, table free, 1–3 chairs near table, fan hexagon in a corner,
        3–6 mini objects (cup/bottle/ball/box/plant/book, 4–6 vertices). Rejection sampling
        (SAT polygon overlap), seed, `free_sides` (robot disc on 4 sides of oriented bbox vs
        walls/objects), `near` (2 nearest landmarks), robot inside door. CLI `--seed`, `--out`.
- [x] 3. `app.py` (Flask): `GET /`, `GET /api/room/new?seed=`, `GET /api/room/latest`,
        `GET /api/room/<seed>`; writes `scenes/`. `static/index.html`: Canvas, y-flip, grid 0.5 m,
        walls lines, door colored gap, polygons filled + class label, robot circle + heading,
        legend, seed box, New room / Download JSON buttons.
- [x] 4. Smoke test: generate 50 seeds, assert invariants in Expected output.
- [x] 5. Run /code-standards-review + /code-logic-review, write `simulation/room/README.md`
        (run + schema), update `agent/plan/simulation_plan.md`, write history file.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[23:12] step 2: room_generator.py written (polygons 3-6 vtx, SAT overlap, free_sides/near) | risk: chairs/minis could block last free side of bed/table -> added protected-object guard + chair fallback | info: use D:/University/Python/Python310/python.exe, plain 'python' is broken on this PC
[23:12] step 4: test_room_generator.py 500/500 seeds pass (inside walls, no overlap, robot free, door on wall)
[23:20] step 3: app.py + static/index.html done, endpoints new/latest/<seed> return 200, scenes/ written | info: page fetches /api/room/latest on load; click object -> JSON panel
[23:22] step 5a: README.md with schema v2.0 table written; offline matplotlib render of seed 7 looks correct
[23:30] standards-review: 6 violations fixed (magic 0.02/1e6/n=8/0.01/50 -> constants, app.py print -> logging, _load_scene corrupt-file guard), 0 deferred | info: index.html JS not covered by checklist, left as is
[23:36] logic-review: 0 mismatches, 1 optimization (door approach zone cached instead of rebuilt per placement try), 0 flagged | info: free_sides naming front/left/back/right = +x/+y/-x/-y matches roadmap L3 "right" = -y offset; seed 7 output byte-identical after all review edits
[23:37] step 5: task -> testing; awaiting user run of app.py
[23:50] step 6 (user change): robot disc -> hexagon chassis + 3 wheels from drawing; JSON robot.{chassis,wheels,footprint} in body frame; free_sides/spawn use real hull facing the object; renderer draws chassis/wheels/hull | info: image frame = body frame rotated 90 deg (image up = body -x, wheel 180); 500/500 seeds pass

## Test result
