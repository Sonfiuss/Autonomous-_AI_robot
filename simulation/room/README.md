# 2D Room Simulator (`simulation/room/`)

Randomly generates a room (walls, door, bed, table, chairs, fan, mini objects), renders it in the
browser, and saves the scene as JSON. The JSON is the Phase-1 stand-in for the perception layer
(L1) in `agent/description/robot_process_llm.md`: it only contains fields a depth camera + object
detector pipeline can produce later.

## Run

```bash
cd simulation/room
python app.py              # http://localhost:5000
python room_generator.py --seed 42            # print scene JSON (no Flask)
python room_generator.py --seed 42 --out x.json
python test_room_generator.py 500             # invariant check over 500 seeds
```

Web page: **New room** (random seed), **Load seed**, **Download JSON**, toggle labels /
free-side markers, click an object to see its JSON. Every generated scene is written to
`scenes/room_<seed>.json` and `scenes/latest.json`.

| Endpoint | Returns |
|---|---|
| `GET /api/room/new?seed=N` | new scene (random seed when omitted), saved |
| `GET /api/room/latest` | last generated scene |
| `GET /api/room/<seed>` | saved scene by seed (generated if missing) |

## Conventions

- Units meters, world frame, origin = bottom-left inner corner, x right, y up.
- `yaw` in radians, counter-clockwise from +x.
- Every object footprint is a **convex polygon with 3..6 vertices**, CCW order.
  Rectangles are 4-gons, round objects (fan, ball) are hexagons.
- Object ids are anonymous (`obj_N`), as a detector would produce. Semantics live in `class`,
  `color`, `near`.
- Robot shape from the user's drawing (2026-09-11): hexagonal chassis 34 cm long (24 cm wide at the
  front, 10 cm at the rear), three 8 × 3 cm wheels 21 cm from center at 60°/180°/300° (body frame,
  +x forward, matches `project_overview.md`). Collision hull = convex hull of chassis + wheels,
  bounding radius 0.225 m.

## Scene JSON v2.0

```json
{
  "version": "2.0", "seed": 42, "units": "m", "frame": "world",
  "source": "simulator", "timestamp": 1757577600.0,
  "room":  {"width": 5.92, "length": 4.08},
  "walls": [{"id": "wall_1", "name": "south", "p1": {"x": 0, "y": 0}, "p2": {"x": 5.92, "y": 0}}],
  "doors": [{"id": "door_1", "wall": "south", "p1": {"x": 2.2, "y": 0}, "p2": {"x": 3.0, "y": 0}, "width": 0.8}],
  "robot": {"x": 2.6, "y": 0.36, "theta": 1.571, "radius": 0.225,
            "chassis": [{"x": 0.145, "y": -0.122}, "... 6 vertices, body frame"],
            "wheels": [{"angle_deg": 60, "polygon": ["4 vertices, body frame"]}, "... x3"],
            "footprint": ["convex hull of chassis + wheels, body frame"]},
  "objects": [
    {
      "id": "obj_2", "class": "table", "confidence": 0.87, "color": "brown",
      "center": {"x": 2.8, "y": 2.0}, "yaw": 0.0,
      "polygon": [{"x": 2.2, "y": 1.6}, {"x": 3.4, "y": 1.6}, {"x": 3.4, "y": 2.4}, {"x": 2.2, "y": 2.4}],
      "free_sides": ["front", "left"],
      "near": ["obj_3", "door_1"]
    }
  ]
}
```

| Field | Meaning | Real-world source |
|---|---|---|
| `walls[]` | line segments `p1`→`p2` | wall extraction from SLAM / occupancy map |
| `doors[]` | gap in a wall, `wall` name + endpoints + `width` | gap detection / detector class "door" |
| `robot` | pose (world) + `chassis` / `wheels` / `footprint` polygons in the **body frame** + bounding `radius` | odometry / localization; shape is a fixed constant |
| `objects[].class` | open-vocabulary label | detector |
| `objects[].confidence` | 0..1 | detector |
| `objects[].color` | dominant color | point-cluster color |
| `objects[].center`, `yaw` | polygon centroid, heading of the object x axis | derived from polygon |
| `objects[].polygon` | 3..6 vertex convex footprint | floor projection of depth points → convex hull → simplify |
| `objects[].free_sides` | `front/left/back/right` (object frame, +x/+y/-x/-y) where a robot disc fits | derived (footprint test) |
| `objects[].near` | 2 nearest landmarks (object ids or `door_1`) | derived |

`room.width/length` is the only field a camera would not know directly. It is kept for the
renderer and can be recomputed from `walls[]`.

## Generation rules

- Room 4–7 m per side, one door 0.8–1.0 m on a random wall, robot spawned just inside the door,
  door approach zone kept free.
- Bed against a wall (not the door wall), table at least 1 m from walls, 1–3 chairs around the
  table, hexagon fan in a corner, 3–6 mini objects (cup, bottle, ball, box, plant, book).
- Rejection sampling with Separating-Axis overlap test. Bed and table always keep at least one
  free side so the robot can reach them.
- Same seed → same room.

## Files

```
room_generator.py       generator + geometry helpers + CLI
app.py                  Flask server
static/index.html       Canvas renderer
test_room_generator.py  invariant smoke test
scenes/                 generated JSON (git-ignored candidates)
```
