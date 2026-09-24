# MV — Path planning

Plans a collision-free route for the robot across a rectangular room with polygon obstacles.
It rasterises the room into an occupancy grid, runs A*, smooths the result to a few straight
legs, and emits motion primitives that MC can execute.

- Namespace `mv`. World frame, metres, radians, θ counter-clockwise from +x.
- Heap-free: fixed-size static buffers, no exceptions. Constants live in `config/constants.h` (`mv::cfg`).
- Uses no RM code. The only link is the `rm::cfg::PI` / `TWO_PI` constants from `config/constants.h`, used for angle wrapping.
- Not re-entrant: one planner instance per process (the A* buffers are static inside the API).
- CM (Python) calls MV through the C API using `project/src/CM/mv_client.py` (ctypes).

```
room + polygons ──OccupancyGrid──► AStar ──► cell path ──smoothPath──► waypoints ──toPrimitives──► primitives ──► MC
```

## Headers

| Header | Provides |
|---|---|
| [types.h](types.h) | `Point`, `Pose`, `Cell` |
| [occupancy_grid.h](occupancy_grid.h) | `OccupancyGrid`: inflated obstacle grid, snapping, line of sight |
| [astar.h](astar.h) | `AStar`: 8-connected grid search |
| [path.h](path.h) | `PrimitiveType`, `Primitive`, `smoothPath`, `toPrimitives`, `pathLength`, `wrapAngle` |
| [mv_api.h](mv_api.h) | The plain C API for foreign callers: `mv_plan`, `mv_grid` |
| [mv_debug.h](mv_debug.h) | `MV_DLOG(...)` debug gate |

## Features

### types.h
| Type | Fields |
|---|---|
| `Point` | `x`, `y` (m) |
| `Pose` | `x`, `y` (m), `theta` (rad) |
| `Cell` | `col` along +x, `row` along +y (row 0 at y = 0) |

### occupancy_grid.h — `OccupancyGrid`
- Rasterises a `roomW × roomL` room into cells of `cfg::GRID_RES_M` = 0.05 m. The maximum grid
  is 240 × 240 cells (12 m × 12 m, about 58 KB, one byte per cell).
- The robot is treated as a point. The room walls and every obstacle polygon are inflated by
  the robot hull radius plus `cfg::SAFETY_MARGIN_M` (0.03 m). A cell is blocked when its centre
  is inside a polygon or within the inflation distance of an edge (an exact distance test, not
  a kernel approximation).
- `reset(roomW, roomL, inflateM)` clears the grid and blocks the wall band. It returns false and
  leaves the grid empty if the room exceeds the maximum size.
- `addPolygon(pts, n)` blocks every cell within the inflation distance of a closed polygon (n ≥ 3).
- `toCell` / `toWorld` convert between world points and cell centres. `inBounds`, `isFree`,
  `index`, `cols`, `rows`, `resolution`, `inflation`, and `data()` (row-major, 1 = blocked).
- `nearestFree(p, out)` finds the nearest free cell within `cfg::SNAP_RADIUS_M` (0.20 m). It is
  used to move a start or goal that sits inside an inflated cell.
- `lineOfSight(a, b)` is true when the segment crosses only free cells (an exact grid ray cast;
  a crossing through a cell vertex checks both adjacent cells).

### astar.h — `AStar`
- 8-connected search with an octile heuristic. Diagonal cost is √2 cells.
- A diagonal move is refused when either orthogonal neighbour is blocked, so the path never cuts an obstacle corner.
- The open list is a binary heap with decrease-key on static arrays (about 1 MB for the
  maximum grid). Keep one instance for the whole program.
- `search(grid, start, goal, out, maxLen)` writes the cell path (start first) and returns its
  length in cells. It returns `AStar::NO_PATH` (-1) if the goal is unreachable, or
  `AStar::TOO_LONG` (-2) if the path is longer than `maxLen`. Start and goal must be free cells.

### path.h — post-processing
- `smoothPath(grid, path, n, out, maxOut)`: greedy line-of-sight smoothing. It keeps the first
  and last points and drops every intermediate point still visible from the last kept one.
  Returns the waypoint count, or -1 if `out` is too small.
- `toPrimitives(waypoints, n, startTheta, goalTheta, holonomic, out, maxOut)`: converts
  waypoints into motion primitives. Returns the count, or -1 if `out` is too small.
  - Turn-go-turn (`holonomic = false`): `ROTATE` to each segment heading, `FORWARD` its length,
    a final `ROTATE` to `goalTheta`, then `STOP`.
  - Holonomic (`holonomic = true`): one `MOVE dx dy` per segment, a final `ROTATE`, then `STOP`.
    The omni chassis drives a leg directly, so no turning is needed between legs.
- Primitive codes are shared with MC as raw ints:

| `PrimitiveType` | Code | `a` | `b` |
|---|---|---|---|
| `ROTATE` | 0 | angle in rad (CCW positive) | unused |
| `FORWARD` | 1 | distance in m along the current heading | unused |
| `MOVE` | 2 | dx in m (world frame) | dy in m (world frame) |
| `STOP` | 3 | unused | unused |

- Legs shorter than `cfg::MIN_SEGMENT_M` (1 mm) emit no `FORWARD`/`MOVE`. Turns smaller than
  `cfg::MIN_ROTATE_RAD` (1e-3) emit no `ROTATE`.
- `pathLength(pts, n)` is the polyline length. `wrapAngle` wraps to (-π, π].

### mv_api.h — C API
The only header a foreign caller (Python ctypes, C, firmware) needs. Plain C structs, and the caller owns every buffer.

- `mv_plan(const MvRequest*, MvResult*) -> int` plans start → goal. Outputs are valid only on `MV_OK`.
  - `MvRequest`: `room_w`, `room_l`, `robot_radius` (hull radius; the safety margin is added
    internally), polygons as a flat `vertices` array plus `poly_sizes` and `n_polys`, `start` and
    `goal` (`MvPose`), and `holonomic` (0 = ROTATE/FORWARD, 1 = MOVE).
  - `MvResult`: caller buffers `path`, `waypoints` and `prims`, each with a capacity and a
    length. It also returns `length_m` (waypoint polyline length) and `snapped_start` /
    `snapped_goal`, the free cell centres actually searched from. A partial path is never returned.
- `mv_grid(req, out, cap, &cols, &rows, &res_m)` writes the inflated grid (row-major, 1 = blocked,
  row 0 at y = 0) for drawing or debugging.
- `mv_status_text(status)` and `mv_version()`.

| `MvStatus` | Code | Meaning |
|---|---|---|
| `MV_OK` | 0 | success |
| `MV_ERR_ARGS` | 1 | null pointer, polygon with fewer than 3 vertices, capacity ≤ 0 |
| `MV_ERR_ROOM_TOO_BIG` | 2 | room exceeds the compiled grid size |
| `MV_ERR_START_BLOCKED` | 3 | no free cell within the snap radius of start |
| `MV_ERR_GOAL_BLOCKED` | 4 | no free cell within the snap radius of goal |
| `MV_ERR_NO_PATH` | 5 | start and goal are in different free regions |
| `MV_ERR_BUFFER` | 6 | an output buffer is too small |

### mv_debug.h
`MV_DLOG(...)` maps to `fprintf(stderr, ...)` when compiled with `-DMV_DEBUG` and to nothing otherwise.

## Example
```c
MvPoint  verts[] = {{1,1},{2,1},{2,2},{1,2}};           /* one square obstacle */
int      sizes[] = {4};
MvRequest req = {5.0f, 4.0f, 0.21f, verts, sizes, 1,
                 {0.5f, 0.5f, 0.0f}, {4.5f, 3.5f, 1.57f}, /*holonomic=*/1};
MvPoint path[4096], wp[256]; MvPrimitive prims[256];
MvResult res = {path, 4096, 0, wp, 256, 0, prims, 256, 0};
if (mv_plan(&req, &res) == MV_OK) { /* res.prims[0..prim_len) goes to MC */ }
```
