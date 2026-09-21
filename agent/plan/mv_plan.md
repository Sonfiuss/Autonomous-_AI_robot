# MV module plan (`project/src/MV`, C++)

## Purpose
Roadmap L4: path planning from the robot pose to a resolved goal pose (from CM) on an occupancy grid
built from the scene JSON. C++ library with a C API; CM (Python, ctypes) calls it to draw the path.

## Active
- [ ] User acceptance run of `python project/src/CM/app.py` (task `agent/tasks/2026-09-12_mv-astar.md`,
      status `testing`): ask for a spot, confirm the purple path appears from the robot to the green hull.

## Next (not started)
- [ ] Executor: feed primitives to the motion layer (RM / ESP32) and re-plan from odometry.
- [ ] Dynamic obstacles / replanning when the scene changes mid-run.
- [ ] Jetson build (`libmv.so`) and CM config for the library path (`tools/build_mv.sh` already picks
      the `.so` name on Linux; only the CM side has been exercised on Windows so far).
- [ ] Plan around the real footprint polygon instead of the circumscribed disc, or let CM drop spots the
      disc cannot reach, so CM and MV agree on reachability (4 of 1452 spots disagree today).

## Done
- [x] A* planner on an inflated occupancy grid + line-of-sight smoothing + ROTATE/FORWARD/MOVE/STOP
      primitives, C API (`mv_plan`, `mv_grid`), CLI, unit tests — all 8 `test_mv` cases pass.
- [x] CM integration: `mv_client.py` (ctypes), `POST /api/plan`, path drawing in the CM canvas,
      Python-side tests in `test_grounding.py`.
