# Simulation module plan

## Purpose
Browser-side tools for the robot: 2D room simulator (Phase 1 L1 stand-in), later path planner + ZMQ bridge.

## Active
- [ ] Room simulator `simulation/room/` — task `agent/tasks/2026-09-11_room-simulator.md`, status testing
  (code complete, 500-seed invariant test passes, both review skills run; awaiting user confirmation).

## Next (not started)
- [ ] Noise mode (`?noise=1`): jitter vertices, drop low-confidence objects, to test the L2 validator.
- [ ] Occupancy grid export from the scene (for L4 A*).
- [ ] Feed scene JSON into L2 grounding prototype (`agent/description/robot_process_llm.md`).
- [ ] Decide whether `room.width/length` stays in the JSON (not camera-observable; recomputable from walls).
- [ ] Re-attach movement/ZMQ bridge (interfaces.md ports 5555/5556) once motivation is rebuilt.

## Done
(none)
