---
id: 2026-09-21_include-module-docs
status: testing
module: motivation
started: 2026-09-21
---

## Task
Write one markdown file per module folder in `project/include/` (RM, MV, MC) describing each header's features.

## Input
User: "Create md file to description feature of each module on folders of the folder /project/include/, like CM, RM, MV."
`project/include/` holds only MC/, MV/, RM/ (17 headers). CM is Python in `project/src/CM/` (has its own README) and has no include folder.

## Expected output
`project/include/RM/README.md`, `project/include/MV/README.md`, `project/include/MC/README.md`.
Each: purpose, position in the CM→MV→MC→RM chain, per-header feature list (classes/functions, behaviour, key limits from constants.h), and how the module is called. Facts taken from the headers + config/constants.h only.

## Plan
- [x] 1. RM/README.md — types, OmniKinematics, VelocityProfile/TrapezoidalProfile, Odometry, DriverStepDir/StepAccumulator, rm_debug
- [x] 2. MV/README.md — types, OccupancyGrid, AStar, path (smooth/primitives), C API (mv_plan/mv_grid, status codes), mv_debug
- [x] 3. MC/README.md — types, speed_limit, Executor pipeline, C API (mc_run/mc_max_speed, McStep/status), mc_debug
- [x] 4. Cross-check numbers/names against headers + constants.h; note CM is out of scope (Python, src/CM/README.md)

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[17:30] step 1: wrote include/RM/README.md | risk: none | info: MAX_WHEEL_OMEGA = 9.817 rad/s from 20 kHz / 12800 steps
[17:31] step 2: wrote include/MV/README.md | risk: none | info: MV uses only rm::cfg::PI constants, no RM code
[17:32] step 3: wrote include/MC/README.md | risk: none | info: ceilings 0.623 fwd / 0.540 side / 2.571 spin confirmed by mc_max_speed
[17:35] step 4: compiled README examples vs src (g++ -std=c++14): mv_plan=0, mc_run=0, rm example ok | risk: none | info: MOVE(1,0.5)+ROTATE 1.57 -> 622 ticks, ends (1.00,0.50,1.57)

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
