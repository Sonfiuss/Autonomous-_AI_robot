---
id: 2026-06-05_example-task
status: done
module: motivation
started: 2026-06-05
---

## Task
Tune PoseController gains so robot reaches 1m goal within ±3cm tolerance.

## Input
- Current gains: kp_lin=0.5, kp_ang=1.0 (hardcoded in PoseController.hpp)
- Test area: 2m × 2m flat floor
- Robot tends to overshoot by ~8cm on forward moves

## Expected output
`./motivation --nav` reaches any goal ≤1m away within ±3cm, no oscillation at target.

## Plan
- [x] 1. Add configurable gain params to PoseController constructor
- [x] 2. Add --kp-lin / --kp-ang CLI flags to main.cpp
- [x] 3. Test kp_lin=0.3 on 1m forward move, measure overshoot
- [x] 4. Iterate gains until tolerance met, write final values to defaults

## Execution log
[10:15] step 1: added PoseController(float kp_lin, float kp_ang) overload | info: old default kp_lin=0.5 caused 8cm overshoot
[10:28] step 2: --kp-lin/--kp-ang wired to main.cpp argv parser
[10:45] step 3: kp_lin=0.3 → 4cm overshoot; kp_lin=0.25 → 2cm, acceptable | risk: oscillation at kp_lin<0.2
[11:02] step 4: defaults updated to kp_lin=0.25, kp_ang=0.8 | info: kp_ang>1.0 causes heading oscillation at goal

## Test result
PASS — 5 runs, max error 2.1cm. No oscillation. kp_lin=0.25 kp_ang=0.8 are the working values.
