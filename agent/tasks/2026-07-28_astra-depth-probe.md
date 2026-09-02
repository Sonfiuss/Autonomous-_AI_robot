---
id: 2026-07-28_astra-depth-probe
status: executing
module: depth-anything
started: 2026-07-28
---

## Task
Click-to-measure depth tool for Astra Pro: click a point on the live depth
colormap, label its depth on the image, save annotated PNG + JSON.

## Input
- Astra Pro depth via OpenNI2 (astra_cloud.py front-end, verified working).
- User choices: click on depth colormap only (no RGB window), manual save with
  `s` key, JSON output format.

## Expected output
`astra_probe.py` runs live; left-click marks P1, P2, ... with `Pn: <mm>` labels
(median 5x5, zeros ignored); SPACE freezes frame, `c` clears, `s` writes
`depth-anything/output/probe/probe_<ts>.png` + `.json` (points with
u, v, depth_mm, x/y/z_mm + intrinsics + timestamp), `q` quits.

## Plan
- [x] 1. Create this task file (intake)
- [ ] 2. Write depth-anything/src/astra_probe.py (reuse astra_cloud helpers)
- [ ] 3. Run /code-standards-review + /code-logic-review (manual, Windows)
- [ ] 4. Hand over test command, set status testing

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[23:20] step 2: astra_probe.py written, reuses astra_cloud helpers, py_compile OK
[23:25] standards-review: 2 violations fixed (label magic numbers -> constants, imwrite return checked), 0 deferred | info: module convention is print() not logging (0 logging usages), kept

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
