---
id: 2026-07-08_golden-points-nogil
status: planning
module: deepmap
started: 2026-07-08
---

## Task
`golden_points` (depth-anything/src/stereo_ruler.py:184) is the identified GIL
hot-spot in the stereo_walk_map pipeline (task 2026-07-04_pipeline-walk-speed):
a per-corner Python loop with up to 4 tiny `cv2.matchTemplate` calls each,
interleaved with pure-Python bytecode (dict bookkeeping, subpixel parabola,
LR-consistency check). Each small cv2 call releases the GIL only briefly, so
the Python bytecode between calls holds the GIL and contends with
FusionMapper's numpy work on another thread — this is the wall-vs-sum-of-stages
gap flagged in stereo_walk_map's --profile design.

Goal: reduce GIL contention from this function without changing its output
(same golden-point schema: x, y, xr, disp, ncc).

## Input
- `depth-anything/src/stereo_ruler.py`: `golden_points`, `_match_row`,
  `_bucket_corners`.
- Baseline: no numba/pybind11/cython currently in the repo (checked 2026-07-08).
- Blocked on step 8 of 2026-07-04_pipeline-walk-speed (Jetson hardware
  `--profile` run) — that run's wall-vs-sum-of-stages number is the evidence
  for whether this hot-spot is worth fixing at all.

## Expected output
Two candidate approaches, ranked; user will pick one (or B as fallback if A
fails) before implementation starts:

- **A — numba `@njit(nogil=True)` (recommended first try)**: rewrite the
  per-corner match loop in numba-JIT-able numpy (no cv2 calls inside the JIT
  region — matchTemplate/NCC reimplemented as manual mean/std correlation),
  releasing the GIL for the whole function body. Lowest effort, no new build
  toolchain, but needs numba/llvmlite verified working on Jetson aarch64.
- **B — pure numpy vectorization (fallback, no new dependency)**: replace the
  per-corner Python loop with `sliding_window_view`-based batched NCC across
  all candidate offsets at once. Higher implementation risk (must reproduce
  `cv2.TM_CCOEFF_NORMED` numerically) but zero new dependencies.

Definition of done for whichever is chosen:
1. Byte-for-byte (within float tolerance) identical golden-point list vs the
   current implementation on the existing test pair.
2. `--profile` on Jetson shows improved wall-vs-sum-of-stages overlap ratio
   for the stereo stage.

## Plan
- [ ] 1. Run step 8 of 2026-07-04_pipeline-walk-speed on the Jetson first —
      confirms whether golden_points is actually the bottleneck (wall ≈
      N×max(stage) already, or real GIL gap).
- [ ] 2. If gap confirmed: verify numba/llvmlite installs and runs on Jetson
      aarch64 (quick smoke test only, not the real port yet).
- [ ] 3a. (Path A) Rewrite match-loop math without cv2 calls, add
      `@njit(nogil=True, cache=True)`, keep `_bucket_corners` untouched
      (runs once per frame, not a hot loop).
- [ ] 3b. (Path B, only if 2 fails or numba underperforms) Vectorize NCC via
      `sliding_window_view` across all candidate disparities per corner.
- [ ] 4. Regression test: old vs new golden_points on the same stored test
      pair, compare every field within float tolerance.
- [ ] 5. Re-run `--profile` on Jetson, compare stereo-stage overlap before/after.

## Execution log
[model-switch] plan captured, no code changes yet — user will resume testing later

## Test result
<!-- pending -->
