# Live Mapping — Timing Reference (Jetson)

Reference for every latency stage in the live 360° mapping run (camera feed
direct, **no** `--test-round360`). Use it to know *what* to measure, *where* it
happens in the pipeline, and *which* numbers to collect for fine-tuning.

All times in **milliseconds**. GPU stages must be measured with a CUDA-synced
timer (`depth_to_3d_timed.Timer`) or numbers will be wrong (async kernels).

---

## Pipeline order (one full run)

```
[STARTUP once]                     model_load
        │
[CALIBRATION once, plan A4]        scale_capture → stepmotor_delay → move_30cm
        │                          → scale_compute   (recover metric k)
        │
[PER ROTATION STEP × 12] ─────────────────────────────────────────────┐
        │  capture_latency → prep → infer → project → merge            │
        │  → movable_space → stepmotor_delay → move_30deg              │
        └──────────────────────── repeat ────────────────────────────┘
        │
[FINALIZE once]                    export (PLY / occupancy grid)
```

---

## Stage table

| # | Stage (key) | What it measures — start → stop | Phase | Units | Source / where to instrument | Status |
|---|-------------|--------------------------------|-------|-------|------------------------------|--------|
| 0 | `model_load` | DA-V2 checkpoint load + `.to(device)` + warmup pass | startup ×1 | ms | `build_map.load_model` (already `[load model]`) | ✅ logged |
| 1 | `capture_latency` | grab signal issued → decoded frame in hand (camera pipeline lag) | per step | ms | wrap `cap.read()`; flush stale buffers first | ✅ `capture_ms` in `timing_log.csv` (incl. imwrite) |
| 2 | `prep` | `image2tensor`: resize 518 + normalize + to device | per step | ms | `depth_to_3d_timed.step_prep` | ✅ timed in merge_360/timed |
| 3 | `infer` | `model.forward` + interpolate to original H×W | per step | ms | `step_infer` (CUDA-sync!) | ✅ timed |
| 4 | `project` | back-project depth+color → 3D point cloud (camera frame) | per step | ms | `scale_calib.project_metric` / `step_project` | ✅ timed |
| 5 | `merge` | add frame cloud to global map (transform + append/voxel) | per step | ms | `build_map` (`merge_ms` col) | ✅ `timing_map.csv` |
| 6 | `movable_space` | depth → drive-area polygon, then LABEL every projected point drivable(floor)/obstacle and colour the map (green/red) | per step | ms | `drive_area.compute_drive_polygon` + `scale_calib.drivable_labels`/`colorize_by_class` | ✅ `timing_map.csv` (`--no-drive-area` skips; `--no-class-color` keeps raw RGB) |
| 7 | `stepmotor_delay` | `T`/`F` command sent → motion actually starts | per move | ms | needs ESP32 ack-on-start | ⏸️ IGNORED (per decision; firmware can't signal motion-start) |
| 8 | `move_30deg` | rotate command sent → `K` ack (full 30° physical move) | per step | ms | `rotate_scan` wait_ack | ✅ `move_ms` in `timing_log.csv` |
| C1| `scale_capture` | capture D1 + D2 relative-depth frames | calib ×1 | ms | `scale_calib.recover_scale_motion_parallax(timing=…)` | ✅ `calib_log.csv` |
| C2| `move_30cm` | `F` forward command sent → `K` ack (30 cm move) | calib ×1 | ms | same fn | ✅ `calib_log.csv` |
| C3| `scale_compute` | ORB detect+match + k_i median (metric scale recovery) | calib ×1 | ms | same fn | ✅ `calib_log.csv` |
| 9 | `export` | write PLY (+ occupancy grid) to disk | finalize ×1 | ms | `build_map` (`export_ms`) / `merge_360.write_ply` | ✅ `timing_map.csv` footer |

Legend: ✅ produced · ⏸️ intentionally skipped for now.

## Output files
- `timing_map.csv` (next to `--out`): per-frame `prep/infer/project/merge/movable_space_ms` + `points`,
  with `AVG`, `load_model_ms`, `export_ms` footer rows. Written by `build_map.py`.
- `timing_log.csv` (in shots dir): per-step `capture_ms`, `move_ms`, expected-vs-odom rotation.
  Written by `rotate_scan.py`.
- `calib_log.csv`: one-shot `model_load/scale_capture/move_30cm/scale_compute`. Pass a `timing`
  dict to `recover_scale_motion_parallax` then `scale_calib.write_calib_log(...)`.

---

## Notes per stage

- **`capture_latency` (1):** current `capture_ms` bundles `cap.read()` **and** `cv2.imwrite`.
  For pure camera latency, time only `cap.read()` after flushing 2–3 stale V4L2 buffers,
  and log imwrite separately. On the live run you may skip imwrite entirely (feed straight
  to the model) — then `capture_latency` ≈ pure sensor+USB+decode lag.
- **`infer` (3):** dominated by first-call JIT (~3 s) unless a warmup pass ran at load —
  see `agent/history/2026-06-28_depth-anything.md`. Always warm up, then measure.
- **`stepmotor_delay` (7): IGNORED for now (per decision).** It cannot be measured today —
  firmware sends `K` only at motion *completion*, and odometry is integrated from commanded
  steps (no encoder/IMU), so there is no "motion began" signal. If needed later, add an ESP32
  ack the moment `move()` is issued (e.g. `B\n`), then `stepmotor_delay = t(B) − t(command)`.
  For now its cost is folded into `move_30deg` / `move_30cm`.
- **`move_30deg` (8) & `move_30cm` (C2):** these are physical-motion durations, set by
  `omega`/`spd` and the accel ramp (`MAX_ACCEL_STEPS`), not by compute. Logging them lets
  you separate compute budget from motion budget.
- **`movable_space` (6):** lives in `drive_area.py`; confirm whether it runs per-frame or
  once on the merged cloud — that changes whether it's a per-step or finalize cost.

---

## Derived stats (compute afterward)
- **compute/step** = prep + infer + project + merge + movable_space  (printed by `build_map`)
- **motion/step**  = move_30deg  (stepmotor_delay folded in)
- **seconds per 360° sweep** ≈ n_steps × (compute/step + motion/step) / 1000
