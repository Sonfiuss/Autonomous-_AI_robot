---
id: 2026-07-04_pipeline-walk-speed
status: testing
module: deepmap
started: 2026-07-04
---

## Task
stereo_walk_map.py is fully sequential per stop:
capture pair → stereo-scaled depth → cloud+merge → drive → (repeat).
Rebuild it as a MULTI-THREAD PIPELINE with one thread per stage (user-specified
architecture), so stages overlap and the walk is drive-bound, not compute-bound.
The design must accept FUTURE workers: voice control and object detection.

## Thread architecture (user requirement: one stage = one thread)
```
                 ┌────────────► DepthWorker (DA-V2, owns GPU model) ──"mono"──┐
CaptureWorker ───┤   "pair"                                                   ├─► FusionMapper
(owns cameras,   └────────────► StereoWorker (rectify+golden pts) ──"golden"──┘   (join by idx →
 persistent open)                                                                  RANSAC fit →
      ▲                                                                            Z=fB/d → cloud →
      │ "capture_req"                                                              merge+save)
Coordinator (main thread, walk state machine)
      │ "move"                       ▲ "move_done"(+pose)
      ▼                              │
MotionWorker (owns UART via core_control→stepper_ctrl, keeps dead-reckon Pose)

FUTURE (same bus, no redesign):
  VoiceWorker  ──"move"──►  MotionWorker   (single owner of the ESP32 port!)
  DetectWorker ◄──"pair"──  CaptureWorker  (subscribes to frames, publishes "objects")
```
Rules that make this safe/fast:
- ONE owner per hardware resource: cameras=CaptureWorker, UART=MotionWorker,
  GPU model=DepthWorker. Everything else is message passing (queue.Queue).
- Capture only while stationary: Coordinator requests capture strictly between
  "move_done" and the next "move". Pose is snapshotted into the pair message.
- DA-V2 (GPU, releases GIL) ∥ golden points ∥ driving (separate stepper_ctrl
  process). GIL caveat: golden_points is a Python loop of many SMALL cv2 calls
  (GIL released only inside each call), so StereoWorker contends with fusion
  numpy more than "releases GIL" suggests — not a blocker (DA on GPU, motion
  blocks on serial), but --profile must expose it (wall vs sum-of-stages).
- Failure policy, two tiers:
  * EXPECTED per-frame failure (compute_metric_depth RuntimeError, <10 golden
    points on a textureless scene): StereoWorker publishes "golden" with
    err set; FusionMapper SKIPS that stop, walk continues. (Sequential mode
    aborts here — pipeline mode deliberately does not; a mid-walk wall must
    not kill the map.)
  * WORKER CRASH (anything else): → shared stop_event. Worker base must catch
    BaseException, NOT Exception — stereo_ruler paths call sys.exit() →
    SystemExit, which except Exception misses → dead worker, Coordinator
    hangs forever. MotionWorker checks stop_event BEFORE every move (no NEW
    move after a fault; an in-flight ~3 s drive completes); partial map saved.
- Stale-frame rule: with cameras held open, V4L2 buffers ~4 frames captured
  DURING the move (blurry, wrong pose). CaptureWorker sets
  CAP_PROP_BUFFERSIZE=1 and discard-grabs (~0.2 s / N grabs) after each
  capture_req before keeping a frame. The offline test CANNOT catch this —
  hardware step verifies it.

## Plan
- [x] 1. `deepmap/pipeline_bus.py` — minimal generic infra (future voice/detect
      reuse it). No external deps.
  - [x] 1.1 `Msg` dataclass: topic, idx, payload (dict), err (str|None).
  - [x] 1.2 `Bus`: `subscribe(topic) -> queue.Queue(maxsize=2)` (one queue PER
        subscriber = fan-out); `publish(msg)` puts to every subscriber queue
        with timeout + stop_event check so a full queue can't deadlock
        shutdown.
  - [x] 1.3 `Worker(Thread)` base: daemon=False, in-queue, shared stop_event;
        run() = get(timeout) loop → `handle(msg)`; `except BaseException` →
        log traceback + stop_event.set() (Exception alone misses SystemExit
        from stereo_ruler's sys.exit paths → dead worker, Coordinator hangs).
  - [x] 1.4 Shutdown: None as poison pill on every queue + `join_all(timeout)`;
        worker exits on pill OR stop_event.
  - [x] 1.5 `__main__` smoke test: 2 chained workers pass 5 msgs; injected
        raise in one → stop_event set, join_all returns.
- [x] 2. CaptureWorker — persistent cameras (kills the 3–5 s open/warmup per
      stop).
  - [x] 2.1 `PersistentStereoCam` helper class (usable WITHOUT the bus — step
        6.3 sequential mode reuses it): open both devices once, set
        width/height + CAP_PROP_BUFFERSIZE=1; on open failure raise
        RuntimeError with sr.capture_pair's diagnostic text (list
        /dev/video*), NOT sys.exit.
  - [x] 2.2 `grab_fresh()`: discard loop (~0.2 s of grab() on both cams) to
        drop frames buffered DURING the move, then read() both; raise on
        failed read. `close()` releases both.
  - [x] 2.3 CaptureWorker(Worker): consumes "capture_req"(idx, pose) →
        grab_fresh → publish "pair"(idx, left, right, pose) → save
        shot_N_left/right.jpg. Camera release in finally.
  - [x] 2.4 Test double: FakeCaptureWorker returns the saved --test pair with
        the same message interface (no cv2.VideoCapture).
- [x] 3. Split `stereo_ruler.compute_metric_depth` into halves + fuse (no
      logic duplicated; pipeline AND sequential path call the same pieces).
  - [x] 3.1 `stereo_half(left_bgr, right_bgr, calib)` → (left_rect,
        right_rect, golden list): the remap + gray + golden_points lines
        currently at the top of compute_metric_depth.
  - [x] 3.2 `depth_half(right_bgr, calib, model, input_size)` → (mono, valid):
        infer_image on the RAW right + remap mono AND valid via calib.map_r
        (fusion samples mono at rectified golden coords, so the remap lives
        here, with the model, in DepthWorker's thread).
  - [x] 3.3 `fuse_metric(golden, mono, valid, calib, min_disp)` → (Z, valid,
        info): golden sampling, <10-pts RuntimeError, fit_affine_ransac,
        Z = fb/disp — the rest of today's function.
  - [x] 3.4 Rewire compute_metric_depth = stereo_half + depth_half +
        fuse_metric. Regression: `stereo_walk_map.py --test` output identical
        to before the split (same pts count, same s/t/rms).
  - [x] 3.5 StereoWorker(Worker): "pair" → stereo_half → publish
        "golden"(idx, golden, right_rect, pose). If len(golden) < 10 publish
        with err set (expected failure, NOT a crash — see rules).
  - [x] 3.6 DepthWorker(Worker): owns the DA-V2 model; "pair" → depth_half →
        publish "mono"(idx, mono, valid).
- [x] 4. FusionMapper(Worker) — join + geometry + accumulate + save.
  - [x] 4.1 Join buffer: dict idx → {golden?, mono?}; process only when both
        halves arrived (arrival order is NOT guaranteed — depth and stereo
        queues drain at different speeds).
  - [x] 4.2 err on either half OR fuse_metric RuntimeError → log + SKIP the
        stop, walk continues (sequential mode aborts here; pipeline must not).
  - [x] 4.3 Happy path: fuse_metric → save depth_N_m.npy → backproject →
        level_to_floor → to_world(pose from the pair msg) → accumulate
        all_pts/all_cols → per-stop print (same fields as today's stop line).
  - [x] 4.4 Incremental PLY every K stops (--save-every, default 1) on the
        fusion thread's own clock — it re-voxelizes ALL points (grows with
        map size) and must never block the join loop.
  - [x] 4.5 Final save (after all workers joined, in Coordinator's finally):
        icp_merge_clouds + voxel_dedup → walk_map.ply + bounds printout.
- [x] 5. MotionWorker — single owner of the ESP32 port.
  - [x] 5.1 Consumes "move"(cm); checks stop_event BEFORE executing (a fault
        elsewhere must never trigger a NEW move).
  - [x] 5.2 Runs robot_drive.drive_forward(cm, port, hz) blocking in its own
        thread; sleep(settle) after; on False result publish "move_done" with
        err (Coordinator ends the walk, map still saved).
  - [x] 5.3 Owns the dead-reckoned Pose: advance on success, publish
        "move_done"(pose snapshot). Coordinator passes that pose into the
        next capture_req.
  - [x] 5.4 FakeMotionWorker for --test: sleep(0.5) + pose.advance, same
        messages, no robot_drive import.
- [x] 6. Coordinator in stereo_walk_map.py.
  - [x] 6.1 Wiring: build Bus, subscribe queues (pair→{depth, stereo},
        golden+mono→fusion, move→motion, move_done+capture-ack→coordinator),
        start workers, load calib/model ONCE and hand to the right worker.
  - [x] 6.2 State machine [capture_req i → pair received → move → move_done →
        i+1]; move is issued as soon as the PAIR lands (not when fusion
        finishes) — that's the whole speedup; stop_event checked every wait
        (with timeout so a hang can't freeze the walk forever).
  - [x] 6.3 `--sequential`: today's loop for A/B, BUT capture through the same
        PersistentStereoCam — otherwise the A/B conflates pipelining with
        persistent cameras (which alone remove 3–5 s/stop) and can't show
        whether the threads earned their complexity.
  - [x] 6.4 `--profile`: each worker records per-idx ms; end-of-run table
        per-stage avg + wall vs sum-of-stages (exposes GIL contention from
        golden_points' small-cv2-call Python loop).
  - [x] 6.5 Shutdown in finally: poison pills / stop_event, join_all(timeout),
        THEN final save (4.5) — partial map survives any fault.
- [x] 7. Offline test (--test: FakeCapture + FakeMotion, no hardware).
  - [x] 7.1 Baseline: --test --sequential → record final pts count + wall.
  - [x] 7.2 --test --pipeline → final count within voxel tolerance of 7.1.
  - [x] 7.3 Timing: with fake stage sleeps, wall ≈ N×max(stage) + ONE compute
        tail (the LAST stop's compute has no drive to overlap), not
        N×sum(stages).
  - [x] 7.4 Inject a worker crash mid-walk → stop_event set, NO further
        "move" consumed, clean join, partial PLY written.
  - [x] 7.5 Inject a <10-golden-points frame → that stop skipped, walk
        CONTINUES to the end.
- [ ] 8. Hardware run — 3 stops × 30 cm.
  - [ ] 8.1 Stopwatch --pipeline vs --sequential (same scene); expect ~2–3×.
  - [ ] 8.2 Map quality: single (non-duplicated) objects, inter-cloud offsets
        ≈ 0.3 m — same acceptance as the sequential task.
  - [ ] 8.3 Captures sharp = stale-frame flush works (offline test CANNOT
        catch this).
  - [ ] 8.4 Two persistent 720p streams don't saturate the Jetson USB bus
        (open failures / dropped frames / lag during the walk).

## Expected result
Per-stop cycle: before ≈ capture 3–5 s + DA 1.2–4 s + stereo 0.3–0.5 s + drive
~3 s ≈ 8–12 s. After ≈ capture ~0.3 s + max(drive ~3 s, DA ~1.5 s, stereo
~0.4 s) ≈ 3–4 s → ~2–3× faster and drive-bound. Voice/detect threads later
plug into the bus without touching the walk logic.

## Out of scope (future)
- VoiceWorker (LLM/voice → "move" commands) and DetectWorker (object detection
  on "pair") — the bus + single-owner rules are designed for them, not built now.
- TensorRT/FP16 export of DA-V2 (2–4× inference) if still compute-bound.
- Binary PLY writer for big maps.

## Execution log
[19:40] step 1: deepmap/pipeline_bus.py — Msg/Bus/Worker/join_all + __main__ smoke test (5-msg chain PASS, injected SystemExit PASS) | risk: none | info: smoke run on Windows dev box, Python D:\University\Python\Python310 (torch CPU)
[19:55] step 3: stereo_ruler.py split — stereo_half/depth_half/fuse_metric extracted, compute_metric_depth rewired to call them (no logic duplicated) | risk: fuse_metric converts fit_affine_ransac's sys.exit (SystemExit) into RuntimeError so the pipeline's skip tier catches it; stereo_ruler main() unaffected (calls fit_affine_ransac directly) | info: per-stop regression identical pre/post split (golden=41 inl=8 rms=0.0888 pts=72696)
[20:05] steps 2+4+5+6: stereo_walk_map.py rebuilt — PersistentStereoCam (BUFFERSIZE=1 + grab-flush --discard-s), Capture/Stereo/Depth/FusionMapper/Motion workers + Fake doubles, Coordinator state machine (--wait-timeout guards every wait), --sequential A/B mode through the SAME persistent cameras, --profile table, --save-every, --inject-crash/--inject-badframe test hooks | risk: incremental PLY re-voxelizes all points on the fusion thread (use --save-every >1 on long walks) | info: coordinator subscribes to "pair" as its capture ack (no extra topic needed)
[20:20] step 7 bugfix: pipeline_bus.join_all — pill was ONE 5s put attempt; a busy DepthWorker with a full in-queue missed it and idled to the 180s deadline. Now the pill is re-offered while joining (extra pill harmless) | risk: none | info: shutdown went 181.2s -> 21.9s wall
[00:15] step 8.2 analysis (user ran hardware walk 23:54, map = 3 disjoint patches): pairwise cloud overlap 0.2%/4.8%/0.0% (<5cm neighbor), per-frame affine scale wildly different (s=0.095/0.063/0.061) → same objects land at different Z per stop → ICP has nothing to align. Root cause: RANSAC inliers (14–20 of 44–58) cluster on NEAR FLOOR (granite texture, golden Z median 0.5–0.6m); affine extrapolates to far scene → whole frame compressed to Z∈[0.34,0.93]m (wall at 2m lands at 0.7m; fitted cam_h=0.02m ≈ camera on the floor = impossible). Golden set also contains garbage (disp<0 → Z=-46m; Z=66m). Known failure mode — refit_with_anchors docstring describes exactly this near-field-cluster trap. Scene also hostile: robot drove INTO a chair (stops 1–2 dark close-range) | risk: 8.2 FAIL as-is | info: scripts in scratchpad analyze_walk.py/analyze_fit.py
[23:45] step 8 prep (user request "output time on each step"): per-stop lines now prefixed [HH:MM:SS] wall-clock (pipeline fused/skipped + sequential); --profile prints a PER-STEP ms table (rows=stop idx, cols=capture/stereo/depth/fusion/motion) before the avg summary — data was already in Worker.stage_ms, only display added | risk: none | info: verified on Jetson --test: pipeline 3 stops wall 9.1s (depth step0 6.6s = CUDA warmup, then ~1s), sequential 2 stops OK; regression values match baseline (golden=41 inl=8 rms=0.0888)
[20:30] step 7 tests (all offline, Windows CPU): 7.1 sequential 28931 pts / 26.6s; 7.2 pipeline 28867 pts (within voxel tolerance; floor-fit RANSAC jitters ±0.2%) / 21.9s; 7.3 wall 21.9s ≈ 3×max(depth 6.9s)+tail ✓, sum-of-stages 25.8s, overlap x1.18 (CPU depth dominates — Jetson GPU + real 3s drives overlap more); 7.4 crash@1 → stop_event, clean join, partial map 1 stop; 7.5 badframe@1 → skipped, walk continued, 2 stops fused | info: PYTHONIOENCODING=utf-8 needed on Windows console only (explore_map prints '→'); Jetson unaffected

## Test result
Offline (step 7): PASS — see execution log. Baseline pre-split --test output
reproduced exactly per stop; pipeline final counts within voxel tolerance.
Hardware (step 8): pending user run on the Jetson:
  python3 deepmap/stereo_walk_map.py --steps 3 --step-cm 30 --profile   # pipeline
  python3 deepmap/stereo_walk_map.py --steps 3 --step-cm 30 --sequential # A/B
Verify: (8.1) stopwatch pipeline vs sequential ≈ 2–3×; (8.2) single objects,
inter-cloud offsets ≈ 0.3 m; (8.3) captures sharp (stale-frame flush works);
(8.4) two persistent 720p streams don't saturate the USB bus.
