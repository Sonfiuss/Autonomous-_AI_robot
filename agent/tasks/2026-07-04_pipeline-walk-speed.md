---
id: 2026-07-04_pipeline-walk-speed
status: planning
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
- [ ] 1. `deepmap/pipeline_bus.py` — minimal generic infra (future voice/detect
      reuse it). No external deps.
  - [ ] 1.1 `Msg` dataclass: topic, idx, payload (dict), err (str|None).
  - [ ] 1.2 `Bus`: `subscribe(topic) -> queue.Queue(maxsize=2)` (one queue PER
        subscriber = fan-out); `publish(msg)` puts to every subscriber queue
        with timeout + stop_event check so a full queue can't deadlock
        shutdown.
  - [ ] 1.3 `Worker(Thread)` base: daemon=False, in-queue, shared stop_event;
        run() = get(timeout) loop → `handle(msg)`; `except BaseException` →
        log traceback + stop_event.set() (Exception alone misses SystemExit
        from stereo_ruler's sys.exit paths → dead worker, Coordinator hangs).
  - [ ] 1.4 Shutdown: None as poison pill on every queue + `join_all(timeout)`;
        worker exits on pill OR stop_event.
  - [ ] 1.5 `__main__` smoke test: 2 chained workers pass 5 msgs; injected
        raise in one → stop_event set, join_all returns.
- [ ] 2. CaptureWorker — persistent cameras (kills the 3–5 s open/warmup per
      stop).
  - [ ] 2.1 `PersistentStereoCam` helper class (usable WITHOUT the bus — step
        6.3 sequential mode reuses it): open both devices once, set
        width/height + CAP_PROP_BUFFERSIZE=1; on open failure raise
        RuntimeError with sr.capture_pair's diagnostic text (list
        /dev/video*), NOT sys.exit.
  - [ ] 2.2 `grab_fresh()`: discard loop (~0.2 s of grab() on both cams) to
        drop frames buffered DURING the move, then read() both; raise on
        failed read. `close()` releases both.
  - [ ] 2.3 CaptureWorker(Worker): consumes "capture_req"(idx, pose) →
        grab_fresh → publish "pair"(idx, left, right, pose) → save
        shot_N_left/right.jpg. Camera release in finally.
  - [ ] 2.4 Test double: FakeCaptureWorker returns the saved --test pair with
        the same message interface (no cv2.VideoCapture).
- [ ] 3. Split `stereo_ruler.compute_metric_depth` into halves + fuse (no
      logic duplicated; pipeline AND sequential path call the same pieces).
  - [ ] 3.1 `stereo_half(left_bgr, right_bgr, calib)` → (left_rect,
        right_rect, golden list): the remap + gray + golden_points lines
        currently at the top of compute_metric_depth.
  - [ ] 3.2 `depth_half(right_bgr, calib, model, input_size)` → (mono, valid):
        infer_image on the RAW right + remap mono AND valid via calib.map_r
        (fusion samples mono at rectified golden coords, so the remap lives
        here, with the model, in DepthWorker's thread).
  - [ ] 3.3 `fuse_metric(golden, mono, valid, calib, min_disp)` → (Z, valid,
        info): golden sampling, <10-pts RuntimeError, fit_affine_ransac,
        Z = fb/disp — the rest of today's function.
  - [ ] 3.4 Rewire compute_metric_depth = stereo_half + depth_half +
        fuse_metric. Regression: `stereo_walk_map.py --test` output identical
        to before the split (same pts count, same s/t/rms).
  - [ ] 3.5 StereoWorker(Worker): "pair" → stereo_half → publish
        "golden"(idx, golden, right_rect, pose). If len(golden) < 10 publish
        with err set (expected failure, NOT a crash — see rules).
  - [ ] 3.6 DepthWorker(Worker): owns the DA-V2 model; "pair" → depth_half →
        publish "mono"(idx, mono, valid).
- [ ] 4. FusionMapper(Worker) — join + geometry + accumulate + save.
  - [ ] 4.1 Join buffer: dict idx → {golden?, mono?}; process only when both
        halves arrived (arrival order is NOT guaranteed — depth and stereo
        queues drain at different speeds).
  - [ ] 4.2 err on either half OR fuse_metric RuntimeError → log + SKIP the
        stop, walk continues (sequential mode aborts here; pipeline must not).
  - [ ] 4.3 Happy path: fuse_metric → save depth_N_m.npy → backproject →
        level_to_floor → to_world(pose from the pair msg) → accumulate
        all_pts/all_cols → per-stop print (same fields as today's stop line).
  - [ ] 4.4 Incremental PLY every K stops (--save-every, default 1) on the
        fusion thread's own clock — it re-voxelizes ALL points (grows with
        map size) and must never block the join loop.
  - [ ] 4.5 Final save (after all workers joined, in Coordinator's finally):
        icp_merge_clouds + voxel_dedup → walk_map.ply + bounds printout.
- [ ] 5. MotionWorker — single owner of the ESP32 port.
  - [ ] 5.1 Consumes "move"(cm); checks stop_event BEFORE executing (a fault
        elsewhere must never trigger a NEW move).
  - [ ] 5.2 Runs robot_drive.drive_forward(cm, port, hz) blocking in its own
        thread; sleep(settle) after; on False result publish "move_done" with
        err (Coordinator ends the walk, map still saved).
  - [ ] 5.3 Owns the dead-reckoned Pose: advance on success, publish
        "move_done"(pose snapshot). Coordinator passes that pose into the
        next capture_req.
  - [ ] 5.4 FakeMotionWorker for --test: sleep(0.5) + pose.advance, same
        messages, no robot_drive import.
- [ ] 6. Coordinator in stereo_walk_map.py.
  - [ ] 6.1 Wiring: build Bus, subscribe queues (pair→{depth, stereo},
        golden+mono→fusion, move→motion, move_done+capture-ack→coordinator),
        start workers, load calib/model ONCE and hand to the right worker.
  - [ ] 6.2 State machine [capture_req i → pair received → move → move_done →
        i+1]; move is issued as soon as the PAIR lands (not when fusion
        finishes) — that's the whole speedup; stop_event checked every wait
        (with timeout so a hang can't freeze the walk forever).
  - [ ] 6.3 `--sequential`: today's loop for A/B, BUT capture through the same
        PersistentStereoCam — otherwise the A/B conflates pipelining with
        persistent cameras (which alone remove 3–5 s/stop) and can't show
        whether the threads earned their complexity.
  - [ ] 6.4 `--profile`: each worker records per-idx ms; end-of-run table
        per-stage avg + wall vs sum-of-stages (exposes GIL contention from
        golden_points' small-cv2-call Python loop).
  - [ ] 6.5 Shutdown in finally: poison pills / stop_event, join_all(timeout),
        THEN final save (4.5) — partial map survives any fault.
- [ ] 7. Offline test (--test: FakeCapture + FakeMotion, no hardware).
  - [ ] 7.1 Baseline: --test --sequential → record final pts count + wall.
  - [ ] 7.2 --test --pipeline → final count within voxel tolerance of 7.1.
  - [ ] 7.3 Timing: with fake stage sleeps, wall ≈ N×max(stage) + ONE compute
        tail (the LAST stop's compute has no drive to overlap), not
        N×sum(stages).
  - [ ] 7.4 Inject a worker crash mid-walk → stop_event set, NO further
        "move" consumed, clean join, partial PLY written.
  - [ ] 7.5 Inject a <10-golden-points frame → that stop skipped, walk
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

## Test result
