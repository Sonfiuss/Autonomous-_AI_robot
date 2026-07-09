---
id: 2026-07-09_walk-scale-fit
status: testing
module: deepmap
started: 2026-07-09
---

## Task
Hardware walk (2026-07-08 23:54, 3 stops) produced 3 DISJOINT patches instead
of one merged map. Diagnosis (logged in 2026-07-04_pipeline-walk-speed):
per-frame affine fit d_mono = s·disp + t collapses each frame to a different
compressed scale (s=0.095/0.063/0.061; wall at ~2 m lands at 0.7 m; fitted
cam_h 0.02 m vs real ~0.32 m) because RANSAC inliers cluster on the near
granite floor; golden set also contains garbage (disp<0 → Z=-46 m, Z=66 m).
Overlap between stop clouds 0.0–4.8 % → ICP has nothing to align.

Fix (user approved "All three fixes" via question, 2026-07-09):
1. Open-scene rerun protocol (user action, final test step).
2. Fit sanitize + span guard: drop disp < min_disp BEFORE RANSAC; if inlier
   disparity span is too narrow and no anchors are available, raise the
   expected-failure RuntimeError so the stop is SKIPPED, not poisoned.
3. Floor-anchor scale: known camera height (0.32 m, explore_map default) +
   fixed mount tilt (~13°) turn floor rows into synthetic far/near anchor
   correspondences fed to the existing refit_with_anchors — pins (s, t) even
   when texture is floor-only.

## Input
- depth-anything/src/stereo_ruler.py (fuse_metric, fit_affine_ransac,
  refit_with_anchors — anchors machinery exists, walk never used it)
- deepmap/stereo_walk_map.py (FusionMapper + sequential path + CLI)
- Bad hardware frames saved in deepmap/output/stereo_walk/ (validation data)
- cam height 0.32 m / tilt 13–15° from explore_map defaults + task notes

## Expected output
- Bad frames either fuse at plausible scale (wall ~1.5–2 m, cam_h ≈ 0.32) with
  anchors ON, or are SKIPPED (span guard) with anchors OFF — never a silently
  compressed cloud.
- Offline --test still passes (new baseline numbers recorded).
- User re-runs the walk in an open scene; stops overlap and merge.

## Plan
- [x] 1. stereo_ruler.fuse_metric: exclude disp < min_disp from the fit set
      (garbage negative/far matches no longer feed RANSAC).
- [x] 2. stereo_ruler.floor_anchors(mono, valid, calib, cam_h, tilt_deg):
      rows between Z≈0.3 m and z_max≈2.5 m below the horizon; per row take a
      low percentile of valid mono in the central column band (floor = far);
      monotonic-with-v filter; Z(v) = cam_h / (dy·cos t + sin t), anchor
      disp = fb/Z(v). ≥4 rows or no anchors.
- [x] 3. fuse_metric: after RANSAC, if cam_h given → refit_with_anchors with
      floor anchors; else enforce min_span_ratio (default 2.0) on inlier
      disparities → RuntimeError (skip tier) when too narrow. Guard s>0.
      info gains 'span', 'n_anchors', 'tilt_deg'. compute_metric_depth
      passes kwargs. PLUS (found during validation): two-pass gating —
      anchors-only line first, drop golden points inconsistent with it
      (aliased NCC matches), THEN weighted refit (single blended LS gave a
      Simpson-paradox slope flatter than either cluster).
- [x] 3b. estimate_tilt (NEW, found during validation): camera sits on the
      TILT SERVO, so mount tilt is run-dependent (this run: ~28°, not 13°!).
      Per frame, bottom-center golden points (stereo = metric) + measured
      cam_h solve t = asin(cam_h/R) − atan2(Z·dy, Z); median ≥3 pts, else
      fall back to --cam-tilt-deg. SGBM cross-check: (H,t) ridge-degenerate
      on flat bottom band — H must come from a MEASUREMENT, tilt per frame.
- [x] 4. stereo_walk_map CLI: --cam-height-m (0.32), --cam-tilt-deg (13.0,
      now fallback-only), --no-floor-anchor; wired through FusionMapper AND
      sequential path; per-stop line prints span/anch/tilt.
- [x] 5. Validate on the SAVED bad frames: anchors OFF → all 3 stops
      correctly SKIPPED (spans x1.37/x1.68/x1.48 < x2.0). Anchors ON →
      tilt est 27–30°, Z ranges sane, overlap 0.2%/4.8% → 20–36%. Residual
      per-frame variance (cloud cam_h 0.05–0.10) = anchor contamination from
      the dark under-chair cavity — pathological scene; open-scene rerun is
      the deciding test. Golden-floor plane fit failed (too few/collinear).
- [x] 6. Offline --test regression: PASS both modes, identical outputs.
      NEW baseline: golden=39 (2 garbage sanitized from 41), inl=2 (gate),
      anch=11, Z[0.35,1.70] (was absurd [0.06,121]), pts=184853/stop.
- [ ] 7. Hardware rerun by user: MEASURE camera height first (floor → lens
      center, update --cam-height-m), open scene (2–3 m clear, textured
      objects at several depths), same A/B commands + --profile.

## Execution log
[00:20] steps 1-4: sanitize + span guard + floor_anchors + CLI wiring | risk: anchors assume central-band floor visibility | info: refit_with_anchors machinery already existed, never used by the walk
[00:26] step 5 iter 1: overlap 24/28% but cloud cam_h 0.13-0.22 — debug showed golden cluster vs anchor line INCONSISTENT (aliased stereo matches: pt with Z_stereo=0.60 has mono of a 1.2 m point); blended LS slope flatter than either cluster | info: added two-pass gating
[00:28] step 5 iter 2: SGBM floor fit revealed tilt ~28-30° NOT 13° — tilt servo position is run-dependent; (H,t) degenerate on flat bottom band | info: added estimate_tilt per frame; H stays a measured input
[00:31] step 6: offline --test PASS pipeline+sequential, new baseline recorded | risk: none | info: test-pair inl=2 after gating (its fit was always weak 8/41); Z range now sane

## Test result
<!-- pending -->
<!-- NOTE 2026-07-10: baseline ở step 6 đã outdated — task
2026-07-09_visual-odometry-align thêm golden-veto cho floor_anchors +
trust horizon vào fuse_metric; baseline --test mới: golden=39, inl=3,
rms=0.0637, anch=5, Z[0.38,1.42]. Bước 7 (hardware rerun) không đổi. -->
