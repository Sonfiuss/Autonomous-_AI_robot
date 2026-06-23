---
id: 2026-06-21_depth-grid-rectify-fix
status: testing
module: stereo-camera
started: 2026-06-21
---

## Task
Fix wrong/sparse depth values in `tools/depth_grid.py` single-frame stereo depth
grid by adding uncalibrated rectification and retuning SGBM. (No live cameras /
checkerboard calibration this session — single saved frame only.)

## Input
- Tool: `stereo-camera/tools/depth_grid.py` (working, but ~60% cells invalid "?")
- Test pair: `stereo-camera/captures/left_20260610_001156.jpg` +
  `right_20260610_001156.jpg` (each 640x480)
- Physical baseline: 54 mm (fixed, known)
- Python: `D:/University/Python/Python310/python.exe` (has cv2 + numpy)
- Grid spec: 72 cols x 36 rows = 2592 depth points, labels in cm.

## Diagnosis (from this session — read before starting)
Ranked root causes of bad output:
1. NO RECTIFICATION — left/right rows not aligned; SGBM searches only horizontal
   epipolar lines, so it fails wherever the same object sits on different rows.
   This causes the ~60% "?" cells. BIGGEST issue.
2. numDisparities=64 too small. Close objects (~30cm) need disparity ~100px
   (= B*f/Z = 54*554/300). They fall outside the 64px search window -> wrong/"?".
3. Focal length is a guess (554px @ 60deg FOV). USB cams likely 70-90deg
   -> real f ~ 320-450px. Wrong f scales ALL absolute depths by a constant factor.
4. No lens distortion correction (CANNOT fix without real calibration — accept).

## Expected output
- New output where rectified left/right epipolar lines are horizontal
  (drawn horizontal reference lines should cross matching features).
- Valid-depth cell coverage substantially > 40% (target >70%).
- Depth values spatially smooth on continuous surfaces (floor, wall) instead of
  random jumps between neighbours.
- Two saved images: rectified-pair preview + depth grid.
- User can run ONE command and visually judge improvement.

## Plan
- [ ] 1. Add `rectify_uncalibrated(left, right)`: ORB (or SIFT) detect + BFMatcher
        + ratio test -> `cv2.findFundamentalMat(RANSAC)` ->
        `cv2.stereoRectifyUncalibrated` -> `cv2.warpPerspective` both images.
        Return rectified pair + match count + mean vertical residual of inliers.
        Print these as quality metrics. Fail-soft: if <20 inliers, warn and fall
        back to original (unrectified) pair so tool still runs.
- [ ] 2. Save a rectified-pair preview: hstack(left_rect, right_rect) with
        horizontal lines every ~40px -> `captures/depth_grid_rectified.jpg`,
        so alignment can be eyeballed.
- [ ] 3. Retune SGBM: numDisparities 64 -> 128, keep block 5; add WLS filter
        (cv2.ximgproc) IF available, else left-right disp12MaxDiff cleanup.
        Run matcher on the RECTIFIED images.
- [ ] 4. Add `--focal` already exists; add `--ndisp` and `--rectify/--no-rectify`
        CLI flags. Add a short focal-calibration note in --help: pick one cell on
        an object of known distance, adjust --focal until cm matches.
- [ ] 5. Recompute depth grid on rectified depth map; print valid-% before/after
        and depth range. Save `captures/depth_grid_out.jpg`.
- [ ] 6. Update `PROJECT_KNOWLEDGE.md` + `agent/plan/stereo-camera_plan.md` and log
        the limitation: uncalibrated rectification fixes geometry/alignment but NOT
        lens distortion or absolute scale; true accuracy needs stereo_calibrate.py
        -> calib/stereo.yml -> use Q matrix with reprojectImageTo3D.

## Run command (for next session, after implementation)
```
cd D:/dev/Autonomous_AI_robot/stereo-camera
D:/University/Python/Python310/python.exe tools/depth_grid.py \
  --left  captures/left_20260610_001156.jpg \
  --right captures/right_20260610_001156.jpg \
  --out   captures/depth_grid_out.jpg \
  --baseline 54 --ndisp 128
```

## Risks / notes
- Uncalibrated rectification quality depends on texture; this scene has rich floor
  texture so feature matching should be OK. If residual stays high, results stay noisy.
- ximgproc WLS may not be installed in this Python -> step 3 must degrade gracefully.
- Absolute cm values remain approximate until real calibration; set expectations.

## Execution log
<!-- [HH:MM] step N: <action> | risk: <note> | info: <fact> -->
[--] step 1: added rectify_uncalibrated (SIFT+BFMatcher+ratio test -> findFundamentalMat RANSAC -> stereoRectifyUncalibrated -> warpPerspective), prints match/inlier counts + vertical residual before/after, fail-soft on <20 inliers AND rejects warp if it worsens row alignment | info: initial RANSAC thresh 3.0 gave degenerate H (residual 1894px); tightening to 1.0/conf 0.999 fixed it (residual 60px->0.34px)
[--] step 2: save_rectified_preview -> captures/depth_grid_rectified.jpg, hstack + green horizontal lines every 40px | info: visually confirmed table leg/cable/connector align on same epipolar lines
[--] step 3: compute_disparity uses ximgproc WLS (lambda 8000, sigma 1.5) with right matcher, degrades to plain SGBM; ndisp 64->128 | info: WLS IS installed in D:/University/Python310
[--] step 4: added --ndisp, --rectify/--no-rectify; focal-calibration note in --help epilog
[--] step 5: prints valid pixel-% and grid-cell-%; depth>MAX clipped to invalid | info: rectified 20.5% cells vs no-rectify 44.0% — warp borders + perspective skew lower coverage
[--] step 6: pending — update PROJECT_KNOWLEDGE.md + stereo-camera_plan.md

## Test result
PARTIAL. Rectification geometry now correct (vertical residual 59.6px -> 0.34px,
visually verified on preview). BUT plan's >70% valid-cell target NOT met: rectified
~20% vs unrectified ~44%, because the uncalibrated warp adds black borders + heavy
perspective skew that block matching can't fill. Tradeoff = correct-but-sparse vs
dense-but-misaligned. User chose to keep --rectify ON as default (correctness over
density). True coverage+accuracy requires real checkerboard stereo calibration
(stereo_calibrate.py -> calib/stereo.yml -> Q matrix + reprojectImageTo3D).
