# Task: Drivable-area detection ported to robot stereo camera

- **Module:** area-detection (new)
- **Status:** testing
- **Created:** 2026-06-23

## Task
Clone `github.com/sajaysurya/drivable_area_detection` (KITTI U/V-disparity + HMM/Viterbi
free-space boundary, pure Python) and refactor it to run on this robot's live USB stereo
cameras, in the `area-detection/` folder.

## Input
- Live USB stereo pair: left=`/dev/video0`, right=`/dev/video2`, 640×480 @ 30 fps
  (matching stereo-camera defaults), OR a `--left/--right` image-file pair for offline test.
- Cameras are NOT factory-rectified; no calibration file exists.

## Expected output
- `area-detection/` Python package that produces a free-space overlay (green = drivable
  ground below the detected boundary) from live cameras or image files.
- Writes annotated frame to `area-detection/captures/`, optional `--display` window.
- No new system/pip dependencies (runs on the existing OpenCV 4.2 + numpy 1.17 stack).

## Decisions (confirmed with user)
- **HMM/Viterbi → reimplemented in pure NumPy** (repo used hmmlearn+sklearn, neither
  installed; the banded-transition Viterbi is ~30 lines).
- **Rectification → uncalibrated SIFT/ORB**, reusing the proven logic from
  `stereo-camera/tools/freespace.py` (no calibration data available).

## Adaptations required vs. upstream
- KITTI-tuned magic numbers → parameterised for 640×480 (disparity range, Hough threshold,
  U-disparity obstacle-height threshold, V-disparity road-plane threshold).
- `np.float` (deprecated) removed; replaced with `float`/`np.float64`.
- `onehot_initialization` (allocates H×W×Dmax ≈ 238 MB) → replaced with a vectorised
  per-column / per-row histogram (bincount) — far lighter on the Jetson.
- matplotlib display (slow, ~1 s/frame upstream) → real-time `cv2` overlay rendering.

## Plan
1. Scaffold `area-detection/` package: `stereo.py`, `disparity.py`, `boundary.py`
   (NumPy Viterbi), `rectify.py` (ported SIFT), `run.py` (driver), `README.md`,
   `captures/`. Copy upstream `left.png`/`right.png` as offline test fixtures; keep a
   short attribution to the original repo + its MIT-style provenance.
2. `stereo.py` — `get_depth_map(l,r,cfg)`: SGBM + WLS (ximgproc) with configurable
   `num_disparities`/`block_size`/WLS λ,σ. Default disparity range raised for 640×480.
3. `disparity.py` — `u_v_disparity(depth, dmax)` via vectorised histogram (no one-hot);
   returns `(v_disparity, u_disparity)`.
4. `boundary.py` — pure-NumPy banded-transition Viterbi (`free_boundary(u_disp)`) +
   V-disparity Hough road-plane fit + `project()` row-mapping. Reproduces upstream math.
5. `rectify.py` — port `_rectify_uncalib` (ORB → fundamental matrix →
   `stereoRectifyUncalibrated`, cached homographies) from stereo-camera/tools/freespace.py.
6. `run.py` — open cameras (warmup) or load image pair → rectify → depth → boundary →
   green cv2 overlay; `--left/--right`, `--display`, `--out`, `--device-left/right` flags.
7. Test offline on the bundled `left.png/right.png`; then a live-camera smoke run.

## Execution log
[10:52] step 1: scaffolded area-detection/ (+captures/), copied upstream left.png/right.png as fixtures | info: target folder existed but was empty
[10:54] step 2: stereo.py — SGBM+WLS via StereoConfig, num_disp default 96 for 640x480, output clamped to [0,Dmax]
[10:54] step 3: disparity.py — u_v_disparity via np.bincount | info: replaces 238 MB one-hot; exact same maps
[10:55] step 4: boundary.py — pure-NumPy banded Viterbi + V-disparity Hough road plane | info: drops hmmlearn/sklearn entirely
[10:55] step 5: rectify.py — ported cached ORB uncalibrated rectification from stereo-camera/tools/freespace.py
[10:56] step 6: run.py + README — live(0/2)/file driver, cv2 green overlay, --display/--once/--rectify/tuning flags
[10:56] step 7a: OFFLINE test on bundled KITTI pair PASSED — green road wraps correctly around the parked car + poles (captures/area_test.jpg)
[10:58] step 7b: LIVE single-frame test — cameras open, ORB rectification cached (447 inliers), pipeline ran; default thresholds found no road plane on the indoor scene (graceful degrade)
[11:00] step 7c: LIVE re-run --v-thresh 15 --hough-thresh 20 --obstacle-height 15 PASSED — green covers carpeted floor, boundary rises around fan + cabinet (captures/area_live2.jpg)

[11:20] follow-up: stereo baseline set to 54 mm. Added StereoConfig.baseline_mm=54.0 +
        focal_px, stereo.disparity_to_distance_mm(), run.py --baseline-mm/--focal-px and a
        "nearest obstacle: X.XX m" overlay readout (only real-obstacle columns).
        info: bundled left/right.png were replaced by user with 640x480 captures; verified
        on them with --rectify + tuned thresholds -> "nearest obstacle: 0.26 m"
        (captures/area_dist.jpg). focal_px is an estimate (457) until calibration.

## Test result
Self-verified PASS on both paths:
  - Offline: `python3 run.py --left left.png --right right.png --no-rectify` -> captures/area_test.jpg (correct).
  - Live: `python3 run.py --once --v-thresh 15 --hough-thresh 20 --obstacle-height 15` -> captures/area_live2.jpg (correct).
Note: default V-disparity thresholds (50/50) are tuned for outdoor/KITTI-scale scenes; indoor close-range needs lower --v-thresh/--hough-thresh. Awaiting user confirmation to mark done.
