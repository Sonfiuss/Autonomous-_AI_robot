# area-detection — drivable free-space from stereo vision

Detects the obstacle-free (drivable) ground in front of the robot from its stereo
camera pair and draws it as a green overlay.

Refactored from [sajaysurya/drivable_area_detection](https://github.com/sajaysurya/drivable_area_detection)
(KITTI demo) to run on this robot's live USB cameras with **no extra dependencies** —
it uses only the OpenCV 4.2 + NumPy already on the Jetson.

## Pipeline
1. **Grab** left/right frames (live cameras or an image pair).
2. **Rectify** — uncalibrated ORB → fundamental matrix → `stereoRectifyUncalibrated`
   (`rectify.py`). Cached after the first frame.
3. **Disparity** — SGBM + WLS smoothing (`stereo.py`).
4. **U/V-disparity** histograms (`disparity.py`).
5. **Free-space boundary** — per-column nearest-obstacle disparity via a banded-transition
   **Viterbi** decode, mapped to image rows with a V-disparity Hough road plane
   (`boundary.py`).
6. **Overlay** — green drivable region blended onto the left frame, written to `captures/`.

## What changed vs. upstream
| Upstream | Here | Why |
|----------|------|-----|
| `hmmlearn` + `scikit-learn` HMM | pure-NumPy Viterbi (`boundary.py`) | neither is installed; same math, no ARM build |
| `H×W×Dmax` one-hot (~238 MB) | `np.bincount` histogram | memory on the Jetson |
| KITTI-rectified input only | ORB uncalibrated rectification | our cameras aren't rectified |
| matplotlib display (~1 s/frame) | real-time `cv2` overlay | live use on the robot |
| KITTI magic numbers (disp=64, thr=100/75) | 640×480 defaults + CLI knobs | smaller frames |
| `np.float` (removed in NumPy ≥1.24) | `float` / `np.float64` | forward-compat |

## Usage
```bash
# Live cameras (left=/dev/video0, right=/dev/video2), writes captures/area.jpg
python3 run.py
python3 run.py --display              # + live window (q to quit)
python3 run.py --once                 # single live frame, then exit

# Offline test on the bundled KITTI sample (already rectified)
python3 run.py --left left.png --right right.png --no-rectify

# Tuning
python3 run.py --num-disp 128 --block 5 --obstacle-height 30 \
               --v-thresh 60 --hough-thresh 60
```

### Key flags
- `--device-left / --device-right` — camera indices (default 0 / 2).
- `--rectify / --no-rectify` — force rectification on/off (default: on for cameras,
  off for `--left/--right` files).
- `--num-disp`, `--block` — SGBM disparity range / block size.
- `--obstacle-height` — U-disparity accumulation (px) before a column counts as blocked.
- `--v-thresh`, `--hough-thresh` — V-disparity road-plane line fitting.
- `--baseline-mm` — physical distance between the two cameras (default **54 mm**).
- `--focal-px` — rectified focal length in pixels for metric distance.

### Metric distance
The overlay prints the distance to the nearest obstacle, from
`distance = focal_px × baseline_mm / disparity` (`stereo.disparity_to_distance_mm`).
The baseline is the measured **54 mm**. `focal_px` defaults to an estimate (~457 px for a
~70° HFOV 640px camera) — for accurate distances replace it with the calibrated focal
length (`P1[0,0]` from a stereo calibration), e.g. `--focal-px 520`.

## Calibration capture
Uncalibrated ORB rectification (`rectify.py`) is the weak link. To collect data for a
**real** stereo calibration, use `capture_chessboard.py`: it saves a synchronized
`captures/calib/left_NN.png` + `right_NN.png` pair only when **both** cameras detect the
full chessboard.

```bash
# Interactive: window with corners + green/red border. SPACE saves, q quits.
python3 capture_chessboard.py --display

# Headless (no monitor on the Jetson): auto-save a pair every 2 s when both detect.
python3 capture_chessboard.py --auto --interval 2

# Custom board (defaults: 9x6 inner corners, 25 mm squares)
python3 capture_chessboard.py --display --cols 9 --rows 6 --square 25
```

Collect 15–25 pairs at varied angles/distances, then run the calibration with the same
board, e.g. `stereo-camera/tools/stereo_calibrate.py` (9×6, 25 mm). Numbering resumes
after existing pairs, so you can append across runs.

## Modules
| File | Role |
|------|------|
| `run.py` | camera/file driver, overlay rendering, CLI |
| `stereo.py` | SGBM + WLS disparity map |
| `disparity.py` | U/V-disparity histograms (vectorised) |
| `boundary.py` | NumPy Viterbi boundary + V-disparity road plane |
| `rectify.py` | uncalibrated ORB rectification (cached) |
| `capture_chessboard.py` | collect synchronized chessboard pairs for calibration |
| `left.png`, `right.png` | bundled KITTI test pair (from upstream) |

## Notes / limitations (inherited)
- Assumes a roughly **flat** ground plane in front of the robot.
- Uncalibrated rectification is the weak link — running a real stereo calibration and
  loading it would improve disparity quality. `rectify.reset_cache()` recomputes after a
  physical knock to the camera rig.
- The leftmost `num_disp` columns have no disparity (SGBM search margin); treat the far
  left edge of the overlay as unreliable.
