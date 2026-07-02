# Session log — 2026-07-02 — Stereo camera alignment + metric depth ruler

Spans: `stereo-camera/` (new alignment + calibration tools), `depth-anything/`
(new stereo_ruler.py). Goal: turn the 2-camera rig into a "ruler" that gives
Depth Anything V2's mono output real metric scale.

---

## What was done

### 1. Chessboard alignment tool (task `2026-07-02_chessboard-align-tool`, status: testing)
Created `stereo-camera/tools/align_from_chessboard.py`. Reads
`chessboard_left*/right*.jpg` pairs (laptop-screen board, partially cropped by
the frame), detects the grid with `findChessboardCornersSBWithMeta` +
`CALIB_CB_LARGER` (handles a cropped board), resolves the left/right grid
correspondence with an ORB scene prior (a flat board fits *any* whole-square
shift equally well — the prior disambiguates), then fits a partial affine
(RANSAC) right→left per pair.

Result on the 2 existing pairs: dy=+102.6px (down), rotation=+1.17°, dx=+14.3px
(mostly real disparity, not misalignment). Saved to
`stereo-camera/calib/alignment.yml` with two warps: `warp` (full align) and
`warp_stereo` (keeps dx since horizontal disparity is the depth signal, only
corrects dy/rotation/scale). Debug overlays: `captures/align_check_pair1/2.jpg`.

### 2. stereo_ruler.py pilot — golden points + DA-V2 scale fit (task `2026-07-02_stereo-ruler-mono-scale`, status: executing)
Created `depth-anything/src/stereo_ruler.py`: normalizes the right image,
extracts sparse high-confidence "golden" stereo correspondences (Shi-Tomasi
bucketed corners, NCC row-match, subpixel, left-right consistency +
uniqueness filter — "quý hồ tinh": a few dozen clean points beat thousands of
noisy ones), runs DA-V2 (`vits`) on the right image, and RANSAC-fits
`d_mono = s*disparity + t`. First pass only found 12 points (5 inliers) —
widened `d_max` 96→140 and loosened matching to get 51 points / 24 inliers.

### 3. Metric correction — discovered the rig is toed-in, not parallel
User measured the real baseline (5.4cm). Plugging that into `Z = f·B/disp`
gave wall distances of 8-9m — clearly wrong. Cross-checked against the
chessboard geometry and found the actual cause: the rig is **not** a simple
side-by-side parallel pair. Full `stereoCalibrate` (anchored by the measured
5.4cm baseline) revealed:
- Toe-in yaw +2.14°, tilt +3.61°, roll −0.87°
- The baseline itself is tilted ~27° — the right camera sits 4.8cm right
  **and** 2.5cm below the left one (not purely horizontal)
- Both chessboard captures happened to be at nearly the same distance
  (0.91m / 0.96m) — so the earlier "translation consistent across depth"
  check in step 1 had no real depth spread to test against; noted as
  incorrect and corrected in memory.

Because of the slanted baseline, row-matching only holds at one depth plane —
proper metric depth needs full stereo rectification, not a single 2D warp.

Created `stereo-camera/tools/stereo_calibrate_2view.py`: runs
`cv2.stereoCalibrate` (joint focal fx=1124px estimated from all 4 chessboard
views, `CALIB_FIX_INTRINSIC`), scales the resulting translation vector so
`|T|` matches the measured 5.4cm baseline, then `cv2.stereoRectify`.
Hit a degenerate case: `alpha=0` collapsed the valid ROI and exploded the
rectified focal to ~166,000px; switched to `alpha=-1` which gives a sane
fx=1123.9px and f·B=60.69 px·m. Verified visually — epipolar lines are
horizontal at every depth after remap (checked with a grid overlay on the
room pair). Output: `stereo-camera/calib/stereo_rectify.yml`.

Rewired `stereo_ruler.py` to rectify both images via this config (with the
old 2D-warp mode kept as a fallback that now warns it can't produce valid
metric output). Re-ran on the room pair: 58 golden points, 19 inliers,
`s=0.0621 t=-2.442`. Distance labels are physically plausible: wall/door
~1.2m, water bottle 0.86-1.05m, clamp tool 0.72m — consistent with the
chessboard having been on the same table at 0.91m.

### 4. Distance labels on golden-point overlay
Added per-point distance labels to `golden_overlay.jpg`, later restricted to
inliers only (outliers stay as bare red-X markers) per user request. Labels
show relative units (`1/disparity`, normalized) when no `f·B` is available,
or real metres once `--fb`/`stereo_rectify.yml` is present.

---

## Decisions made
- **Golden-point philosophy stays "quý hồ tinh"**: RANSAC keeps ~1/3 of raw
  matches; the rejected ones are consistently textureless/repetitive-floor
  false matches, confirming the filter is working as intended.
- **warp_stereo (2D) is now legacy/fallback only** for this rig — it's valid
  near the ~0.9m plane where it was measured, not as a general metric tool.
  `stereo_rectify.yml` (full calibration) is the source of truth going
  forward for anything needing real distances.
- Metric scale currently rests on fx=1124px estimated from only 4 planar
  chessboard views (~±15% uncertainty per the user-facing caveat given).
  Not yet corrected against a hand-measured real-world distance.

## Interface changes
- New config file `stereo-camera/calib/stereo_rectify.yml` (K, dist, R, T,
  R1/R2/P1/P2/Q, fx, baseline_m, square_m, fb, stereo_rms_px,
  angles_deg_tilt_yaw_roll) — this is the new default alignment source for
  `depth-anything/src/stereo_ruler.py`.
- `stereo-camera/calib/alignment.yml` (from step 1) still exists and is used
  as fallback only if `stereo_rectify.yml` is missing.

## Pending / follow-ups
- **Accuracy**: fx has ~±15% uncertainty (only 2 chessboard poses, both at
  ~0.9m). Offered to the user: measure one real-world distance to a visible
  object to correct fx. Not yet done.
- Stereo-ruler task plan steps 8-9 (integrate `--scale-mode stereo-ruler`
  into `build_map.py`/`explore_map.py`, full error measurement + 360°
  map consistency test) are not started — current work only covers steps
  1-7 on a single pilot pair.
- Both task files (`2026-07-02_chessboard-align-tool` status `testing`,
  `2026-07-02_stereo-ruler-mono-scale` status `executing`) are awaiting user
  confirmation to move to `done`.
- `agent/plan/stereo-camera_plan.md` does not exist yet — no module plan file
  was updated (module has no plan file to update; consider creating one if
  this becomes an ongoing pipeline).
