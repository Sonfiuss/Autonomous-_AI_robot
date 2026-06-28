# Task: Strip drivable-area logic → depth-only + room align + border fix

- **Module:** stereo-area-detection
- **Status:** testing
- **Input:** `stereo-area-detection/captures/left.jpg`, `right.jpg` (1280x720)
- **Expected output:** clean `drive_disp_color.png` (AD-Census disparity), with
  left/bottom boundary artifacts removed; no more drivable overlay.

## Background
Last session `drive_disp_color` was ~80% good but left edge + top/bottom bands
were wrong = boundary artifacts (left occlusion strip + warp black borders).

## Plan (approved via AskUserQuestion: blank-to-black, edit in place)
1. Default `--align` → `captures/_room_align.yml` (350-inlier room calib). [done]
2. Stage 1: warp a white mask with same matrix → `valid_mask` (warp content). [done]
3. After reconstruct: `invalidate_borders()` zeros disparity outside valid_mask
   + leftmost `maxd` columns (occlusion). [done]
4. `colorize()` paints disp==0 BLACK (not JET blue). [done]
5. Removed Stage-3 drivable code: `_HMM`, `onehot_uv`, `ground_projection`,
   `free_bound_hmm`, `render_overlay`, and `_drivable/_udisparity/_vdisparity`
   outputs + scipy/hmmlearn imports. [done]

## Execution log
- step 1-5: edited run_drivable.py in place | info: now pure depth pipeline
- run: `python run_drivable.py --adc-width 512` → 512x288 maxd=26, 4.1s
  | info: border mask zeroed 28660 px (left 26 cols + top warp band)

## Follow-up: bottom floor speckle (granite texture mismatch)
User: bottom of image still noisy. Cause = repetitive granite floor texture →
AD-Census false matches (cyan/green blobs where near floor should be red).
Added `denoise_disparity()`: cv2.filterSpeckles (zero small/divergent blobs) +
median smooth. New flags `--speckle-size(600) --speckle-diff(1.5) --median(7)`.
Run `--speckle-size 600 --median 7` removed 5197 px noise → floor now smooth
red→orange gradient; remaining cyan bottom-left = real clutter. Defaults bumped.

## Follow-up: black areas
User asked why many black regions. Explained: (1) top band = warp ty=88.5px down
shift, (2) left strip = occlusion + rotated corners, (3) interior holes = AD-Census
disp==0 + speckle-removed pixels. User chose "fill interior holes". Added
`fill_holes()` (cv2.inpaint TELEA) limited to valid_mask region (borders stay
black). `--no-fill` to disable. Run: filled 5412 px interior holes; only geometry
border black remains. Floor/objects now solid.

## Follow-up: review plan_upgrade_output.md (2026-06-26)
Plan assumed OpenCV SGBM; real engine = AD-Census (color, Census+AD). Applied
ONLY Phase 2 preprocessing (user choice): match_histograms_color R→L (self-impl,
no skimage) + clahe_color on Lab-L before warp. Flags --no-preprocess --clahe-clip.
Skipped SGBM/WLS (downgrade vs AD-Census), plane-fit fill + metrics (user deferred),
calibration (needs checkerboard data). Added OPENBLAS_NUM_THREADS=1/OMP=2 to exe env
to fix OpenBLAS OOM on low-RAM box. Full decision table in plan_upgrade_output.md.

## Test result
PASS (eyeball): disp_color shows clean floor gradient; black top band (88px ty
warp) + black left strip mark invalid regions. Bottom-left speckle remaining is
REAL clutter (red box/machines), not artifact.
Command: `python run_drivable.py` (in stereo-area-detection/)
Output: captures/drive_disp_color.png, drive_disp.png, drive_right_aligned.jpg
