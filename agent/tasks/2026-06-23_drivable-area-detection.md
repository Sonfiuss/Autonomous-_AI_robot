# Task: Drivable-area detection from AD-Census stereo

- **Module:** stereo-area-detection
- **Status:** executing
- **Input:** `stereo-area-detection/captures/left.jpg`, `right.jpg` (1280x720)
- **Expected output:** drivable/free-space overlay image + disparity, using
  AD-Census depth (from `stereo-camera-AD-Census/adcensus_depth.exe`) fed into
  the `sajaysurya/drivable_area_detection` free-space (u/v-disparity + HMM) algo.

## Pipeline
```
left.jpg ───────────────────────────────────┐
right.jpg → warpAffine(alignment.yml, tx=0) ─┴► AD-Census .exe → disparity
                                                     │
                                       reconstruct int disparity
                                                     │
                              u/v-disparity + HoughLine ground + HMM free bound
                                                     │
                                            drivable_result.jpg
```

## Plan
1. Deps: Python310 + cv2/scipy/sklearn (+ installed hmmlearn). Clone repo. [done]
2. Stage 1: align right via alignment.yml — apply rotation+vertical only (keep
   horizontal disparity = depth signal); flag `--full-warp` for the full matrix.
3. Stage 2: run AD-Census exe on (left, right_aligned); parse disparity range
   from stdout; reconstruct integer disparity from the normalized PNG.
4. Stage 3: `run_drivable.py` reimplements freespace algo (patched: no np.float,
   no matplotlib popup, robust Hough fallback) and saves overlay + debug images.
5. Run end-to-end, show output.

## Execution log
- step 1: cloned repo, installed hmmlearn into Python310 | info: Python310 has cv2 4.5.5 + ximgproc; only hmmlearn missing
- step 2: align right with alignment.yml, tx=0 (keep horizontal disparity) | info: room imgs are 1280x720 = same res as yml, no tx/ty scaling needed
- step 3: AD-Census exe needs MinGW OpenCV DLLs on PATH (C:\msys64\mingw64\bin); else exit 0xC0000135 | risk: hard dependency on msys64
- step 3b: AD-Census bad_alloc at 1280x720 → downscale to 640x360 maxd=32 (~7s) | info: exe is O(W*H*ndisp) RAM, only safe ≤640px
- step 4: u/v-disparity + HoughLine ground + HMM free-bound; reimplemented (no np.float, no matplotlib popup, RAM-safe onehot) | info: HoughLine found ground rho=17 theta=171°
- step 5: ran end-to-end → drive_drivable.jpg, green free-space covers floor, excludes foreground clutter + boxes

## Test result
PASS (proof of concept). Command:
`python run_drivable.py` (in stereo-area-detection/)
Output: captures/drive_drivable.jpg (green = drivable), drive_disp_color.png,
drive_udisparity.png, drive_vdisparity.png, drive_right_aligned.jpg.
Known limitations: disparity speckled on textured floor → occasional false
"spikes" in the boundary; tune --obstacle-disp / --adc-width / --maxd to improve.

## CORRECTION (user flagged: "not using config to standardize before depth")
- The checkerboard alignment.yml (210px) was INVALID for these room images.
  Re-measured on the actual pair (350 inliers): shift_x=+13.5px, shift_y=+88.5px,
  rotation=+1.13°. The real misalignment is VERTICAL (88px)+rotation, not horizontal.
- Standardization principle: remove vertical+rotation (rows align = epipolar),
  KEEP horizontal (that IS the depth disparity). Applied via tx=0 warp.
- Re-ran: `python run_drivable.py --align captures/_room_align.yml --adc-width 512`
  → disparity now a CLEAN ground-plane gradient (was speckled); green drivable
  area smooth, boundary only rises at real objects. Output: captures/drive2_*.
- Fixes: run_adcensus now abspath()s all paths (relative --out-prefix broke the
  exe whose cwd is the AD-Census dir). AD-Census needs <1GB free → adc-width 512
  when RAM tight (machine had 0.9GB free).
