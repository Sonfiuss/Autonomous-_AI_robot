---
id: 2026-06-22_arm-mask-freespace
status: testing
module: stereo-camera
started: 2026-06-22
---

## Task
Mask the robot's own arm out of the stereo result so depth processing ignores it,
then identify objects, their distance, and the free movement space.

## Input
- Module: `stereo-camera-AD-Census/` pipeline (`tools/preprocess_run.py`), with
  metric-depth helper in `stereo-camera/tools/depth_grid.py`.
- Test pair: `stereo-camera-AD-Census/captures/left_20260610_001156.jpg` + `right_*`.
- Arm facts (from user):
  - Arm is fixed to the robot body, but its image position VARIES with joint angles
    (so NO fixed polygon mask). Usually appears on the RIGHT side.
  - Silver/metallic appearance + red/yellow ribbon wires.
  - In the current test capture the arm is bottom-LEFT (left camera); it is the
    nearest object in frame and connects to the image border.
- Output choice (from user): **free-space depth grid** (reuse 72x36 grid; arm cells
  blanked; remaining cells classified near/clear).

## Expected output
- `preprocess_run.py` gains an arm-mask stage (after AD-Census, before clean/zone)
  that zeros arm pixels in the disparity map. Produces `_armmask.png` overlay for
  tuning. `--no-arm-mask` disables it.
- Object zoning (`_zones.png`) and the new free-space grid (`_freespace.png`) operate
  on arm-free disparity — arm is never reported as an object/obstacle.
- User eyeballs `_armmask.png` (arm correctly covered, scene not over-masked) and
  `_freespace.png` (objects + distance + free cells sensible), tuning thresholds.

## Plan
- [x] 1. Confirm env: OpenCV importable in this Python; note ximgproc availability.
- [x] 2. Implement `arm_mask(disp8, left_bgr)` fusing cues that fail differently:
       (a) near-disparity (closest blob), (b) border-connectivity (arm touches image
       edge; free objects don't), (c) right/lower region prior, (d) optional
       saturated red/yellow wire seed. Morph-close + keep largest border-connected
       near-component. Tunable flags: `--arm-near-cm`, `--arm-side {right,left,auto}`,
       `--no-arm-mask`. Write `_armmask.png` overlay.
- [x] 3. Apply mask: zero arm pixels in disparity BEFORE `clean_disparity` /
       `zone_objects` so objects exclude the arm. (also re-blanked after clean)
- [x] 4. Add free-space grid output: per-cell metric depth (port from depth_grid.py),
       arm cells blanked, classify near vs clear, label distance (cm). -> `_freespace.png`.
- [ ] 5. Run on the existing test pair; user tunes thresholds on the overlays. (DONE
       once; awaiting user eyeball + threshold tuning.)

## Open questions to resolve at start of next session
- RESOLVED: arm identified by border-connectivity (largest near+border blob);
  `--arm-near-cm` default 60 is only a soft near gate / tiebreaker, not the decider.
- RESOLVED: baseline=54mm, focal=554px reused (uncalibrated). Exposed as
  `--baseline`/`--focal` so user can recalibrate against a known distance.
- STILL OPEN: "suggested directions" annotated image was never located in captures/.

## Decisions / key facts (this session)
- Env: no working Python+OpenCV existed (base C:\Python\Python310 removed). Installed
  `mingw-w64-x86_64-python-opencv` (OpenCV 4.13, ximgproc=True) into MSYS2.
  Run the wrapper with: `C:\msys64\mingw64\bin\python tools/preprocess_run.py ...`.
- `_disp.png` from the C++ binary is MIN-MAX NORMALIZED to 0..255; absolute px scale
  is only printed to stdout ("Disparity range: [min, max] px"). `run_adcensus` now
  captures stdout + parses that line so metric depth can be recovered by
  de-normalizing (`disp8_to_cm`). Without it, depth is relative only.
- Arm mask placed after AD-Census, before clean+zone; arm pixels zeroed in disparity
  (and re-zeroed after clean so guidedFilter can't bleed it back).

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[--:--] step 1: found no working python+cv2; installed MSYS2 python-opencv 4.13 | info: ximgproc available
[--:--] step 2: added arm_mask() (near+border+side+wire fusion), arm_overlay() | info: keeps largest border-touching near blob
[--:--] step 3: zero arm disparity before clean/zone, re-zero after clean | risk: guidedFilter could bleed masked region back
[--:--] step 4: added disp8_to_cm() (de-norm) + free_space_grid() 72x36 | info: depth needs stdout range line
[--:--] step 5: ran on test pair | info: arm=15.5% masked; freespace clear=2161 near=22 arm=408

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
