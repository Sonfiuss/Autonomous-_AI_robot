# Task: Freespace / navigable-ground detector
**Date:** 2026-06-23
**Module:** stereo-camera (tools/)
**Status:** executing

## Task
Detect navigable ground in front of the robot from a live stereo camera pair and
produce a green/red overlay image: green = free to drive, red = obstacle.

## Input
- Left/right USB cameras (idx 0/1, 640×480, 30fps — same as StereoCamera::Config)
- Optional: calibration file `calib/stereo.yml`
- Offline mode: supply --left-img / --right-img to test without cameras

## Expected output
- `captures/freespace_out.jpg` — left camera frame with green (free) / red (obstacle) overlay
- Printed per-frame stats: time, % free area, fitted ground-plane coefficients
- `--display` flag for live OpenCV window

## Algorithm
1. Capture synchronized stereo pair (grab+retrieve pattern from StereoCamera.cpp)
2. Rectify: uncalibrated (SIFT feature match + fundamental matrix) or calibrated (.yml)
3. SGBM disparity (same params as StereoCamera.cpp)
4. V-disparity ground-plane fit: for each row find 75th-pct disparity → fit d_ground(v) = a·v + b
5. Per-pixel classify: ground if |d - d_ground(v)| < thresh, obstacle otherwise
6. Flood fill from bottom row → keep only reachable free space
7. Morphological close → remove holes
8. Color overlay: green=free, red=obstacle, alpha blended onto rectified left frame

## Plan
1. Write `stereo-camera/tools/freespace.py`
2. Test offline with existing captures/left_20260610_001156.jpg pair
3. Test with live cameras

## Execution log
- [09:10] step 1: task file created
- [09:20] step 2: wrote stereo-camera/tools/freespace.py — ORB rectify, SGBM, v-disparity ground fit, connected-component flood fill, green/red overlay
- [09:25] step 3: fix ORB (nfeatures=2000, ratio 0.85), NaN errstate, flood fill → connected-component reachability, --no-rectify flag
- [09:30] step 4: offline test PASSED — 36.9% free, dt=289ms, ground d=0.014v+122.4, output 640×480 RGB saved

## Test result
Offline test: PASS
- With rectification: 36.9% free, 289ms, no warnings
- Without rectification: 0% free (expected — SGBM needs rectified rows)

## Status: testing
