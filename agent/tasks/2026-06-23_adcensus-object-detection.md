---
id: 2026-06-23_adcensus-object-detection
status: testing
module: stereo-camera
started: 2026-06-23
---

## Task
Free-space (navigable ground) detector powered by AD-Census disparity.

## Input
- Existing `stereo-camera-AD-Census/main.cpp` — produces disparity map
- Existing `stereo-camera/tools/freespace.py` — V-disparity ground plane logic (SGBM)
- User wants: AD-Census disparity → fit floor → green free space + red obstacles

## Expected output
`stereo-camera-AD-Census/tools/freespace.py`:
- Takes stereo pair (camera or file mode)
- Calls `adcensus_depth` binary → raw 16-bit disparity PNG
- V-disparity fit → ground plane equation
- Flood from bottom → reachable free space
- Outputs overlay: green=free space, red=obstacles, white boxes; + _free.png/_obstacles.png

## Plan
- [x] 1. Modify `main.cpp` to also save `_disp_raw.png` (16-bit, disparity×16, 0=invalid)
- [x] 2. Create `tools/freespace.py` (AD-Census freespace pipeline)
- [x] 3. Rebuild `adcensus_depth` binary + smoke-test
- [x] 4. Add distance-to-obstacle (depth = baseline*focal/disp), box labels + nearest in HUD

## Execution log
[now] step 1: added 16-bit raw disparity output to SaveDisparityMap() in main.cpp
[now] step 2: PIVOT from object_detector → freespace.py (reuses original V-disparity logic)
[now] step 3: rebuilt binary; smoke-test ran end-to-end | info: _BINARY needs resolve() abs path
[now] step 4: distance via baseline=54mm focal=554px (from depth_grid.py); per-box + nearest
[now] info: free=0% on old calib pair (desk scene, no floor) — needs live floor scene to verify

## Test result
<!-- Filled when user confirms -->
