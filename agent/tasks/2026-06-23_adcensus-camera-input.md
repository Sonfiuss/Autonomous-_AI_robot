# Task: AD-Census live camera input
**Date:** 2026-06-23
**Module:** stereo-camera-AD-Census
**Status:** testing

## Task
Add a `--camera` mode to `adcensus_depth` so it can read a stereo pair directly
from the Jetson's USB cameras instead of requiring pre-saved image files.

## Input
- Left/right USB cameras already working in `stereo-camera` module (indices 0/1,
  640×480, 30 fps — same defaults as `StereoCamera::Config`).

## Expected output
- `adcensus_depth --camera [--left N] [--right N] [--width W] [--height H]
   [--fps F] [--out prefix] [--min-disp 0] [--max-disp 128] [--save]`
  captures one stereo pair, runs AD-Census, writes `*_disp.png` + `*_disp_color.png`.
- Optionally saves captured frames to `captures/live_left.png` / `captures/live_right.png`
  when `--save` is given.
- Existing image-file mode unchanged.

## Plan
1. Add `videoio` to `CMakeLists.txt` OpenCV component list.
2. Restructure `main.cpp`:
   - Detect `--camera` as first arg.
   - Parse camera options (`--left`, `--right`, `--width`, `--height`, `--fps`,
     `--out`, `--min-disp`, `--max-disp`, `--save`).
   - Warm up cameras (5 frames), then grab+retrieve sync pair.
   - Hand the captured `cv::Mat` pair directly to the existing AD-Census pipeline.
3. Update `run_ad-census.sh` to pass `--camera` through to the binary.

## Execution log
- [09:00] step 1: task file created | risk: none | info: camera indices 0/1, 640×480
- [09:05] step 2: added `videoio` to CMakeLists.txt; CMake confirms opencv4 components all found
- [09:06] step 3: rewrote main.cpp — split into FileMode/CameraMode/RunADCensus/PackBGR; grab+retrieve sync pattern from StereoCamera.cpp; 5-frame warmup; optional --save flag
- [09:07] step 4: updated run_ad-census.sh to detect --camera as first arg and pass remaining args through
- [09:08] step 5: CMake build clean (all 100%); file mode verified end-to-end (21.8s, output PNGs written)

## Test result
File mode regression: PASS (captures/adcensus_out_disp.png written as before).
Camera mode: ready to test with physical cameras — run:
  `./run_ad-census.sh --camera`                        # live capture + disparity
  `./run_ad-census.sh --camera --save`                 # also saves live_left.png / live_right.png
  `./adcensus_depth --camera --left 0 --right 2`       # custom camera indices (Jetson UVC offset)
