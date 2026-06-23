---
id: 2026-06-22_adcensus-port
status: done
module: stereo-camera
started: 2026-06-22
---

## Task
Apply the AD-Census stereo matching solution
(https://github.com/ethan-li-coding/AD-Census) to compute a depth/disparity
image from captured image pairs. New self-contained folder
`stereo-camera-AD-Census/`, keep IMAGE-FILE input (existing captures), build for
Jetson (Linux/aarch64).

## Input
- Upstream repo: ethan-li-coding/AD-Census (C++, OpenCV only for I/O)
- Test pair: `stereo-camera/captures/left_20260610_001156.jpg` + `right_…` (640x480)
- Output target: Jetson aarch64 + OpenCV

## Decisions (user-approved)
- Vendoring: git clone + copy algorithm sources (self-contained, no submodule).
- Output: disparity-only (upstream behavior) — grayscale + JET colormap PNGs.

## Plan
- [x] 1. Clone repo, copy algorithm .cpp/.h into `stereo-camera-AD-Census/adcensus/`
        (skip vcxproj, 3rdparty, Data, upstream main.cpp).
- [x] 2. Convert vendored sources GBK→UTF-8 (GCC rejects GBK comments).
- [x] 3. Add missing standard headers (<cstdio>/<cstring>/<cmath>) MSVC pulled
        in transitively but GCC does not.
- [x] 4. Write headless portable `main.cpp` (image-file in, disparity PNGs out;
        no imshow/system("pause")/fopen_s).
- [x] 5. Write CMakeLists.txt (Linux/Jetson, OpenCV core/imgproc/imgcodecs,
        aarch64 -O3 -march=native).
- [x] 6. README + this task file + action log.

## Run command (on Jetson / Linux with OpenCV + CMake)
```
cd stereo-camera-AD-Census && mkdir -p build && cd build && cmake .. && make -j$(nproc)
./adcensus_depth \
  ../../stereo-camera/captures/left_20260610_001156.jpg \
  ../../stereo-camera/captures/right_20260610_001156.jpg \
  adcensus_out 0 128
```
Expect: `adcensus_out_disp.png` + `adcensus_out_disp_color.png`.

## Execution log
[--] step 1: cloned to temp, copied 6 .cpp + 7 .h + ReadMe + LICENSE to adcensus/
[--] step 2: iconv GBK->UTF-8 all sources; revalidated all OK | info: first iconv -o pass silently failed (printed to stdout), redid with redirection
[--] step 3: added headers via sed | info: errors were printf/memcpy/exp/sin/lround/memset undeclared on GCC 6.3
[--] step 4: wrote headless main.cpp (BGR byte packing, SaveDisparityMap gray+JET)
[--] step 5: wrote CMakeLists.txt (static adcensus lib + adcensus_depth exe)
[--] step 6: README.md written
[--] verify: g++ -std=c++14 -fsyntax-only on all 6 algorithm .cpp = clean. main.cpp
     not compiled locally (no OpenCV/cmake on Windows dev box) — builds on Jetson.

## Test result — PASS (built + ran on Windows via MSYS2)
Installed MSYS2 (winget) + mingw-w64-x86_64-{gcc,opencv,pkgconf} + make.
Built with g++ 16.1.0 + OpenCV 4.13 (no CMake, via run_ad-census.sh).
Ran on captures/left_+right_20260610_001156.jpg (640x480, disp[0,128]):
  match 25.0s (cost 5.6 / aggr 10.7 / scanline 1.8 / refine 6.6), range 0-126px.
Output OK: captures/adcensus_out_disp.png + _disp_color.png.
QUALITY: noisy/patchy — input pair is UNRECTIFIED so epipolar search scatters.
Port is correct; to get clean depth, feed a rectified pair (depth_grid.py rectify
or real checkerboard stereo calibration). One-shot reproducer: run_ad-census.sh.
Toolchain to run again on Windows: use MSYS2 'MinGW 64-bit' shell (C:\msys64).

## Risks / notes
- Existing captures are UNRECTIFIED; AD-Census assumes rectified input. Disparity
  will be noisy until a rectified pair is supplied. See depth_grid.py rectify or
  checkerboard calibration.
- AD-Census CPU cost is high (~seconds/frame at 640x480).
