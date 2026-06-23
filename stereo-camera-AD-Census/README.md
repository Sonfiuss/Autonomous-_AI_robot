# stereo-camera-AD-Census

AD-Census stereo matching ported to Jetson/Linux to produce a depth (disparity)
image from a **left/right image-file pair** (e.g. the captures in
`../stereo-camera/captures/`).

Algorithm: **AD-Census** (Xing Mei et al., 2011), C++ implementation by
Yingsong Li — https://github.com/ethan-li-coding/AD-Census. The algorithm files
in `adcensus/` are vendored **unmodified except**:
- converted from GBK to UTF-8 so GCC accepts the (Chinese) comments;
- added missing standard headers (`<cstdio>`, `<cstring>`, `<cmath>`) that MSVC
  pulled in transitively but GCC does not.

The upstream `main.cpp` (Windows-only: `imshow`, `system("pause")`, `fopen_s`)
is replaced by a headless `main.cpp` here.

## Layout
```
stereo-camera-AD-Census/
  adcensus/        vendored AD-Census algorithm (ADCensusStereo, cost_computor, …)
  main.cpp         headless entry: image-file in → disparity PNGs out
  CMakeLists.txt   Linux/Jetson build (OpenCV for I/O only)
```

## Build (on Jetson / any Linux with OpenCV + CMake)
```bash
cd stereo-camera-AD-Census
mkdir -p build && cd build
cmake .. && make -j$(nproc)
```
OpenCV is used **only** for `imread`/`imwrite` and the JET colormap; the matcher
itself is library-independent.

## Run
```bash
./adcensus_depth <left> <right> [out_prefix] [min_disp] [max_disp]

# example using the existing stereo captures (640x480):
./adcensus_depth \
  ../../stereo-camera/captures/left_20260610_001156.jpg \
  ../../stereo-camera/captures/right_20260610_001156.jpg \
  adcensus_out 0 128
```
Defaults: `out_prefix=adcensus_out`, `min_disp=0`, `max_disp=64`.

## Output
- `<out_prefix>_disp.png` — normalized 8-bit grayscale disparity
- `<out_prefix>_disp_color.png` — JET colormap visualization

Console prints the valid disparity range (px) and init/match timing.

## Preprocessing wrapper (recommended for the raw captures)

The raw captures are **unrectified**, so feeding them straight to `adcensus_depth`
gives a scattered/noisy map. `tools/preprocess_run.py` wraps the binary with a
prepare→match→clean pipeline (C++ tool unchanged):

1. **rectify** (uncalibrated, on the sharp originals — must come before denoise)
2. **denoise + CLAHE** on the rectified pair to strengthen texture
3. run **adcensus_depth**
4. **speckle removal + guided/bilateral smoothing** of the disparity
5. **depth-band zoning** + object outlines

```bash
python tools/preprocess_run.py \
  --left  captures/left_20260610_001156.jpg \
  --right captures/right_20260610_001156.jpg \
  --out-prefix captures/adcensus_pre --ndisp 128
```
Outputs (prefix `captures/adcensus_pre`): `_rectified.png`, `_L/_R.png` (prepared
pair), `_disp(_color).png` (raw matcher), `_clean(_color).png` (cleaned),
`_zones.png` (segmented). Toggle stages with `--no-rectify/--no-denoise/
--no-clean/--no-zone`. Order matters: rectify is applied **before** denoise
because denoising blurs the keypoints rectification relies on (residual 0.34px
vs 17.9px when reversed).

Needs Python with `opencv-contrib` (`cv2.ximgproc`). On Windows the script adds
`C:\msys64\mingw64\bin` to PATH so the C++ tool finds its OpenCV DLLs.

## Notes
- Input images are assumed **already rectified** (matching rows on epipolar
  lines). The existing captures are raw/unrectified; for best results feed a
  rectified pair (see `../stereo-camera/tools/depth_grid.py` rectification, or a
  proper checkerboard stereo calibration).
- AD-Census on CPU is compute-heavy; expect seconds per 640x480 frame. On Jetson
  the `aarch64` branch in CMakeLists adds `-O3 -march=native`.
- To convert disparity → metric depth: `Z = fx · baseline / disp`
  (baseline = 54 mm for this rig). Not emitted by default per task scope
  (disparity-only output); add it in `SaveDisparityMap` if needed later.
