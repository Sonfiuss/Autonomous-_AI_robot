# Task: Chessboard capture tool for area-detection

- **Module:** area-detection
- **Status:** testing
- **Created:** 2026-06-23

## Task
Add a feature to `area-detection/` that captures synchronized left/right chessboard
image pairs from the live USB stereo cameras, for stereo calibration. This addresses
the module's documented weak link (uncalibrated ORB rectification — see README).

## Input
- Live USB stereo pair: left=`/dev/video0`, right=`/dev/video2`, 640×480
  (same defaults as `run.py`).
- A printed checkerboard. Default pattern matches `stereo-camera/tools/stereo_calibrate.py`:
  **9×6 inner corners, 25 mm squares** (CLI-overridable).

## Expected output
- New script `area-detection/capture_chessboard.py`.
- Saves synchronized pairs to `area-detection/captures/calib/left_NN.png` +
  `right_NN.png` only when BOTH cameras detect the full chessboard.
- Live `--display` window: draws detected corners, green/red border = both-found/missing,
  on-screen pair counter; SPACE saves, `q` quits.
- Headless fallback (`--auto`): auto-saves a pair every `--interval` seconds whenever
  both detect (debounced), for when no display is attached.
- No new pip/system deps (OpenCV 4.2 + NumPy only).

## Plan
1. `capture_chessboard.py` — argparse CLI: `--device-left/right` (0/2), `--rows` (6),
   `--cols` (9), `--square` (25.0, recorded only), `--out-dir` (captures/calib),
   `--display`, `--auto`, `--interval` (2.0), `--width/--height` (640/480).
2. Open both cameras with warmup (reuse `run.py` `_open_camera` settings); per frame
   grab L+R, `cv2.findChessboardCorners` (ADAPTIVE_THRESH | NORMALIZE_IMAGE) on both,
   refine with `cornerSubPix` (same criteria as stereo_calibrate.py).
3. Save logic: write pair only when both found; auto-increment NN; print
   `[capture] saved pair NN` and running count. `--auto` debounces by `--interval`;
   `--display` waits for SPACE.
4. `--display` overlay: `drawChessboardCorners`, colored border, `pairs: N` / status text;
   `q` to quit. Headless mode runs until Ctrl-C.
5. Update `area-detection/README.md`: short "Calibration capture" section + how the
   saved pairs feed `stereo-camera/tools/stereo_calibrate.py` (or future calib loader).
6. Test: offline sanity (script imports, runs with `--help`); then a live smoke run
   capturing 1–2 pairs (user-run, since cameras + a physical board are needed).

## Execution log
[approved] confirmed: standalone script, save-only (calibration run separately).
[step 1-4] capture_chessboard.py: argparse CLI (devices 0/2, 9x6, 25mm, 640x480),
  dual-camera warmup, per-frame findChessboardCorners+cornerSubPix on both, save pair
  only when both detect. --display (SPACE saves, q quits, drawn corners + green/red
  border + counter) and --auto headless (debounced by --interval). Index resumes via
  _next_index over existing left_*.png.
[step 5] README: added "Calibration capture" section + module table row.
[step 6] Self-test PASS: py_compile OK; --help OK; _find_corners returns None on the
  non-board left.png (graceful); _next_index 0 -> 2 after saving; filenames
  left_NN.png/right_NN.png correct. Live camera+board loop is user-run only.

## Test result
Self-verified PASS (offline): compiles, CLI help, detection/index/save helpers all
correct. PENDING user live test:
  python3 area-detection/capture_chessboard.py --display     # SPACE to save when green
  python3 area-detection/capture_chessboard.py --auto        # headless
Awaiting user confirmation on real cameras + printed 9x6/25mm board to mark done.
