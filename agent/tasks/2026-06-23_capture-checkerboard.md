# Task — Capture checkerboard pairs + calibrate (CLI)

- **Module:** stereo-camera
- **Status:** testing
- **Input:** 2 USB cameras (left idx 0, right idx 2), printed checkerboard (9x6 inner corners, square mm known)
- **Expected output:**
  - Raw left/right image pairs saved to `captures/checkerboard/<session>/`
  - `calib/stereo.yml` (M1,D1,M2,D2,R,T,R1,R2,P1,P2,Q,baseline) — compatible with `StereoCamera::loadCalibration`

## Decisions
- New standalone tool `tools/capture_checkerboard.py` (do NOT modify stereo_calibrate.py).
- UI: local `cv2.imshow` window (no browser). Keys: SPACE=capture, c=calibrate, q=quit.
- Capture + calibrate in one tool, but ALSO save raw images (tool cũ chỉ giữ corners trong RAM).
- Reuse detect/calibrate logic from stereo_calibrate.py.

## Plan
1. CLI args (rows/cols/square/left/right/out/img-dir/width/height/min-pairs).
2. Open both cameras at native res; per-frame chessboard detect + overlay (border xanh/đỏ, pair count).
3. SPACE: if both detected + sharp → save left_NN.jpg/right_NN.jpg + store full-res corners.
4. c: run stereoCalibrate on collected pairs (>=15) → write calib/stereo.yml + print fx/baseline/RMS.
5. q: quit.
6. Add run_capture_checkerboard.sh launcher (cv2-capable python auto-detect).

## Execution log
[--:--] step 1-5: tạo tools/capture_checkerboard.py (CLI cv2.imshow, SPACE/c/r/q) | info: tái dùng detect+stereoCalibrate từ stereo_calibrate.py, thêm lưu ảnh gốc + sharpness gate
[--:--] step 6: tạo run_capture_checkerboard.sh (auto-dò python cv2) | info: py_compile OK, cv2 4.13
[--:--] verify: py_compile OK trên MSYS2 python; --help lỗi cp1258 chỉ do console Windows, không phải bug (Jetson UTF-8 ok)

## Add-on — Offline calibration tool (define stereo config from saved images)
- **Status:** executing → testing
- New `tools/calibrate_from_images.py` + `run_calibrate_from_images.sh`.
- Đọc cặp left_NN/right_NN từ 1 thư mục session → detect corners → calibrateCamera
  ×2 → stereoCalibrate (R,T) → stereoRectify (R1,R2,P1,P2,Q) → ghi calib/stereo.yml
  (đúng format StereoCamera::loadCalibration). In baseline + góc lệch 2 cam + RMS.
- Ảnh debug: `<dir>/_calib/detect_NN.jpg` (corners/FAIL từng cặp) + `rectified.jpg`
  (nắn thẳng + đường epipolar ngang để mắt thường kiểm tra alignment).
- Board geometry user xác nhận: 8x8 Ô → 7x7 GÓC TRONG (default --rows 7 --cols 7),
  square=18mm. Lưu ý 7x7 đối xứng → giữ cùng hướng giữa các shot.
- Verify: py_compile OK; chạy trên 4 cặp cũ → 0/4 detect (bàn cờ bị che màn hình +
  cắt mép) → tool báo lỗi đúng + xuất detect_*.jpg, KHÔNG sinh config rác.
- PENDING user: chụp lại bằng bàn cờ IN GIẤY, full khung ở cả 2 cam, 15-25 tư thế,
  rồi `DIR=<session> ./run_calibrate_from_images.sh`.

## Test result (chờ user xác nhận trên Jetson)
Chạy: `cd stereo-camera && ./run_capture_checkerboard.sh`
hoặc: `python3 tools/capture_checkerboard.py --cols 9 --rows 6 --square <mm> --left 0 --right 2`
Kỳ vọng: cửa sổ left|right, border xanh khi thấy bàn cờ → SPACE chụp 15-25 cặp → c → in fx/baseline/RMS + ghi calib/stereo.yml; ảnh gốc ở captures/checkerboard/<session>/
