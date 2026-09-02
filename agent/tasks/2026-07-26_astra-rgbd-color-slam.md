---
id: 2026-07-26_astra-rgbd-color-slam
status: testing
module: stereo-camera
started: 2026-07-26
---

## Task
Nâng astra_slam lên RGB-D hoàn chỉnh: calib depth<->color (mode B), màu chính
xác từng pixel, RGB-D odometry (đo chuyển động thật từ ảnh, không dùng motor),
TSDF fusion cho map sạch/đẹp có màu.

## Input
- `depth-anything/src/astra_cloud.py` + `astra_slam.py` (đã chạy live ~7-9 FPS,
  RGB mode A xấp xỉ, ICP-only odometry).
- Astra Pro: depth+IR qua OpenNI2 (chung sensor), color qua UVC/cv2 (device
  riêng, không đồng bộ hardware).
- User chốt: KHÔNG dùng tín hiệu motor (trượt bánh) — chuyển động đo từ camera.
- Checkerboard sẵn có từ stereo-camera captures (cols/rows/size truyền qua CLI).

## Expected output
- `astra_calib.py`: capture cặp IR+color checkerboard → `astra_calib.npz`
  (intrinsics color + R,t depth→color), RMS reproj < 0.5 px.
- `astra_rgbd.py`: `align_color_to_depth()` màu khớp 1:1 pixel depth (mode B).
- `astra_slam.py` mở rộng: `--calib` (mode B màu), `--rgbd-odom` (hybrid
  color+depth odometry, fallback ICP), `--tsdf` (map cuối = TSDF fusion màu),
  record/replay có color (.npz).
- User test: đi thẳng 2 m đo thước → dist báo sai ≤ ~5%; tường không nhân đôi;
  màu khớp mép vật thể 0.5–3 m.

## Plan
- [x] 1. `astra_calib.py --capture`: mở depth lấy intrinsics → chuyển IR stream
       + UVC color, SPACE lưu cặp ảnh checkerboard (~15-20 pose).
- [x] 2. `astra_calib.py --calibrate`: calibrateCamera (color) +
       stereoCalibrate (fix intrinsics 2 bên) → `astra_calib.npz`, in RMS.
- [x] 3. `astra_rgbd.py`: load_calib + align_color_to_depth (vectorized:
       backproject→R,t→project→sample; undistort color trước) + preview CLI.
- [x] 4. astra_slam: `--calib` → màu mode B (thay same-pixel hack; mode A vẫn
       là fallback khi không có file calib).
- [x] 5. astra_slam: record/replay lưu/đọc color (`frame_*.npz`, giữ tương
       thích `.npy` cũ).
- [x] 6. astra_slam: `--rgbd-odom` = compute_rgbd_odometry (hybrid term),
       init = motion trước, fallback ICP khi fail; đổi hệ trục qua F=diag(1,-1,1).
- [x] 7. astra_slam: `--tsdf` = ScalableTSDFVolume tích keyframe RGBD với pose
       đã optimize → cloud/mesh màu sạch làm output cuối.
- [~] 8. Test offline PASS (synthetic replay: depth-only ICP, rgb+rgbd-odom,
       mode B calib, tsdf đều chạy sạch) → CÒN test live vòng kín (cần HW).

## Execution log
<!-- [HH:MM] step N: <summary> | risk: <...> | info: <...> -->
[--:--] step 1-2: astra_calib.py (capture IR+UVC + stereoCalibrate FIX_INTRINSIC, T lưu mm) | risk: IR speckle phá corner detect -> PHẢI dán che projector khi capture; IR stream chưa test trên HW thật
[--:--] step 3: astra_rgbd.py align_color_to_depth vectorized + unit test identity & baseline-shift (31px) PASS | info: undistort qua remap table cache (initUndistortRectifyMap 1 lần) thay cv2.undistort mỗi frame
[--:--] step 4-7: astra_slam +--calib/--rgbd-odom/--tsdf/record npz | info: RGBD odom trả T ở hệ y-down -> conjugate F=diag(1,-1,1,1) khớp hệ y-up của map; MAX_STEP_M=0.3 chặn jump; TSDF dùng opt_poses, raw map lưu *_raw
[--:--] step 8: offline smoke PASS (3 cấu hình: icp-only, rgbd mode A, rgbd mode B+tsdf; f1..f7 đều odo=rgbd, TSDF ra cloud màu) | info: replay synthetic → dist chỉ định tính, cần HW đo thật
[--:--] standards-review: 9 violations fixed (magic values→constants, ok_o→odo_ok, imwrite check, MIN_PAIRS/RMS_WARN_PX), 0 deferred
[--:--] logic-review: 2 mismatches fixed | info: keyframe bỏ lưu RGBD khi color drop (tránh mảng đen trong TSDF RGB8); undistort map cache hot-path

## Hướng dẫn test (status: testing)
1. Calib (1 lần, che projector bằng băng keo):
   `python astra_calib.py --capture calib/ --rgb-index 0` (SPACE ~20 pose)
   `python astra_calib.py --calibrate calib/ --cols 9 --rows 6 --square-mm 25`
   → đạt: stereo rms < 0.5px.
2. Kiểm màu khớp: `python astra_rgbd.py --preview` (mép vật trùng mép depth).
3. Chạy full: `python astra_slam.py --rgb --calib ../output/astra_calib.npz
   --rgbd-odom --tsdf` — đi 1 vòng kín, đi thẳng 2 m đo thước so cột dist.
   → đạt: dist sai ≤ ~5%; astra_map.ply tường không nhân đôi, màu đúng vật.

## Test result
