# Session 2026-07-26 (b) — RGB-D upgrade: calib mode B, RGB-D odometry, TSDF

## Context
User request: 3D map từ camera ORBBEC, chuyển động đo từ ẢNH (không tin motor
vì trượt bánh), map rõ + có màu đẹp. Phần lớn nền tảng đã có (astra_slam ICP
visual odometry + RGB mode A) → session này đóng 3 khoảng trống: màu chính xác,
odometry dùng texture ảnh, map sạch. Task: `2026-07-26_astra-rgbd-color-slam`
(status: testing).

## Implemented
- **astra_calib.py** (mới): `--capture` = depth self-calib intrinsics → chuyển
  sensor sang IR stream + UVC color, SPACE lưu cặp checkerboard (PHẢI dán che
  laser projector kẻo speckle phá corner detect); `--calibrate` =
  calibrateCamera(color) + stereoCalibrate(CALIB_FIX_INTRINSIC, dist_ir=0) →
  `astra_calib.npz` (K_c, dist_c, R, T_mm — T đơn vị mm khớp depth).
- **astra_rgbd.py** (mới): `align_color_to_depth()` vectorized
  (backproject y-down → R,T → project → sample; undistort qua remap table cache
  1 lần thay cv2.undistort mỗi frame); `--preview` blend màu-đã-align lên depth
  colormap. Unit test: identity alignment + baseline 31px shift PASS.
- **astra_slam.py** (mở rộng, astra_cloud KHÔNG sửa):
  - `--calib NPZ` → màu mode B thay same-pixel mode A (mode A vẫn fallback).
  - `--record --rgb` → `frame_*.npz` (depth+color); ReplaySource đọc cả
    .npy/.npz, `has_color`.
  - `--rgbd-odom` → `compute_rgbd_odometry` hybrid term, init = motion trước,
    fallback ICP; **hệ trục**: o3d RGBD/TSDF dùng y-DOWN, map dùng y-UP của
    astra_cloud → conjugate `flip_T(T)=F@T@F`, `F=diag(1,-1,1,1)`.
    `MAX_STEP_M=0.3` chặn jump 1 frame (áp cả cho ICP). Log thêm cột odo
    (`rgbd`/`icp`).
  - `--tsdf [--tsdf-voxel 0.015]` → keyframe lưu RGBD image (RGB8), cuối chạy
    ScalableTSDFVolume với pose ĐÃ pose-graph-optimize (`opt_poses`), extract
    cloud → `--out`; raw accumulate giữ ở `*_raw.ply`. Keyframe bị drop color
    thì KHÔNG lưu RGBD (tránh mảng đen trong volume màu).

## Tests (offline, synthetic, Windows python D:\University\Python\Python310)
- py_compile 3 file OK; --help đầy đủ cờ.
- Replay depth-only: ICP path chạy như cũ (regression OK).
- Replay npz: `--rgb --rgbd-odom --tsdf` và `--rgb --calib fake --rgbd-odom
  --tsdf` → mọi frame odo=rgbd, TSDF ra cloud màu, không exception.
- Reviews: standards (9 fix), logic (2 fix — keyframe color-drop guard,
  undistort cache).

## Pending (user test, xem task file mục "Hướng dẫn test")
1. HW calib thật (IR stream OpenNI2 CHƯA test trên phần cứng — rủi ro chính).
2. `astra_rgbd.py --preview` kiểm mép màu.
3. Live vòng kín + đi thẳng 2 m đo thước (≤ ~5%) → chốt ngưỡng, set done.

## Notes
- Không dùng odom bánh xe đúng yêu cầu user (read_wheel_odom vẫn stub, chỉ là
  init tùy chọn cho tương lai).
- Chưa git commit (kể cả đợt xoá legacy session trước).
