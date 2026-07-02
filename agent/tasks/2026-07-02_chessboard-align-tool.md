---
id: 2026-07-02_chessboard-align-tool
status: testing
module: stereo-camera
started: 2026-07-02
---

## Task
Tạo tool đọc 2 cặp ảnh bàn cờ (chessboard hiển thị trên laptop, chụp bằng 2 camera) trong
`stereo-camera/tools/captures/` → tính config align (warp) giữa camera phải và camera trái
→ lưu YAML để "normalize" (chuẩn hoá) ảnh camera phải về khung camera trái trước khi xử lý stereo.

Liên quan: đây chính là "config 2 camera" mà task [[2026-07-02_stereo-ruler-mono-scale]]
bước 1 đang chờ user cung cấp (phần độ lệch/alignment).

## Input
- 2 cặp ảnh: `chessboard_left1/right1.jpg` (gần), `chessboard_left2/right2.jpg` (xa), 1280x720.
- Đã kiểm tra trước (scratchpad): `findChessboardCornersSBWithMeta` + `CALIB_CB_LARGER`
  detect được 96–112 góc/ảnh dù bàn cờ bị cắt mép (grid 16x7, 16x6, 14x7 tuỳ ảnh).
- Bài học cũ (claude_action_log 2026-06-23): warp đo từ bàn cờ ở MỘT khoảng cách (210px)
  KHÔNG đúng ở khoảng cách khác (thực tế phòng chỉ lệch 13.5px ngang). Translation phụ
  thuộc depth; rotation/scale thì ổn định → tool phải so 2 cặp (gần vs xa) và cảnh báo.

## Expected output
- Tool mới `stereo-camera/tools/align_from_chessboard.py`:
  - Mode tính: đọc các cặp `chessboard_left*.jpg/right*.jpg` → detect góc → khớp lưới L↔R
    → estimateAffinePartial2D (RANSAC) right→left cho TỪNG cặp → in dx, dy, góc xoay, scale,
    RMS, inliers → so sánh 2 cặp (consistency theo depth) → lưu config + ảnh overlay debug.
  - Mode apply: `--apply right.jpg` đọc config → warpAffine → ảnh right đã chuẩn hoá.
- Config `stereo-camera/calib/alignment.yml`: ma trận warp 2x3, các thành phần
  (dx, dy, angle, scale), stats từng cặp, kích thước ảnh, ngày đo.
- Ảnh debug `captures/align_check_*.jpg`: blend left + right-đã-warp để mắt thường kiểm tra.

## Plan
- [x] 1. Viết `align_from_chessboard.py` — phần detect: load cặp ảnh, detect góc bằng
      `findChessboardCornersSBWithMeta` (CALIB_CB_LARGER, seed 9x6), trả về góc + toạ độ lưới.
- [x] 2. Khớp tương ứng lưới L↔R: vì bàn cờ bị cắt khác nhau ở 2 camera, origin lưới có thể
      lệch nguyên ô → thử các shift nguyên (±6 ô), chọn shift cho residual affine thấp nhất.
- [x] 3. Ước lượng warp từng cặp: `estimateAffinePartial2D` (RANSAC) right→left; in
      dx/dy/angle/scale + RMS + số inliers.
- [x] 4. So sánh cặp gần vs xa: nếu dx/dy lệch nhau nhiều → cảnh báo translation phụ thuộc
      depth (chỉ dùng warp cho vùng depth tương ứng); angle/scale phải khớp nhau.
- [x] 5. Lưu `stereo-camera/calib/alignment.yml` (cv2.FileStorage) + ảnh overlay debug
      trước/sau vào `captures/`.
- [x] 6. Mode `--apply`: load YAML, warpAffine ảnh right bất kỳ → ảnh chuẩn hoá (dùng làm
      pre-process trước stereo depth / stereo-ruler).
- [x] 7. Chạy tool trên 2 cặp hiện có, kiểm tra overlay + số liệu, báo user lệnh test.

## Execution log
[03:10] step 1: detect OK cả 4 ảnh (96–112 góc/ảnh dù bàn cờ bị cắt mép) | info: phải dùng SBWithMeta+CALIB_CB_LARGER, findChessboardCorners thường fail vì board không nguyên vẹn
[03:15] step 2: khớp lưới bằng shift nguyên ±6 ô + thử flip 180° | risk: bàn cờ PHẲNG nên mọi shift đều fit affine tốt → PHẢI dùng ORB prior trên toàn cảnh (nền, viền laptop) để chọn shift đúng; nếu không sẽ lệch nguyên ô (~70px)
[03:20] step 3: pair1 dx=+16.6 dy=+102.4 rot=+1.25° rms=1.09px (82 inl); pair2 dx=+12.0 dy=+102.8 rot=+1.10° rms=0.70px (98 inl)
[03:20] step 4: 2 cặp (gần vs xa) NHẤT QUÁN: dy lệch 0.4px, dx lệch 4.6px (<15px), rot lệch 0.15° | info: dy≈102.6 ổn định theo depth → đúng là mounting offset; dx nhỏ ≈ disparity thật
[03:22] step 5: lưu calib/alignment.yml (warp trung bình + per-pair stats) + align_check_pair1/2.jpg | info: overlay AFTER bàn cờ chồng khít (vàng/đen), vật gần vẫn ghost (đúng lý thuyết depth)
[03:25] step 6: thêm warp_stereo (giữ dx=disparity, chỉ khử dy+rotation+scale) theo nguyên tắc memory project-stereo-alignment; --apply --stereo dùng biến thể này; test trên righ.jpg OK

## Test result
(chờ user xác nhận)
