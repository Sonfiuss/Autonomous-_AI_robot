# 2026-07-13 — stereo-camera: calib v2 từ eval session, promote ACTIVE

## Bối cảnh
User cung cấp 6 cặp chessboard (left/right_{81,82,97,112,120,180}, z=cm trong tên,
chụp tối 12/07 với camera NHÌN THẲNG — hardware change, trước đó cúi 30°).
Board tường 19x10 inner corners. Task: `agent/tasks/2026-07-11_stereo-accuracy.md`.

## Chuỗi phát hiện (mỗi bước do user đính chính/cung cấp số đo)
1. Calib đầy đủ từ 6 cặp → fx nổ 2560. Chẩn đoán ban đầu "toàn frontal" SAI —
   user đính chính các cặp có góc lệch (PnP yaw −43..+26° xác nhận).
2. Truy tiếp → **board ô KHÔNG vuông** (user đo: 4 ô = 9.0cm ngang / 10.4cm dọc
   → ô 22.5×26.0mm) và hơi cong (homography residual 0.5–1.2px).
3. Tỷ lệ pixel ảnh 1.101 vs tỷ lệ ô 1.156 → **pixel camera không vuông
   fy/fx≈0.95** (tape 0.955/0.957, Zhang 0.944/0.940 — độc lập, cả 2 cam).
4. Cặp _112 outlier thật (rig xê dịch giữa 2 lần grab lần lượt): dy held-out
   −13.7px, Z err +3.6% → loại khỏi mọi ước lượng.

## Kết quả — calib plan-B v2 (ACTIVE, user duyệt promote)
- `calib/stereo_rectify.yml` = bản 20260713 (cũ 02/07 → archive/
  stereo_rectify_ACTIVE_pre20260713_*.yml).
- K fx≠fy neo thước dây: L 1088.9/1040.3, R 1097.7/1050.6; pp=center, dist=0.
- R: tilt +5.54°, yaw +0.51°, roll −2.76°; |T|=5.54cm square-trusted (~đo tay 5.4).
- fb=57.91 danh định; **fb_measured=61.29 → Z=61.29/disp**, d0 dư chỉ +0.27px.
- Kiểm định thước dây: 5 cặp sạch +1.8/−3.2/+0.7/−0.7/−1.2% (mục tiêu ≤3%).
- Epipolar dy p50 0.57–2.05px (calib cũ: 22–123px trên cùng ảnh).
- Báo cáo trực quan: https://claude.ai/code/artifact/1310644e-96cb-44c2-b6e3-b1d0581539f1

## Code mới/sửa
- **`pitch_from_board.py`** (NEW): pitch/yaw/roll từng cam từ board tường
  (solvePnP, +pitch=cúi, `--square-y-mm` cho ô chữ nhật) + dy per góc;
  selftest PASS 0.0000°. Dùng cho servo sweep (đang hoãn theo user).
- **`depth_eval.py` FIX** (dòng ~371): write_fb ghi fb_median kèm d0 lstsq
  (2 model trộn nhau → Z bias) → nay ghi fb_lstsq khi |d0|>0.3.

## Giới hạn calib v2
Tin cậy vùng GIỮA khung hình, 0.8–1.8m, tư thế nhìn thẳng. Distortion chưa mô
hình hóa (board tường không đủ chuẩn). Ảnh/PLY trước tối 12/07 (hướng cúi cũ)
KHÔNG tương thích — calib cũ khớp ảnh cũ.

## Pending
1. Session calib 20–25 cặp board nghiêng ±30° phủ 4 góc — chờ user có board
   chuẩn (khuyến nghị PDF 9x6 25mm in, dán phẳng) → distortion + gate p90<0.5px.
   Nếu dùng board tường: stereo_calibrate_2view cần thêm hỗ trợ ô chữ nhật.
2. Servo pitch sweep (hoãn): p00/p10/p20/p30, robot đứng yên → pitch_from_board.
3. Stage E replay: cần walk shots MỚI (hướng nhìn thẳng).

## Interface changes
- yml active đổi nội dung (schema per-camera, fx≠fy, fb_measured mới 61.29 —
  cũ 60.69). Key mới `square_y_m`. Mọi consumer qua calib_io không cần sửa.
