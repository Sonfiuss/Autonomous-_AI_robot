---
id: 2026-07-02_stereo-ruler-mono-scale
status: executing
module: deepmap
started: 2026-07-02
---

## Task
Dùng stereo (2 camera) CHỈ làm "thước đo" để chuyển depth relative của Depth Anything V2
(monocular) sang metric tuyệt đối + đồng bộ scale giữa các frame. Map3D vẫn dựng hoàn toàn
bằng monocular; stereo chỉ đóng góp 2 tham số affine (s, t) mỗi frame.

## Input
- deepmap MONO-only (DA-V2), đã bỏ hướng stereo-làm-map vì nhiễu (camera lệch + chất lượng thấp)
  → xem memory [[project_deepmap_mono_only]].
- Vấn đề gốc: DA-V2 là affine `d_pred = s·(1/Z) + t`, pipeline hiện dùng `k/d` (bỏ shift t) →
  sai hệ thống, sàn "cong" ở xa (scale_calib.py, ground_profile_residual đang vá hậu kỳ).
- User SẼ cung cấp config 2 camera: baseline B, tiêu cự f, hệ số méo (distortion), độ lệch.
- Nguyên tắc chốt: "quý hồ tinh" — chỉ cần vài chục điểm stereo VÀNG là đủ giải (s, t).
- Điểm plug-in: giữa bước `depth → back-project` trong build_map.py / explore_map.py.

## Expected output
- Module mới `depth-anything/src/stereo_ruler.py`: nhận (frame trái+phải, config, d_mono) →
  trả về `(s, t)` robust + Z_abs dense = `s/(d_mono − t)`.
- build_map.py / explore_map.py có option `--scale-mode stereo-ruler` dùng Z_abs này thay cho
  k/d hiện tại; map PLY + BEV giữ nguyên định dạng.
- Test: trên một cặp ảnh stereo mẫu, in ra (s, t), số điểm inlier, và so sánh Z_abs tại vài
  điểm với đo tay → sai số chấp nhận được; map3D mono không méo scale giữa các frame.

## Plan
- [ ] 1. **Nhận + validate config camera.** Đọc config (B, f, k1..k3/p1/p2, độ lệch) do user
      cung cấp → chuẩn hoá về dạng OpenCV (cameraMatrix, distCoeffs, R, T). Xác định rig là
      CỨNG (calibrate-once đủ) hay XÊ DỊCH (cần floor-pin kèm) → chọn nhánh bước 6.
- [ ] 2. **Undistort + rectify.** Từ config dựng rectify maps; ép epipolar về ngang. Giới hạn
      vùng làm việc ở TÂM ảnh (méo ít, overlap chắc). Kiểm tra rectify bằng vài epipolar line.
- [ ] 3. **Chọn điểm texture mạnh (anchor candidates).** Shi-Tomasi/Harris min-eigenvalue cao;
      spatial bucketing (lưới ô, mỗi ô 1–2 điểm) để ĐA DẠNG độ sâu; loại cạnh thuần + vùng phẳng.
- [ ] 4. **Match sang camera phải + subpixel.** Tìm dọc cùng dòng (epipolar), block-match NCC
      trong [d_min, d_max]; parabola subpixel refine disparity.
- [ ] 5. **Lọc confidence (khử nhiễu).** Left-right consistency (±1px) + peak uniqueness (ratio
      đỉnh 1/đỉnh 2) + NCC-min + chỉ giữ disparity lớn (gần–trung, nơi baseline ngắn đáng tin).
      Kỳ vọng còn ~vài chục điểm sạch.
- [ ] 6. **Giải thước (s, t).** Z_stereo = f·B/disparity; ghép (d_mono, 1/Z_stereo); RANSAC/Huber
      fit trong MIỀN inverse-depth (1/Z), trọng số theo NCC + độ gần. Nếu rig XÊ DỊCH: dùng stereo
      cho consistency tương đối + floor-height (metric_floor sẵn có) pin 1 hằng số tuyệt đối.
- [ ] 7. **Áp cho toàn frame + fallback.** Z_abs = s/(d_mono − t). Nếu số inlier < ngưỡng hoặc
      residual cao → fallback floor-anchor; cross-check s_stereo vs s_floor để bắt lỗi/slip.
- [ ] 8. **Tích hợp vào pipeline.** Thêm `--scale-mode {kd, floor, stereo-ruler}` vào build_map.py
      + explore_map.py; thay đoạn dựng Z metric bằng stereo_ruler khi bật. Giữ PLY/BEV format.
- [ ] 9. **Test + đo sai số.** Cặp ảnh stereo mẫu → in (s,t)+inliers; so Z_abs vài điểm với đo tay;
      dựng map 360 và kiểm tra frame không lệch scale nhau (vật trùng thay vì "3 bản sao").

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[03:40] step 1: config alignment có từ task [[2026-07-02_chessboard-align-tool]] → calib/alignment.yml (warp_stereo: khử dy=102.6px + rot 1.17°, GIỮ disparity) | info: chưa có f·B (user chọn "để sau") → scale ở đơn vị disparity-px, chưa metric
[03:45] step 2-6: viết `depth-anything/src/stereo_ruler.py` (pilot trên cặp 3 left.jpg/righ.jpg): warp_stereo → golden points (Shi-Tomasi bucketed + NCC row-match ±1 dòng + subpixel + LR-consistency + uniqueness) → DA-V2 vits (CPU) → RANSAC fit d_mono = s·disp + t
[03:50] step 2-6: lần 1 chỉ 12 golden/5 inlier → nới d_max 96→140, grid 16x10, ±1 row, ncc 0.70 → 51 golden/24 inlier, s=0.0970 t=0.582 rms=0.085 | risk: cụm outlier disp 60–140px là match sai trên sàn granite lặp vân + vùng occlusion cận cảnh — RANSAC loại đúng
[04:20] step 1-2 LÀM LẠI ĐÚNG: user đo baseline B=5.4cm → phát hiện warp 2D không đủ: rig THỰC TẾ toe-in (yaw +2.14°, tilt +3.61°, roll −0.87°), baseline NGHIÊNG ~27° (right cam ở phải +4.8cm VÀ dưới +2.5cm) → epipolar XIÊN, row-match chỉ đúng ở 1 depth | info: 2 bàn cờ thật ra cùng ở ~0.9m (Z=0.91/0.96m) nên check "consistent theo depth" trước đó không có giá trị; ô bàn cờ = 4.37cm
[04:25] step 1-2: tool mới `stereo-camera/tools/stereo_calibrate_2view.py`: stereoCalibrate (fx=1124px joint, K cố định) + neo scale bằng |T|=5.4cm + stereoRectify alpha=-1 (alpha=0 degenerate: focal nổ 166000px) → calib/stereo_rectify.yml, f·B=60.69 px·m; kiểm tra visual: epipolar ngang chuẩn ở mọi depth
[04:30] step 3-7: stereo_ruler.py chuyển sang pipeline rectified (remap 2 ảnh + mono map; fallback warp cũ nếu thiếu yml, có cảnh báo metric); label mét trên inlier; kết quả cặp 3: 58 golden/19 inlier, s=0.0621 t=−2.442, Z hợp lý (tường ~1.2m, bình nước ~0.9-1.05m, kẹp 0.72m — khớp board 0.91m cùng mặt bàn) | risk: fx=1124 sai số ~±15% → Z scale theo; cần đo tay 1 khoảng cách để verify
[03:52] step 7 (một phần, ĐÃ THAY BẰNG RECTIFIED Ở TRÊN): dense pseudo-disparity map OK (tường xa thấp, sàn gradient mượt, vật gần cao); output tại depth-anything/output/stereo_ruler/ (scale.yml, golden_points.csv, overlay, scatter, npy) | info: Z_metric = f·B/disp_px khi có f·B; fit (s,t) tự hấp thụ offset ngang không đổi của mounting

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
