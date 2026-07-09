---
id: 2026-07-09_visual-odometry-align
status: testing
module: deepmap
started: 2026-07-09
---

## Task
Walk map hiện đặt cloud theo dead-reckoning từ lệnh động cơ (không phát hiện
trượt bánh) → các stop lệch nhau, ICP cuối chỉ cứu được khi đã gần đúng.
User yêu cầu: dùng CHÍNH HÌNH ẢNH giữa 2 stop liên tiếp để đo chuyển động
thực (tiến thẳng ⇒ ảnh phóng to, quay ⇒ khung hình dịch trái/phải) và dùng
pose đó đặt cloud, số liệu động cơ chỉ làm giá trị khởi tạo/tham chiếu.

Cách làm tổng quát hơn heuristic "tỉ lệ zoom": match feature giữa 2 ảnh trái
liên tiếp; mỗi feature đã có depth metric từ pipeline hiện tại ⇒ mỗi cặp match
là một cặp điểm 3D↔3D; RANSAC + Umeyama rigid fit cho ra (R, t) đầy đủ —
khoảng cách tiến, góc yaw, cả lệch ngang — chính là điều zoom/shift phản ánh
nhưng chính xác và đo được độ tin cậy (số inlier, residual).

## Input
- deepmap/stereo_walk_map.py — FusionMapper + sequential path, to_world/Pose,
  icp_merge_clouds; mỗi stop đã có (left_rect, metric depth, golden points)
- depth-anything/src/stereo_ruler.py — fuse_metric trả depth metric per-stop
- Frame walk đã lưu ở deepmap/output/stereo_walk/ (dữ liệu validate offline)
- Lệnh động cơ (cm per hop) làm init + sanity bound

## Expected output
- deepmap/visual_odom.py: estimate_motion(prev_frame, cur_frame, K) →
  (R, t, n_inliers, rms) hoặc None khi không đủ tin cậy.
- stereo_walk_map: pose mỗi stop = pose VO nếu tin cậy, fallback dead-reckon;
  log mỗi stop in "vo: dist=X cm (cmd Y cm) yaw=Z° inl=N" — chênh lệch
  VO vs lệnh động cơ chính là số liệu calib trượt bánh.
- Cloud các stop chồng khít hơn trước khi ICP; ICP cuối vẫn giữ làm refine.
- Offline --test pass; validate trên frame đã lưu.

## Plan
- [x] 1. deepmap/visual_odom.py: ORB trên ảnh RIGHT-RECTIFIED (depth ở frame
      này, không phải ảnh trái raw) 2 stop liên tiếp; ratio-test match; nâng
      keypoint lên 3D metric + đưa về frame đã LEVEL (sàn Y=0) → chuyển động
      chỉ còn phẳng: yaw + tịnh tiến XZ (3 DOF, chắc hơn 6 DOF).
- [x] 2. RANSAC planar 2-point + 2D-Kabsch refine: trả (yaw, t), inliers,
      rms. Gate: ≥12 inlier, rms ≤ 4 cm, |yaw| ≤ 20°, |t| trong
      [0.3×, 2.5×] quãng lệnh động cơ.
- [x] 3. Wire vào FusionMapper (pipeline) + run_sequential; fallback KHÔNG
      dùng dead-reckon tuyệt đối mà chain BƯỚC dead-reckon tương đối lên pose
      đã gán trước đó (1 stop hỏng không làm map nhảy). CLI --no-vo,
      --vo-min-inliers; thêm --replay chạy lại từ shot đã lưu.
- [x] 4. Cross-check scale: median tỉ lệ khoảng-cách-giữa-các-điểm matched
      prev/cur (cảnh rigid ⇒ 1.0); lệch >15% = HARD reject + tag SCALE?.
      Lưu ý giới hạn: nén scale ĐỒNG ĐỀU cả 2 frame thì ratio vẫn =1,
      chỉ sanity bound theo lệnh động cơ đỡ được.
- [x] 5. Validate replay trên 3 bộ ảnh đã lưu (output/stereo_walk_vo/):
      stop1 REJECTED đúng (SCALE x0.79 — 2 frame lệch scale 21%, d=0.04 vs
      cmd 0.30); stop2 accepted d=0.136/yaw −7.3° (scale pair nhất quán nhưng
      data này depth vẫn nén ~0.4× nên chưa kết luận đúng/sai); icp fitness
      0.237/0.141 vs baseline 0.240/0.126 (ngang). --test regression PASS cả
      2 mode: baseline y hệt (golden=39, inl=2, anch=11, Z[0.35,1.70]); VO đo
      d=0.003–0.007 m với 1999/1999 inl trên ảnh lặp → tự loại theo bound.
- [ ] 6. Hardware walk (user): cùng protocol open-scene của task
      2026-07-09_walk-scale-fit (ĐO chiều cao camera trước), chạy A/B
      --no-vo vs mặc định; kỳ vọng dòng "vo: d=... (cmd ...)" ≈ nhau và
      overlap các stop tăng.

## Execution log
[--:--] steps 1-3: visual_odom.py (ORB + lift 3D leveled + planar RANSAC +
VOTracker với dr-relative fallback) + wire vào FusionMapper & sequential;
level_to_floor trả thêm (R, floor_y); CLI --no-vo/--vo-min-inliers/--replay
| risk: ORB có thể ít feature trên nền granite | info: depth ở right-rect
frame nên VO match trên right_rect, không phải ảnh raw
[23:20] step 5 iter 1: replay A/B trên 3 bộ ảnh — VO phát hiện scale lệch
21% giữa stop0/1 (đúng chẩn đoán walk-scale-fit) | risk: scale lệch mới chỉ
là warning, stop2 accepted trên depth nén | info: nâng scale thành hard gate
[23:23] step 5 iter 2 + regression: --test PASS pipeline+sequential, baseline
y hệt; VO đo d≈0 trên ảnh lặp với 1999/1999 inl rms 2-6mm → máy đo chính
xác, gate theo lệnh hoạt động | info: VO thêm ~0.1-0.2s/stop, không đáng kể
[23:50] addendum (user report: cloud nghiêng vì tilt chưa dùng để dựng):
level_to_floor giờ XOAY TRƯỚC theo tilt ước lượng per-frame (info['tilt_deg'])
rồi mới plane-fit tinh chỉnh; fit lệch >20° so với tilt = loại (bám nhầm đồ
đạc) → fallback tilt-only + floor_y = median điểm đáy khung. Kết quả trên 3
frame lưu: cam_h 0.28/0.27/0.24 (~0.32 thật, trước 0.15/0.11/0.06 do fit bám
mặt cong). Sàn xa vẫn DỐC XUỐNG trong cloud = depth nén làm cong tấm sàn —
không xoay nào sửa được, chỉ scale đúng mới hết | risk: fit_floor_plane
sample không seed → frame borderline có thể đổi mode plane-fit/tilt-only
giữa 2 lần chạy | info: --test baseline mới ~10.9k/9.6k pts, cam_h 0.26
(tilt-only)
[00:16] addendum 2 (user report: điểm phía trên/vật cản bị deep gần hơn
thực tế): floor_anchors giờ nhận golden làm nhân chứng metric — golden trong
dải cột trung tâm có Z_stereo < 0.6×Z_sàn(v) ⇒ vật cản; MỌI hàng phía trên
chân vật cản bị che sàn → cắt khỏi anchor (v_cut = max row + 10px).
fuse_metric thêm trust_beyond=1.5: pixel ngoại suy sâu hơn 1.5× bằng chứng
metric xa nhất (golden inlier + anchor) → UNRESOLVED, loại khỏi cloud (lỗ
trống trung thực thay vì tường giả). Kết quả 3 frame: cam_h 0.29/0.29/0.29
(hoàn toàn nhất quán), hết rác Z=121m; GIỚI HẠN CÒN LẠI: tường/hộp thật ở
~2m vẫn bị nén về ~0.8m vì frame không còn bất kỳ bằng chứng metric xa nào
(inl 2-5 điểm toàn gần) — chỉ cảnh có texture xa mới sửa được, không phải
thuật toán. --test PASS 2 mode, baseline mới: anch=5 (veto cắt 6 hàng nhiễm),
inl=3, rms 0.0637 (trước 0.38 — fit chặt hơn hẳn), Z[0.38,1.42],
~8.7k/8.4k pts

## Test result
<!-- pending -->
