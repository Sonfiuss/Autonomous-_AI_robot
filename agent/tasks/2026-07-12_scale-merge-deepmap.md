---
id: 2026-07-12_scale-merge-deepmap
status: testing
module: deepmap
started: 2026-07-12
---

## Task
Đưa các frame DA-V2 về chung 1 scale (mét) bằng khoảng cách đo từ stereo, rồi merge point cloud. Chạy offline trên Windows từ 3 cặp shot_0/1/2 (12/07, stereo-camera/captures). Chấp nhận calib hiện tại chưa đạt (dy ~8px, fx ±15%) — user sẽ cung cấp chessboard và calib lại sau; script phải chạy lại được với yml mới.

## Input
- 3 cặp shot_0/1/2 (12/07 15:06): cùng cảnh, robot tiến dần (Z center 0.67→0.61→0.57m).
- calib/stereo_rectify.yml (fb=60.69, single-K) — đo hôm nay: dy p50 ~7.5px sau rectify.
- stop_0/1/2.ply cũ (09–10/07) KHÁC session — giữ nguyên, không re-scale (user đã duyệt).
- Sẵn có: stereo_ruler.fuse_metric (neo scale DA bằng golden points), visual_odom.VOTracker, DA-V2 vits weights, python D:\University\Python\Python310\python.exe.

## Expected output
- Script replay offline: từ mỗi cặp → DA-V2 + golden/SGBM anchors → fuse_metric → depth mét → stop_i.ply mới (session folder riêng, không ghi đè cũ).
- Bảng scale s_i + n_inliers per frame; kiểm chéo cùng-vật-cùng-khoảng-cách giữa 3 frame (% lệch).
- merged.ply từ VOTracker (+ ICP nếu fitness tốt).

## Plan
- [x] 0. Phân tích + đo baseline độ lệch 3 cặp (epipolar dy p50 7.5px / p90 17px; SGBM cov 12–26%) — user duyệt dùng tạm
- [x] 1. Script deepmap/replay_shots.py: rectify → golden points (dy_search nới) → DA-V2 → fuse_metric → Z_i (m); fallback anchor SGBM nếu inliers < 8
- [x] 2. Xuất stop_i.ply (mét, chung scale) vào depth-anything/output/pointcloud/session_20260712/
- [x] 3. Cross-check scale giữa 3 frame (cùng vật trừ quãng tiến) — báo % lệch
- [x] 4. Merge: VOTracker pose (yaw, tx, tz) + ICP refine → merged.ply + báo cáo

## Execution log
<!-- [HH:MM] step N: <summary> | risk: <if any> | info: <key fact> -->
- [15:40] step 0: epipolar + SGBM baseline trên 3 cặp; raw dy ~135px (rig cố định), sau rectify p50 7.5px | risk: golden inliers sẽ ít khi dy lớn | info: 3 shot = mini walk session tiến dần
- [16:05] step 1: replay_shots.py chạy lần 1 KHÔNG cam_h → stop_1/2 rejected (span<2.0, không floor anchors), stop_0 fit sai (ctr 0.16m) | info: BẮT BUỘC --cam-height-m để có floor anchors với cảnh sàn-gần
- [16:10] step 2-4: chạy lại --cam-height-m 0.32 (default robot) → cả 3 stop fuse OK, đều qua fallback SGBM anchors (golden inliers < 8 do dy 8px) | info: cam_h ước lượng từ floor fit = 0.300/0.296/0.297m — lệch nhau chỉ 1.3% → 3 frame ĐÃ về chung scale; VO stop_1 accept (d=8.4cm, rms 0.009), stop_2 reject vì scale ratio x0.80 → dead-reckon fallback (không nhiễm map)
- [16:10] output: session_20260712/{stop_0,1,2.ply, depth_N_m.npy, merged_walk.ply 13065 pts, report.csv}; PLY cũ 09-10/07 giữ nguyên

## Test result
<!-- Filled when user confirms -->
