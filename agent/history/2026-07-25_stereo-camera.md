# Session 2026-07-23..25 — stereo-camera / Astra Open3D SLAM

## Task
`agent/tasks/2026-07-23_astra-open3d-slam.md` (status: testing) — dựng map live
cho robot di chuyển tự do dùng depth Astra Pro (OpenNI2) + Open3D pose-graph SLAM
(ICP + loop closure). Bản trung gian trước khi đầu tư RTAB-Map ROS2.

## Đã implement
Tạo mới `depth-anything/src/astra_slam.py` (import astra_cloud, KHÔNG sửa nó):
- LiveSource (OpenNI2, có WARMUP 10 frame bỏ đầu) / ReplaySource (.npy) / record().
- make_pcd → voxel down + normals + FPFH (FPFH chỉ tính ở keyframe cho nhanh).
- icp_odom point-to-plane; phát hiện mất tracking (FIT_MIN 0.30 / RMSE_MAX 0.06).
- Tích luỹ map = cloud ĐÃ downsample (không phải full 240k) → nhanh; voxel IN-PLACE
  mỗi 10 frame (rebind object làm đơ Visualizer — đã fix bằng `map_pcd.points = ds.points`).
- Guard frame degenerate: <100 pts sau downsample → skip (tránh crash
  OrientNormals "No normals"); cảnh báo <2000 pts (camera dí quá gần).
- optimize_pose_graph offline (RANSAC-FPFH loop edge + global_optimization).
- read_wheel_odom() stub cho ESP32 (step 7); CLI đủ 12 cờ (T8 pass).

## Test đã chạy
- `--help` (T8) OK, defaults khớp bảng ngưỡng.
- Offline replay tổng hợp + 120 frame thật đã record (`rec/`, valid ~78%): pipeline
  chạy hết không exception. Sau tối ưu accumulate: **6.5–8.3 FPS** (>5 ngưỡng).
- intrinsics ReplaySource dùng default 554/554/320/240 (chưa self-calib khi replay).

## Vấn đề còn mở (QUAN TRỌNG cho lần sau)
1. **Live chưa xoay xong 1 vòng thành công.** Nhiều lần restart. Lần cuối
   (task bcfiyq9id) depth stream CHẾT giữa chừng: ~3900 frame liên tiếp 0 pts
   (SKIP). Nghi: Astra Pro rớt USB/stream timeout khi chạy lâu, HOẶC camera bị
   che. Cần: thêm cơ chế phát hiện "N frame liên tiếp 0 pts → dừng sạch + cảnh báo
   reconnect" thay vì spin vô hạn.
2. Nhiều lần record/replay ở background bị exit code 127 (shell wrapper), không
   phải lỗi script — cần chạy foreground hoặc cd tách riêng.
3. Loop-closure (T5) CHƯA verify trên vòng kín thật (chưa quay xong vòng nào).
4. Camera hay bị dí quá gần → <300 pts/frame, ICP trượt. Nhắc user giữ ≥1 m.

## Việc tiếp theo
- Fix "dead-stream detection" (điểm 1) rồi cho user quay lại 1 vòng chậm ≥1 m.
- Xem `astra_map_live.ply` bằng mắt: tường/sàn không nhân đôi.
- Chốt FIT_MIN/RMSE_MAX từ log ICP thực (mới thấy 1 dòng fit=0.11 → LOST đúng).
- Khi user xác nhận map OK: set task done, cập nhật `agent/plan/deepmap_plan.md`.
