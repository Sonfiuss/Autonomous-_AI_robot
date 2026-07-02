---
id: 2026-07-02_depth-review-bugfix
status: testing
module: deepmap
started: 2026-07-02
---

## Task
Sửa 3 bug nặng nhất phát hiện trong review pipeline depth (mono DA-V2), theo thứ tự
đã xếp hạng trong review. User đã duyệt cả 3, chọn giải pháp tốt nhất cho từng lỗi.

## Input
- Review 2026-07-02: build_map.py / scale_calib.py / explore_map.py / obstacle_grid.py.
- B1: build_map height mode + `--voxel/--icp` → điểm đi vào o3d_clouds, `all_pts/all_cls`
  rỗng → BEV bị skip IM LẶNG (điều kiện `if height_mode and all_cls:`).
- B2: `classify_height` trong build_map không có ngưỡng trên `h_max` → trần nhà / vật
  trên đầu robot bị gán obstacle, đổ vào BEV (obstacle_grid.classify CÓ h_max = 2.0).
- B3: `scale_calib.apply_scale` clamp metric depth về `[0, d_max]` → mọi điểm xa dồn
  đúng d_max = "vỏ cầu" điểm giả quanh robot trong PLY + BEV.

## Expected output
- B1: BEV luôn được xuất ở height mode, kể cả khi bật `--voxel/--icp`.
- B2: build_map có `--h-max` (default 2.0 m); điểm cao hơn → unknown (0), không phải obstacle.
- B3: điểm ngoài d_max bị LOẠI (invalid = 0.0, quy ước ROS depth) thay vì clamp;
  không phá slam/depth_node (node này đã dùng 0.0 = invalid sẵn).

## Plan
- [x] 1. B3 — `scale_calib.apply_scale`: thay clip trên bằng `metric[metric > d_max] = 0.0`;
      điểm 0 tự bị near-clip của `project_metric` (0.26 m) và `depth_node` loại bỏ.
- [x] 2. B2 — build_map: thêm arg `--h-max` (default 2.0); `classify_height(..., h_max)`;
      obstacle chỉ khi `h_obs < height <= h_max` (khớp semantics obstacle_grid.classify).
- [x] 3. B1 — build_map: khi `o3d` active + height mode, vẫn append `wpts`/`cls` vào
      `all_pts`/`all_cls` (BEV cần class per-point mà merge o3d không giữ được).

## Execution log
[22:15] intake: tạo task file, user đã approve 3 fix từ review → status executing
[22:20] step 1: apply_scale bỏ clamp trên, set >d_max = 0.0 | risk: caller mới của apply_scale phải hiểu 0=invalid | info: slam/depth_node đã dùng 0.0=invalid sẵn → tương thích
[22:20] step 2: build_map +--h-max, classify_height nhận h_max, obstacle = (h_obs, h_max] | info: default 2.0 khớp obstacle_grid
[22:20] step 3: build_map append wpts/cls cả nhánh o3d khi height_mode → BEV không mất | info: PLY vẫn từ o3d merge, BEV từ điểm thô đã phân loại
[22:25] verify: py_compile OK cả 2 file; unit test apply_scale pass (far→0.0, near giữ nguyên) → status testing, chờ user chạy trên Jetson

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
