---
id: 2026-07-23_astra-open3d-slam
status: testing
# live pipeline verified after USB replug; RGB(mode A)+debug-log+snapshot added;
# closed-loop visual check still pending before -> done
module: stereo-camera
started: 2026-07-23
---

## Task
Dựng map trực tiếp (live) cho robot di chuyển tự do, dùng depth Astra Pro
(OpenNI2) + Open3D pose-graph SLAM (ICP + loop closure). Bản trung gian/kiểm
chứng trước khi đầu tư RTAB-Map (ROS2) — con đường B đã chốt với user.

## Input
- `depth-anything/src/astra_cloud.py` (đã có): open_depth_stream / read_depth_mm /
  calibrate_intrinsics (fx,fy,cx,cy tự-calib) / backproject / save_ply_xyz.
- Open3D 0.18.0 đã cài trên `D:\University\Python\Python310\python.exe`.
- Astra Pro: depth-only 640x480@30fps (không RGB qua OpenNI2). Chưa cắm odom ESP32.
- Robot di chuyển tự do → cần registration + loop closure, không dùng angle-merge.

## Expected output
- Script mới `depth-anything/src/astra_slam.py` (KHÔNG sửa astra_cloud.py — import
  lại các hàm từ nó).
- Chạy live: stream depth, ghép frame dần thành 1 map lớn, hiển thị map đang lớn
  dần trong cửa sổ Open3D; kết thúc lưu `map.ply`.
- User xác nhận: cầm camera đi 1 vòng phòng → map.ply ghép đúng hình học, tường/
  sàn không bị nhân đôi lệch nhiều.

## Plan chi tiết

Ký hiệu: **[o3d]** = open3d.geometry / pipelines.registration. Đơn vị nội bộ
= MÉT (chia depth_mm/1000). Voxel mặc định `v = 0.03 m`.

Chiến lược test: mỗi step có testcase chạy ĐỘC LẬP bằng dữ liệu tĩnh trước
(record vài frame ra .npy) rồi mới test live — để tách lỗi thuật toán khỏi lỗi
phần cứng/stream.

### Step 0 — Bộ dữ liệu test tĩnh (làm trước để test các step sau)
- Mục tiêu: có sẵn chuỗi ~30 depth frame để test offline lặp lại được.
- Công cụ: thêm chế độ `--record N out_dir/` vào astra_slam (hoặc script phụ
  `record_depth.py`): lưu `frame_000.npy` (uint16 mm) mỗi ~5 khung hình khi cầm
  camera đi chậm 1 đoạn.
- Testcase T0: `--record 30 rec/`; kiểm tra có 30 file .npy, mỗi file shape
  (480,640) dtype uint16.
- Ngưỡng: ≥ 60% pixel mỗi frame có depth>0 (nếu thấp hơn → camera che/xa quá,
  ghi cảnh báo).

### Step 1 — Khung script + tái dùng astra_cloud
- Mục tiêu: `astra_slam.py` import `open_depth_stream, read_depth_mm,
  calibrate_intrinsics, backproject, save_ply_xyz` từ astra_cloud. Mở stream +
  calib intrinsics 1 lần. Có nguồn frame trừu tượng: live (OpenNI2) HOẶC
  `--replay rec/` (đọc .npy) để test offline.
- Công cụ: argparse, numpy, import module cùng thư mục.
- Testcase T1: `--replay rec/ --max-frames 1 --no-display` → in
  `[slam] intrinsics fx≈.. cx≈..` và `frame 0: N pts`.
- Ngưỡng: fx,fy ∈ [500,620]; cx ∈ [300,340]; cy ∈ [220,260] (đã verify Astra
  Pro); N > 100_000 điểm depth>0.

### Step 2 — Frame → PointCloud + preprocess
- Mục tiêu: hàm `make_pcd(depth_mm) -> (pcd_down, fpfh)`:
  backproject→ Nx3 (mét) → `o3d.geometry.PointCloud`,
  `voxel_down_sample(v)`, `estimate_normals(radius=2v, max_nn=30)`,
  `orient_normals_towards_camera_location([0,0,0])`,
  FPFH `compute_fpfh_feature(radius=5v, max_nn=100)`.
- Công cụ: [o3d] PointCloud, registration.compute_fpfh_feature.
- Testcase T2: trên frame replay: in số điểm trước/sau downsample.
- Ngưỡng: sau voxel 3cm còn 8_000–60_000 điểm; 100% điểm có normal (không NaN).
  Nếu >150k sau downsample → tăng voxel, cảnh báo cấu hình.

### Step 3 — Odometry ICP frame-to-frame (point-to-plane)
- Mục tiêu: `icp_odom(src_down, tgt_down, init) -> (T_4x4, fitness, rmse)`
  dùng `registration_icp(..., TransformationEstimationPointToPlane,
  max_correspondence_distance=1.5v, criteria max_iteration=50)`. Init = pose
  tương đối trước đó (constant-velocity) hoặc odom ESP32 (step 7) hoặc identity.
- Xử lý mất tracking: nếu `fitness < FIT_MIN` HOẶC `rmse > RMSE_MAX` →
  coi là mất tracking: KHÔNG cập nhật pose bằng T này, giữ pose cũ, tăng bộ đếm
  `lost`; nếu `lost >= 5` liên tiếp → dừng/cảnh báo "mất tracking, đi chậm lại".
- Công cụ: [o3d] pipelines.registration.registration_icp.
- Testcase T3a: ICP giữa frame i và i (chính nó) → T≈I. Ngưỡng: ‖T−I‖ dịch <
  1mm, góc < 0.1°, fitness > 0.95.
- Testcase T3b: ICP giữa frame i và i+1 (camera nhích nhẹ) → translation trả về
  cùng hướng chuyển động thực. Ngưỡng: fitness > 0.6, rmse < 0.5·v.
- Ngưỡng chốt tham số: **FIT_MIN = 0.30**, **RMSE_MAX = 0.06 m** (2·v). Tinh
  chỉnh lại sau khi xem log T3b thực tế.

### Step 4 — Tích luỹ map + quản RAM
- Mục tiêu: `T_world = T_world_prev @ T_icp`; `map += (pcd_full_frame).transform
  (T_world)`; sau mỗi K=10 frame `map = map.voxel_down_sample(v)`.
- Công cụ: [o3d] PointCloud.transform, voxel_down_sample.
- Testcase T4: replay 30 frame; đo số điểm map cuối + RAM.
- Ngưỡng: map < 3_000_000 điểm cho 1 phòng (nếu vượt → voxel to hơn hoặc
  down-sample thường xuyên hơn). Không có exception OOM.

### Step 5 — Loop closure + pose-graph (bản đầu: OFFLINE lúc kết thúc)
- Mục tiêu: giảm drift khi quay lại chỗ cũ. Bản đầu làm ĐƠN GIẢN:
  1. Lưu keyframe mỗi khi di chuyển ≥ `KF_DIST=0.3 m` hoặc xoay ≥ `KF_ANG=15°`
     kể từ keyframe trước (lưu pcd_down + fpfh + T_world).
  2. Xây `o3d.pipelines.registration.PoseGraph`: node = pose keyframe; edge
     tuần tự (odometry, uncertain=False) từ ICP.
  3. Loop edge: với mỗi cặp keyframe (i,j) mà `‖t_i − t_j‖ < KF_DIST·2` và
     j−i > 5 → thử `registration_ransac_based_on_feature_matching` (FPFH) rồi
     refine ICP; nếu `fitness > LOOP_FIT=0.4` → thêm edge uncertain=True.
  4. `global_optimization(GlobalOptimizationLevenbergMarquardt,
     GaussNewton criteria, option(max_correspondence_distance=1.5v,
     edge_prune_threshold=0.25, reference_node=0))`.
  5. Ghép lại map bằng pose đã tối ưu → ghi map.ply.
- (Nâng cấp sau: chạy loop-closure định kỳ trong lúc live thay vì chỉ cuối.)
- Công cụ: [o3d] registration_ransac_based_on_feature_matching, PoseGraph,
  global_optimization.
- Testcase T5: quay 1 vòng kín về gần điểm đầu; so sai lệch pose đầu–cuối
  TRƯỚC và SAU optimization.
- Ngưỡng: sau optimization, khoảng cách pose đầu–cuối giảm ≥ 50% so với trước;
  tường trong map.ply không nhân đôi > 1 voxel (~3cm) nhìn bằng mắt.

### Step 6 — Hiển thị live
- Mục tiêu: `o3d.visualization.Visualizer` non-blocking:
  `create_window`, `add_geometry(map)`, mỗi frame `update_geometry` +
  `poll_events` + `update_renderer`. `--no-display` bỏ qua toàn bộ.
- Công cụ: [o3d] visualization.Visualizer.
- Testcase T6: chạy replay có `--display`; cửa sổ hiện map lớn dần, không crash,
  đóng cửa sổ → thoát sạch (stream.stop + unload).
- Ngưỡng: ≥ 5 FPS vòng lặp ở voxel 3cm trên máy hiện tại (nếu < 2 FPS → tăng
  voxel / giảm tần suất render).

### Step 7 — Chỗ cắm odom ESP32 (chuẩn bị, chưa bắt buộc chạy)
- Mục tiêu: `--odom` bật; `read_wheel_odom() -> T_rel_4x4` (stub trả I + TODO).
  Nếu bật, dùng T_rel làm `init` cho ICP (step 3) thay identity → ICP chỉ tinh
  chỉnh, robust hơn nhiều khi xoay nhanh.
- Công cụ: pyserial (chưa nối), protocol O/T/F/M/S (xem
  agent/description/interfaces.md).
- Testcase T7: `--odom` với stub → chạy y hệt không odom (init=I), không lỗi.
  Nối serial thật để test sau (ngoài phạm vi bản này).
- Ngưỡng: không có (stub). Ghi TODO rõ ràng.

### Step 8 — CLI + ghi chú
- Cờ: `--replay DIR`, `--record N DIR`, `--out map.ply` (mặc định
  depth-anything/output/astra_map.ply), `--voxel 0.03`, `--max-frames`,
  `--icp-fitness-min 0.30`, `--icp-rmse-max 0.06`, `--keyframe-dist 0.3`,
  `--keyframe-ang 15`, `--loop-fit 0.4`, `--no-display`, `--odom`.
- Ghi chú đầu file: bản TRUNG GIAN/kiểm chứng; bản chính thức = RTAB-Map ROS2
  (agent/tasks/2026-06-28_rtabmap-slam.md). Port Jetson: OpenNI2 linux/arm64.
- Testcase T8: `--help` liệt kê đủ cờ; giá trị mặc định đúng như trên.

## Bảng ngưỡng tổng hợp (tinh chỉnh sau khi có log thực)
| Tham số | Giá trị đầu | Ý nghĩa |
|---------|-------------|---------|
| voxel v | 0.03 m | độ phân giải map / downsample |
| ICP max_corr_dist | 1.5·v = 0.045 m | bán kính ghép điểm ICP |
| FIT_MIN | 0.30 | dưới ngưỡng = mất tracking, bỏ frame |
| RMSE_MAX | 0.06 m | trên ngưỡng = ghép tệ, bỏ frame |
| lost liên tiếp | 5 | cảnh báo/dừng |
| KF_DIST / KF_ANG | 0.3 m / 15° | tạo keyframe mới |
| LOOP_FIT | 0.40 | chấp nhận loop-closure edge |
| map cap | 3.0 M điểm | trần RAM 1 phòng |
| FPS tối thiểu | 5 (report nếu <2) | hiệu năng live |

## Thứ tự thực thi đề xuất
Step 0 → 1 → 2 → 3 (test T3a/T3b kỹ, chốt FIT_MIN/RMSE_MAX từ log) → 4 → 6
(xem live sớm) → 5 (loop closure) → 7 → 8.

## Plan (checklist)
- [x] 0. Record dữ liệu test tĩnh (.npy) — `--record N DIR` implement xong (cần HW để chạy)
- [x] 1. Khung script + tái dùng astra_cloud + nguồn live/replay
- [x] 2. Frame → PointCloud + normals + FPFH
- [x] 3. ICP odometry + phát hiện mất tracking
- [x] 4. Tích luỹ map + quản RAM
- [x] 5. Loop closure + pose-graph offline
- [x] 6. Hiển thị live non-blocking
- [x] 7. Hook odom ESP32 (stub)
- [x] 8. CLI + ghi chú + help

## Execution log
<!-- [HH:MM] step N: <summary> | risk: <...> | info: <...> -->
[--:--] step 1-8: viết `depth-anything/src/astra_slam.py` (import astra_cloud, KHÔNG sửa) | info: LiveSource/ReplaySource/record; make_pcd+FPFH; icp_odom point-to-plane; tích luỹ map + voxel mỗi 10 frame + cap 3M; optimize_pose_graph (RANSAC-FPFH + global_optimization); Visualizer non-blocking; read_wheel_odom() stub
[--:--] step 8: T8 `--help` liệt kê đủ 12 cờ, giá trị mặc định đúng bảng ngưỡng | risk: none
[--:--] test offline: dựng 12 frame .npy tổng hợp, `--replay --no-display` chạy hết pipeline không exception (intrinsics→backproject→ICP→map→map.ply 606k pts) | info: Windows python KHÔNG hiểu path MSYS /c/... phải dùng path C:\\; loop-closure chưa kích hoạt vì slab phẳng <3 keyframe → cần geometry phòng thật để test T5

## Test result
