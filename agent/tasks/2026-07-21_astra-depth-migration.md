---
id: 2026-07-21_astra-depth-migration
status: planning
module: stereo-camera
started: 2026-07-21
---

## Task
Bỏ hẳn hướng depth bằng mono (depth-anything) và stereo thủ công (stereo-camera C++);
thay bằng camera Orbbec Astra Pro làm nguồn depth. Đầu ra cần: point cloud 3D + phát
hiện vật cản / vùng đi được.

## Input
- Phần cứng: Orbbec Astra Pro đã cắm (USB `2bc5:0501` = depth sensor Orbbec,
  `2bc5:0403` = UVC/serial; RGB là UVC camera thường trên /dev/video*).
- Driver: OpenNI2 đã cài sẵn ở hệ thống (`/usr/lib/libOpenNI2.so`, `/usr/include/openni2`,
  `/etc/openni2/OpenNI.ini`). CHƯA có Python binding.
- Quyết định của user:
  - Xóa: `depth-anything/`, `stereo-camera/`. Giữ: `area-detection/`, `deepmap/`.
  - Module depth Astra mới viết bằng **C++ + OpenNI2**.
  - Downstream: **point cloud 3D** + **phát hiện vật cản/vùng đi được**.

## Ràng buộc phát hiện được (QUAN TRỌNG — ảnh hưởng plan)
- `deepmap/` (GIỮ) phụ thuộc sâu vào `depth-anything/src`: import
  `merge_360`, `depth_to_3d(_timed)`, `obstacle_grid`, `drive_area`, `stereo_ruler`.
- `slam/calibrate_scale.py` và `slam/ros2_ws/src/depth_anything_node/` cũng import
  từ `depth-anything/src` + model `.pth`.
- Các helper trên **import DepthAnythingV2 ở top-level** → không tách sạch được; phải
  TRÍCH các hàm không-phụ-thuộc-model ra file riêng trước khi xóa, nếu không deepmap + slam vỡ.
- Hàm model-free tái dùng được cho pipeline Astra: `depth_to_points`, `fit_floor_plane`,
  `write_ply`, `transform_to_world`, `icp_merge`, `build_occupancy`/`classify`.

## Kiến trúc đề xuất
```
astra-depth/                 (C++ + OpenNI2, thay depth-anything + stereo-camera)
  src/astra_capture.{h,cpp}  mở device OpenNI2, đọc depth (mm) + color, registration D→C
  src/point_cloud.{h,cpp}    depth + intrinsics → back-project → ghi .ply
  src/obstacle_bev.{h,cpp}   height-above-ground → BEV occupancy + vùng đi được
  apps/astra_scan.cpp        CLI: chụp → point cloud .ply
  apps/astra_obstacle.cpp    CLI: depth → grid occupancy / drive-area
  CMakeLists.txt             link OpenNI2 + OpenCV
common/geom/                 (Python model-free helpers trích từ depth-anything)
  depth_geom.py              depth_to_points, fit_floor_plane, write_ply, transform_to_world, occupancy
```

## Quyết định cần user chốt trong lúc duyệt plan
1. Số phận các script mono của deepmap (build_map/explore_map/stereo_walk_map/replay_shots):
   chúng vốn CHẠY BẰNG mono depth. Đề xuất: **retire** (chuyển sang `deepmap/legacy/`),
   giữ lại phần mapping tái dùng (world transform + BEV + PLY merge) qua `common/geom`.
2. `slam/ros2_ws/src/depth_anything_node/`: xóa hay thay bằng `astra_depth_node`?
   Đề xuất giai đoạn sau (không nằm trong scope lần này) — chỉ đảm bảo không vỡ build.

## Expected output
- `astra-depth/` build được: `astra_scan` xuất `.ply` point cloud từ Astra Pro,
  `astra_obstacle` xuất grid vật cản/vùng đi được.
- `depth-anything/` và `stereo-camera/` đã xóa; deepmap + slam vẫn import được
  (trỏ sang `common/geom`), không ImportError khi chạy phần được giữ.
- Tài liệu `agent/description/` + `interfaces.md` cập nhật (bỏ 2× USB stereo cam,
  thêm Astra Pro + OpenNI2).

## Plan
- [ ] 1. Xác nhận Astra Pro stream được qua OpenNI2 (test `NiViewer`/mẫu C++ đọc 1 frame depth), ghi lại intrinsics + resolution + registration mode.
- [ ] 2. Tạo `common/geom/depth_geom.py`: trích các hàm model-free từ depth_to_3d/merge_360/obstacle_grid/drive_area (không kéo theo torch/DepthAnythingV2). Thêm test import.
- [ ] 3. Rewire import của `deepmap/` + `slam/calibrate_scale.py` sang `common/geom`; các script mono chuyển vào `deepmap/legacy/` (giữ chạy được nếu model còn, nhưng không chặn xóa).
- [ ] 4. Dựng khung `astra-depth/` (CMake + OpenNI2 link) và `astra_capture` đọc depth+color, in ra frame test.
- [ ] 5. `point_cloud` + app `astra_scan`: depth → .ply; verify mở được trong viewer.
- [ ] 6. `obstacle_bev` + app `astra_obstacle`: height-above-ground → BEV occupancy/drive-area (port logic obstacle_grid).
- [ ] 7. Xóa `depth-anything/` và `stereo-camera/`; grep toàn repo đảm bảo không còn tham chiếu treo (trừ agent/history + legacy).
- [ ] 8. Cập nhật docs: `project_overview.md`, `interfaces.md`, readme các module; viết `agent/plan/astra-depth_plan.md`.

## Execution log
<!-- One line per event. [HH:MM] step N: <action> | risk: <note> | info: <fact> -->
[22:50] pivot: user gác mono (archive để sau, KHÔNG xóa); ưu tiên #1 = test output Astra.
[22:50] step 1: viết astra-depth/test/astra_probe.cpp (OpenNI2→grab depth→lưu png), build OK.
[22:52] step 1: probe chạy → OpenNI2 devices=0. info: driver PS1080 stock (VID 1d27), Astra=2bc5:0501 chưa có udev/quyền ghi (/dev/bus/usb/001/008 root:root 0664). risk: có thể cần Orbbec OpenNI2 SDK nếu chmod không đủ.
[23:05] step 1: xác nhận stock driver KHÔNG nhận 2bc5 kể cả sau chmod → cần Orbbec OpenNI SDK. Tải OpenNI 2.3.0.86 arm64 (orbbec/OpenNI_SDK) vào astra-depth/vendor/. info: gói zip lồng RAR v5 → cần `unrar` (unar fail solid rar5).
[23:20] step 1: staged Orbbec redist → astra-depth/lib/ (libOpenNI2.so + OpenNI2/Drivers/liborbbec.so). Build probe vs SDK. udev 557-orbbec-usb.rules (2bc5→0666) đã cài → node 0666 OK.
[23:26] step 1: OpenNI2 ENUMERATE OK "Astra|Orbbec" nhưng open() USB transfer timeout. info: depth = interface vendor-specific của 2bc5:0403 If0 (libusb); 2bc5:0501=RGB UVC (/dev/video0).
[23:33] step 1: cắm thẳng (chỉ còn hub nội bộ camera). Vẫn timeout. dmesg TRỐNG (không over-current/reset) → LOẠI giả thuyết nguồn.
[23:40] step 1: bật OpenNI verbose log (astra-depth/test/last_openni_diag.log). info: libusb claim iface0 OK, đọc được FW=5.8.22. If0 chỉ có 2 bulk-IN 0x81/0x82 (KHÔNG có bulk-OUT) → lệnh đi qua control transfer. FAIL: control transfer chập chờn — "Get version timeout" rồi retry OK, "Received NACK:2", "Get mode failed timeout" khi đọc COLOR modes (Astra Pro color nằm ở UVC 0501, không phải device này). risk: (a) cáp/tín hiệu marginal, (b) driver beta6 vs FW5.8.22 không hợp, (c) driver kẹt ở enum color mode. TODO: thử cáp khác; plan B tải OpenNI 2.3.0.63 known-good cho Astra Pro.

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
