---
id: 2026-07-23_orbbec-dual-capture
status: testing
module: stereo-camera
started: 2026-07-23
---

## Task
Viết script đọc + hiển thị/lưu ảnh RGB từ camera Orbbec Astra Pro (mono, độc lập —
KHÔNG ghép stereo).

## Input
- Ban đầu tưởng là 2 camera Orbbec riêng biệt → điều tra thực tế: user chỉ có
  **1 camera Astra Pro** (VID_2BC5&PID_0501, "Astra Pro HD Camera" trong Device
  Manager).
- Đã thử cài `pyorbbecsdk` (PyPI 1.3.2 = build macOS sai, gỡ bỏ) rồi
  `pyorbbecsdk2` 2.1.1 từ GitHub release chính thức (win_amd64/cp310, import
  name vẫn là `pyorbbecsdk`) — **kết luận: OrbbecSDK v2 KHÔNG hỗ trợ Astra Pro**
  (theo README chính thức: "Astra Pro Plus: limited maintenance ở SDK v1,
  not supported ở SDK v2"). Driver Windows đã cài đúng (obdrv4 v4.3.0.22,
  OpenNI-protocol) nhưng SDK v2 vẫn lỗi `setXu failed / propertyId 1000` khi
  đọc device info — không phải lỗi cấu hình, camera này không nằm trong danh
  sách hỗ trợ.
- **Hướng đi đúng: bỏ pyorbbecsdk hoàn toàn.** Astra Pro tự lộ ra Windows như
  1 UVC webcam màu chuẩn — `cv2.VideoCapture(0, cv2.CAP_DSHOW)` đọc RGB
  640x480 thành công (đã test). Không cần SDK/driver đặc biệt cho luồng màu.
- Mono, không ghép stereo — không đụng `stereo_cloud.py`/`stereo_ruler.py`.

## Expected output
- Script mới, độc lập, style giống `area-detection/capture_chessboard.py` nhưng
  1 camera: mở `cv2.VideoCapture` tại index camera Astra Pro, hiển thị live
  preview, SPACE lưu ảnh PNG đánh số, q thoát.
- User xác nhận: chạy script trên Windows, thấy cửa sổ preview màu từ Astra Pro,
  SPACE lưu được file ảnh.

## Update (2026-07-23, sau khi test sâu hơn)
User thực ra muốn dùng depth sensor của Astra Pro để **thay thế cụm stereo-matching
thủ công + DA-V2** hiện tại (không chỉ đọc RGB). Đã cài + verify xong toàn bộ chain
trên Windows:
- Driver: `obdrv4 v4.3.0.22` (đã có sẵn, đúng bản chính thức từ
  `github.com/orbbec/OpenNI_SDK` release `openni-device-windows-driver-4.3.0.22.zip`).
- OpenNI2 SDK Windows: giải nén từ
  `OpenNI_2.3.0.86_..._beta6_windows_x64.zip` (cùng release, có bản `arm64`/`linux_x64`
  dùng được cho Jetson sau này) vào `C:\Users\admin\Downloads\openni_sdk\extracted\...`.
- Thiếu `MSVCR120.dll` (VC++ 2013 runtime) → đã cài `vcredist_x64.exe` (Microsoft
  official) → hết lỗi load `OpenNI2.dll`.
- Python binding: `pip install openni` (package `openni`, import `from openni import
  openni2`). Phải gọi `os.add_dll_directory(<thư mục chứa OpenNI2.dll>)` trước
  `openni2.initialize(...)` trên Windows, nếu không LoadLibrary báo thiếu dependency.
- **Verify thành công:** depth stream 640x480@30fps, pixelFormat
  `ONI_PIXEL_FORMAT_DEPTH_1_MM` (đơn vị mm/pixel). Device URI thực tế của phần depth
  là `vid_2bc5&pid_0403` (khác PID với phần color `pid_0501` đã thấy trong Device
  Manager) — Astra Pro lộ ra Windows như **2 thiết bị USB riêng**: color (UVC,
  đọc bằng cv2) và depth (OpenNI2, đọc bằng `openni2`).
- Astra Pro KHÔNG có auto-registration RGB-D (không giống Gemini/Femto đời mới) →
  cần tự align/warp 2 luồng nếu muốn RGB-D chồng khớp.

## Update 2 (2026-07-23) — quyết định cuối: THAY THẾ HẲN code cũ, KHÔNG tô màu cloud
- Đã thử `oniCoordinateConverterDepthToColor` (hardware depth↔color registration
  của OpenNI2) → **lỗi cứng `ONI_STATUS_ERROR`**, và color stream qua OpenNI2 trên
  Astra Pro treo vô thời hạn ở `read_frame()` (không bao giờ trả frame — Astra Pro
  không xuất RGB qua OpenNI2, đúng như ghi nhận cộng đồng Orbbec).
- User quyết định: **bỏ qua việc tô màu cloud**. Point cloud chỉ cần XYZ từ depth
  Astra Pro, không ghép ảnh màu.
- `oniCoordinateConverterDepthToWorld` (chỉ cần depth stream, không cần color) đã
  verify OK — dùng hàm này để back-project mọi pixel depth (u,v,mm) → XYZ (mm),
  thay hoàn toàn cho bước back-project bằng P2 rectify + toàn bộ stereo-matching/
  DA-V2 mono-scale trong `stereo_cloud.py`/`stereo_ruler.py`.
- **Code cũ (`depth-anything/src/stereo_cloud.py`, `stereo_ruler.py`,
  stereo-camera rectify/align) coi như legacy/thay thế hẳn** theo module mới này —
  không xoá file (giữ tham khảo) nhưng không còn là pipeline chính.

## Plan (final)
- [x] 1. Tạo `depth-anything/src/astra_cloud.py` (module mới, KHÔNG sửa
      stereo_cloud.py/stereo_ruler.py):
      - `open_depth_stream()`: `os.add_dll_directory(OPENNI2_REDIST)` +
        `openni2.initialize(...)` + `dev.create_depth_stream()`.
      - Đọc 1 frame depth (`VideoFrame` → numpy uint16 mm, 640x480).
- [x] 2. Back-project: với mỗi pixel (u,v) có depth>0, gọi
      `oniCoordinateConverterDepthToWorld(depth_handle, u, v, z_mm, ...)` (vectorize
      bằng cách gọi hàm C cho từng pixel — nếu chậm, cache theo lưới rồi
      nội suy, hoặc tự suy fx/fy/cx/cy tuyến tính từ vài điểm mẫu đã verify
      [(320,240)->(0,0), (0,0)->(-561,421), (639,479)->(559,-419)] rồi
      back-project bằng công thức pinhole thường cho toàn frame — nhanh hơn
      nhiều so với gọi C API 307200 lần/frame).
- [x] 3. Xuất PLY (xyz only, không màu, đơn vị mm hoặc đổi mét cho khớp
      `stereo_cloud.py` cũ) — tái dùng hàm ghi PLY hiện có trong
      `depth-anything/src` nếu có, hoặc viết writer PLY tối giản.
- [x] 4. Thêm `--display` (hiển thị depth colormap qua cv2.applyColorMap để
      debug live) và `--out` (đường dẫn .ply).
- [x] 5. Ghi chú cài đặt (driver `obdrv4` + OpenNI2 SDK Windows đã giải nén tại
      `C:\Users\admin\Downloads\openni_sdk\extracted\...` + vcredist x64 2013 +
      `pip install openni`) vào đầu file `astra_cloud.py`, kèm ghi chú port sang
      Jetson: dùng bản `linux_x64`/`arm64` cùng GitHub release
      `orbbec/OpenNI_SDK`, không cần vcredist (đó là Windows-only).
- [x] 6. Cập nhật `agent/plan/deepmap_plan.md`: đánh dấu stereo-matching + DA-V2
      mono-scale là legacy, nguồn depth chính chuyển sang Astra Pro/OpenNI2.

## Execution log
[14:10] step 0: điều tra pyorbbecsdk — không hỗ trợ Astra Pro | info: OrbbecSDK v2 bỏ Astra Pro, dùng cv2.VideoCapture(0) thay thế
[14:35] step 0: cài OpenNI2 SDK + vcredist x64 2013, verify depth stream 640x480@30fps qua python `openni` package | info: Astra Pro lộ 2 USB device riêng (color pid_0501 UVC, depth pid_0403 OpenNI2), không auto-register RGB-D
[15:05] step 0: test depth-to-color registration → lỗi cứng, color stream treo | info: Astra Pro không hỗ trợ RGB qua OpenNI2; user chốt bỏ màu, chỉ lấy XYZ qua oniCoordinateConverterDepthToWorld (đã verify đúng)
[exec] step 1-4: tạo astra_cloud.py — open_depth_stream/read_depth_mm/calibrate_intrinsics/backproject/save_ply_xyz + --out/--meters/--display | info: KHÔNG gọi C converter 307200x/frame; tự-calib pinhole fx/fy/cx/cy bằng 4 sample điểm từ convert_depth_to_world (X=a·u+b tuyến tính), rồi back-project vectorize numpy
[exec] step 3: save_ply_xyz binary LE xyz-only, mirror stereo_cloud.save_ply_xyz; --meters chia 1000 cho khớp đơn vị cloud cũ
[exec] step 5: ghi chú setup (obdrv4 + OpenNI2 SDK + vcredist + pip openni + os.add_dll_directory) và port Jetson (linux_x64/arm64, không cần vcredist) ở đầu file; OPENNI2_REDIST đọc từ env var
[exec] step 6: deepmap_plan.md thêm mục "Depth source" đánh dấu stereo-match + DA-V2 mono-scale là legacy, nguồn depth chính = Astra Pro/OpenNI2

## Test result
