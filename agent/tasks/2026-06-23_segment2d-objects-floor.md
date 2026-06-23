---
id: 2026-06-23_segment2d-objects-floor
status: testing
module: stereo-camera
started: 2026-06-23
---

## Task
Khoanh vùng vật thể bằng xử lý ảnh 2D thuần (KHÔNG dùng depth/stereo) trên ảnh
camera trái. Mục tiêu: bo viền các vật thể; phần còn lại sau khi loại vật thể chính
là NỀN SÀN. Mỗi bước xử lý xuất 1 ảnh trung gian để user kiểm tra + tinh chỉnh.

## Bối cảnh / khó khăn (user nêu)
- Sàn nhà có hiệu ứng ĐỐM (speckle granite/terrazzo) + ÁNH SÁNG KHÔNG ĐỀU
  (góc tối/sáng khác nhau) -> ngưỡng màu đơn giản sẽ thất bại.
- Ý tưởng: model nền sàn -> cái gì KHÁC nền = vật thể -> bo viền -> phần còn lại = nền.
- Vật thể trong ảnh test: chân bàn (nâu, mịn), cáp đen/trắng, tay máy (bạc + dây
  đỏ/vàng), vài vật trắng. Sàn = xám đốm.

## Input
- Ảnh đơn: `stereo-camera-AD-Census/captures/left_20260610_001156.jpg` (640x480).
- Tool mới `tools/segment2d.py` (độc lập, chạy ~1s, KHÔNG cần binary AD-Census).

## Expected output (mỗi stage 1 file, prefix vd `captures/seg`)
- `_01_input.png`      ảnh làm việc (bản sao trái)
- `_02_illum.png`      chuẩn hóa ánh sáng (làm phẳng sáng không đều)
- `_03_floorseed.png`  vùng nền tự lấy mẫu (overlay) để dựng model màu sàn
- `_04_floorprob.png`  heatmap "độ giống nền" (khoảng cách màu + texture đốm)
- `_05_objmask_raw.png` mặt nạ "không phải nền" thô
- `_06_objmask.png`    mặt nạ vật thể đã làm sạch (morphology, lấp lỗ, bỏ blob nhỏ)
- `_07_objects.png`    viền + bounding box vật thể trên ảnh gốc
- `_08_background.png` NỀN cô lập (đã bỏ vật thể -> còn lại sàn)
- Flags tinh chỉnh từng bước: `--floor-dist`, `--min-obj-area`, `--morph`,
  `--tex-win`, `--illum {clahe,divide,none}`.

## Ý tưởng thuật toán (chốt khi duyệt)
- Sàn đốm = TEXTURE TẦN SỐ CAO; vật mịn (chân bàn, cáp) = phương sai cục bộ THẤP.
  -> dùng cả (a) khoảng cách màu so với model nền trong kênh Lab a/b (ít nhạy sáng),
     và (b) bản đồ phương sai cục bộ để tách đốm sàn khỏi bề mặt mịn.
- Lấy mẫu nền tự động từ vùng lớn + chạm biên (giả định sàn chiếm phần lớn khung),
  dựng phân phối màu (mean/cov) -> Mahalanobis -> floor prob.
- Object mask = nghịch đảo floor prob, làm sạch bằng open/close + fill + lọc diện tích.
- Nền = nghịch đảo object mask.

## Plan
- [x] 1. Tạo `tools/segment2d.py` khung CLI + load ảnh + ghi `_01_input.png`.
- [x] 2. Stage chuẩn hóa ánh sáng (`--illum`): CLAHE / divide-by-gaussian -> `_02`.
- [x] 3. Tự lấy mẫu nền + dựng model màu Lab(a,b) -> `_03_floorseed.png`.
- [x] 4. Floor prob = khoảng cách màu + phương sai texture -> `_04_floorprob.png`.
- [x] 5. Ngưỡng -> object mask thô `_05`; làm sạch morphology/diện tích -> `_06`.
- [x] 6. Bo viền + bbox vật thể `_07_objects.png`; cô lập nền `_08_background.png`.
- [ ] 7. Chạy trên ảnh test; user xem từng stage và tinh chỉnh ngưỡng. (đã chạy 1
       lần, chờ user soi + chỉnh threshold)
- [x] 8. `run_segment2d.sh` chạy một phát + liệt kê output.

## Open questions (RESOLVED)
- Ưu tiên: MẶT NẠ NỀN SÀN (`_08_background.png`).
- Chỉ khoanh vùng, KHÔNG phân loại/đặt tên vật thể.

## Decisions / key facts
- Tool độc lập 2D, KHÔNG đụng stereo/binary; chạy ~1s. Chạy bằng
  `C:\msys64\mingw64\bin\python tools/segment2d.py` (hoặc `./run_segment2d.sh`).
- Cốt lõi: floor_like = colour_like(Mahalanobis Lab a/b so model nền) * tex_like
  (local std L). Sàn đốm = texture cao -> giữ là nền; vật mịn cùng màu xám vẫn lộ ra
  vì texture thấp. Model nền tự lấy từ ĐỈNH histogram a/b (nền chiếm đa số -> seed 79%).
- Chuẩn hóa sáng mặc định = divide-by-gaussian (làm phẳng sáng không đều).
- Kết quả lần 1: seed nền 79%, object mask 26%, 8 vùng vật. `_08` tách nền sạch.
- Hạn chế quan sát: cáp trắng trên sàn còn lẫn nền; mảng tường mịn trên cùng bị tính
  là vật (đúng nghĩa "không phải sàn" nhưng không phải obstacle). Chỉnh bằng flags.

## Execution log
<!-- [HH:MM] step N: <action> | risk: <note> | info: <fact> -->
[--:--] step 1-6: viết tools/segment2d.py (8 stage output) | info: floor_like=colour*texture
[--:--] step 7: chạy ảnh test | info: seed 79%, obj 26%, 8 vùng; _08 tách nền OK
[--:--] step 8: thêm run_segment2d.sh (EXTRA= để truyền flag) | info: tự dò python cv2

## Test result
<!-- Pass / Fail / Partial -->
