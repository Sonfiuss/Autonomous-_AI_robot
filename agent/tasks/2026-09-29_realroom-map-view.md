---
id: 2026-09-29_realroom-map-view
status: testing     # user 09-29: phía trước = 1 bánh (W1 0°), 2 bánh ±120° → xoay hình simulation 180°
module: realroom (+ vision)
started: 2026-09-29
---

## Task
Bản đồ phòng thật dễ đánh giá: tương phản rõ vật cản / sàn trống / chưa thấy, xe vẽ đúng hình như simulation,
xuất PNG riêng; thêm màu cơ bản của vật vào Scene JSON (trường `color` của schema simulation).

## Input
- User 2026-09-29 (ảnh chụp viewer :5002, chế độ tối): bản đồ thiếu tương phản giữa vật cản và không gian trống;
  hình xe chưa đúng như simulation; cần một file PNG map riêng để đánh giá; bổ sung logic lấy màu cơ bản của vật vì
  JSON simulation có trường này.
- Sự thật đã kiểm:
  - Viewer `realroom/static/index.html` theo theme: tối → sàn `#243222`, chưa thấy `#2a2a28` (gần như trùng), vật cản
    `#c9c9c4`; xe = vòng tròn r 0.225 + mũi tên. Không có lưới / thước tỉ lệ / chú giải trên canvas.
  - Scene realroom `robot` chỉ có x, y, theta, radius — THIẾU `chassis` / `wheels` / `footprint` mà schema v2.0 có
    (`vision/scene_export.to_scene`).
  - Simulation (`room_generator.py`, KHÔNG sửa): khung lục giác 34 cm (rộng 24 cm ở +x, 10 cm ở −x), 3 bánh 8×3 cm
    cách tâm 0.21 m ở **60/180/300°**, hull lồi, r 0.225. Robot thật (project_overview, chốt 09-25): bánh W1 ở **0°**
    (+x trỏ vào W1), W2 120°, W3 240° → hình simulation xoay 180° mới khớp bộ bánh thật (mũi hẹp phía trước).
  - `color` = "unknown" cứng ở `scene_export.py:195`. Dữ liệu có sẵn mỗi run: `raw/*.jpg` (RGB 640×480, 294 frame),
    `floor/*.png` (nhãn pixel: 2 = vật cản) + `floor/index.jsonl` `contacts` (u, v) = hàng vật cản thấp nhất mỗi cột
    = chân vật → ô bản đồ qua `floor_hit` (24 phân tích ở run 222009). Hầu hết vật là "unknown" (không có hộp YOLO)
    → màu phải lấy từ pixel vật cản, không chỉ từ hộp YOLO.
  - Bảng màu simulation: red, orange, yellow, green, blue, brown, black, white, gray.
  - Có sẵn cv2, matplotlib 3.7.5, PIL 10.4.

## Expected output
1. `realroom/maps/map_<id>.png` (bản đồ như dựng) + `realroom/maps/latest.png` (vị trí robot hiện tại + vệt đi),
   tự cập nhật khi publish và sau mỗi lệnh; `python3 realroom/render.py [--map ID] [--out FILE]`.
   Sàn trống sáng, vật cản gần đen, chưa thấy xám có gạch chéo; lưới 0.5 m + thước tỉ lệ + chú giải; vật tô màu của
   nó + nhãn `obj_N class (color)`; xe = khung + 3 bánh + hull nét đứt + mũi tên hướng.
2. Viewer :5002 cùng bảng màu và cách vẽ xe như PNG (cả sáng lẫn tối), nút tải PNG.
3. Scene realroom `robot` có `chassis` / `wheels` / `footprint` (thân xe, bánh 0/120/240).
4. Mỗi vật có `color` (một tên trong bảng simulation, "unknown" khi thiếu bằng chứng) + `color_rgb` "#rrggbb"
   (trung vị) + tỉ lệ phiếu; drive_map ghi vào scene.json, `--publish` đưa sang realroom.
5. Test offline: màu thuần tổng hợp → đúng tên (kể cả đen / trắng / xám theo độ bão hoà), ít pixel → unknown;
   phiếu màu rơi đúng ô của chân vật; PNG: ba vùng tương phản đủ (ΔL ≥ 40), xe nằm đúng pose.

## Plan (chờ duyệt)
- [ ] 1. Hình xe thật: `realroom/robot_shape.py` lấy khung + bánh + hull từ `room_generator` (import, không sửa
      simulation) rồi XOAY 180° → bánh 0/120/240 như xe thật; `scene_export.to_scene` ghi `chassis` / `wheels`
      (`angle_deg` 0/120/240) / `footprint` vào `robot`; `map_store.publish_run` bổ sung cho map cũ chưa có.
- [ ] 2. Màu vật — `vision/object_color.py`: mỗi phân tích sàn dùng được (như `build_floor`), mỗi cột có contact:
      các pixel nhãn VẬT CẢN liền phía trên chân (tối đa ~80 hàng) → RGB từ `raw/<seq>.jpg` cùng seq → bỏ pixel quá
      tối / cháy sáng → phiếu theo ô của chân vật (tên màu HSV + tổng RGB). Tên màu: S thấp → black / gray / white theo V;
      còn lại theo hue (red / orange / yellow / green / blue), brown = cam–đỏ tối.
      `scene_export`: màu vật = tên có phiếu nhiều nhất trên các ô của vật nếu ≥ 50 pixel và ≥ 40 % phiếu, không thì
      "unknown"; `color_rgb` = trung vị. drive_map gọi, test với run 222009 (in bảng màu từng vật để user so thực tế).
- [ ] 3. Publish lại bản đồ 222009 có màu + hình xe, GIỮ vị trí robot hiện tại (publish cùng map id không reset pose;
      `--reset-pose` để đặt lại như trước).
- [ ] 4. `realroom/render.py`: vẽ PNG (cv2, 100 px/m, lề + chú giải); bảng màu cố định không theo theme:
      trống `#f4f4f0`, vật cản `#1e1e1e`, chưa thấy `#9a9a94` + gạch chéo; vật: viền + tô nhạt `color_rgb`; xe như
      simulation; vệt pose_history; plan nếu có. `map_store` gọi sau publish / save_pose → `map_<id>.png`, `latest.png`.
- [ ] 5. Viewer: cùng bảng màu (canvas giữ nền "giấy" cả ở chế độ tối), vẽ xe bằng chassis / wheels / footprint,
      lưới 0.5 m + thước, chú giải có ô màu, nhãn có màu vật; `GET /api/map/<id>.png` + nút "Tải PNG".
- [ ] 6. Test: `vision/test_object_color.py` + thêm ca render / robot_shape vào `realroom/test_realroom.py`; chạy
      lại test_drive_map, test_realroom.
- [ ] 7. Docs (realroom/README, interfaces: robot chassis/wheels ở realroom, `color_rgb`, PNG + endpoint) +
      `/code-standards-review` + `/code-logic-review`.
- Ngoài phạm vi: dùng màu để phân biệt vật khi ra lệnh ("đi tới cái ghế đen") — CM đã đọc trường `color`, rule của
  `goto.py` chưa; làm sau nếu user muốn.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[19:20] approved | user: phía trước xe = 1 bánh (W1), 2 bánh ±120° → hình simulation xoay 180°; không sửa bước nào
[19:25] step 1: `scene_export.robot_shape()` = khung / bánh / hull của room_generator xoay REAL_ROBOT_TURN_DEG 180 (bánh 0/120/240, xếp từ W1), `to_scene` ghi vào `robot` | info: bỏ ý "map_store bổ sung cho map cũ" — map cũ vẽ dự phòng bằng đĩa, map 222009 dựng lại ở bước 3
[19:50] step 2: `vision/object_color.py` ColorVotes (phiếu theo ô chân vật, 11 tên màu cơ bản theo HSV), drive_map `object_colors(dm)` (0.25 s / 24 phân tích), `to_scene(..., colors)` → `color` + `color_rgb` | risk (đã sửa): ảnh kiểm tra cho thấy GHẾ XANH LÁ ra "black" — floor_segment tô cả hộp YOLO trong hình thang = sàn / dây / bóng giữa chân ghế, nhãn vật cản không lên tới mặt ghế → chân ở đáy hộp (phần giữa 60 % bề rộng) leo tới đỉnh hộp + ưu tiên màu có sắc (≥ 25 % pixel; tường / sàn / bóng không có sắc) | info: run 222009: ghế green #2b5e21 (28 % xanh, 53 % đen = bảng đen sau ghế), vali black, obj_4 green (chậu xanh), obj_2 / obj_5 gray, obj_6 unknown (27 px)
[19:58] step 3: `map_store.publish_run(reset_pose=False)`: latest.json đã là map này, cùng map_offset → giữ pose + pose_history (khác offset → cảnh báo, đặt lại); CLI `publish --reset-pose`; `show` in màu | info: sao lưu realroom/maps vào scratchpad; `drive_map --publish` 222009: pose giữ (0.607, 0.734, −1.7°, 4 mục lịch sử), bánh 0/120/240, id vật không đổi
[20:05] step 4: `realroom/map_image.py` (tên thay cho render.py — tránh trùng module) draw / png_bytes / write_png (ghi nguyên tử) + CLI; bảng màu cố định trống #f4f4f0 / chiếm #1c1c1c / chưa thấy #84847e gạch chéo, lưới 0.5 m chỉ trên sàn trống, trục mét, chú giải + thước 1 m, nhãn vật có ô màu (tránh chồng nhãn), xe = hull nét đứt + khung + 3 bánh + mũi tên (map cũ: đĩa), vệt pose_history, plan + chấm đích; `map_store._draw` sau publish (map_<id>.png + latest.png) và save_pose (latest.png), lỗi vẽ chỉ cảnh báo | info: 914 × 591 px cho phòng 3.75 × 1.95 m
[20:15] step 5: viewer cùng bảng màu (canvas nền giấy cả chế độ tối), xe từ chassis / wheels / footprint, lưới clip theo ô trống, trục, chú giải HTML, nhãn có ô màu, ô vẽ tròn pixel (hết vệt); app: `GET /api/map/latest.png[?spot=]` (+ đường plan), `/api/map/<id>.png`, nút "Tải PNG" theo spot đang chọn | info: test client: 200 / 404 / 400 đúng; ảnh chụp Firefox headless (bản tĩnh nhúng dữ liệu) không lỗi JS
[20:35] step 6: `vision/test_object_color.py` 6/6 (tên 11 màu; phiếu đúng ô chân vật, trần FACE_ROWS, không phiếu ngoài tầm; ghế giả: không hộp → black, có hộp → green, đếm pixel lõi / cạnh hộp; ưu tiên sắc 30 % → green, 20 % → black, chia đều → unknown, 1 chân < MIN_PIXELS; hình xe W1 tâm (0.21, 0), trước rộng 0.10 / sau 0.244, bánh trong hull; to_scene color / color_rgb / robot fields); `test_realroom` +2 (publish lại giữ pose, reset_pose, khác khung → đặt lại, PNG có sau publish, latest.png vẽ lại khi robot đi; PNG: ΔL* ≥ 40 giữa trống / chiếm / chưa thấy, mỗi loại đúng chỗ, khung xe xanh, bánh W1 phía trước không có bánh phía sau) 5/5 | info: test_drive_map 11/11, test_vision 10/10, test_mapping 9/9, test_drive + test_leg_executor ALL OK, pyflakes sạch; bản đồ thật không bị test chạm
[20:40] step 7 (docs): realroom/README (PNG, màu vật, hình xe thật, publish lại giữ pose / --reset-pose, endpoint PNG, bảng màu L* 96 / 11 / 55), interfaces (scene: color / color_rgb / robot outline; publish_run reset_pose; PNG endpoints)
[20:45] standards-review (2 pass): 22 sửa, 0 hoãn | map_image: số viết thẳng → hằng (vạch chia, nhãn trục, chú giải, thước, tiêu đề, khoảng nhãn, ARROW_TIP_SHARE, THIN_PX / BOLD_PX, cv2.FILLED), `_ticks` dùng chung cho lưới + trục, bỏ mặc định 'unknown' thừa, docstring `_title` / `_text_size`; index.html: TICK_* / DASH / ARROW_HEAD_* / LABEL_* ; app `_png` docstring; test_object_color: FakeVotes (lambda gắn vào namedtuple) → class FixedVotes, RED_BGR / WHITE_BGR / FULL_SHARE / ONE_FOOT_COL / BLOCK_CELLS | info: ảnh PNG + trang web sau sửa giống hệt trước; test 6/6 + 5/5, pyflakes sạch
[20:58] logic-review (2 pass; đối chiếu plan 1–7, expected output 1–5, interfaces, comm_plan, history 09-29): 4 sửa, 1 báo user | (1) LỆCH PLAN bước 4 / EO1 "vật tô màu của nó": chỉ có ô màu trong nhãn → thêm lớp tô nhạt color_rgb (α 0.25: sàn trống dưới vật đen vẫn L* ~76, xa màu chưa thấy 55) ở PNG + web; (2) LỆCH EO4 "tỉ lệ phiếu": scene thêm `color_share` (null khi unknown), drive_map in ra, docs; (3) HIỆU NĂNG: nền nhãn chép CẢ ẢNH mỗi nhãn (map khám phá 50 vật × 5 MB) → trộn đúng vùng nhãn; (4) test_live_parser: khoảng --first/--last rỗng (hoặc --first 0) → chia 0 / IndexError sau khi đã tạo client → báo và thoát 1 trước khi gọi LLM | báo user: CM `candidates.py` kiểm chỗ đứng bằng `room_generator.robot_footprint` (hình simulation, bánh 60/180/300) — xe thật xoay 180°; có từ trước, ngoài phạm vi | info: lệch plan có chủ đích: 200 px/m thay 100 (5 mm/px, phòng 3.75 m → 914 px); publish lại 222009: pose giữ, color_share ghế 0.28; test 6/6 + 5/5, pyflakes sạch; review gate (standards + logic trên phần sửa): sạch
[21:05] review gate — standards (2 pass, skill) trên phần sửa của logic-review: 2 sửa, 0 hoãn | scene_export `SHARE_DIGITS` (confidence + color_share, trước `2` viết thẳng); test_live_parser `FIRST_CASE` (gốc đánh số ca) | info: wash / nền nhãn ROI / color_share / FixedVotes đạt; pyflakes sạch, test_object_color 6/6
[21:08] review gate — logic (2 pass, skill) trên cùng phần: 1 sửa, 0 báo user | test_live_parser: `--last` âm bị Python cắt từ cuối (--last −5 = ca 1..35 âm thầm) → --last < --first là khoảng rỗng, báo và thoát 1 | info: wash 1 bản sao / lần vẽ (điểm ngoài vật giữ nguyên qua addWeighted), nền nhãn ROI cắt biên ảnh, color_share null khi unknown, 9..24 vẫn 16 ca; test_realroom 5/5, test_object_color 6/6, pyflakes sạch

## Test result
