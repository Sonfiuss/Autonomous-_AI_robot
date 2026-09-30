---
id: 2026-09-29_big-object-landmarks
status: testing     # 23:45 approve; user sau đó: chấp nhận vật (−0.01, −1.04), THÊM nhãn person vào realroom (làm ở bước 2b)
module: vision (drive_map landmarks)
started: 2026-09-29
---

## Task
Vật lớn (người) đứng / nằm yên một chỗ nhưng xe di chuyển thấy từng phần (đầu … chân) ở các frame khác nhau → bản đồ
hiện nhiều "person" giả. Gộp về đúng một vật, không gộp nhầm vật nhỏ khác nhau.

## Input
- User 2026-09-29 (sau khi publish map 20260929_231612): "vật lớn như người vẫn đứng yên 1 vị trí, khi robot di chuyển
  nhiều frame bắt được các bộ phận từ đầu tới chân → nhiễu nhiều người".
- Sự thật đã kiểm (run 231612, 213 lần thấy person, scratchpad, không ghi vào run):
  - Landmark = đáy hộp YOLO (giữa cột) chiếu xuống sàn (`row_sightings`) → cụm tham lam bán kính 0.3 m / lớp, ≥ 3 lần
    (`cluster_landmarks`); chỉ vẽ trên map.png / map.mp4 + `objects.json` (scene / realroom không đọc landmark).
  - Ảnh gốc: người **nằm trên giường gỗ thấp** dọc bên phải xe, dài ~1.6 m → 5 cụm person: (2.31, −0.71) ×193 chân
    (đáy hộp = mặt nệm), (1.96, −0.97) ×5 hộp cả người (rộng 420 px, frame 1220/1759), (2.70, −0.85) ×5 chỉ bàn chân ở
    mép phải tại B (đáy hộp nhảy 304–358 px khi xe đứng yên), (0.65, −0.49) ×3 đầu + thân gần (đáy hộp 476–478 px =
    sát mép dưới ảnh, `FOOT_EDGE_PX` 2 không chặn), (−0.01, −1.04) ×4 vật tối cao sát mép phải (frame 1265/1704) — CHƯA
    RÕ là người thứ hai hay khăn / rèm.
  - Ba cơ chế: M1 đáy hộp sát mép dưới (chân ngoài ảnh) → gần hơn thật; M2 vật không đứng trên sàn (nằm trên giường)
    / chân bị che → đáy hộp cao hơn sàn → xa hơn thật, đổi theo góc nhìn; M3 vật dài bị thấy từng phần → nhiều cụm.
  - Thử offline: (a) mép 8 px: 5 → 4 cụm person; (b) + gộp cụm cùng lớp mà MỘT hộp đã bao cả hai (tâm cụm chiếu vào
    khoảng cột của hộp ± 10 px) VÀ cách nhau ≤ bề rộng hộp tính ra mét tại tầm của nó + 0.15 m: **4 → 2** (người ×206
    + vật ở (−0.01, −1.04)); 3 chai cùng hướng nhìn vẫn riêng (hộp chai ~0.07 m); toilet 2 → 1 (cách 0.15 m). Không có
    điều kiện bề rộng thì 4 cụm chai bị gộp nhầm thành 1.
  - Thử "dò tia tới vật cản đầu tiên" (chân = mép giường): không giảm số cụm (tách cụm chính làm đôi 2.30 / 1.91) →
    không đưa vào plan.

## Expected output
1. Run 231612 dựng lại: map.png / objects.json còn **1 person** cho người nằm trên giường (+ vật (−0.01, −1.04) nếu đó
   thật là người), có `extent` (hộp bao các phần đã thấy); số chai / ghế / vali không đổi.
2. `objects.json` mỗi landmark thêm `extent_m` [xmin, ymin, xmax, ymax] và `parts` (số cụm đã gộp).
3. Test offline: người nằm thấy từng phần từ 3 pose → 1 landmark; 2 chai cùng hướng cách 1 m → 2; hộp đáy cách mép
   dưới 5 px → bỏ; check_landmarks cũ vẫn đạt.

## Plan (chờ duyệt)
- [ ] 1. `drive_map.FOOT_EDGE_PX` 2 → 8 (mép hộp YOLO dao động vài px; 8 px ≈ 1.7 % chiều cao ảnh).
- [ ] 2. `cluster_landmarks(sightings, boxes, …)`: sau khi phân cụm tham lam (trước lọc ≥ 3 lần), union-find gộp các cụm
      cùng lớp mà một hộp đã bao cả hai (điều kiện ở Input), rồi mới lọc ≥ LANDMARK_MIN_SIGHTINGS; vị trí = trung vị mọi
      thành viên; thêm `extent_m`, `parts`. `object_sightings` trả thêm các hộp (pose, cột, bề rộng m). Hằng mới:
      `MERGE_MARGIN_PX` 10, `MERGE_WIDTH_SLACK_M` 0.15.
- [ ] 3. Test trong `vision/test_drive_map.py` (3 ca của EO3); chạy lại test_drive_map, test_map_check, test_realroom.
- [ ] 4. Dựng lại 231612 (+ publish lại: cùng map id → giữ pose realroom), in bảng landmark trước / sau cho user.
- [ ] 5. Docs (interfaces: objects.json `extent_m` / `parts`; vision README nếu có mục landmark) +
      `/code-standards-review` + `/code-logic-review`.
- Ngoài phạm vi: nhãn "person" cho vùng giường trong scene (phiếu nhãn vẫn đặt ở chân chiếu, rơi sau mép giường vào ô
  chưa thấy) → CM "đi tới người" chưa tìm được; YOLO nhận nhầm lớp (giường = "bench", "boat", "toilet").

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[23:40] intake + plan | info: 5 cụm person của 1 người nằm trên giường; mép 8 px + gộp theo hộp → 2; prototype trong scratchpad (combined_proto.py)
[23:45] approved + user mở rộng: chấp nhận vật (−0.01, −1.04); **thêm nhãn person vào realroom** (bước 2b mới); tự publish + chat trên web realroom → task riêng
[23:55] step 1–2: FOOT_EDGE_PX 8; Sighting + BoxView (lens, bearing 2 mép ± MERGE_MARGIN_PX, bề rộng m); greedy_clusters / in_view / merge_by_box (union-find) / cluster_landmarks + extent_m, parts; OccupancyMap.cell_index | info: 231612 person 5 → 2 (×206 parts 5 + ×4), chai 3 cụm giữ nguyên, toilet 2 → 1
[00:00] step 2b (nhãn realroom): support_points (7 tia dưới 60 % giữa đáy hộp → ô vật cản đầu tiên > 10 cm trước điểm sàn) → phiếu đặt lên đó; scene_export: _label tính phiếu trong 2 ô quanh vùng (LABEL_REACH_CELLS), đường Hough có ≥ WALL_OBJECT_MIN_VOTES 10 của một lớp → vật riêng từ ô của chính đường đó (_object_region), không trộn với vật trước nó | risk (đã sửa): lần đầu mép giường đổi thành vật bị dính vùng chai trước giường → nhãn "bottle" (135 vs 108) → giữ dải của đường, vật = ô của đường | info: mép giường bị Hough coi là TƯỜNG 1.1 m (tường không nhận nhãn) + 2/3 phiếu person rơi trên ô trống ngay trước mép → không vùng nào nhận person; A/B cùng bản đồ 4 run: không nhãn nào mất; 231612 + person (mép giường 1.15 m, 81 %) + bench + bottle; 184936 đường 1.45 m bên trái → suitcase (62 %)
[00:03] step 3: test_drive_map + check_big_object_landmarks (người nằm 3 phần → 1 landmark 3 phần; 2 chai cùng hướng → 2; đáy cách mép 5 px bỏ / 10 px giữ) + check_support_votes (phiếu lên mép giường, vùng person / bottle / wall) 14/14; đột biến: tắt support_points / wall→object / merge → test FAIL đúng chỗ | info: test_map_check, test_mapping, test_object_color, test_vision, test_realroom: 0 FAIL, pyflakes sạch
[00:05] step 4: drive_map 231612 --publish (cùng map id → giữ pose 1.71, 1.91, +11.6°): realroom 17 vật, **obj_14 person** (mép giường, free side left), obj_5 person (vật nhiễu đã chấp nhận), obj_7 bench, 2 bottle, chair, toilet; web :5002 phục vụ bản mới
[00:08] step 5 (docs): interfaces (nhãn vùng, objects.json extent_m / parts), realroom README "Nhãn vật lớn", vision README

[00:08] standards-review (2 pass; drive_map, scene_export, occupancy_map, test_drive_map): 8 sửa, 0 hoãn | box_view `[y2] * 4` → theo số cột + đổi tên `left` bị dùng hai nghĩa → `side`; logger.debug số phiếu đặt lên vật đỡ + mỗi lần gộp cụm; scene_export UNKNOWN_CLASS thay "unknown" viết thẳng; test: `k` không dùng → `_`, SAMPLES_PER_CELL / BOTTLE_TURNS_DEG / EDGE_BOX_* / EDGE_PROBE_PX (dò mép quanh FOOT_EDGE_PX thay 5 / 10 viết thẳng), tách biểu thức chai dài | info: test_drive_map 14/14, pyflakes sạch

[00:14] logic-review (2 pass; đối chiếu plan 1–5 + 2b, EO 1–3, interfaces, overview; chưa có vision_plan.md → task file là plan): 2 sửa, 0 báo user | (1) LOGIC: _label giãn từng vùng 2 ô → một phiếu giữa hai vật gần nhau tính cho CẢ HAI (vật không nhãn cạnh chai nhận "bottle") → _vote_areas: ô trong 2 ô chỉ thuộc vật GẦN NHẤT (distanceTransformWithLabels, 1 lần / bản đồ); dòng Hough vẫn xét phiếu quanh nó (_near_cells, đĩa bán kính 2) để quyết định tường hay cạnh vật, _walls trả mask cạnh vật → gán nhãn cùng các vùng khác; (2) HIỆU NĂNG: cluster_landmarks lọc lại mọi sighting cho mỗi lớp → dùng thành viên của các cụm lớp đó | info: 231612 chai thứ hai (3.33, −0.15) về unknown như luật cũ (phiếu gần chai kia hơn), person mép giường 0.80, bench, suitcase (184936) giữ; không run nào mất nhãn so với luật cũ; publish lại 231612 (pose giữ); docs sửa theo; test_drive_map 14/14, test_mapping / test_realroom / test_object_color 0 FAIL, pyflakes sạch
[00:15] review gate — standards trên phần sửa: 1 sửa | NO_OWNER thay −1 viết thẳng 4 chỗ trong _vote_areas | info: 14/14, test_mapping 0 FAIL
[00:19] review gate — standards (2 pass, skill) trên phần sửa của logic-review (scene_export _near_cells / _vote_areas / _label / _object_region / _walls / extract_regions; drive_map cluster_landmarks): 3 sửa, 0 hoãn | extract_regions thiếu docstring (hàm public, nay trả walls + vật cạnh + vật) → thêm; `masks` (vừa là cạnh vật vừa là cụm) → `object_masks`; logger.debug số tường / vật / cạnh vật | info: test_drive_map 14/14, test_mapping 0 FAIL, pyflakes sạch
[00:21] review gate — logic (2 pass, skill) trên cùng phần: 1 sửa, 0 báo user | hai định nghĩa "trong 2 ô" lệch nhau: _near_cells (quyết tường / cạnh vật) giãn bằng ellipse 5×5 gồm cả ô (1, 2) cách √5, _vote_areas lấy L2 ≤ 2 → _near_cells cũng distanceTransform L2 ≤ LABEL_REACH_CELLS | info: đúng: mask cạnh vật ⊂ dải, cụm ngoài dải → không chồng; ô có chủ thuộc chính nó; thành viên cụm = sightings của lớp; nhãn 231612 / 184936 không đổi (bản đồ đã publish vẫn đúng); 14/14, test_mapping 0 FAIL
[00:15] → testing: user reload http://192.168.1.90:5002 → obj_14 person ở mép giường; map.png run 231612 còn 2 vòng person

## Test result
