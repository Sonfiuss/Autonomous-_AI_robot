---
id: 2026-09-28_natural-command
status: testing     # user 09-28: Gemini cho parser; Phần B = module realroom riêng (simulation không đổi)
module: communication
started: 2026-09-28
---

## Task
Robot hiểu lệnh lái tự nhiên hơn (tiếng Việt có / không dấu, gõ sai, tiếng Anh), hỏi lại có ngữ cảnh thay vì bắt
gõ lại từ đầu.

## Input
- User 2026-09-28: "hiện phần communicate quá đơn giản — tăng cường giúp robot hiểu ngữ cảnh tự nhiên hơn". Phiên lỗi:
  `demo_drive --provider rule --yes "tại chỗ quay trái 180 độ. rồi quay phải 360 độ rồi quay phải 180 độ để trở về vị
  trí ban đâu. sau đó đi thẳng 90 cm rôi quay lại 180 độ"` → hỏi "Chưa hiểu phần '180 do'" 3 lần; trả lời "180 độ",
  "degree", "quay tại chỗ", rồi cả câu tiếng Anh ("self rotating 180 degree by the left then selt rotating 360 degree
  by the right and self rotating 180 to back the base angel. After this, go straight 90cm and self rotating 180
  degree") đều không hiểu.
- Nguyên nhân đã kiểm (cmd_parser.py):
  1. `_TURN_AROUND_RE` bắt "quay lai" nhưng KHÔNG lấy góc sau nó → "180 do" còn lại → câu hỏi. Validator cũng bỏ qua
     số của turn_around (luôn 180).
  2. Vòng hỏi–đáp ở chế độ rule parse câu trả lời MỘT MÌNH, không ghép với lệnh gốc → "180 độ" đứng riêng lại không hiểu.
  3. Từ vựng thiếu: "quay tại chỗ" (không có phía), "tự quay", "xoay tròn", "vòng / nửa vòng", "chiều kim đồng hồ",
     tiếng Anh "rotate/rotating/spin", "by the left", "clockwise"...
  4. Không có LLM: `project/src/CM/.env` không tồn tại (không API key) → auto rơi về rule; Jetson Python 3.8,
     có SDK openai 1.89 (Gemini dùng endpoint OpenAI-compatible), anthropic 0.72 (thiếu tính năng CM dùng).
- Ràng buộc giữ nguyên (comm_plan 09-25): LLM chỉ chép, CODE quyết định; mọi số phải có trong lời user, đúng thứ tự.
- User 2026-09-28 (lần 2): có Gemini 2.5 Flash trong `project/src/CM/.env` (hiện chỉ CM grounding dùng) → dùng cho cả
  phân tích lệnh ban đầu. Nối simulation ↔ tạo bản đồ: bản đồ tạo ra được GHI LẠI vào JSON như thiết kế cũ (Scene JSON
  v2.0, `simulation/room/scenes/`), và các lệnh về sau dùng nó.
- Sự thật đã kiểm: `.env` có CM_PROVIDER=gemini + key → `demo_drive --provider auto` đã gọi Gemini, prompt thì chưa đủ.
  Thiết kế L2 (`robot_process_llm.md`): LLM trả `goal | ask | primitive`; `goal` → CM GroundingSession (target → side →
  spot) → MV plan → primitives. `drive_map` đã xuất `map/scene.json` v2.0 (source astra_map) nhưng KHÔNG đưa vào
  `scenes/`; `seed` = None; robot ở x −0.38 NGOÀI khung phòng (camera không thấy chỗ xe đứng) → MV sẽ phải snap start.
  `scenes/latest.json` là của simulator (nút New room ghi đè) → không dùng làm bộ nhớ của robot.

## Expected output
1. Đúng câu lệnh trên (rule, offline, 1 lần, không hỏi) → quay trái 180°, quay phải 360°, quay phải 180°, tiến 90 cm,
   quay đầu 180°; in hướng tích luỹ sau mỗi bước + "sau bước 3: 0° — khớp 'trở về vị trí ban đầu'".
2. Bản tiếng Anh của user cũng hiểu (typo "selt", "angel" không làm hỏng).
3. Hỏi–đáp có ngữ cảnh: câu hỏi nêu các bước ĐÃ hiểu + phần chưa hiểu; câu trả lời điền vào phần đó (hoặc thay cả lệnh
   nếu tự nó là lệnh đủ).
4. Quay không nói phía: 180° / 360° / "vòng" → trái (quy ước như quay đầu); góc khác → hỏi "trái hay phải?".
5. Gemini là parser chính (`--provider auto`, mặc định); rule là dự phòng offline với cùng từ vựng.
6. Test offline: bộ ≥ 25 câu tự nhiên (có / không dấu, gõ sai, tiếng Anh) → đúng các bước; chạy thật bộ đó qua Gemini
   ≥ 95 % đúng (số nào không có trong câu → validator chặn 100 %).
7. Module `realroom/` (tương tự simulation/room, cho phòng thật): giữ bản đồ thật (`maps/latest.json`, Scene v2.0),
   xem trên web, plan đường đi trên nó; mỗi lệnh chạy xong cập nhật vị trí robot (VO đo). `simulation/` không đổi.
8. Lệnh theo bản đồ: "đi tới cái ghế" / "go next to the suitcase" → Gemini chọn loại `goal` → CM grounding trên
   `map_latest.json` (hỏi lại nếu mơ hồ) → MV plan từ vị trí robot đã lưu → leg_executor (đo + bù) → cập nhật vị trí.

## Plan
### Phần A — Hiểu lệnh (Gemini chính, rule dự phòng)
- [x] 1. Validator + schema: turn_around nhận góc user nói (có trong câu, 1..360), không nói = 180. Quay không phía:
      180 / 360 / vòng → trái (quy ước như quay đầu); góc khác → hỏi "quay trái hay phải?" (không đoán).
- [x] 2. Prompt v1.1 (Gemini): quay tại chỗ / tự quay / xoay / rotate / spin, phía trước hoặc sau góc, chiều kim đồng
      hồ, "N vòng / nửa vòng", "quay lại N độ", câu mục đích ("để trở về...") không phải bước, gõ sai, tiếng Anh; ví dụ
      gồm đúng câu của user và bản tiếng Anh. demo_drive in "parser: gemini-2.5-flash" để biết đang dùng gì.
- [x] 3. Rule parser (dự phòng offline): cùng từ vựng ở bước 2.
- [x] 4. Hỏi–đáp có ngữ cảnh: câu hỏi liệt kê bước đã hiểu + phần chưa hiểu; ở rule, câu trả lời thay phần chưa hiểu rồi
      parse lại cả lệnh (không được thì thử như lệnh mới); ở Gemini, lịch sử hội thoại đã có — thêm ví dụ trả lời ngắn.
- [x] 5. Kiểm ý định: in hướng tích luỹ sau mỗi bước; câu "trở về hướng / vị trí ban đầu", "back to the start / base
      angle" → báo bước nào đưa hướng về 0° (không có → cảnh báo, không chặn).
- [x] 6. Test: bộ ≥ 25 câu (rule + FakeClient cho hợp đồng Gemini) + `test_live_parser.py` chạy bộ câu qua Gemini thật
      (tay, tốn vài chục lượt gọi) → tỉ lệ đúng.

### Phần B — Module `realroom/`: bản đồ thật + plan cho phòng thật (user 09-28: simulation là module RIÊNG, không sửa;
viết module TƯƠNG TỰ `simulation/room` cho phòng thật)
- [x] 7. `realroom/map_store.py` (≈ scenes/ của simulator): `maps/map_<run>.json` + `maps/latest.json`, Scene JSON v2.0
      cùng schema; `drive_map --publish` ghi vào; id = run; khung scene bao cả quỹ đạo robot (hiện robot ở x −0.38 ngoài
      khung); lưu / đọc pose robot.
- [x] 8. `realroom/planner.py` (≈ MV trong CM cho phòng giả): goal (x, y, θ) → MV plan trên bản đồ thật từ pose đã lưu
      → primitives FORWARD / ROTATE (MOVE tách thành T + F) → leg_executor.
- [x] 9. `realroom/app.py` + `static/index.html` (≈ simulation/room/app.py, cổng 5002): xem bản đồ thật, vị trí robot,
      quỹ đạo, vật + nhãn, đường plan; `GET /api/map/latest`, `GET /api/map/<id>`, `POST /api/plan`.
- [x] 10. Vị trí robot liên tục: demo_drive đọc pose từ `realroom` lúc bắt đầu (ghi vào run.json), cộng chuyển động VO đo
      được lúc kết thúc, lưu lại. Chạy không `--compensate` → pose theo lệnh × k, gắn cờ.
- [x] 11. Bộ định tuyến L2: prompt lệnh thêm loại `goal` (người dùng nhắc tới vật: chép nguyên cụm từ) → CM
      GroundingSession trên bản đồ thật (Gemini, hỏi lại khi mơ hồ) → realroom.planner → leg_executor → cập nhật pose.
      Không có bản đồ / vật không có → hỏi lại, không chạy.
- [ ] 12. Test offline (FakeClient + scene mẫu) [x] + thử thật "đi tới cái ghế" trên bản đồ run 222009 [chờ user].
- [x] 13. `/code-standards-review` + `/code-logic-review`; docs (project_overview bảng module, interfaces: realroom
      maps + API, run.json start_pose; comm_plan; realroom/README).
- Ngoài phạm vi (explore-map Pha 2): ghép nhiều run thành một bản đồ (scan_match) — task này dùng bản đồ mới nhất.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[23:55] approved | user: dùng Gemini 2.5 Flash (.env CM) cho phân tích lệnh; bản đồ thật ghi JSON như thiết kế cũ, dùng cho lệnh sau; simulation là module riêng → viết module tương tự `realroom/` (bản đồ thật + plan) | info: làm A (1–6) rồi B (7–13)
[23:57] step 1: validator — hành động thô `turn` (không phía): 180 / 360 → turn_left (quy ước), góc khác → `Question` "Quay N độ sang trái hay sang phải?", không góc → hỏi; turn_around nhận góc user nói (không nói = 180); đơn vị `rev` (vòng, ×360 trong CODE; không số = 1 vòng); `Question` không làm LLM thử lại, ghi chú câu hỏi vào cùng tin assistant (tránh 2 tin assistant liền) | info: cfg SIDELESS_TURN_DEG, REV_UNIT, ANGLE_UNITS_DEG, DEFAULT_REVS; test_drive ALL OK
[23:58] step 2: prompt drive_command v1.1 (động từ quay + phía trước/sau góc, chiều kim đồng hồ, `turn` không phía, turn_around có góc, đơn vị rev, câu mục đích / chữ đệm không phải bước, gõ sai, trả lời = cả lệnh; ví dụ câu user không dấu + bản tiếng Anh + vòng); CommandParser.model → demo_drive in "Parser (gemini, gemini-2.5-flash)" | risk: GỌI THẬT Gemini → 403 "API key was reported as leaked" — key trong CM/.env bị Google khoá; .env không bị git theo dõi (.gitignore), cây làm việc không file nào chứa key AIza; lộ ở đâu chưa rõ (git hỏng, không xem được lịch sử) → user tạo key mới; kiểm thật chờ key
[00:04] step 3: rule parser — cụm quay = động từ (re/queo/quay/xoay/turn/rotate/spin/pivot/u-turn) + phần bất kỳ thứ tự (vòng / góc có hoặc không đơn vị / chiều kim đồng hồ / phía trước-sau góc kể cả 'by the left' / quay đầu / chữ đệm 'tại chỗ'); vế nối không động từ ('rồi nửa vòng', 'rồi 40 cm nữa') kế thừa phía / chiều; câu mục đích 'để / to ... ban đầu / base angle' bỏ qua; 'quay lại vị trí ban đầu' không có 'để' → hỏi (QUESTION_RETURN), không hiểu thành quay đầu; câu hỏi trích nguyên văn có dấu | info: câu user (VN + EN) → 5 bước đúng, không hỏi; user báo đã có key Gemini mới
[00:09] step 4: hỏi–đáp có ngữ cảnh (rule) — rule_parse trả `understood` + `pending` (chỗ đặt câu trả lời: thay phần chưa hiểu, hoặc SAU động từ thiếu quãng / cụm quay thiếu phía), bước thô có `span`; Question mang chỉ số bước; câu hỏi nối "Đã hiểu: 1) …"; thứ tự thử: câu trả lời đủ bước (≥ total) → thay cả lệnh; ghép đúng chỗ; (câu hỏi phía) thay cả cụm quay; KHÔNG nhận cách đọc ít bước hơn lệnh chờ; không được → giữ lệnh đã ghép, hỏi tiếp; 'X bao xa?' cho động từ không quãng; số chữ cho quãng ('nửa mét', 'một mét') chỉ khi có đơn vị ngay sau | risk (đã sửa): trả lời 'nửa mét' từng làm RƠI bước lùi âm thầm (thay 'lùi lại' bằng 'nửa mét', phần dư không có từ bị cờ) → chặn bằng số bước + đơn vị lẻ (mét / cm / deg) vào danh sách phần dư
[00:10] step 5: cmd_parser `track_steps` (hướng / x / y tích luỹ theo lệnh), `intents` (câu mục đích: vị trí nếu nhắc chỗ / vị trí / start, không thì hướng), `check_intents` (bước đầu tiên về lại ban đầu, ±1° / ±1 cm; chỉ báo, không sửa bước); TURN_SIGN / LINEAR_SIGN chuyển sang cmd_parser (motion_plan dùng lại); demo_drive stage [2] in hướng / vị trí mỗi bước + ✓ / ! | info: câu user → "✓ 'để trở về vị trí ban đâu': sau bước 3"
[00:33] step 6: `natural_commands.json` (36 ca: VN có / không dấu, gõ sai, EN, câu hỏi, hỏi–đáp, intent) dùng chung; test_drive + test_natural_corpus_rules / test_intents / test_turn_validator / test_llm_question_is_not_retried / test_fallback_finishes_the_exchange_with_rules → ALL OK; `test_live_parser.py` (provider bắt buộc, giãn 12.5 s, --first) | info: GEMINI thật: 10 / 10 ca gọi được ĐÚNG (gồm câu user VN + EN), 2–3 s/lệnh; free tier gemini-2.5-flash = 5 req/phút + **20 req/NGÀY** (GenerateRequestsPerDayPerProjectPerModel-FreeTier) → hết hạn mức sau ~20 lượt, 26 ca còn lại chưa chạy; auto rơi về rule (rule đọc được mọi ca offline) | risk (đã sửa): LLM lỗi giữa hỏi–đáp → reset xoá lệnh chờ → câu trả lời bị hiểu một mình; nay rule đọc lại cả cuộc hỏi–đáp và giữ đến hết lệnh; CM grounding dùng CHUNG key + model → chung 20 lượt/ngày
[00:35] step 7: `realroom/map_store.py` (publish_run / load_latest / load_map / load_grid / save_pose / move_robot / compose, CLI publish | show); `maps/map_<id>.json` + `_grid.npz` (int8 1/0/−1, res, origin trong khung scene) + `latest.json` (robot = pose HIỆN TẠI + pose_history); vision: `OccupancyMap.copy / integrate_footprint / known_grid`, drive_map xuất scene + `known_grid.npy` từ bản có dấu thân robot (đĩa 0.225 m dọc quỹ đạo = trống), `--publish` | info: run 222009 → phòng 3.75 × 1.95 m, robot TRONG khung (x 0.573, trước −0.38), 6 vật (ghế obj_1 cách ~0.8 m), lưới 79 chiếm / 1247 trống; map.png vẫn từ lưới gốc
[00:37] step 8: `realroom/planner.py` plan(scene, grid, goal) → RoomPlan: MV (CM mv_client, không holonomic: T–F–T) từ pose robot đã lưu; kiểm đĩa thân robot quét dọc đường trên KnownGrid: chạm ô chiếm → từ chối, sàn chưa thấy > 35 % → từ chối, MV dời điểm đầu > 5 cm → từ chối; legs [(ROTATE rad | FORWARD m)] | info: map 222009, 10 spot CM (ghế, vali, …): tất cả ok, 0 ô chiếm, chưa thấy 9–26 % (cao nhất ở đường ngắn: vùng mù 0.78 m trước xe) | risk: ngưỡng 35 % cho đi qua sàn chưa thấy (vùng mù) — robot không có cảm biến va chạm; demo phải in tỉ lệ này và chờ Enter
[00:39] step 9: `realroom/app.py` (Flask :5002; / , /api/map/latest | <id> (scene + lưới cắt theo phòng, hàng chuỗi 'o' / '.' / ' '), /api/spots (CM candidates), POST /api/plan {{spot | goal}}) + `static/index.html` (canvas: lưới đã biết, vật + nhãn, robot + hướng, vệt pose_history, spot bấm được → đường plan + legs; sáng / tối) | info: test client: map 75 × 39 ô, 10 spot, plan cạnh trái ghế ok 0.87 m; goal vào giữa vật → MV "no free cell nearby"
[00:45] step 10: demo_drive `_map_start` (run.json start_pose từ realroom latest), `_map_update` sau run: --compensate → measured_pose (VO), thường → lệnh × k (slip_model) cờ predicted, run dừng sớm / Ctrl-C không có đo → KHÔNG đặt, cảnh báo; run.json end_pose; `drive_config.add_realroom_to_path`; leg_executor dùng `map_store.compose` (bỏ bản trùng) | info: test_drive + test_leg_executor ALL OK
[00:57] step 11: định tuyến L2 — ParseResult.goal; rule `_find_goal` (động từ + giới từ chỗ; 'đến' đứng một mình được, 'tới' không vì = 'tôi'; từ chỉ hướng không phải chỗ; lẫn bước khác → hỏi tách); prompt v1.2 trường `goal` (chép lời user) + `goal_error` (chữ của goal phải có trong lời user, trừ mạo từ); `communication/goto.py` ground (CM GroundingSession với LLM, hỏi lại; LLM lỗi / không có → rule: CLASS_WORDS VN/EN → lớp COCO, nhiều vật → hỏi số, spot gần robot nhất) + route (realroom.planner); `motion_plan.plan_from_legs`; demo_drive: goal → _plan_goal → stage [2] "Đi tới …" → _execute như lệnh thường (+ cập nhật pose) | info: dry-run "đi tới cái ghế" → obj_1:left:corner_a, 0.60 m, chưa thấy 26 %, T 29° / F 0.601 / T −119°, 12.7 s; 'the kitchen' → "Bản đồ thật không có …, có: chair, suitcase"; corpus +4 ca goal, 'go to the kitchen' giờ là goal | risk: mô tả spot của CM bằng tiếng Anh
[00:59] step 12 (offline): `realroom/test_realroom.py` 3/3 (publish + gốc lưới về khung scene + move_robot / lịch sử / cờ predicted; quét hành lang; planner với MV thật: phòng trống → T/F, 1 ô chiếm → từ chối, dải chưa thấy → từ chối, điểm đầu trong lề vật → từ chối); test_drive + goal LLM (chép / bịa bị chặn) + rule grounding (vali, 2 ghế → hỏi số, spot gần nhất, 'kitchen' → nói bản đồ có gì) ALL OK; drive_map --publish thật OK | info: chờ: Gemini thật cho ca goal (hết 20 lượt / ngày), thử robot "đi tới cái ghế" (user)
[01:02] step 13 (docs): communication/README (hai nhánh L2, --compensate, từ vựng luật, quay không phía, hỏi–đáp, kiểm ý định, hạn mức Gemini, file mới), realroom/README (dữ liệu, khung, vị trí robot, luật lập đường + ngưỡng 35 %, web), project_overview (run modes), CLAUDE.md (bảng module + realroom), interfaces (drive command JSON v1.2, realroom, run.json, slip_profiles.json + vo_scale.json — mục logic-review 09-28 báo thiếu), comm_plan (quyết định + done + pending) | info: còn 2 review
[01:09] standards-review (2 pass): 19 sửa, 0 hoãn | cmd_parser: dòng chú thích lặp đôi, docstring module (nhánh goal, hỏi–đáp), `rest` của goal dùng `_source` (điều kiện cũ so sai chuỗi), tên phần cụm quay → _PART_*, regex viết thẳng → _COUNT_RE / _DIGIT_RE, chú thích dài; demo_drive: docstring module + _parse_until_steps, 2 dòng dài; goto: lý do / lớp 'unknown' → hằng, docstring _nearest; map_store / planner: docstring _history_entry / _refuse; app: số chữ số → M_DIGITS; ngắt dòng leg_executor ×2, occupancy_map, test_realroom | info: độ dài dòng repo vốn tới ~130 (drive_map) — chỉ ngắt dòng mới > 128; tất cả test pass
[01:13] logic-review (2 pass; đối chiếu plan 1–13, expected output, interfaces, comm_plan, explore-map): 5 sửa, 0 báo user | (1) planner kiểm hành lang trên đường lưới DÀY của MV trong khi leg chạy THẲNG giữa waypoint (đích obj_4: 2 đoạn 1.27 + 1.33 m, path 53 điểm uốn khác) → kiểm + trả về polyline waypoint; (2) 'đi tới A rồi đi tới B' / 'rồi đi thẳng' → phần sau bị BỎ âm thầm → có bất kỳ gì ngoài chỗ (bước, chỗ thứ hai, phần không đọc được) thì hỏi tách; (3) parser không reset sau goal → lịch sử lẫn sang lệnh sau; (4) dự phòng giữa chừng: tin đầu đã đủ lệnh vẫn bị câu trả lời đè → dừng ở tin hoàn chỉnh; (5) --compensate dừng vì robot_link lỗi / mất camera / Ctrl-C vẫn cập nhật vị trí bằng pose thiếu leg cuối → không ghi measured_pose, bản đồ không đổi + cảnh báo | info: +3 test (goal đi một mình, replay dừng, polyline = quãng leg); tuyến thật chưa thấy 9–29 % (trước 9–26 %), vẫn < 35 %; MV không holonomic chỉ cho leg TIẾN → tránh lỗi firmware cắt leg lùi
[01:16] review gate — standards (2 pass) trên phần sửa của logic-review: 1 sửa, 0 hoãn | test_realroom: POSE_TOL * 10 → LENGTH_TOL_M | info: phần còn lại (planner polyline, rule_parse goal + phần còn lại, replay dừng, measured_pose có điều kiện, 3 test) đạt
[01:19] review gate — logic (2 pass) trên cùng phần: 1 sửa, 0 báo user | RoomPlan.length_m vẫn là độ dài đường lưới dày của MV (demo_drive / web in quãng sai) → độ dài polyline robot chạy (obj_4: 2.605 m = Σ FORWARD) | info: polyline trùng điểm đầu vô hại; STUCK vẫn đo được → vẫn cập nhật (cờ predicted); test_realroom 3/3
[01:19] status → testing | chờ user: (1) `test_live_parser.py --first 9` khi hạn mức Gemini hồi (ca 9–40); (2) robot thật "đi tới cái ghế" (--compensate) trên bản đồ 20260928_222009 — đặt robot đúng chỗ run đó kết thúc
[01:20] review gate — standards (2 pass) trên sửa length_m của planner: 1 sửa, 0 hoãn | chú thích RoomPlan 122 ký tự → 2 dòng | info: test_realroom 3/3
[18:14 09-29] step 6 (Gemini thật, hạn mức ngày reset 14:00 VN = 00:00 PT): `test_live_parser.py` thêm `--last` (chạy N..M, chừa lượt cho robot — parser + CM grounding chung key); ca 9–24 → **16 / 16 đúng**, không giá trị bịa, ~2 s/lệnh | info: tích luỹ Gemini 26 / 26 (ca 1–8 + 9–24; 10/10 ngày 09-28 gồm cả lượt thử); còn ca 25–40 (16 ca, ~19 lượt) → ngày mai `--first 25`; dry-run rule "đi tới cái ghế" sau khởi động lại: obj_1:left:corner_a, T 29° / F 0.601 / T −119°, pose bản đồ không đổi (0.573, 0.737, −0.3°); `robot_link --stop` exit 0 | risk: còn ~4 lượt Gemini hôm nay cho phép thử robot

## Test result
