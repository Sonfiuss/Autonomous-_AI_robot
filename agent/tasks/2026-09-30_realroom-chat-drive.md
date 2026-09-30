---
id: 2026-09-30_realroom-chat-drive
status: testing     # 08:28 user "tiếp tục task tối qua" = duyệt plan; 09:23 code + review xong, chờ thử thật
module: realroom (+ communication)
started: 2026-09-30
---

## Task
Web realroom (:5002) thành chỗ điều khiển robot thật như simulation: chat lệnh → xem kế hoạch trên bản đồ → Chạy / Dừng
→ vị trí robot và bản đồ tự cập nhật; sau khi robot lập bản đồ thì tự publish sang realroom, chỉ cần reload (hoặc trang
tự làm mới).

## Input
- User 2026-09-30: "logic cần tự động sau khi robot lập bản đồ thì cập nhật vào bản đồ realroom luôn. tôi chỉ cần reload
  web realroom thì bản đồ mới được cập nhật. tại đây tôi có thể chat để điều khiển robot di chuyển như trong simulation".
- Sự thật đã kiểm:
  - "Chat như simulation" = CM web (`project/src/CM/app.py`, :5001): `/api/ground/say` (GroundingSession LLM) → đích →
    `/api/trajectory` (MV + MC) → phát lại mô phỏng bằng nút Play. KHÔNG lái robot thật.
  - Robot thật chỉ chạy qua `communication/demo_drive.py` (CLI): parser (Gemini / rule, hỏi lại khi thiếu) → steps hoặc
    goal (`goto.ground` hỏi qua callback `ask` chặn stdin, `goto.route` trên bản đồ thật) → in kế hoạch → **Enter** →
    recorder (vision/record.py) + `robot_link --run-plan` (hoặc `leg_executor` với `--compensate`) → `_map_update` dời
    pose (đo VO khi compensate, dự đoán khi không) → run.json. Ctrl-C = SIGINT tới cả robot_link → hủy plan + gửi S.
  - Bản đồ realroom chỉ đổi khi chạy tay `drive_map --run <run> --publish` (thay cả bản đồ; cùng map id thì giữ pose).
    Ghép nhiều run chưa có (multi-stop Pha D bước 10; S4 hôm qua ghost 53 %).
  - `realroom/app.py`: chỉ xem bản đồ + spots + `/api/plan` (vẽ đường), "không lái robot". Trang tải bản đồ 1 lần (nút
    Tải lại).
  - Parser chưa biết "lập bản đồ / quét phòng". Quét tại chỗ trái 90 – phải 180 – trái 90 là kịch bản đã đạt (184936).

## Expected output
1. Web :5002 có khung chat: gõ "đi thẳng 50 cm", "quay trái 90 độ", "đi tới người", "lập bản đồ" → bot trả lời (hoặc
   hỏi lại như CLI) → kế hoạch hiện trên bản đồ (đường đi, các leg, thời gian, sàn chưa thấy) → nút **Chạy** → robot
   chạy (trạng thái: camera, leg i/N, đang đo, đang dựng bản đồ, xong: kết quả) → nút **Dừng** dừng ngay (như Ctrl-C).
2. Sau mỗi lệnh: pose robot trên bản đồ tự cập nhật; sau lệnh "lập bản đồ": `drive_map --publish` tự chạy, bản đồ mới
   hiện trên trang (trang tự làm mới khi `latest.json` đổi; reload cũng được).
3. CLI dùng chung: `demo_drive.py "lập bản đồ"` / `--map` cũng tự publish; `--plan-json` chạy kế hoạch web đã duyệt.
4. Test offline (không robot, không LLM): chat → plan → chạy `--dry-run --no-vision` → xong; hỏi lại (2 ghế) → trả lời
   → plan; Dừng giữa chừng → aborted, pose không dời; chạy lần 2 khi đang chạy → từ chối; "lập bản đồ" → kế hoạch quét
   + cờ publish.

## Plan (chờ duyệt)
- [ ] 1. `demo_drive`: (a) `--plan-json FILE` = kế hoạch đã duyệt ở nơi khác {command, provider, raw, goal, steps | legs,
      map} → bỏ qua parser / grounding, chạy như CLI (`--yes`); (b) lập bản đồ: cờ `--map` hoặc lệnh có "lập bản đồ /
      quét phòng / quét xung quanh" (so khớp đã bỏ dấu) → một mình thì là quét tại chỗ trái 90 – phải 180 – trái 90, đi
      kèm lệnh khác ("quay trái 360 độ rồi lập bản đồ") thì chạy lệnh đó; chạy xong (complete hoặc đã đo) →
      `vision/drive_map.py --run <run> --publish` (log `map_build.log`, run.json event `map_published` / `map_failed`,
      status "đang dựng bản đồ"). Lệnh thường: chỉ dời pose như bây giờ.
- [ ] 2. `realroom/drive_chat.py`: `ChatSession` — say(text) → {type ask | plan | error, text, plan: legs, path, length,
      sàn chưa thấy, cờ map}; câu hỏi của parser và của goto (callback `ask` chặn) → luồng riêng + hàng đợi, câu chat kế
      tiếp là câu trả lời; `RunManager` — start(plan, compensate) → tiến trình con `demo_drive --yes --plan-json` trong
      process group riêng (một run một lúc), status() từ status.txt / run.json / đuôi log của run, stop() → SIGINT cả
      group (đúng đường Ctrl-C đã kiểm: robot_link hủy + gửi S, video đóng đúng).
- [ ] 3. `realroom/app.py`: `POST /api/chat {text}`, `POST /api/chat/reset`, `POST /api/run {compensate}` (chạy kế hoạch
      của câu trả lời cuối; 409 khi đang chạy), `POST /api/run/stop`, `GET /api/run/status`, `GET /api/map/version`
      (mtime latest.json).
- [ ] 4. `realroom/static/index.html`: khung chat + trạng thái; kế hoạch vẽ trên bản đồ (đường + leg); nút Chạy (chỉ khi
      có kế hoạch), Dừng (nổi bật khi đang chạy), "Lập bản đồ"; ô "đo từng leg (--compensate)" mặc định bật; trang hỏi
      `/api/map/version` mỗi 2 s, đổi thì tải lại bản đồ.
- [ ] 5. Test `realroom/test_chat.py` (EO4, parser `rule`, `--dry-run --no-vision`); chạy lại test_drive, test_leg_executor,
      test_realroom, test_drive_map.
- [ ] 6. Docs (realroom README, interfaces: endpoints, `--plan-json` / `--map`, run.json map events; overview run modes;
      comm_plan) + `/code-standards-review` + `/code-logic-review`.
- [ ] 7. Thử thật (user): reload :5002 → chat "lập bản đồ" → Chạy → bản đồ mới tự hiện; "đi tới người" → Chạy → pose.
- Ngoài phạm vi: ghép bản đồ nhiều run (Pha D bước 10), đăng nhập (web chạy trong LAN, ai vào :5002 cũng lái được),
  hàng đợi nhiều lệnh.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[00:18] intake + plan | info: simulation chat = CM :5001 (mô phỏng); robot thật chỉ qua demo_drive CLI (Enter / Ctrl-C) | risk: web lái robot thật không có cảm biến va chạm → luôn cần nút Chạy + Dừng; publish thay CẢ bản đồ (chưa ghép run)
[08:28] approved ("tiếp tục task tối qua") → executing | info: vision/output chỉ còn 20260930_002825 + manual_360 (các run 09-28/29 đã bị xoá khỏi đĩa; realroom/maps còn nguyên); run 002825 (quét + đi 80 cm, --compensate) failed: leg quay phải bị firmware cắt (2.38 / 4.18 s) → VO −44° / −90° → stuck
[08:36] step 1: demo_drive `prepare()` (dùng chung CLI + web: ask / say callback), `plan_json` / `load_plan_json` (`--plan-json`, kiểm giới hạn validator; route mang `from_pose` → lệch map id / pose > 1e-3 thì từ chối), `--map` + cmd_parser `split_map_request` (bỏ cụm + liên từ hai đầu; chỉ còn từ đệm → quét MAP_SCAN), `build_map` (drive_map --publish, map_build.log, event map_published / map_failed / map_skipped, status building map), `--run-dir`; thêm (ngoài plan): flock `vision/output/.robot.lock` (CLI + web không cùng lái), map_store `REALROOM_MAPS_DIR` + `map_version()` | info: drive_map run 002825 (35 s) mất 17 s; dry-run "quay trái 90 độ rồi lập bản đồ" → 1 leg + map_skipped (dry run)
[08:52] step 2: `realroom/drive_chat.py`: ChatSession (một luồng / lệnh chạy demo_drive.prepare, ask() chờ câu chat kế; parser mới mỗi lệnh; hết 90 s → huỷ; take_plan() tiêu kế hoạch: bấm Chạy 2 lần không chạy 2 lần), preview (leg, thời gian, đường trên map = pose ghép từng leg, ô vật cản / sàn chưa thấy dưới đĩa robot: route đã qua kiểm, bước chỉ cảnh báo), RunManager (demo_drive --yes --plan-json <run>/approved_plan.json --run-dir, start_new_session, console.log; stop = killpg SIGINT 1 lần; status từ status.txt / đuôi log / run.json + sự kiện map) | risk (đã sửa): tiến trình nền có SIGINT bị bỏ qua thì con kế thừa → Dừng vô hiệu → demo_drive main đặt lại default_int_handler | info: thử trên bản đồ thật (rule): 'đi thẳng 50 cm' → đường 0.5 m, sàn chưa thấy 24 %; 'quay 90 độ' → hỏi phía; 'đi tới người' → hỏi obj_5 / obj_14
[08:55] step 3: app.py: POST /api/chat (busy khi đang chạy), /api/chat/reset, /api/run {compensate} (400 chưa có plan, 409 đang chạy → trả plan lại), /api/run/stop, GET /api/run/status, /api/map/version (mtime_ns dạng chuỗi: vượt độ chính xác số JS); `--provider`, `--dry-run` (Chạy = phát thời gian, không robot / camera), `--port`
[08:56] step 4: index.html: khung chat (bong bóng, câu hỏi viền xanh, lỗi viền đỏ), nút Lập bản đồ / Huỷ lệnh; Kế hoạch (leg, thời gian, sàn chưa thấy, cảnh báo ô vật cản, 'sau đó dựng bản đồ'), ô --compensate mặc định bật, Chạy / Dừng (Esc); trạng thái 1 s / lần khi chạy (camera khởi động, leg i/N, đo, dựng bản đồ, xong); version bản đồ 2 s / lần → tự tải lại; kế hoạch vẽ nét đứt | risk (đã sửa): chạy xong mà pose không đổi (dry-run) thì đường kế hoạch còn trên bản đồ → draw() khi xong | info: Firefox headless qua proxy scratchpad (bản sao maps, cổng 5003): 'đi tới người' → hỏi → '2' → đường tới obj_14 1.43 m; 'đi thẳng 2 m' → cảnh báo 28 ô vật cản + 41 % chưa thấy → Chạy → Dừng → aborted; 'quay trái 90 độ' → complete; bản đồ thật không đổi, 2 run dry-run thử đã xoá
[08:57] step 5: realroom/test_chat.py 8/8 (plan + dry-run + plan đã tiêu, pose không dời; 2 ghế hỏi → '2' → route obj_2 có from_pose, chạy được, robot dời 10 cm → demo_drive exit 2; Dừng giữa chừng → aborted khi server BỎ QUA SIGINT; chạy 2 → RunBusy / 409 / chat busy; lập bản đồ → quét 5 leg + cờ, dry → map_skipped; build_map stub 0 / 3 → published / failed; --plan-json từ chối 6 file xấu; version đổi theo pose) + test_drive `test_map_request`; đột biến: bỏ default_int_handler → check_stop_midway FAIL (run chạy hết dù đã Dừng) | risk (đã sửa): Dừng lúc demo_drive còn import → chết vì SIGINT, không run.json → status 'cancelled' (không gì được gửi tới robot) | info: test_chat ~55 s, test_drive ALL OK, test_leg_executor ALL OK, test_realroom 5/5, test_drive_map 14/14, pyflakes sạch
[09:02] step 6 (docs): realroom README (lệnh chạy, endpoint mới, mục 'Điều khiển robot từ trang', REALROOM_MAPS_DIR, lập bản đồ tự publish), communication README (tuỳ chọn mới, mục 'Lập bản đồ', an toàn: khoá, SIGINT, --plan-json; file run mới), interfaces (endpoint chat / run / version, demo_drive options + run.json), overview run modes, comm_plan (quyết định + done)
[09:10] standards-review (2 pass; drive_chat, app, test_chat, map_store, index.html, demo_drive, cmd_parser, drive_config, test_drive): 15 sửa, 0 hoãn | drive_chat: ELAPSED_DIGITS, ERROR_TEXT (chuỗi 'Lỗi:' lặp), `(run or {})` ×4 gọn lại, chú thích '- 1' (STOP), logger.debug (preview, reply), docstring _read_text; app: `--provider` có choices (tên sai trước đây → 500 mỗi câu chat), COMPENSATE_DEFAULT, comment 'main()' sai; demo_drive: _lock_robot OSError được bắt (trước: traceback), map id tính 1 lần, logger.debug (prepare, _moved_since, build_map); cmd_parser: _MAP_RE tách dòng + debug; test_chat: CHAIR_* / BOX_HALF_M / MOVED_M / MTIME_GAP_S / cfg thay số thẳng, gộp 2 helper run.json; index.html: STATUS / LEG_WORDS_SEP thay 'leg ' / slice(5) | info: dòng > 120 còn lại đều là code cũ; test_chat 8/8, test_drive ALL OK, pyflakes sạch
[09:22] logic-review (2 pass; đối chiếu plan 1–6, EO 1–4, interfaces, overview, comm_plan, history 09-29): 5 sửa, 0 báo user | (1) LOGIC: SIGINT tới sau khi robot xong (đuôi video 2 s / đóng video — nút Dừng web bật suốt pha này) → except gọi _map_update LẦN HAI (pose cộng đôi), trong finally thì mất run.json, và bản đồ vẫn dựng → `_note_stops`: từ khi drive trả về tới khi lưu run.json SIGINT chỉ đặt cờ `stop_requested` (handler không print: ghi stdout lồng nhau ném lỗi), kết quả giữ nguyên, _map_after bỏ với lý do 'stopped by the user', handler mặc định trả lại cho pha dựng bản đồ; (2) kiểm from_pose chuyển SAU khi giữ khoá robot (trước đó run khác có thể dời robot giữa kiểm và chạy; áp cả cho đích CLI); (3) from_pose hỏng → PlanError thay KeyError; (4) HIỆU NĂNG: _tail đọc cả console.log vào list mỗi giây → deque(maxlen); (5) trang báo 'vị trí KHÔNG cập nhật' cả khi vừa publish bản đồ mới → bỏ | info: test mới check_stop_after_the_drive; đột biến tắt _note_stops → FAIL (run đã xong thành 'cancelled'); test_chat 9/9, test_drive / test_leg_executor ALL OK, test_realroom 5/5, pyflakes sạch; trang Firefox headless vẫn chạy; docs (interfaces, 2 README) thêm stop_requested
[09:23] → testing: chờ user thử thật (bước 7)
[09:24] review gate — standards (2 pass, skill) trên phần sửa của logic-review (demo_drive _note_stops / _execute / _map_after / _prepare_command / main / load_plan_json; drive_chat _tail; index.html finished(); test_chat check_stop_after_the_drive): 5 sửa, 0 hoãn | load_plan_json dùng map_store.POSE_KEYS thay tuple viết thẳng; CLI in một dòng khi Ctrl-C chỉ được ghi nhận (không in trong handler); index.html: REPLY / STATE / RESULT / MAP_EVENT thay giá trị giao thức viết thẳng (8 chỗ) + tách dòng dài; test_chat: FIRST_LEG_PREFIX, DONE_PREFIX (= STATUS_DONE.format) thay 'leg 1/' / 'done:' | info: check_stop_midway / check_stop_after_the_drive / check_plan_json_limits PASS, pyflakes sạch
[09:28] review gate — logic (2 pass, skill) trên cùng phần (đối chiếu plan 1–6, EO 1–4, interfaces, comm_plan, history 09-29; truy các nhánh: Dừng lúc lái / trong robot_link / chờ camera / prompt Enter / đuôi video / recorder.stop / dựng bản đồ; khoá rồi mới kiểm from_pose; file kế hoạch hỏng): 3 sửa, 0 báo user | (1) LOGIC: _moved_since so '> tol' → from_pose có NaN (json đọc được NaN) coi như chưa dời → đường chạy từ vị trí sai; nay 'not (… <= tol)' (NaN = đã dời), test mới + đột biến so sánh cũ → FAIL; (2) trang báo 'dựng bản đồ lỗi' cả khi user tự Dừng lúc dựng → hiện lý do (stopped / drive_map thoát N); (3) check_stop_after_the_drive báo rõ khi poll trễ quá đuôi 2 s thay vì fail mơ hồ | info: cửa sổ vài bytecode giữa lúc drive trả về và _note_stops (Ctrl-C ở đó ghi aborted cho run đã xong, không cộng pose đôi) chấp nhận; test_chat 9/9, test_drive / test_leg_executor ALL OK, test_realroom 5/5, pyflakes sạch; trang Firefox headless: chat → Chạy → complete với hằng mới; run dry-run thử đã xoá, bản đồ thật không đổi
[12:47] step 7 (CLI, robot thật, firmware bật lại): 'quay trái 90 độ' --compensate → đo +91.8° complete (124436); 'lập bản đồ' --compensate → 7 leg, mỗi góc lệch ≤ 0.4° (leg 2, 3, 6 vẫn bị firmware cắt, bù bằng leg compensation) → drive_map tự publish bản đồ 20260930_124516 (2.15 x 4.65 m, 13 vật, pose 'map built' đo) | risk: firmware vẫn cắt một số leg quay | info: còn phần web :5002 do user thử
[13:15] step 7: run 130635 / 130943 failed (acks 0, undecodable 100+) | info: ESP32 kẹt vòng panic lúc boot: Interrupt WDT trong ServoDriver::begin → ledc_channel_config → ledc_ll_set_duty_start chờ bit duty_start (SW_CPU_RESET không reset ngoại vi LEDC → lặp mãi); reset cứng qua RTS (= nút EN) → firmware chạy lại, telemetry P/O bình thường | risk: firmware chưa sửa (cần periph_module_reset(PERIPH_LEDC_MODULE) đầu begin()); nguyên nhân panic đầu tiên chưa rõ

## Test result
