---
id: 2026-09-29_multi-stop-map-test
status: executing   # 20:40 user "approve"; câu hỏi chưa trả lời → mặc định: 1 lệnh / kịch bản, ngưỡng như đề xuất, cách sửa fx chọn ở bước 6
module: vision (+ communication, realroom)
started: 2026-09-29
---

## Task
Kiểm thử (và sửa khi cần) bản đồ TỔNG HỢP từ nhiều vị trí của robot: quét tại A → đi 80 cm tới B → quay đầu /
quét tại B → một bản đồ không bóng ma. Quét tại chỗ đã đạt; nhiều vị trí chưa đạt. Đây là explore-map Pha 2
(bước 8–9) làm theo hướng "đo trước, sửa sau".

## Input
- User 2026-09-29: chất lượng map (quét tại chỗ) đạt; khi xe quay / đi lên 80 cm rồi quay ngược lại để có một
  bản đồ tổng hợp từ nhiều vị trí thì chưa đạt → cần test cho trường hợp này, lên plan.
- Sự thật đã kiểm (2026-09-29, run 184936 "quay trái 90, phải 180, trái 90", dựng lại trong scratchpad, không ghi
  vào run):
  - Bản đồ dựng MỘT run/lần; `--publish` THAY bản đồ realroom (README: "ghép nhiều run… Pha 2, chưa làm"). Không có
    cơ chế ghép run thứ hai vào bản đồ đang có.
  - Pose = VO sàn nối frame-với-frame (stride 5), không loop closure, không đối khớp bản đồ → sai số hướng cộng dồn.
  - **VO đọc mỗi lệnh quay 90° thành 95–99.5°** (tỉ số 1.06; leg bù kéo về 90° theo VO). Với động cơ bước, quay
    thật > lệnh là vô lý (trượt chỉ làm thiếu) → nghi VO đọc dư ~6 % do fx 570 CHƯA đo (`camera_rgb.json`:
    "not measured on this camera"). Sai tỉ lệ này bị triệt tiêu khi quét trái-phải-trái (vì sao quét tại chỗ trông
    ổn) nhưng KHÔNG triệt tiêu khi quay đầu 180° rồi nhìn lại → tường bị xoay lệch.
  - **Độ lặp VO khi quay**: cùng một leg 90°, cùng frame, đo 2 lần (leg_executor vs drive_map, lệch pha keyframe):
    94.8/95.05, −96.87/−96.40, −98.48/−97.61, 97.73/99.46° → lệch tới **1.7°/90°**. Cuối run: executor báo
    hướng −0.05°, drive_map +4.7°. Drift khi đứng yên = 0.02° (không phải nguồn lỗi).
    Sai 3° ở 2.5 m = 13 cm tường lệch → đủ tạo bóng ma.
  - **Trục quay theo ảnh ở (+0.039, 0.000) m** thân xe (từng leg 0.026–0.067): VO báo tâm xe dịch 6–9 cm mỗi lần
    quay 90° tại chỗ (end_pose realroom lệch 5.8 cm sau run chỉ có quay). Chưa biết tâm xe dịch thật hay lens
    thật ra cách tâm 0.146 m chứ không phải 0.185 m.
  - Có sẵn: `drive_map.rotation_axis`, `calib_floor.py` (fit pitch + fx + cx từ mốc dán sàn), `test_drive_map`
    render sàn tổng hợp, `leg_executor` (`--compensate`), parser hiểu các lệnh test dưới đây (đã thử rule_parse).
  - Leg lùi vẫn lỗi đồng hồ firmware (09-28 cắt 40 %, 09-29 dài 1.21×) → kịch bản test CHỈ dùng tiến + quay.

## Expected output
1. `python3 vision/map_check.py --run vision/output/<run> [--truth truth.json]` → `<run>/map/stops.png` (bản đồ
   con mỗi điểm dừng một màu, chồng lên nhau: thấy bóng ma bằng mắt) + `<run>/map/check.txt`: độ lệch giữa các
   điểm dừng đo bằng chính bản đồ (không cần thước), khoảng cách vật cản A↔B, pose cuối / vật mốc so với thước,
   độ lặp + tỉ số + trục quay VO.
2. Hiệu chuẩn quay: biết VO đọc góc đúng hay dư bao nhiêu, tâm xe có dịch thật khi quay không; sửa fx / mount
   theo số đo (user duyệt).
3. Bảng kết quả S2–S4 trên robot thật, ngưỡng đạt (đề xuất, user xác nhận):
   pose cuối khứ hồi ≤ 10 cm / ≤ 3° (thước); vật mốc ≤ 10 cm (EO 5 explore-map); độ lệch tốt nhất giữa 2 điểm
   dừng ≤ 5 cm / ≤ 1.5°; khoảng cách vật cản A↔B trung vị ≤ 5 cm (1 ô).
4. Không đạt → sửa (bước 9–10) rồi chạy lại map_check trên CHÍNH các run đã ghi (offline) tới khi đạt.

## Plan (duyệt 2026-09-29 20:40)

### Pha A — Thước đo (offline, không cần robot)
- [ ] 1. `vision/scan_match.py`: đối khớp 2 lưới occupancy (dx, dy, dθ trong ±0.3 m / ±8°): likelihood field của
      ô vật cản + phạt vật cản rơi vào ô trống, thô (5 cm / 1°) rồi tinh (1 cm / 0.2°); trả độ lệch + độ nhọn đỉnh
      (đỉnh không rõ → "không tin"). Ở pha này chỉ dùng để ĐO (= explore-map bước 8).
- [ ] 2. `vision/map_check.py`: tách run thành điểm dừng (đoạn giữa hai leg FORWARD, gồm các leg ROTATE của nó; frame
      trong leg FORWARD thuộc "đang đi") → bản đồ con mỗi điểm dừng bằng đúng luật của drive_map (`floor_usable`,
      pose drive_map) → `stops.png` + `check.txt`: (a) scan_match giữa từng cặp điểm dừng, (b) khoảng cách mỗi ô
      vật cản của A tới ô vật cản gần nhất của B trên vùng cả hai cùng thấy (trung vị / p90), (c) `truth.json` (nếu
      có): pose cuối + vật mốc vs thước, (d) VO: tỉ số quay từng leg, trục quay (`rotation_axis`), độ lặp = đo lại
      mỗi leg quay với 5 pha keyframe (stride 5 lệch 0..4) → độ phân tán.
- [ ] 3. `vision/test_map_check.py`: bản đồ con giả (tường + hộp) dời/xoay cài sẵn → scan_match tìm lại ±2 cm /
      ±0.5°, đỉnh phẳng (hành lang dài) → "không tin"; khoảng cách = 0 khi trùng, tăng đúng theo độ dời; tách điểm
      dừng trên timeline giả; so truth. Chạy lại test_drive_map, test_realroom, test_leg_executor.
- [ ] 4. map_check trên run 184936 (quét tại chỗ, "đạt") → số chuẩn; in bảng cho user.

### Pha B — Hiệu chuẩn quay (robot + user, ~20 phút)
- [ ] 5. S1 `python3 communication/demo_drive.py "quay trái 360 độ"` (KHÔNG --compensate, để xe quay tự nhiên):
      trước khi chạy dán băng keo theo 3 bánh / khung xe. Sau: (a) user đo khung xe dịch bao nhiêu (tâm có dịch thật
      không; VO nói 4–9 cm mỗi 90°), (b) map_check so ảnh đầu – ảnh cuối (cùng hướng nhìn) → góc thật = 360° + phần dư
      ảnh (gần như không phụ thuộc fx) → VO đọc đúng / dư bao nhiêu.
- [ ] 6. Đề xuất sửa theo số đo của 5 (CHỜ USER DUYỆT, vì đổi hằng số hiệu chuẩn): fx qua `calib_floor.py` với mốc
      dán ngang (~10 phút) hoặc từ tỉ số 360°; lens forward_m (dọi) hoặc ràng buộc "tâm đứng yên" trong leg ROTATE
      nếu tâm thật không dịch. fx / mount đổi → hệ số `vo_scale.json` hết hiệu lực (đang khoá theo mount, cần khoá
      thêm intrinsics) → đo lại bằng leg tiến 80 cm của S3 + thước.

- [ ] 6a. (user duyệt 23:30) Hệ số góc quay VO: `vision/vo_scale.json` thêm `turns` [{run, truth_deg, vo_deg}] (manual_360:
      359.47 / 398.38) → `frame_motion.vo_turn_scale(mount)` = Σthật / ΣVO của đúng mount (0.9023); áp vào drive_map
      (tư thế, số đo leg quay, alignment.txt, map_meta), leg_odometry.measure_leg / leg_executor (bù quay), map_check
      (đo lại pha, closure); mẫu quay cũ trong slip_profiles.json học với VO chưa hiệu chỉnh → xử lý; test offline.
- [ ] 6b. (user duyệt 23:30) `ROBOT_RADIUS_M` 0.21 → **0.2217** (tam giác bánh cạnh 38.4 cm, 38.4/√3) + `WHEEL_POS_*`;
      trần chassis đổi (quay chậm hơn ~5 %) → drive_config; build lại Jetson (libmc/libmv/link/seq/robot_link) + test C++;
      build firmware + nạp ESP32; test_drive / test_leg_executor / test_drive_map; docs (overview, interfaces).

### Pha C — Kịch bản nhiều vị trí (robot + user)
- [ ] 7. Chuẩn bị: sàn trống ≥ 1.3 m trước A; băng keo khung xe tại A; thước đo 3–4 vật cố định (x trước, y trái
      tính từ tâm xe tại A) → `truth.json` (mẫu do map_check in ra). Mỗi kịch bản MỘT lệnh = một run, `--compensate`:
      - S2 `"đi 80 cm rồi quay đầu"` — 2 vị trí nhìn ngược nhau.
      - S3 `"đi 80 cm, quay đầu, đi 80 cm rồi quay đầu"` — khứ hồi: về A, cùng hướng → pose cuối vs băng keo.
      - S4 `"quay trái 90 độ, quay phải 180 độ, quay trái 90 độ, đi 80 cm, quay trái 90 độ, quay phải 180 độ,
        quay trái 90 độ"` — quét ở A + quét ở B (đúng trường hợp user nêu).
- [ ] 8. drive_map + map_check từng run → bảng đạt / không đạt theo EO 3, kèm stops.png; báo user.

### Pha D — Sửa (tuỳ kết quả C; chờ user duyệt lại phạm vi)
- [ ] 9. Sai do trôi hướng VO: (a) giảm nhiễu yaw (trung bình nhiều pha keyframe khi quay, theo số đo bước 2);
      (b) drive_map đăng ký bản đồ con mỗi điểm dừng vào bản đồ chung bằng scan_match (khởi tạo = pose VO, đỉnh
      không rõ → giữ VO + cờ) → pose đã sửa cho các frame của điểm dừng đó; (c) leg_executor và drive_map dùng
      CÙNG một phép đo cho mỗi leg (hiện lệch tới 4.7° cuối run → pose realroom ≠ bản đồ). Chạy lại map_check trên
      S2–S4 đã ghi, không cần chạy robot lại.
- [ ] 10. Nhiều lệnh riêng (mỗi lệnh 1 run): `drive_map --publish --merge` đặt run vào bản đồ realroom đang dùng tại
      `start_pose` của nó (+ scan_match), realroom giữ danh sách run thành viên + phép đặt, dựng lại lưới gộp từ
      log-odds. Test: S4 chạy thành 3 lệnh riêng → bản đồ gộp khớp bản đồ S4 một lệnh (map_check giữa hai bản).
- [ ] 11. Docs (interfaces: map_check / check.txt / truth.json / stops.json, `--merge`; vision + realroom README),
      cập nhật explore-map bước 8–9, `/code-standards-review` + `/code-logic-review`.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[20:32] intake + plan | info: run 184936 dựng lại trong scratchpad: VO 90° → 95–99.5° (tỉ số 1.06, fx chưa đo), độ lặp quay 1.7°/90°, trục quay +3.9 cm, drift đứng yên 0.02° | risk: fx / mount đổi làm vo_scale.json hết hiệu lực; leg lùi còn lỗi firmware → test chỉ tiến + quay

[20:40] approved | user "approve"; 3 câu hỏi không trả lời → mặc định (1 lệnh / kịch bản, ngưỡng đề xuất, fx chọn ở bước 6)
[20:55] step 1: `vision/scan_match.py` match(ref, moving, pivot) → Match(dx, dy, dθ, spread, trusted, why): likelihood field bilinear (σ 5 cm) − 0.5·ô trống, lưới thô 5 cm/1° ±0.3 m/±8° rồi tinh 1 cm/0.2°; tin = mỗi trục profile tụt ≥ 10 % trong ±0.1 m/±3°, không nằm ở mép cửa sổ | info: thử phòng giả: lỗi (7, −4 cm, 3°) / (−12, 5 cm, −5.5°) → sau sửa còn ≤ 2 cm / 0.1°; lỗi < nửa ô (2 cm, 0.7°) không thấy được (hai map cùng ô); hành lang 2 tường → "not pinned along dx"; 0.23 s / lần | risk: độ phân giải = ô 5 cm → ngưỡng test ±2.5 cm (không phải ±2 cm như plan)
[21:25] step 2: `vision/map_check.py` (+ stops.png / check.txt / stops.json): điểm dừng = khung ngoài leg FORWARD đang chạy (leg dời < 0.15 m không tách), `--sweeps` tách thêm chỗ đổi chiều quay; bản đồ con theo luật drive_map; cặp: scan_match + khoảng cách vật cản trung vị / p90 trước–sau; closure = ORB + LK khớp thẳng ảnh đầu–cuối; truth; VO: drive_map vs executor + đo lại 5 pha keyframe; `--truth-template`; override camera như drive_map | info: thử trên bản sao 184936 `--sweeps`: **ảnh đầu → cuối = +2.93° thật** (executor tin −0.05°, drive_map +4.71°) → leg bù theo VO để xe lệch 2.9°; đo lại 5 pha: lệch 1.8–3.0°/90° (VO quay ±1°); sweep 2 vs 3 lệch −2.2° (bóng ma thấy ở tường xa); 1 cặp frame nhoè (47°/s, 28 track) hỏng mọi pha → lấp bằng trung bình cặp kề | info: `pyflakes` trên PATH là python2 → dùng `python3 -m pyflakes`; ~1.5 phút / run (đo lại pha chiếm phần lớn, `--no-repeat` bỏ)
[21:50] step 3: `vision/test_map_check.py` 7/7 (scan_match: phòng giả 4 lỗi pose → ≤ 3.5 cm / 0.5°; hành lang / bản đồ 1 ô / lỗi ngoài cửa sổ → không tin; ghost; tách điểm dừng + leg bù + --sweeps; closure 360° đọc dư 6 %; truth + load_truth; 5 pha keyframe trên sàn render ±0.2°; closure ảnh render ±5 mm / 0.1°, ảnh nhìn ngược → None); test_drive_map 11/11, test_realroom 5/5, test_leg_executor ALL OK, pyflakes sạch | risk (đã sửa, LỆCH PLAN EO3): trung vị khoảng cách vật cản KHÔNG thấy bóng ma (phòng giả dời 10 cm → trung vị 5 cm, vì tường dọc hướng lệch vẫn trùng chính nó) → thay bằng **tỉ lệ ô vật cản không có ô tương ứng trong 7.5 cm ≤ 20 %** (184936 các sweep: 12–16 %; dời 5 cm 4 %, 10 cm 33 %, 15 cm 60 %); trung vị / p90 giữ để tham khảo | info: độ phân giải scan_match ~3 cm (ô 5 cm, hai map lượng tử độc lập)
[21:58] step 4: baseline 184936 (`drive_map` + `map_check --sweeps`, ghi vào run/map, không publish): 3 sweep; ghost 16 / 16 / 12 % (PASS); pose 1-2 5 cm / 0°, 1-3 5 cm / 0° (PASS), **2-3 5.8 cm / −2.2° (FAIL)**; closure ảnh đầu→cuối **+2.93° thật** vs tích phân +4.71° → trôi 1.78° (PASS ≤ 3°); VO/lệnh 90° = 1.06–1.10 cả 4 leg; 5 pha: lệch 1.7–3.0° / 90° (TB 2.1°); trục quay +3.9 cm | info: xe THẬT lệch +2.9° sau L90/R180/L90 có bù — bù theo VO không giữ được hướng vì VO quay chỉ lặp lại ±1°/90° | chờ user: bước 5 (S1 quay 360° + băng keo)
[22:15] standards-review (3 pass; scan_match, map_check, test_map_check): 19 sửa, 0 hoãn | scan_match: bỏ chặn 1e3 thừa (exp(−inf) = 0), NO_SPREAD, logger.debug kết quả thô/tinh, `cell_points` dùng chung (map_check bỏ bản chép); map_check: CLOSURE_LK_LEVELS, ARROW_LINE_PX / TRACK_LINE_PX / cv2.FILLED, NO_VALUE, docstring pair_lines, hàm lồng truth_report lên đầu, repeat_turn chịu frame đầu không đọc được (trước: AttributeError), debug log tách điểm dừng / cặp / closure; test: lambda → def, TIMELINE / START / FULL_WEIGHT / CORRIDOR_ERROR / TURNED_AROUND / TURN_OVERREAD / END_* / LANDMARK_XY, MIN_HITS_PER_CELL thay 3 | info: test_map_check 7/7, pyflakes sạch (python3 -m)
[22:25] logic-review (2 pass; đối chiếu bước 1–4 + EO, interfaces, history 09-28/29; chưa có agent/plan/vision_plan.md → task file là plan): 1 sửa, 1 báo user | sửa HIỆU NĂNG: truth_report tính lại occupied_points của mỗi map cho TỪNG vật mốc → tính một lần / map (đã kiểm standards phần sửa: sạch) | báo user: ngưỡng lệch pose 5 cm ≈ độ phân giải scan_match (~3 cm, baseline ra đúng 0.050 → PASS sát biên) → ghost share là phán quyết chắc hơn; ngưỡng không đổi | info: lệch plan có chủ đích đã ghi (ghost share, dung sai 3.5 cm); --sweeps / closure / stops.json nằm trong plan (bước 4 cần so trong 1 điểm dừng, 5b, 11); test_map_check 7/7, baseline 184936 không đổi, check.txt đầy đủ ghi lại
[21:15] step 5 (thử 1, run 210957 "quay trái 360 độ" không bù): 360 bị chia 3 × T 120 (TURN_CHUNK_MAX_DEG 170: SEQ gói hướng vào (−180, 180]), mỗi leg một hình thang riêng (MC 4.72 s, tăng/giảm tốc 2.1 s mỗi đầu) → user thấy "ngắt quãng, không giữ vận tốc"; **cả 3 leg bị firmware CẮT**: K sau 2.73 / 3.20 / 3.06 s (58–68 %), VO: chạy 2.16 / 2.73 / 2.56 s, đỉnh 47–58°/s (đúng tốc độ) → cả hình thang bị nén thời gian ~1.7–2.2× như leg lùi 09-28 → xe quay **+237.6°** (user: thiếu ~10 cm nữa mới tới 2/3 vòng) | risk: lỗi đồng hồ firmware nay có ở leg QUAY (T120 trong 1 plan nhiều leg); 184936 (T90, mỗi leg 1 lần gọi --compensate) không bị; chưa rõ cơ chế | risk: chạy thường không đo → realroom giữ pose cũ (predicted = lệnh) trong khi xe thật lệch ~122° → phải đặt xe lại theo băng keo | info: closure không dùng được (ảnh đầu/cuối lệch 122°); log robot_link `undecodable=2`
[21:40] step 5 (thử 2, run 211656 `--compensate` 4 × "quay trái 90 độ", user tự chạy theo đề xuất): **12 / 12 leg bị firmware cắt** (T90 chính: K sau 2.4–2.9 s / 4.08 = 59–70 %; cả leg bù) — lần này cả leg gọi riêng cũng bị (184936 chiều nay thì không); executor bù ≤ 2 lần mỗi 90° → VO 88.3 / 86.6 / 88.5 / 86.4 = **349.75°** (drive_map 351.45°); user: xe còn thiếu một cung có **dây cung 12 cm** → nếu đo ở bánh (0.21 m từ tâm) = 33.2° → thật ≈ 326.8° → VO đọc dư ~7 % (khớp 1.06–1.10 ở 184936) | info: closure ảnh đầu–cuối không khớp (ORB 12 cặp, AKAZE ~0 điểm trên sàn); thử BEV (chiếu sàn từ trên + NCC): tự kiểm đúng ở 5° (NCC 0.84), đọc NHỎ hơn VO khi góc lớn (VO 14.56 → 14.0, 29.35 → 27.0), ảnh cuối không có đỉnh (chung sàn quá ít) | info: quét fx (scratchpad, không ghi run): VO tổng 570 → 351.45, 600 → 341.19, 610 → 337.49, 620 → 334.42; muốn 326.8 cần fx ≈ 640, và trục quay càng xa tâm (+0.063 → +0.079 m) → fx một mình KHÔNG giải thích | risk: trục quay theo ảnh +6 cm trước tâm → dây cung phụ thuộc bánh nào (W1 cách trục ~0.15 m → 48°, bánh sau ~0.25 m → 28°) → cần dây cung cả 3 bánh | risk: realroom pose sau run = VO 348° trong khi xe thật ≈ 327°
[21:55] user: 3 bánh là tam giác đều cạnh **38.4 cm** → tâm → điểm chạm bánh = 38.4/√3 = **22.17 cm** (constants.h ROBOT_RADIUS_M 0.21, WHEEL_POS_* theo 0.21) → mọi lệnh quay vật lý thiếu ~5.3 % trước trượt; dây cung 12 cm ở bán kính 22.17 → 31.4° → thật ≈ 328.6° → VO/thật 1.064–1.070; user chọn **đo lại fx** (bước 6 = calib_floor) | risk: số liệu chưa khớp nhau: 184936 (T90 không bị cắt, vật lý ≤ 85.3° nếu L 0.2217) VO 95–99° → dư 11–16 %, còn dây cung 211656 → dư 6–7 % → cần fx đo tĩnh để phân xử; đổi L = đổi constants.h + build lại + nạp ESP32 → task riêng, chờ user xác nhận 38.4 đo giữa điểm nào
[22:20] step 6 (calib_floor lần 1, offline trên ảnh chụp TB 10 frame, --no-save; user: O = hình chiếu camera, h 0.24): mốc 1/2/4/5 là vệt băng keo (tâm vệt tự động: (331.5, 426.5) / (315.6, 367.9) / (108.0, 374.6) / (502.8, 363.9)), mốc 3 vẫn là bình xịt → bỏ, check không đặt → fit **fx 589–594, pitch −0.91…−1.02°, cx 315 (yaw −0.4…−0.5°)**, RMS 8 px, **FAIL 3.0 % > 2.5 %** | risk: hình học mốc không khớp: mốc 1 lệch 16 px (~2 cm) ngang so với mốc 2 (cùng trục y = 0 thì phải cùng cột u), mốc 4 thấp hơn mốc 5 10.7 px (~7 cm gần hơn) — khớp đúng với đặt mốc 4 theo đường chéo 1.20 m (x thật 1.13) thay vì x 1.20 (chéo phải 1.265) | info: fx ~592 chỉ giải thích ~½ phần VO đọc dư (quét fx: 600 → 341°, thước ~327–329°)
[22:40] step 6 (calib_floor lần 2, user thay mốc 3 bằng giấy trắng + vạch đen; mốc 1/4 chưa sửa): mốc 3 = (313, 319) (tâm tự động bắt nhầm khe gạch dọc → đọc tay theo hàng tối nhất); mốc 2 và 3 thẳng cột (u 314–316), **mốc 1 lệch 17 px ≈ 2.3 cm sang phải** | fit (a) 5 mốc theo plan: fx 589.4, pitch −0.99°, RMS 7.3 px, FAIL 2.8 %; (b) bỏ mốc 1: fx 589.8, RMS 4.3 px, FAIL 3.1 %; **(c) bỏ mốc 1 + mốc 4 đặt theo đường chéo 1.20 m (x 1.131): fx 571.9, pitch −1.17°, cx 312.7 (yaw −0.68°), RMS 1.8 px, PASS 1.6 %** | info: nếu (c) đúng thì fx 570 vốn đúng → VO đọc dư khi quay KHÔNG do fx | chờ user: đo thẳng O→4 và O→5 (1.20 hay 1.265 m), sửa / đo lại mốc 1
[23:05] step 6 (tiếp): user đo O→4 = 1.25 m (đúng 1.265, chéo 1.20) → mốc 4 x ≈ 1.184 → giả thiết (c) SAI; row mốc 4 vs 5 lệch 10.7 px ≫ 1.6 cm → **camera roll**; fit có roll (scratchpad, xoay điểm ảnh quanh tâm quang học; calib_floor không có roll): **fx 586.0, pitch −1.012°, cx 307.9, roll −1.25°, RMS 3.05 px, PASS 1.2 %** (mốc 2–5; mốc 1 lệch 2.3 cm bỏ) | VO với hiệu chuẩn mới (vá floor_hit/floor_project có roll, không ghi cache): 211656 tổng 351.45 → 346.55°, 184936 T90 95.0/−96.4/−97.6/99.5 → 93.3/−94.8/−96.7/97.3, trục quay vẫn ~+5.7 cm → hiệu chuẩn chỉ giải thích ~¼ phần đọc dư | thử chuẩn bằng ảnh: toàn cảnh ORB+homography chỉ tin ≤ ~15°; "con quay cảnh" (trên chân trời, 3 frame/cặp): 211656 = 317.8°, nhưng từng leg lệch VO từ 0.2 % tới ×4 → không tin được | risk: chưa có chuẩn góc quay đáng tin (thước phụ thuộc bánh + điểm xoay) → đề xuất xoay TAY 1 vòng khớp lại băng keo (motor tắt nguồn) + record.py → chuẩn = 360° + closure ảnh đầu–cuối
[23:20] step 5 (chuẩn góc quay, run `vision/output/manual_360`: user xoay TAY 1 vòng trái, motor tắt, 44 s, 1326 frame): closure ảnh đầu→cuối −0.53° / 3 cm (187 inlier) → **thật 359.47°**; VO sàn hiệu chuẩn cũ **398.38° (+10.8 %)**, hiệu chuẩn mới có roll 392.32° (+9.1 %) → **VO sàn đọc dư góc quay ~10.8 %, có hệ thống**, hiệu chuẩn camera chỉ giải thích ~⅙ | info: hệ số 0.9023 làm khớp các run cũ: 211656 351.45 → 317.1° (con quay cảnh 317.8°), 184936 T90 → 85.8–89.7° (giới hạn không trượt với L 0.2217 = 85.3°) → đây là NGUYÊN NHÂN CHÍNH của bản đồ nhiều vị trí hỏng: quay đầu 180° lệch ~18°, và leg_executor bù quay theo VO nên xe thật quay thiếu ~10 % | risk: nguyên nhân gốc trong mô hình sàn chưa rõ (dịch VO đọc NGẮN 6 %, quay đọc DÀI 10.8 % → nghi méo không đẳng hướng dọc/ngang); mới 1 mẫu, 1 chiều quay | chờ user: duyệt hệ số quay VO (sửa code) + lặp lại xoay tay (phải, trái)
[23:55] step 6a: `frame_motion.vo_turn_scale(mount)` (+ `_scale_data` dùng chung với vo_scale), `vo_scale.json` `turns` [manual_360 359.47 / 398.38] → 0.9023; `leg_odometry.odometry_track / measure_leg(turn_scale)`; drive_map: align_legs / integrate_poses (turn_scale; biến lấp lệnh đổi tên turn_fill), build, DriveMap.vo_turn_scale, `--vo-turn-scale`, alignment.txt + map_meta; leg_executor đo leg với vo_turn_scale, run.json `vo_turn_scale`; map_check: đo lại pha × turn scale, closure trực tiếp × vo_scale / vo_turn_scale (trước: tịnh tiến closure không nhân vo_scale — lệch với pose), `--vo-turn-scale`; slip_profiles.json: 4 mẫu quay đã học (1.053–1.094, VO chưa hiệu chỉnh) → không học, why "stale" (sao lưu scratchpad) → k quay về mặc định 1.0 | info: test_drive_map 12/12 (+check_vo_turn_scale, check_vo_scale thêm turns), test_map_check 7/7 (+closure theo turn scale), test_leg_executor ALL OK (+test_turn_scale_reaches_the_odometry), test_drive ALL OK, pyflakes sạch | info: MAX_TURN_RATE 0.8 rad/s giờ so với tốc độ quay THẬT (trước: VO, dư 10 %) → ít frame sàn bị bỏ hơn
[00:20] step 6b: constants.h ROBOT_RADIUS_M 0.21 → **0.2217** (0.384/√3) + WHEEL_POS (0.2217, 0) / (−0.11085, ±0.191998); build lại libmc / libmv / link / seq / test_rm (tay, cờ build_mc) / robot_link → test_rm / mc / mv / link / seq PASS; firmware: host test_motion PASS, `esp32_flash.sh --no-monitor` → **nạp ESP32, 3 vùng hash OK**, `--stop` 0, `--monitor` telemetry sạch (undecodable=1 lúc mở cổng); T90 → **17736 bước / bánh** (trước 16800, +5.6 %), 4.18 s, đỉnh 8486 Hz; trần mới 1.77 rad/s quay, 0.361 rad/s² → drive_config MAX_YAW_RATE_RAD_S 1.8 → 1.75, stream_bench; test_drive_map check_run_scale lấy thời lượng từ chính MC (ROTATE 6.08 s, trước ghi cứng 6.04); docs: overview (L, 197.1 bước/°, trần, vo_turn_scale), interfaces (vo_scale.json turns, run.json vo_turn_scale, trần), system_architecture, project/RM/MC README | info: test_drive / test_leg_executor ALL OK, test_drive_map 12/12, test_map_check 7/7, test_realroom 5/5, test_vision, test_mapping PASS | risk: simulation (room_generator 0.21) và hình xe realroom (lấy từ simulation) giữ 0.21 — module riêng, chỉ ảnh hưởng hình vẽ
[00:30] standards-review 6a/6b (2 pass; frame_motion, leg_odometry, drive_map, map_check, stream_bench, leg_executor, drive_config, constants.h, 3 test): 1 sửa, 0 hoãn | drive_map: dòng map_meta nối vo_turn_scale dài 156 ký tự → tách | info: codebase vốn dòng ≤ ~135, tên / docstring / hằng / logging đạt; pyflakes sạch (map_builder `sys` thừa có từ trước, ngoài phạm vi); test_drive_map 12/12
[00:40] logic-review 6a/6b (2 pass; đối chiếu plan 6a/6b, interfaces, overview, history 09-28/29): 1 sửa, 0 báo user | sửa: map_check cột "executor" đọc `executed_legs.measured` ở thang của lúc chạy (run cũ = 1.0) cạnh cột drive_map ở 0.9023 → lệch giả 10 %; nay quy về dm.vo_turn_scale bằng run.json `vo_turn_scale` (thiếu = 1) | info: kiểm đúng: cache motion.npz vẫn hợp lệ (VO thô, hệ số nhân sau), turn_fill lấy tỉ số đã hiệu chỉnh, executor bù + slip học theo góc đã hiệu chỉnh, firmware / libmc / robot_link cùng L mới, run cũ rơi về hình thang thời lượng ghi; rotation_axis giữ VO thô (chẩn đoán) | info: baseline 184936 sau hiệu chỉnh: T90 → 85.8–89.7° (drive_map ≈ executor ≤ 1.6°), sweep 2–3 **5.8 cm / −2.2° FAIL → 0 / 0° PASS**, ghost 16/16/12 → 11/11/9 %, closure trôi 1.78 → 1.59°; test_map_check 7/7, test_drive_map 12/12, test_leg_executor ALL OK
[00:45] review gate — standards (2 pass) trên phần sửa của logic-review (map_check vo_report / main): 1 sửa, 0 hoãn | `LEGACY_TURN_SCALE` thay `1.0` viết thẳng ở 2 chỗ (mẫu drive_timeline.LEGACY_WHEEL_RADIUS_M) | info: pyflakes sạch, test_map_check 7/7
[00:47] review gate — logic (1 pass) trên cùng phần: 0 sửa, 0 báo user | info: đo executor = VO thô × thang lúc chạy → × dm / thang lúc chạy = cùng thang drive_map (run mới: × 1; run cũ không khoá: LEGACY 1.0; --vo-turn-scale ghi đè: cả hai cột theo); executor luôn ghi thang > 0 → không chia 0; 184936 hai cột khớp ≤ 1.6°
[23:27] step 8 (S4, run 231612, L 0.2217 + vo_turn_scale 0.902, `--compensate`; user: dựng lại realroom từ kết quả mới nhất): drive_map → **publish** (realroom `latest.json` = map 20260929_231612: phòng 4.9 × 3.4 m, 16 vật, robot (1.71, 1.91) +11.6° theo VO; sao lưu maps cũ ở scratchpad `maps_backup_before_231612`), app :5002 chạy lại | map_check: 2 điểm dừng, **ghost 53 % FAIL** (sau match 33 %), scan_match không tin (không ghim trục nào), closure n/a | risk: run kết thúc "failed" (bước 5 "stuck", R180 thứ hai chưa xong); **8/14 leg bị firmware CẮT**, gồm leg quay chính 3 (−76°) và 13 (−63.5°) → bản đồ + pose chưa đạt EO3 | info: leg quay không bị cắt đọc 89.2–95.0° (TB 1.04 ở thang 0.902); trục quay +4.1 cm; độ lặp pha 1.82°/90°; forward 0.889 + bù −0.092

## Test result
