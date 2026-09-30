---
id: 2026-09-27_explore-map
status: executing   # 09-27 duyệt "cách a"; 09-28 user duyệt Pha 1' (VO là chuẩn) + hệ số thước đã đo
module: vision (+ communication)
started: 2026-09-27
---

## Task
Dựng bản đồ cả một phòng bất kỳ: robot tự di chuyển giữa nhiều điểm dừng (quét → ghép → chọn điểm tiếp →
đi an toàn → quét lại). Trọng tâm (user 2026-09-27): **hạn chế trượt bánh** bằng cơ chế thích nghi đo sai số
lệnh vs thực tế theo từng loại sàn.

## Input
- User 2026-09-27 (lần 1): map từ việc tự quay đã chạy (run `vision/output/20260926_173930`), nhưng một vị trí
  không đủ overview → xin cơ chế di chuyển.
- User 2026-09-27 (lần 2):
  - Cản trở chính là **trượt bánh** → lệnh không bám thực tế → cần cơ chế hạn chế.
  - **Vùng tin cậy**: 0.8 m phía trước robot lúc khởi chạy đầu tiên hoàn toàn trống (user bảo đảm).
  - Tin kết quả odometry, nhưng vẫn giữ hiệu chỉnh.
  - Trượt khác nhau theo loại sàn → **cơ chế thích nghi**: sai số trung bình lệnh vs thực tế, ví dụ lệnh 30 cm,
    đo 25 cm → trượt 5/30; chạy nhiều lần, lưu trung bình.
- User 2026-09-27 (lần 3): 0.8 m tính **từ tâm xe**; tên sàn **hệ thống tự nhận**; phần cứng (IMU / optical
  flow) bổ sung sau; hỏi chờ READY 2 s mỗi leg ảnh hưởng gì (trả lời: thời gian, không phải độ chính xác/an toàn).
- User 2026-09-28: Jetson có nguồn riêng, motor có nguồn riêng; sửa nguồn / rớt hub USB để SAU. Ưu tiên bây giờ:
  **di chuyển để tạo bản đồ**, **tin VO** khi di chuyển.
- Sự thật đã kiểm (2026-09-27):
  - Chờ READY: `sequencer.cpp` trạng thái WAIT_READY chờ tới `READY_WAIT_MS` 2000 rồi mới gửi leg đầu, MỖI lần
    gọi `--run-plan` (run 173930: status leg 1 lúc +0.035 s, xe chạy thật +2.25 s). `--stop` không qua sequencer
    (gửi `S` + settle), không chờ. Trong lúc chờ xe đứng yên.
  - **"Odometry" thấy được trượt = odometry thị giác (VO sàn, `frame_motion`)**. Odometry firmware `O` tích
    phân từ số bước ĐÃ LỆNH (stepper, không encoder) → luôn báo đúng 30 cm, không bao giờ thấy trượt.
    VO tự nó phụ thuộc h 0.24 m + pitch −0.92° (giá camera không cứng) → phải kiểm VO bằng thước dây một lần
    và mỗi khi chạm camera: thước dây → VO → hệ số trượt bánh.
  - VO dịch chuyển tịnh tiến CHƯA thử trên leg FORWARD thật (drive-map 4b treo); yaw VO tốt (bước ≤ 5.4°/cặp).
  - (user lần 4: sàn trắng) Khoảng cách KHÔNG lấy từ độ sâu DA: `floor_hit` = hình học sàn (h, pitch, fx); DA chỉ
    phân loại pixel sàn / vật cản / hụt. Điểm yếu thật của sàn trơn là VO (Shi-Tomasi + LK trên dải sàn ≤ 2.5 m).
    Thí nghiệm (scratchpad `texture_sweep3.py`, 72 cặp quay stride 5 của run 173930, vân sàn trong dải + 40 px phía
    trên nhân α, giữ bóng sáng σ 150 px, thêm lại nhiễu cảm biến 4.8 mức xám; sàn hiện tại: độ lệch cục bộ 13.4):
    α 1 → 72/72 cặp, 248 inlier; 0.25 → 70/72, 154, sai yaw 0.04°/cặp; 0.10 → 53/72; 0.05 → 31/72, sai 0.29°
    (p95 0.94°); 0 → 4/72 = VO chết. Chỉ thử leg QUAY (chưa có dữ liệu leg tiến), nhiễu mô phỏng độc lập từng frame.
  - `F` âm = đi lùi (`MotionState::beginForward` lấy dấu làm hướng; cmd_parser có "lùi").
  - Số đo cũ: quay thật/lệnh 0.79 (fx 570), trái đi xa hơn phải ~6.5°/180°; quãng 0.733 (r_eff 4.03, chưa đo lại).
  - Cảm biến: chỉ RGB Astra, FOV ngang ~58.6°, sàn DA tin được ≤ ~2.5 m. Vùng mù: tia đáy ảnh chạm sàn 0.60 m
    trước ống kính ≈ 0.78 m trước tâm xe → vùng tin cậy 0.8 m phủ đúng vùng mù phía trước lúc khởi động.
  - MV planner chỉ nhận phòng chữ nhật + polygon, không nhận occupancy grid.
  - Mỗi lần gọi `robot_link --run-plan` chờ READY tới 2 s (READY_WAIT_MS) và từng hỏng dòng đầu (E2).

## Expected output
1. **Hệ số trượt thích nghi**: `communication/slip_profiles.json` theo sàn TỰ NHẬN (`san_01`, `san_02`… kèm ảnh
   mẫu `slip_profiles/san_XX.png` để user xem đó là sàn nào) × loại chuyển động
   {tiến, lùi, quay trái, quay phải}: k = quãng đo / quãng lệnh, % trượt = 1 − k, n, độ lệch chuẩn, lệch
   ngang/lệch hướng mỗi mét khi đi thẳng. Cập nhật sau MỖI leg; in bảng sau mỗi lần chạy.
2. **Leg bù trượt**: `demo_drive --compensate "đi 30 cm"` → gửi 30/k, đo lại bằng VO, bù phần thiếu nếu
   lệch > 3 cm / 2° → thước dây ra 30 ± 3 cm (hiện tại lệnh 30 cm chỉ đi ~22 cm).
3. **Hiệu chuẩn khởi động** trong vùng tin cậy 0.8 m (~2 phút): `explore_room.py --calibrate-only`.
4. `python3 communication/explore_room.py [--auto] [--max-stops N]`: tự quét–đi–quét trong phòng;
   `vision/output/<run>/map/`: `map.png`, `stops.json`, `slip.txt`, `map.mp4`, `scene.json`. Không `--auto`:
   hỏi user Enter trước mỗi lần đi.
5. Nghiệm thu phòng thật: vật đứng yên thấy từ ≥ 2 điểm dừng lệch ≤ 10 cm; về điểm xuất phát sai ≤ 15 cm
   (thước dây); không va chạm.
6. Test offline không camera/robot: bộ ước lượng hội tụ + loại ngoại lai + phát hiện đổi sàn; leg executor
   với phép đo giả; scan_match tìm lại dịch/xoay đã cài; khám phá mô phỏng phủ ≥ 90 %.

## Plan (lập lại 2026-09-27 theo phản hồi user — chờ duyệt)

### Chống trượt: 4 tầng
1. **Đo thật**: pose lấy từ VO sau mỗi leg (lệnh × k chỉ là dự đoán / dự phòng khi VO kém).
2. **Bù trước (feedforward)**: lệnh gửi = mục tiêu / k của sàn hiện tại.
3. **Sửa sau (feedback theo leg)**: đo xong còn thiếu > 3 cm / 2° → gửi leg bù (tối đa 2).
4. **Học liên tục**: mọi leg (hiệu chuẩn, quét, di chuyển) là một mẫu (lệnh trên dây, đo được) → cập nhật k.

### Pha 0 — Thước đo tin cậy (cần robot)
- [x] 1. Orchestrator gọi `robot_link --stop` trước mỗi `--run-plan` (tránh E2 dòng đầu, không sửa C++).
- [x] 2. Kiểm VO bằng thước dây trong vùng tin cậy: tiến 0.4 m rồi lùi 0.4 m (×2), user đo thước quãng tiến thật →
      sai số VO; đạt khi ≤ 3 %, không đạt thì đo lại pitch (`calib_floor`) trước khi đi tiếp.
      Lặp lại trên SÀN TRẮNG THẬT: trải giấy / tấm xốp trắng phủ vùng tin cậy → VO còn đo được bao nhiêu (thay cho
      thí nghiệm mô phỏng).

- [x] 2b. (sau bước 2, chờ user duyệt) Sửa hằng số hình học theo số đo user 2026-09-27: `WHEEL_RADIUS_M` 0.055 →
      **0.040** (⌀8 cm); `ROBOT_RADIUS_M` 0.21 → 0.215 − w/2 (21.5 cm = tâm xe → mặt NGOÀI bánh; cần bề rộng bánh w);
      `WHEEL_POS_*`; build lại Jetson (robot_link, libmc, libmv) + project tests; build + nạp lại firmware ESP32
      (`tools/esp32_flash.sh`); `run.json` ghi bán kính bánh đã dùng để lệnh → drive_map scale = 0.040 / r của run
      (run cũ 0.727, run mới 1.0) thay cho 4.03/5.5 cố định; docs (project_overview) + memory. Chạy lại thước:
      kỳ vọng ~37 cm cho lệnh 40 cm (k ≈ 0.93) + một vòng quét 4×T135 (khép vòng → tỉ số quay thật, gỡ L và fx).

### Pha 1' — VO là chuẩn khi di chuyển (lập lại 2026-09-28 theo user — CHỜ DUYỆT)
Lý do: lệnh không bám thực tế vì 2 nguyên nhân khác nhau — trượt (tiến k 0.94) và leg lùi bị ESP32 cắt ngắn ~40 %
(đồng hồ vòng motion nhanh ~1.6×, chưa rõ cơ chế, sửa nguồn để sau). VO đo được cả hai → pose = VO, lệnh chỉ là dự đoán.
- [x] A. `drive_map` lấy VỊ TRÍ từ VO (dx, dy mỗi cặp frame, quay theo heading VO), không còn từ quãng đã lệnh; cặp VO
      hỏng mới lấp bằng lệnh × tỉ số đo của chính leg đó (như heading đang làm). alignment.txt: mỗi leg lệnh / VO / % cặp
      + cờ `fw_short` khi K về sớm hơn thời lượng MC > 10 %. Test: 5 run 09-28 → vị trí cuối khớp VO (35 / −23 / … cm),
      test_drive_map thêm run giả leg bị cắt.
- [x] B. `vision/leg_odometry.py` (= bước 3 cũ, bản gọn): tách `measure_motion` khỏi drive_map để dùng chung;
      `measure_leg(run_dir, t0, t1)` → dx, dy, dθ, % cặp, inlier. % cặp < 75 → "VO yếu" → dùng lệnh × k, gắn cờ.
      Chuỗi dự phòng (khép vòng, mốc phía trước) → hoãn.
- [x] C. `communication/leg_executor.py` + `demo_drive --compensate` (= bước 5 cũ): recorder chạy SUỐT phiên; mỗi leg:
      lệnh = mục tiêu / k → `--run-plan` 1 leg → K + 0.5 s → đo VO (B) → thiếu > 3 cm / 2° thì gửi leg bù (tối đa 2)
      → trả pose VO. Leg lùi bị cắt 40 % hội tụ sau 2 leg bù (40 → 24 → 34 → ~38). Camera mất (hub rớt) → dừng, báo.
- [x] D. `communication/slip_model.py` bản gọn (= 4b cũ, 1 hồ sơ sàn `san_01`, chưa tự nhận sàn): k theo
      {tiến, lùi, quay trái, quay phải} từ mẫu VO tốt; mẫu `fw_short` KHÔNG học (là lỗi firmware, không phải trượt).
- [x] E. Thang VO: VO/thước hôm nay 0.945 (hôm qua 0.974) → nhạy pitch (~0.75° ≈ 5.5 %). Chạy lại `calib_floor`
      (dán mốc trên sàn, ~10 phút, cần user) rồi kiểm 1 lần bằng thước; không kịp thì tạm nhân hệ số thước 1/0.945.
- [ ] F. Thử thật: `demo_drive --compensate "đi 40 cm"` / `"lùi 40 cm"` ×3 mỗi chiều → thước 40 ± 3 cm; T±90 → kiểm
      leg quay có bị cắt như lùi không.
- Khám phá (Pha 3) ưu tiên T + F TIẾN (MOVE vốn đã tách T rồi F dương) → tránh leg lùi cho tới khi sửa phần cứng.
- Hoãn tới khi di chuyển chạy được: 3b (kiểm vân sàn), 4a (tự nhận sàn), chuỗi dự phòng của 3, chẩn đoán nguồn / hub.

### Pha 1 — Cơ chế trượt thích nghi (bản 09-27; phần chưa nằm trong Pha 1' → hoãn)
- [ ] 3. `vision/leg_odometry.py`: chuyển động thật của một leg (dx, dy, dθ, chất lượng) từ raw frame giữa hai seq
      (tách `measure_motion` khỏi drive_map để dùng chung); chất lượng = tỉ lệ cặp đo được, inlier, rms.
      Chuỗi dự phòng khi sàn thiếu vân (mỗi kết quả ghi rõ nguồn):
      quay → khép vòng 360° của vòng quét (so cả ảnh: tường, đồ đạc, không cần vân sàn); tiến → chênh khoảng cách tới
      mốc chạm sàn phía trước (chân tường / vật, DA + hình học) trước và sau leg; cả hai hỏng → lệnh × k, gắn cờ.
- [ ] 3b. Kiểm vân sàn lúc khởi động (cạnh `light_check`): số góc bám được trong hành lang tin cậy + tỉ lệ cặp VO
      đạt trong chuỗi hiệu chuẩn → "đủ vân / yếu / không đo được" (ngưỡng từ thí nghiệm: < ~75 % cặp đạt = yếu),
      cảnh báo, lưu vào hồ sơ sàn (cũng là một thành phần chữ ký 4a). Sàn yếu → hop ngắn lại (≤ 0.8 m) để sai số
      còn nằm trong cửa sổ scan_match.
- [ ] 4a. `vision/floor_signature.py` — tự nhận sàn: trên pixel FREE của `floor/<seq>.png` cách xe 0.8–1.6 m, vector
      = histogram màu chuẩn hoá theo độ sáng (rg 8×8) + histogram texture (gradient / LBP) + tỉ lệ điểm bóng loá
      (sàn bóng thường trơn); trung vị qua nhiều frame. So với chữ ký các sàn đã lưu (chi-square): gần nhất trong
      ngưỡng → dùng hồ sơ đó; không → hồ sơ mới `san_NN` + ảnh mẫu, k ban đầu = sàn gần nhất / mặc định.
      Chữ ký lấy theo HÀNH LANG ĐƯỜNG ĐI của mỗi hop (pixel chiếu xuống sàn rơi vào đường sẽ đi, từ frame lúc quét)
      → phòng có nhiều loại sàn (gạch + thảm) vẫn chọn đúng k. Khởi động: chữ ký hành lang tin cậy 0.8 m.
      Thống kê trượt là trọng tài cuối: hình giống nhưng trượt khác ổn định → cảnh báo (sàn ướt/bụi?) + mở cửa sổ mới.
- [ ] 4b. `communication/slip_model.py` + `slip_profiles.json`: mỗi sàn (4a) × {tiến, lùi, quay trái, quay phải} lưu mẫu
      (lệnh, đo, nguồn đo, tốc độ, chất lượng, thời điểm; mẫu "lệnh × k" KHÔNG được học); k = Σđo / Σlệnh trên 10 mẫu hợp lệ gần nhất (leg dài nặng ký hơn,
      bám được sàn đổi); bỏ mẫu VO kém, leg bù < 10 cm / 10°, ngoại lai > 3 MAD; k < 0.5 → sự kiện KẸT, không
      phải mẫu; 3 mẫu liền ngoài ±3σ → cảnh báo "sàn khác", mở cửa sổ mới; n < 3 → dùng hệ số lưu của sàn hoặc
      mặc định (0.733 / 0.79). Tiến: thêm lệch ngang + lệch hướng mỗi mét → bù hướng. Có mẫu nhiều độ dài thì báo
      thêm fit đo = a·lệnh + b (trượt dồn ở pha tăng/giảm tốc hay tỉ lệ đều).
- [ ] 5. `communication/leg_executor.py`: mục tiêu → lệnh = mục tiêu / k → `--run-plan` 1 leg → chờ K + 0.5 s đứng
      yên → đo (3) → cập nhật (4b) → bù nếu > 3 cm / 2° (tối đa 2) → trả pose đo được. `demo_drive --compensate`.
      Giảm số lần chờ READY 2 s: leg KHÔNG cần sửa ngay (4×T135 của vòng quét, chuỗi tiến/lùi hiệu chuẩn) gửi chung
      MỘT plan, tách leg sau bằng `align_legs`; chỉ leg di chuyển có bù mới gọi riêng. Bỏ qua cặp frame đứng yên
      (chênh ảnh ≈ 0) khi đo để 2 s chờ không tốn thêm VO.
- [ ] 6. Hiệu chuẩn khởi động (vùng tin cậy): gieo ô trống hành lang 0.8 m × 0.51 m vào map (tính 0.8 m từ TÂM xe
      cho an toàn); [tiến 0.4, lùi 0.4] ×3 (về đúng vệt cũ, so frame đầu–cuối = trôi tổng); quét trái 4×T135 +
      quét phải 4×T135 (khép vòng = đúng 360° thật → mẫu quay chính xác nhất, tách luôn fx); so hệ số lưu của sàn.
- [ ] 7. Test offline: ước lượng hội tụ về k cài sẵn, loại ngoại lai, phát hiện đổi sàn ≤ 5 mẫu; executor với phép
      đo giả hội tụ trong ≤ 2 leg bù; chữ ký sàn: cùng sàn khác độ sáng ±30 % → cùng hồ sơ, sàn tổng hợp khác
      màu/texture → hồ sơ mới. Chạy thật: `demo_drive --compensate "đi 30 cm"` ×5 → thước dây 30 ± 3 cm.

### Pha 2 — Nhiều điểm dừng, lộ trình do người đặt
- [ ] 8. `vision/scan_match.py`: đăng ký local map vào map chung (dx, dy, dθ trong ±0.3 m / ±8°), likelihood field +
      phạt free↔occupied, thô rồi tinh; đỉnh không rõ → giữ pose đo + gắn cờ. Test: dịch/xoay cài sẵn → ±2 cm / ±0.5°.
- [ ] 9. `drive_map`: tách điểm dừng (chuỗi T) và hop; pose từ leg_executor (VO) → local map → scan_match → map chung +
      submap; `stops.json`, `slip.txt`. Chạy thật lộ trình tay 2 điểm dừng (hop 1 m) → không bóng ma.

### Pha 3 — Tự chọn điểm tiếp (hỏi user trước mỗi lần đi)
- [ ] 10. `vision/explore_planner.py`: đi được = chỉ ô FREE đã thấy (+ hành lang tin cậy), phồng 0.255 m; frontier gom
      cụm (bỏ < 0.3 m); điểm tiếp 0.8–1.5 m, gain = ô unknown thấy được trong 2 m − λ·đường; A* 8 hướng + rút gọn →
      ROTATE/FORWARD ≤ 1.5 m. Test mô phỏng phủ ≥ 90 %.
      (user lần 5: đề xuất bám tường) Thứ tự ưu tiên "bám tường": trong các ứng viên gain gần nhau (≥ 70 % gain cao
      nhất), chọn điểm tiếp nối dọc biên đã biết theo MỘT chiều quay (ngược chiều kim đồng hồ), cách tường ~1 m
      (≥ 0.9 m để chân tường nằm ngoài vùng mù 0.78 m khi nhìn vào) → vòng chu vi trước, sau đó mới lấp frontier bên
      trong. Frontier không giải quyết được (nhìn 2 lần vẫn unknown: bóng loá, sau vật) hoặc không tới được → danh sách đen.
      Rào ảo: không theo frontier xa > R (mặc định 8 m) tính từ điểm xuất phát hoặc qua khe hẹp < 0.7 m (cửa) trừ khi cho phép.
      Test mô phỏng thêm: phòng chữ L, phòng có bàn giữa, hành lang, phòng có cửa mở.
- [ ] 11. Look-before-go (sau T, pixel đáy hành lang 0.51 m phải FREE) + KẸT từ (4b) → `S`, đánh dấu chặn, chọn lại.
- [ ] 12. `communication/explore_room.py`: recorder suốt phiên, hiệu chuẩn (6) → vòng {quét → xử lý → chọn → hỏi → đi
      bằng leg_executor}, `explore.json`, Ctrl-C → `S`.

### Pha 4 — Tự động hoàn toàn
- [ ] 13. `--auto` + ngân sách (mặc định 15 điểm dừng / 20 phút) + về điểm xuất phát → sai lệch khép vòng lớn.
      Tự xử lý sự cố, không cần người: KẸT → lùi theo đúng đường vừa đi (đã biết trống), đánh dấu vật cản, lập lại;
      scan_match hỏng 2 lần liền → quay về điểm dừng gần nhất có đăng ký tốt, quét lại để định vị lại; VO mất (sàn
      trơn) → hop ≤ 0.8 m; hết frontier / hết ngân sách → về gốc, xuất map + danh sách vùng còn unknown.
- [ ] 14. `scene.json` cho CM/LLM; docs (interfaces: slip_profiles.json, explore.json, stops.json; README);
      `agent/plan/vision_plan.md`; `/code-standards-review` + `/code-logic-review`.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[--:--] re-plan (user) | info: trọng tâm = trượt bánh thích nghi theo sàn; vùng 0.8 m trước xe lúc khởi động trống (user bảo đảm); "odometry" thấy trượt chỉ có thể là VO (O firmware = bước đã lệnh); F âm = lùi | risk: mỗi leg một lần gọi robot_link → +2 s chờ READY; về sau nên có chế độ robot_link giữ cổng, nhận leg qua stdin (motivation)

[--:--] re-plan (user lần 3) | info: 0.8 m từ tâm xe; sàn tự nhận → thêm 4a floor_signature (chữ ký theo hành lang đường đi, trượt là trọng tài); chờ READY 2 s đã xác nhận trong sequencer.cpp WAIT_READY → gộp leg không cần sửa vào 1 plan | risk: ngưỡng nhận sàn chỉ kiểm được khi có sàn thứ hai thật; ~10–15 % thời gian khám phá là chờ READY cho tới khi robot_link có chế độ giữ cổng
[--:--] câu hỏi user (sàn trắng) | info: khoảng cách từ hình học sàn, không từ DA; VO chịu được vân còn 1/4 sàn hiện tại, chết khi ~0 (4/72 cặp) → thêm chuỗi dự phòng (khép vòng / khoảng cách tới mốc trước mặt), 3b kiểm vân sàn, thử sàn trắng thật ở Pha 0 | risk: sàn trắng + vật trắng tương phản thấp → DA có thể bỏ sót vật (không có Astra depth để đối chiếu); phản chiếu sàn bóng → nhãn "hụt" → lỗ trong vùng free (an toàn nhưng map thiếu)
[--:--] câu hỏi user (tự động hoàn toàn, bám tường?) | info: frontier không cần biết trước hình phòng; bám tường thuần không hợp camera này (chỉ nhìn trước 58°, vùng mù 0.78 m che tường sát bên, chỉ phủ chu vi, không có lệnh đi ngang có ack) → giữ frontier/NBV làm lõi + ưu tiên bám tường làm thứ tự chọn, danh sách đen, rào ảo, tự xử lý sự cố (bước 10, 13)
[22:18] approved | user: "approve cách a" → Pha 0 → 1 theo thứ tự
[22:22] step 1: `demo_drive._flush_link` = `robot_link --stop` trước mỗi `--run-plan` (log sự kiện `link_flush`, lỗi chỉ cảnh báo), `drive_config.ROBOT_LINK_STOP_TIMEOUT_S` 5 s; test_drive ALL OK | info: chạy thật trên /dev/ttyUSB0: ok, 0.33 s
[22:25] step 2 (chuẩn bị): dry-run "đi 40 cm" / "lùi 40 cm" → `F ±0.400 0.150`, 3.86 s/leg; `drive_map` alignment.txt: leg FORWARD in 3 chữ số (mm) + dòng "forward: measured / commanded" (Σ|đo|/Σ|lệnh|, đúng k của slip model) + trôi ngang / lệch hướng mỗi leg; test dữ liệu giả (0.30 + 0.28)/0.8 = 0.725 đúng; run 173930 cho kết quả quay y hệt; drive_map 7/7 | info: bước 2 không cần chờ bước 3 — `align_legs` đã đo tỉ số từng leg | chờ user: đặt xe + đo thước
[22:31] standards-review (2 pass): 1 sửa, 0 hoãn | drive_map: chuỗi trôi ngang/lệch hướng dùng FORWARD_DIGITS/DEG_DIGITS thay số viết thẳng, bỏ cắt mảng lặp | info: giữ tên sự kiện "link_flush" viết thẳng như các sự kiện khác của demo_drive; drive_map 7/7, test_drive ALL OK, pyflakes sạch
[22:36] logic-review (2 pass; đối chiếu bước 1–2, interfaces `--stop`/`E` rebaseline, history 09-26): 2 sửa, 0 báo user | sửa: (1) k leg tiến dùng Σ|đo| → leg đi SAI CHIỀU được tính là tiến; nay Σ(đo·dấu lệnh)/Σ|lệnh|; (2) cặp VO hỏng tính là đứng yên → trên sàn trắng leg hụt đo trông như trượt; nay in "% pairs" mỗi leg + chặn chia 0 / span rỗng | info: E do fragment + S của flush (≤ 1 Hz) rơi trong 2 s WAIT_READY → bị rebaseline, không làm hỏng plan; Ctrl-C lúc flush → _execute vẫn tắt recorder; cột `measured` của leg QUAY cũng đếm cặp hỏng là 0 (có từ trước, ngoài phạm vi; run 173930 đo đủ 850/850)
[22:50] step 2 (kết quả): user đo 4 run 223542 / 223653 / 223747 / 223827 (run 223130 hỏng, bỏ): +40 → 26.0, −40 → 28.0, +40 → 27.0, −40 → 27.5 cm (user điền nhầm cột "lệch ngang"; vật 176 → 178 cm khớp tổng −2.5 cm). VO: 25.2 / 27.6 / 26.2 / 26.7 cm, 100 % cặp, ~300 inlier → **VO/thước 0.974 (−2.6 %, đạt ≤ 3 %)**, cùng mẫu tiến ngắn hơn lùi. Lệch hướng mỗi leg ≤ 0.8° | info: tỉ số thước/lệnh 0.678 (tiến 0.663, lùi 0.694)
[22:58] kiểm chuỗi chuyển đổi (user yêu cầu; user: bánh ⌀8 cm, tâm → mặt ngoài bánh 21.5 cm): IK `omni_kinematics` ω_i = (−sin α·u + cos α·v + L·r)/a ĐÚNG; F 0.4 → W2/W3 ∓12831 bước (0.866·0.4/0.055 rad × 2037.2) = khớp dry-run, đỉnh 4812 Hz; firmware lấy mẫu profile giữa tick, Hz = ω·STEPS_PER_RAD, LEDC mỗi 20 ms, không mất xung có hệ thống (ngưỡng 20 Hz ≈ 1 xung/leg) → **thuật toán đúng, HẰNG SỐ SAI**: WHEEL_RADIUS_M 0.055 (chép từ PinConfig.h firmware cũ) vs bánh thật 0.040 → mọi quãng × 0.727 (dự kiến 29.1 cm/40). Phần còn lại là mất thật (trượt + bán kính lăn hiệu dụng của con lăn omni): tiến 91.1 %, lùi 95.4 %, TB 93.2 % | info: "r_eff 4.03 cm" từ 06-29 chính là bán kính bánh, không phải trượt; tỉ số quay không trượt dự kiến (0.21/L)·0.727 = 0.764 nếu L 0.20 (VO đo 0.79 với fx 570 chưa hiệu chuẩn) | risk: `ledc_set_freq` không kiểm giá trị trả về — với APB + 10 bit, tần số < ~76 Hz không đặt được, LEDC giữ tần số cũ ≤ 1 tick ở đầu/cuối leg (vài xung; leg đầu sau khởi động có thể 1 tick ở 1000 Hz ≈ 20 xung) — không phải nguyên nhân, ghi cho module motivation
[23:05] step 2b (user: bánh rộng 1 cm → L = 21.5 − 0.5 = 0.21 giữ nguyên; "sửa lại hằng số bán kính bánh"): `constants.h` WHEEL_RADIUS_M 0.055 → 0.040 (+ chú thích C, trần rim 0.39 m/s, gia tốc rim 0.08); build lại libmc/libmv/link/seq + robot_link, test_rm build tay: 5/5 bộ C++ pass; firmware build + host test_motion pass → **nạp ESP32** (3 vùng hash OK), `--monitor` sạch, `--stop` 0; `mc_client.chassis()` đọc constants.h → `run.json plan.wheel_radius_m`; `drive_timeline`: Run.wheel_radius_m (thiếu = LEGACY 0.055), `distance_scale(run)` = r thật / r của run (thay 4.03/5.5 cố định), profile MC lệch thời lượng ghi > 1 tick → hình thang theo thời lượng ghi; drive_map/stream_bench dùng scale của run, trần quay 1.87; drive_config trần demo 0.4 m/s / 1.8 rad/s; docs (overview, architecture, interfaces, 3 README) | info: trần mới 0.45 m/s tiến / 0.39 ngang / 1.87 rad/s, gia tốc 0.092 / 0.080 / 0.381; F 0.4 → 17642 xung, 6616 Hz, 4.30 s; 186.7 bước/độ; hồi quy run 173930: tỉ số quay 0.804/0.812/0.751/0.794 (trước 0.803/0.813/0.752/0.794), khởi đầu leg lệch ≤ 30 ms; test: drive_map 8/8 (+check_run_scale), vision 10/10, mapping 9/9, test_drive ALL OK (+test_plan_matches_the_chassis_header) | risk: VO quay 0.79 > 0.727 hình học (L 0.21 → không trượt tối đa 0.727) → VO yaw đọc DƯ ~9 % (fx 570 thấp?) hoặc L thật nhỏ hơn — vòng quét 4×T135 khép vòng sẽ gỡ; xe giờ chạy nhanh hơn 37 % ở cùng lệnh (0.15 m/s thật)
[23:10] standards-review 2b (2 pass): 4 sửa, 0 hoãn | drive_timeline: `_mc_client()` thay 2 bản chép chèn sys.path + import; stream_bench: bỏ biến trung gian `speed`, chú thích ra khỏi danh sách tham số, chú thích COMMANDED_TURNS theo hằng số hiện tại; test_drive_map dùng LEGACY_WHEEL_RADIUS_M | info: khoá dict "wheel_radius_m" viết thẳng như các khoá plan khác ("cruise_speed"); raise McError không `from` giống load_library; drive_map 8/8, test_drive ALL OK, pyflakes sạch
[23:12] logic-review 2b (2 pass; đối chiếu bước 2b, interfaces MC/run.json, overview): 2 sửa, 0 báo user | sửa: (1) `_mc_profile` ghép t ĐẦU tick (MC gán `out->t = elapsed_` trước khi cộng dt) với vị trí CUỐI tick → profile sớm 1 tick + 2 điểm t = 0; nay t + tick_s, profile chạy 0..duration_s. Chính lỗi đó làm phép so thời lượng mới (ngưỡng 1 tick) nhảy theo nhiễu float: ROTATE 1.5708 hiệu 0.0200000005 > 0.02 → run MỚI rơi vào hình thang; nay ngưỡng nửa tick `DURATION_TOL_S`, check_run_scale thêm leg quay + khẳng định dùng profile MC; (2) test_plan_matches_the_chassis_header ±1 xung mong manh (F 0.4: 17642 vs 17643.2) → ±0.1 % | info: stream_bench chạy thật: trần quay 1.87 → 2.05 rad/s thực = 2.57 × 0.79 cũ (cùng tốc độ bánh vật lý); run cũ không đổi (vẫn nhánh hình thang); regex chassis: tên hằng duy nhất trong header
[23:13] review gate (hook bật sau 2 review; phần sửa của logic-review chưa qua standards): standards + logic trên `_mc_profile` / `DURATION_TOL_S` / 2 test | 4 sửa, 0 hoãn: test_drive_map tốc độ run giả → `RUN_CRUISE_M_S` / `RUN_YAW_RAD_S` (trước chép 2 nơi), điều kiện "dùng MC" tính 1 lần (`uses_mc`); test_drive `PULSE_REL_TOL`; logic: ngưỡng 1e-6 cho t cuối của MC quá chặt (đồng hồ MC là float32 cộng dồn từng tick) → dùng `DURATION_TOL_S` | info: drive_map 8/8, test_drive ALL OK, pyflakes sạch
[22:35 09-28] step 2b (thước sau khi nạp r 0.040; user đo 5 run 221618 / 221725 / 221843 / 222009 / 222125): +40 → 38, −40 → 24, −40 → 34, +40 → 37, −40 → 23 cm. **Tiến đạt: k 0.94** (dự kiến 0.93). VO 35.2 / 22.9 / (11.6, cắt) / 35.1 / 21.9 cm → VO/thước 0.93–0.95 (−5.5 %, trước −2.6 %). **Lùi 2 và 5 KHÔNG phải trượt**: VO cùng vận tốc đỉnh ~13.5 cm/s nhưng cả profile bị nén thời gian ~1.6× (chạy 2.6 s thay 4.3 s, dốc tăng/giảm gấp ~1.7); K tới +2.75 / +2.82 s sau khi gửi (tiến +4.44 / +4.52); odometry `O` vẫn cộng đủ −0.400 → vòng motion 20 ms của ESP32 chạy ~12 ms/tick trong 2 leg đó, chỉ ~60 % xung ra thật (40 / 1.6 = 25 cm). Firmware logic đối xứng (host: ±0.4 → 215 tick, 17642 xung, đỉnh 6616 Hz cả hai chiều) → lỗi chỉ có trên phần cứng. Run 3 (lùi sau lùi): ramp bình thường, rồi **cả hub USB 1-2 (CH340 + Astra + WiFi) rớt 22:18:59** lúc +1.9 s vào leg (kernel log; rớt lần 2 22:19:55) → ESP32 tự chạy hết leg (34 cm), CH340 về lại thành ttyUSB1 | risk: cơ chế đồng hồ nhanh chưa rõ (không PM/tickless, FREERTOS_HZ 100, log level 0, không FW_DEBUG); rớt hub USB giữa leg = mất điều khiển + camera → nghi nguồn / nhiễu từ driver bước (xung nhanh hơn 37 % từ 2b) | info: lùi sau tiến 2/2 bị nén; lùi sau lùi 1/1 bình thường; tiến 2/2 bình thường — mẫu nhỏ; chờ user duyệt chẩn đoán
[22:48 09-28] approved Pha 1' | user: Jetson / motor nguồn riêng, sửa nguồn sau; ưu tiên di chuyển tạo map, tin VO; bước E = dùng hệ số thước đã đo (4 leg 09-28, không chạy calib_floor)
[22:59 09-28] step A + E: drive_map vị trí = VO × vo_scale theo heading VO (cặp hỏng: lệnh × tỉ số đo của chính leg trên các cặp đo được, leg đo < 25 % thì tỉ số cả run, theo chiều từng leg); `vision/vo_scale.json` (4 leg thước/VO 09-28 + mount đo) → `frame_motion.vo_scale(mount)` = Σthước/ΣVO = **1.063**, mount khác (refit) → 1.0; `--vo-scale`; `drive_timeline`: Leg.ack_t (status leg sau = lúc K về), `ready_wait_s()` đọc READY_WAIT_MS qua `mc_client.constant()` (tách từ chassis), `firmware_duration`, `cut_short` (< 90 % plan); alignment.txt cột pairs_% (theo THỜI GIAN leg, frame không có tính là hụt), fw_s, plan_s, CUT; tỉ số trượt chỉ lấy leg `trusted` (không CUT, pairs ≥ 75 %) | info: 5 run 09-28 → cuối +0.374 / −0.244 / −0.123 / +0.373 / −0.233 m (thước 0.38 / 0.24 / 0.34 / 0.37 / 0.23: lệch ≤ 1 cm, trừ run 3 mất camera → "weak 43 %"); run 2, 5 CUT (fw 2.75 / 2.82 s vs 4.30); k tiến 0.934 / 0.929; test_drive_map 10/10 (+check_vo_positions, +check_vo_scale), test_drive ALL OK, test_vision / test_mapping PASS, pyflakes sạch | risk: VO trong lúc QUAY giờ cũng đưa tịnh tiến vào pose (trước = 0) — chưa đo trên run quay thật (run 173930 đã xoá)
[23:14 09-28] step B: `vision/leg_odometry.py` — chuyển từ drive_map: measure_motion, odometry_track (+ tham số scale), COL_*, RAW_*, ODOMETRY_STRIDE, load_jsonl (bỏ qua dòng cuối đang ghi dở), VO_MIN_PAIRS 0.75; mới: `read_frames`, `measured_share` (drive_map dùng chung cho pairs_%), `measure_leg(run_dir, t0, t1, intr, mount, scale, wait_s)` → LegMotion(dx, dy, dθ, pairs, weak, frames, inliers, t_first, t_last); drive_map / stream_bench / test trỏ sang leg_odometry | info: run thật (t0 = status + 1.5 s, t1 = K + 0.5 s, scale 1.063): +0.370 / −0.243 / −0.232 m (thước 0.38 / 0.24 / 0.23), 99–100 % cặp, 1.4–2.1 s tính mỗi leg; test_drive_map 11/11 (+check_measure_leg: sàn tổng hợp 0.20 m / 0.03 m / 5° ± 5 mm / 0.2°, dòng index dở, span quá frame cuối → weak), pyflakes sạch
[23:17 09-28] step C + D: `communication/leg_executor.py` LegExecutor.run(kind, target): lệnh = phần còn lại / k → `--stop` + `robot_link --run-plan` 1 leg (plan `leg_NN.txt`, giữ kiểm lỗi của sequencer, tốn READY 2 s/leg) → K + SETTLE 0.5 s → `measure_leg` (chờ frame ≤ 3 s) → lệch > 3 cm / 2° thì bù (≤ 2); dừng bù khi VO yếu / kẹt (< 0.5 lệnh, chỉ xét leg ≥ 10 cm / 10°) / mất camera (frame mới nhất cũ hơn cuối span 1 s → dừng cả run) / robot_link lỗi / Ctrl-C; cut_short = K về trước 90 % thời lượng MC → vẫn bù, không học; pose = hợp các chuyển động đo (SE2). `communication/slip_model.py` (1 sàn san_01): k theo forward/backward/left/right = Σđo/Σlệnh 10 mẫu học gần nhất, < 3 mẫu → mặc định 0.93 / 0.93 / 1 / 1; không học: cut short, VO yếu, < 10 cm / 10°, kẹt; lưu mọi mẫu + lý do vào `communication/slip_profiles.json`, in bảng. `demo_drive --compensate` (từ chối với --dry-run / --no-vision), in pose đo + bảng trượt; `robot_link_cli.py` (flush_link + mẫu dòng robot_link, dùng chung); `motion_plan.plan_leg` (tách từ build_plan); `drive_config.add_vision_to_path`; run.json: executed_legs (waits_ready), requests, sự kiện leg_acked / leg_measured, status "leg N/N" mỗi leg → `drive_timeline.load_run` đọc executed_legs + leg_acked, Leg.waits_ready | info: test_leg_executor ALL OK (robot giả: tiến 40 trúng 1 leg; lùi bị cắt 60 % → 3 leg trong 3 cm, 3 mẫu "cut short" không học; quay 90° → 2 leg; k học 0.80 → lệnh 0.625, vượt → leg bù lùi; kẹt dừng sau 1 leg; VO yếu / mất camera; pose quay + tiến = +y 0.40); test_drive ALL OK, test_drive_map 11/11, test_vision / test_mapping PASS, pyflakes sạch; dựng executor trên run thật: scale 1.0627, READY 2.0 s | risk: chưa chạy trên robot; mỗi leg tốn ~2.3 s chờ (flush + READY); góc quay đo bằng VO dựa fx 570 chưa hiệu chuẩn → bù quay theo VO có thể lệch vài %
[23:22 09-28] standards-review A–E (2 pass): 14 sửa, 0 hoãn | leg_executor: kết quả call "complete/failed/aborted" → CALL_*, sự kiện K dùng `drive_timeline.ACKED_EVENT` (hợp đồng 2 module), K_DIGITS / SHARE_DIGITS, logger `__name__`, ngắt dòng dài; slip_model: `ROTATE` từ motion_plan, chuỗi điều kiện 4 tầng → `_why_not`, TIME_DIGITS / JSON_INDENT, `statistics.pstdev`, docstring save; drive_map: điều kiện lồng của forward_fill → `next(...)`; test_leg_executor: số giả đặt tên, bỏ FRAME_GAP_S không dùng | info: tên sự kiện khác giữ viết thẳng như các sự kiện sẵn có của demo_drive (quyết định 09-27); test_leg_executor ALL OK, test_drive ALL OK, test_drive_map 11/11, pyflakes sạch
[23:27 09-28] logic-review A–E (2 pass; đối chiếu Pha 1' A–E, overview, interfaces, comm_plan): 5 sửa, 1 báo user | sửa: (1) LỆCH PLAN bước B: VO yếu → executor cộng quãng VO thiếu vào pose; nay pose = lệnh × k (× fw_s / plan_s nếu leg bị cắt), cờ `predicted` trong executed_legs / requests; (2) `measured_share` bỏ sót cặp vắt qua t1 → giao chính xác từng cặp với [t0, t1]; (3) HIỆU NĂNG: chờ frame đọc lại TOÀN BỘ index mỗi 50 ms (phiên khám phá ~36k dòng) → `_last_capture` đọc 4 KB đuôi, đọc đủ 1 lần; (4) demo_drive chạy tiếp các bước sau leg KẸT → dừng lệnh; (5) PlanError của leg bù văng traceback → bắt, RESULT_FAILED | info: số trên run thật không đổi (+0.374 / −0.243 CUT / weak 43 % / −0.233 CUT; measure_leg +0.370 / −0.243); test_leg_executor ALL OK (+ dự đoán khi VO yếu, có / không cắt), test_drive_map 11/11 (+ đuôi index) | báo user: interfaces.md chưa ghi run.json executed_legs / requests / leg_acked, slip_profiles.json, vo_scale.json — để bước 14 (docs) theo plan
[23:30 09-28] review gate — standards (2 pass) trên phần sửa của logic-review: 3 sửa, 0 hoãn | leg_odometry: chú thích nhánh parse lỗi của `_last_capture` viết lại cho đúng; test_leg_executor PREDICT_REL_TOL; test_drive_map INDEX_TIME_TOL_S | info: test_leg_executor ALL OK, test_drive_map 11/11, pyflakes sạch
[23:31 09-28] review gate — logic (2 pass) trên cùng phần: 1 sửa, 0 báo user | PlanError của leg bù thoát khỏi `LegExecutor.run` trước khi cộng pose → mất quãng leg chính đã chạy + không ghi requests; nay bắt trong run() → FAILED (dừng lệnh), pose giữ các leg đã chạy; bỏ phần bắt thừa ở demo_drive | info: measured_share (giao từng cặp, O(n) vector), `_last_capture` / `_frames_through`, `_predicted`, dừng khi KẸT: đúng; test_leg_executor ALL OK (+test_unplannable_top_up_keeps_the_pose), test_drive ALL OK, pyflakes sạch
[18:40 09-29] step F (robot thật, 1 lần mỗi chiều): run 182642 "đi 40 cm" → F 0.430 (k 0.93 mặc định), VO 38.05 cm, lệch 1.95 < TOL 3 cm → không bù, **thước 38** (VO khớp thước); run 182735 "lùi 40 cm" → F −0.430, VO −45.8 cm (vượt 5.8) → bù F +0.062 → +5.4 → cuối −40.5 cm, **thước 40**; user thấy xe lùi quá rồi tự tiến lại | info: VO/thước giờ khớp ≤ 0.5 cm (vo_scale 1.063 đúng); tiến 0.885 × lệnh (09-28: 0.94); leg lùi KHÔNG bị cắt lần này mà **K về MUỘN**: firmware 5.43 s / plan 4.50 s = 1.21× → ra nhiều xung hơn (ngược với 09-28 nhanh 1.6×) → lỗi đồng hồ vòng motion vẫn còn, cả hai chiều | risk: slip_model đã HỌC mẫu lùi 1.065 (chỉ chặn leg cắt ngắn < 90 %, không chặn leg kéo dài) → khi đủ 3 mẫu, k lùi > 1 sẽ làm lệnh lùi ngắn đi; đề xuất chặn `fw_long` (> 110 %) — chờ user duyệt

## Test result
