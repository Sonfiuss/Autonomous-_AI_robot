---
id: 2026-09-25_llm-drive-demo
status: executing
module: communication (+ vision, motivation)
started: 2026-09-25
---

## Task
Demo: câu lệnh chữ ("đi thẳng 30 cm, sau đó rẽ trái, đi tiếp 60 cm") → LLM phân tích → từng bước
chuyển thành primitive → lệnh F/T → thông số từng bánh → robot chạy đúng lệnh (không dùng bản đồ).
Song song: vision detect + ghi video có chú thích vào `vision/output/`.

## Input
- Lệnh chữ tiếng Việt/Anh, chỉ gồm chuyển động tương đối (tiến/lùi X cm|m, rẽ/quay trái/phải [X độ]).
- Pin config ESP32: user cung cấp sau → thay placeholder trong `fw/config.h`.
- Hạ tầng có sẵn: `CM/llm_client.py` (gemini|openai|anthropic|fake), SEQ + `robot_link --run-plan`
  (one leg in flight, chờ `K`), MC ctypes (per-tick w/hz/dir qua RM), `vision/perception.py` (YOLO + depth).

## Expected output
1. `python3 communication/demo_drive.py "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"` in từng
   tầng: JSON LLM → bước đã validate → primitive → dòng wire (`F 0.300 0.150`, `T 90.000 0.800`, ...)
   → mỗi bánh: số bước, Hz đỉnh, chiều, thời gian.
2. Robot đi 30 cm, quay trái 90°, đi 60 cm (đo bằng thước).
3. `vision/output/<run_id>/` có `detections.mp4` (khung YOLO + khoảng cách + overlay bước đang chạy),
   `detections.jsonl` (từng frame), `run.json` (lệnh, plan, mốc thời gian từng leg, kết quả).
4. `--dry-run` chạy toàn bộ (kể cả ghi video) mà không gửi gì xuống ESP32.

## Plan
- [x] 1. `communication/cmd_parser.py`: prompt + JSON schema cho LLM (tái dùng `CM/llm_client.py`),
      output `{steps:[{action: forward|backward|turn_left|turn_right, value|null, unit}], question}`.
      Validator trong code: action hợp lệ, giới hạn (≤3 m/bước, ≤360°/bước, ≤10 bước), **mọi số LLM
      trả về phải xuất hiện trong câu lệnh** (chặn bịa số); "rẽ trái" không có góc → code điền 90°.
      Kèm parser luật (regex VI/EN) làm provider `rule` offline khi không có mạng/API key.
- [x] 2. `communication/motion_plan.py`: steps → primitive (`FORWARD m`, `ROTATE rad`, trái = +CCW)
      → dòng wire như SEQ sẽ gửi → thông số bánh qua MC (`mc_client.py`, cùng RM với firmware):
      bước/bánh, Hz đỉnh, chiều, thời gian. Ghi `plan.txt` đúng format `--run-plan`.
- [x] 3. `vision/record.py`: vòng headless dùng lại grabber/detector/`measure_object`/`render_view`
      của perception; ghi `detections.mp4` + `detections.jsonl`; overlay bước đang chạy (đọc file
      status do orchestrator ghi); fallback RGB-only nếu không có depth; SIGINT/SIGTERM → đóng video sạch.
- [x] 4. `communication/demo_drive.py`: nhận lệnh (argv hoặc hỏi) → parse → in plan, hỏi xác nhận →
      bật recorder, chờ camera có frame → chạy `robot_link --run-plan` (hoặc `--dry-run` giả lập theo
      thời lượng MC) → theo dõi tiến độ, cập nhật status → tắt recorder sau 2 s → ghi `run.json`.
      Ctrl-C: robot_link tự abort + gửi `S`, video vẫn được đóng.
- [x] 5. Test offline: unit test parser (câu ví dụ → `[FORWARD 0.30, ROTATE +90°, FORWARD 0.60]`,
      số bịa bị chặn, vượt giới hạn bị chặn) + motion_plan; dry-run end-to-end ra video thật.
- [x] 6. Môi trường Jetson: cài `ultralytics` **không đụng torch NVIDIA** (`--no-deps` + deps lẻ),
      binding `openni` (lib `/usr/lib/libOpenNI2.so` đã có), SDK LLM + `python-dotenv`, weights YOLO;
      build `robot_link` + `libmc.so` trên Jetson.
- [ ] 7. **[đã build + flash; còn smoke test chạy bánh]** điền `fw/config.h`, cài ESP-IDF (`tools/esp32_setup_idf.sh`),
      build + flash; smoke test `robot_link --monitor`, `--forward 0.1`, `--turn 90` (kiểm dấu trái=+).
- [x] 8. `communication/README.md`, `agent/plan/comm_plan.md`, cập nhật `system_architecture.md`;
      `/code-standards-review` + `/code-logic-review`.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[--:--] intake | info: Jetson py3.8, torch CUDA OK, cv2 4.2; thiếu ultralytics/openni/openai/anthropic/dotenv; không có CM/.env; ESP-IDF chưa cài; robot_link + firmware chưa từng build
[--:--] intake | risk: memory cũ (06-29) ghi bán kính lăn hiệu dụng 4.03 cm vs 5.5 cm trong constants → robot có thể chỉ đi ~73% quãng; đo thước ở bước 7 trước khi tin số

[11:25] approved | info: pin: W1 PUL26/DIR27, W2 PUL12/DIR13, W3 PUL4/DIR5; servo qua PCA9685 I2C SDA21/SCL17 (firmware đang giả định LEDC 18/19); mic INMP441 25/32/33
[11:35] step 1: communication/{drive_config,cmd_parser}.py + prompts/drive_command.json | info: rule parser pass câu ví dụ; LLM chỉ chép số, validator chặn số không có trong câu; turn không góc → 90° do code điền
[11:45] step 2: communication/motion_plan.py; build libmc.so + robot_link trên Jetson (sạch) + pip python-dotenv | info: 30cm → W1/W3 ∓9623 bước @4.8kHz, W2=0; 90° → 12218 bước/bánh @6.2kHz; turn chia chunk <180° vì SEQ wrap (-180,180]
[11:47] step 3: vision/record.py (VideoSink pace theo wall-time, overlay status, RGB-only fallback); result_records → object_distance.py | risk: torchvision ~/.local là wheel sai ABI (undefined symbol) → YOLO NMS crash; đang build v0.16.1 từ source (CUDA sm_87) | info: ultralytics 8.4.162 --no-deps, torch NV giữ nguyên
[11:50] step 4: communication/demo_drive.py (stdbuf -oL robot_link --run-plan, parse "primitive N/M" → status; recorder session riêng, SIGINT để đóng video) | info: dry-run --no-vision pass, ra plan.txt/run.json
[11:52] step 5: test_drive.py 12 test pass; ESP32 giả trên pty: robot_link+SEQ gửi F/T/F/S đúng 1 leg/K, Ctrl-C giữa leg → S ngay | info: dòng "plan failed at primitive N/M" khớp regex tiến độ → phải so end-mark trước
[12:16] step 6: torchvision 0.16.1 build từ source (CUDA sm_87, wheel ở ~/wheels), NMS CUDA OK; pip --user: ultralytics 8.4.162 --no-deps, tqdm, py-cpuinfo, thop, openai 1.89, anthropic 0.72, openni; dry-run đầy đủ ra mp4 140 frame | risk: anthropic 0.72 (cuối cho py3.8) thiếu output_config/fallbacks → CM AnthropicClient không chạy trên Jetson, dùng gemini/openai | risk: depth: OpenNI2 hệ thống (PS1080) không thấy Astra, cần SDK Orbbec ARM64 → hiện RGB-only
[12:05] step 7 (phần code): fw/config.h pin thật + DIR_INVERTED[]; step_dir_driver XOR cờ; sdkconfig.defaults LOG_NONE (UART0 là link); test_motion pass | risk: ESP-IDF cần sudo apt gperf python3-venv ninja-build ccache dfu-util → chờ user; chưa flash

[12:20] standards-review: 13 vi phạm sửa (magic string action/tên file/exit code/digits, dead Leg.primitive, log leak khi Popen lỗi, video path lấy từ ready file thay listdir), 0 deferred | info: pass 2 sạch, pyflakes sạch, test pass
[12:20] user: video phải có vùng sàn trống (Depth Anything) + khoảng cách vật → mở rộng step 3
[12:41] step 3b: record.py + FloorWorker (DA + analyze_floor 1 thread, newest wins), blend_floor overlay, clear-ahead hành lang ±0.25m; box_floor_range (đáy bbox + cao/pitch camera) khi không có depth, nhãn "~"; range_source trong jsonl; test_vision check_box_floor_range; dry-run 80/84 frame có sàn | info: DA V2 Small fp16 518px ~290ms trên Orin; pip transformers 4.40.1 + tokenizers 0.19.1/safetensors 0.4.3/hf-hub 0.23.4 (--only-binary, bản mới cần Rust) | risk: khoảng cách "~" sai với vật không đứng trên sàn; pitch 0 chưa đo trên robot
[12:55] logic-review: 4 mismatch sửa, 1 flag | info: (1) validator thêm kiểm thứ tự số (LLM đảo 2 số thật lọt check cũ), chỉ 1 message; (2) age DA âm vì đo từ đầu cycle; (3) Popen robot_link lỗi → run.json ghi "cancelled" + crash; (4) FloorWorker log traceback mỗi frame. blend_floor 8.8ms/frame: không tối ưu. Flag: step 7 chưa xong (ESP-IDF cần sudo apt), Expected output #2 chưa kiểm được

[12:48] standards-review (lần 2, file sửa sau review 1): 7 vi phạm sửa, 0 deferred | info: self.p → self.pipeline; chuỗi/offset bar + MS_PER_S + ESTIMATE_MARK thành hằng; biến forward thừa trong box_floor_range; pass 2 sạch
[12:50] logic-review (lần 2): 1 mismatch sửa, 1 flag giữ nguyên | info: fallback LLM→luật chỉ xoá history, giữ user_texts → số từ câu cũ vẫn "hợp lệ" và tắt check thứ tự; nay reset cả exchange + test_fallback_resets_the_exchange. Đã xác nhận chỉ box_floor_range tạo valid_fraction None (nhãn "~" không nhầm số đo). Flag: step 7 (flash) chờ sudo apt

[13:02] user: "xe vẫn chưa chạy" → chẩn đoán | info: lần chạy 12:53 là --dry-run (không gửi UART, đúng thiết kế); ESP32 đang chạy firmware Arduino CŨ (O 2 số lẻ, READY\r\n) mà F/T tính sai (mọi bánh cùng số bước, W2 cũng chạy, bỏ dấu lùi); ESP-IDF vẫn chưa cài (5 gói apt còn thiếu) | risk: fw/config.h đã gán SAI bánh↔pin (theo thứ tự bảng user) → sửa theo PinConfig.h cũ đã chạy (git a96ff99^): W1 12/13, W2 4/5, W3 26/27; DIR_INVERTED={true×3} (firmware cũ đảo DIR, memory 06-29); test_motion pass
[13:03] standards-review (fw/config.h): 0 vi phạm | info: chỉ đổi giá trị hằng + comment nguồn gốc
[13:04] logic-review (fw/config.h): 0 mismatch | info: thứ tự bánh = PinConfig cũ (M1 12/13=W1…); DIR: đường W đã chạy đảo dấu IK + FastAccelStepper move(+)=HIGH → IK+ = LOW = forward XOR true ✓; IK core-control (-sin,cos) trùng RM nên quay cùng đúng nếu tịnh tiến đúng; GPIO12 đã boot được với firmware cũ
[13:14] step 7 check trạng thái flash: CHƯA flash firmware mới | info: ESP-IDF v5.1.7 ở ~/esp/esp-idf + đủ gói apt, idf.py/esptool 4.12 chạy được; chip ESP32-D0WD-V3 rev3.1, flash 4MB, /dev/ttyUSB0 (CH340), auto-reset OK, cổng rảnh; chưa có sdkconfig/build/*.bin; boot không có log bootloader IDF (sdkconfig.defaults giữ log đó) + READY\r\n → vẫn là firmware Arduino cũ
[13:23] step 7 build + flash: OK | risk: bánh chưa chạy lần nào với firmware mới → pin map/DIR_INVERTED chưa kiểm | info: 2 lỗi build đã sửa: (1) build_test_motion.sh ghi vào build/ → idf.py set-target từ chối fullclean → chuyển sang build_host/ (+README, system_architecture, .gitignore); (2) component project/ không đặt chuẩn nên dùng gnu++2b của IDF → <atomic> kéo unistd.h `int link()` đụng `namespace link` → set CXX_STANDARD 14 trong project/CMakeLists.txt nhánh IDF (main/ đã C++14 sẵn). 0 warning, app 184 KB (18% phân vùng). Boot: bootloader IDF 13:18, READY, O 10 Hz / P 50 Hz, không log IDF lọt vào UART; R → không K (đúng: K chỉ cho leg F/T), không E
[13:40] step 7 smoke test → đổi hệ trục | info: user test --forward: đi thẳng đúng trục (W2 đứng yên) nhưng sai hướng; đầu xe thật là W3 (26/27) → WHEEL_ANGLE_DEG {60,180,300} → {120,240,0} + WHEEL_POS_X/Y; đi thẳng giờ W1 −9623 / W2 +9623 / W3 0 (30 cm); trần tốc độ không đổi (xoay 60° = đối xứng 3 bánh, cùng độ lớn hệ số); sửa test_mc (bánh idle = w[2]), test_drive.py, comment fw/config.h, README communication/RM, project_overview, system_architecture; test_rm/mc/mv/link/seq/motion + test_drive pass; rebuild libmc.so + robot_link; firmware build 0 warning + flash, telemetry OK | risk: chiều DIR chưa kiểm với hệ mới (xe phải tiến VỀ PHÍA W3); simulation/room giữ 60/180/300 vì đa giác thân xe (đầu rộng giữa 2 bánh) theo bản vẽ 09-11 → hỏi user
[13:41] standards-review (constants.h, test_mc.cpp, test_drive.py, fw/config.h, project/CMakeLists.txt, build_test_motion.sh): 2 vi phạm sửa, 0 deferred | info: comment đầu constants.h còn trỏ PinConfig.h/OmniKinematics.h làm nguồn → sửa; comment test_mc ghi sin(120) cạnh code sin(PI/3) → ghi rõ bằng nhau; pass 2 sạch
[13:42] logic-review (cùng bộ file): 0 mismatch sửa, 2 flag cho user | info: {120,240,0} = xoay cứng +60° giữ thứ tự CCW; W3 ở (L,0), hệ số tiến −sin0=0 → đứng yên, xe tiến về W3 ✓; WHEEL_POS = L·(cos,sin) ✓ (không ai dùng); FK là nghịch đảo tổng quát, speed-limit/odometry sinh từ ma trận IK → không có chỗ nào khác giả định hệ cũ; không có offset yaw camera trong vision; CXX_STANDARD 14 → -std=gnu++14 đứng sau gnu++2b (thắng). Flag: (1) chiều DIR_INVERTED chưa suy ra được (lần test cũ không cho biết xe đi ra xa hay về phía W2) → kiểm bằng --forward 0.1; (2) simulation/room thân xe theo bản vẽ, chờ user
[13:53] step 7 smoke test lần 2 → sửa hệ trục + DIR | info: user: {120,240,0} chạy lệch TRÁI 60°, đầu xe thật là bánh "13-14" = 12/13 (W1; không có GPIO 14). Mô hình duy nhất khớp CẢ 2 lần test: W1 0° (trước), W2 120° (sau-trái), W3 240° (sau-phải) + đảo dấu mọi bánh (lần 1 60/180/300 → dự đoán chạy VỀ PHÍA W2 = "thẳng nhưng sai hướng", W1 là bánh kề W2 theo chiều kim đồng hồ = câu đầu của user; lần 2 → dự đoán lệch trái 60° ✓) → WHEEL_ANGLE_DEG {0,120,240}, WHEEL_POS, DIR_INVERTED {false×3}; đi thẳng 30 cm: W1 0 / W2 −9623 / W3 +9623; test_motion "released stop" ghi cứng bánh 0 quay khi đi thẳng → đổi thành "có bánh nào quay"; mọi test pass, rebuild libmc/robot_link, flash OK, telemetry OK | risk: cặp đảo-dấu + xoay 180° cho cùng kết quả tịnh tiến nhưng ngược chiều quay → --turn 90 phải quay TRÁI mới xác nhận được DIR
[13:55] standards-review (constants.h, fw/config.h, test_mc.cpp, test_drive.py, test_motion.cpp): 0 vi phạm, 0 deferred | info: chuyển sang logic-review: check rest của test_mc đọc w[0] (nay là bánh idle)
[13:56] logic-review: 1 mismatch sửa, 1 flag | info: mô phỏng số FK trên layout vật lý {0,120,240}: test 1 (60/180/300, inv) → hướng 120° = về phía W2, W2 idle ✓; test 2 (120/240/0, inv) → +60° trái ✓; cấu hình mới (0/120/240, không inv) → 0° về W1, W1 idle, quay CCW ✓. Sửa: test_mc "starts/ends at rest" đọc w[0] (bánh idle khi tiến) → luôn pass, vô nghĩa → đổi sang w[1]; test pass; chỉ đổi test, firmware không cần flash lại. Flag: phương án xoay 180°+đảo DIR cho cùng tịnh tiến nhưng quay NGƯỢC → phải xác nhận bằng --turn 90 (quay trái)
[13:58] standards-review (gate, test_mc.cpp check rest sửa lúc logic-review): 0 vi phạm | info: nhãn "Wheel 2" = w[1] theo cách đếm 1-based sẵn có của file
[13:58] logic-review (gate, cùng thay đổi): 0 mismatch | info: w[1] = W2 @120°, hệ số tiến −sin120 = −0.866 ≠ 0 → check rest giờ kiểm thật ramp lên/xuống; test_mc pass

## Test result
