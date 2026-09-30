# communication/ — lệnh chữ → LLM → bánh xe

Gõ một câu như *"đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"* hoặc *"đi tới cái ghế"*. Chương trình phân
tích câu lệnh, chuyển dần qua từng tầng thành thông số cho từng bánh, rồi cho robot chạy. Trong lúc đó
`vision/record.py` detect vật và quay video. Tầng L2 trong `agent/description/robot_process_llm.md` có hai nhánh:
- `primitive` — các bước đi / quay tương đối, bỏ qua bản đồ, L3 và L4;
- `goal` — một chỗ trong phòng: bản đồ thật (`realroom/`) chọn vị trí đứng (CM grounding) và lập đường đi.

```bash
python3 communication/demo_drive.py "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
python3 communication/demo_drive.py "đi tới cái ghế"             # cần bản đồ thật: realroom/README.md
python3 communication/demo_drive.py --compensate "lùi 40 cm"     # mỗi leg đo bằng camera, thiếu thì bù
python3 communication/demo_drive.py --dry-run "..."      # chạy mọi thứ trừ UART, video vẫn quay
python3 communication/demo_drive.py --compensate "lập bản đồ"   # quét tại chỗ, rồi dựng + publish bản đồ phòng
python3 communication/demo_drive.py                       # hỏi lệnh
python3 communication/test_drive.py                       # unit test, không cần key/camera/robot
python3 communication/test_leg_executor.py                # executor trên robot giả
python3 communication/test_live_parser.py [--first N]     # bộ câu natural_commands.json qua LLM thật
```

Tuỳ chọn: `--provider auto|rule|gemini|openai|anthropic`, `--compensate`, `--yes` (không chờ Enter),
`--port /dev/ttyUSB0`, `--speed 0.15` (m/s), `--yaw-rate 0.8` (rad/s), `--no-vision`, `--no-depth`, `--no-floor`,
`--color-index N`, `--map` (dựng bản đồ sau lần chạy), `--plan-json FILE` (chạy kế hoạch đã duyệt ở trang
realroom), `--run-dir DIR` (thư mục run, mặc định `vision/output/<ngày_giờ>`), `-v`.

Trang `realroom` (cổng 5002) có khung chat làm đúng việc này trên web: câu lệnh đi qua cùng `prepare()` (parser,
hỏi lại, tìm chỗ trên bản đồ thật), kế hoạch hiện trên bản đồ, bấm **Chạy** thì trang gọi
`demo_drive --yes --plan-json` — xem `realroom/README.md`.

## Lập bản đồ

*"lập bản đồ"*, *"quét phòng"*, *"quét xung quanh"*, *"scan the room"* (hoặc `--map`) được tách khỏi câu trước khi
parse (`cmd_parser.split_map_request`, cùng liên từ hai đầu như *rồi*, *sau đó*):
- chỉ có cụm đó (và từ đệm *hãy*, *nhé*, *robot*…) → robot quét tại chỗ: quay trái 90°, phải 180°, trái 90°
  (`MAP_SCAN`, kịch bản đã dựng bản đồ 20260929_184936);
- có kèm lệnh khác (*"quay trái 360 độ rồi lập bản đồ"*, *"đi tới cái ghế rồi quét phòng"*) → chạy lệnh đó.

Chạy xong, `vision/drive_map.py --run <run> --publish` dựng bản đồ từ video của chính lần chạy (log
`map_build.log`, ~17 s cho run 35 s trên Orin) và thay **toàn bộ** bản đồ phòng thật (chưa ghép nhiều run).
Không dựng khi `--dry-run`, hoặc khi lệnh không chạy hết (trừ `--compensate` đã đo được vị trí) — cùng luật cập nhật
vị trí. `run.json`: `build_map`, sự kiện `map_published` / `map_failed` / `map_skipped`. Cần camera: `--no-vision`
bị từ chối.

## Luồng

```
câu lệnh
 [1] parser     LLM (CM/llm_client.py) hoặc luật regex → {"steps":[{action,value,unit}],"question"}
 [2] validator  action hợp lệ, số nằm trong giới hạn, MỌI SỐ PHẢI CÓ TRONG CÂU LỆNH; "rẽ trái" không có góc → 90°
 [3] primitive  FORWARD m / ROTATE rad / STOP  → plan.txt (format của robot_link --run-plan)
     UART       F 0.300 0.150 / T 90.000 0.800 / S   (SEQ gửi từng dòng, chờ K rồi mới gửi dòng sau)
 [4] bánh       MC (cùng chuỗi RM với firmware): số xung DM556 có dấu (= mức DIR), Hz đỉnh, rad/s đỉnh
 chạy           robot_link --run-plan  ∥  vision/record.py → vision/output/<run_id>/
```

Với câu lệnh ví dụ, bảng [4] in ra:

| Leg | UART | W1 (0°, đầu xe, 12/13) | W2 (120°, 4/5) | W3 (240°, 26/27) |
|---|---|---|---|---|
| 1 | `F 0.300 0.150` | 0 | −9623 xung, 4812 Hz | +9623 xung, 4812 Hz |
| 2 | `T 90.000 0.800` | +12218, 6223 Hz | +12218 | +12218 |
| 3 | `F 0.600 0.150` | 0 | −19246 | +19246 |

Đầu xe chỉ thẳng vào W1. Khi đi thẳng, W1 đứng yên vì hướng lăn của nó vuông góc với trục +x, còn W2 và W3
quay ngược chiều nhau để đẩy xe tiến về phía W1.
Đây cũng là cách kiểm tra thứ tự bánh trên xe thật.

## Chọn parser

- `auto` (mặc định): dùng LLM mà CM đang cấu hình (`CM_PROVIDER`, mặc định gemini-2.5-flash; key đặt trong
  `project/src/CM/.env`, ví dụ `GEMINI_API_KEY=...`). Stage [1] in parser và model đang dùng. Nếu thiếu SDK,
  thiếu key, mất mạng hoặc **hết hạn mức** thì tự chuyển sang luật, **in cảnh báo**, và luật đọc lại cả cuộc
  hỏi–đáp đang dở rồi làm tiếp đến hết lệnh đó.
  - Gemini 2.5 Flash gói miễn phí: **5 lượt / phút và 20 lượt / ngày** (mỗi model, mỗi project; đo 2026-09-29).
    CM grounding (lệnh "đi tới …") dùng chung key và model, nên dùng chung hạn mức đó.
- `rule`: offline, cùng từ vựng với prompt:
  - đi / lùi: *đi thẳng / tiến / đi tiếp / lùi / back up X cm | m | mm | phân | mét*, số chữ một từ cho quãng
    (*nửa mét*, *một mét*);
  - quay: *rẽ / quẹo / quay / xoay / tự quay / rotate / spin* với phía, góc, số vòng theo **bất kỳ thứ tự** nào:
    *quay trái 90 độ*, *quay 90 độ sang trái*, *rotating 180 degree by the left*, *xoay tròn một vòng*,
    *nửa vòng ngược chiều kim đồng hồ*, *quay lại 180 độ*, *quay tại chỗ 360 độ*;
  - vế nối không động từ theo bước trước: *đi thẳng 30 cm rồi 40 cm nữa*, *quay trái 90 độ rồi 45 độ nữa*;
  - câu mục đích *để trở về vị trí ban đầu*, *to get back to the base angle* không phải bước (xem "Kiểm ý định");
    không có *để* thì *quay lại vị trí ban đầu* là một yêu cầu, robot hỏi lại chứ không quay đầu;
  - đi tới một chỗ: *đi tới / đến cạnh / lại gần / go to / go next to <vật>*.
  Có dấu hay không dấu, gõ sai chữ không phải động từ đều được. Phần nào luật không đọc được (còn một số, một
  động từ, một đơn vị) thì robot hỏi lại chứ **không bao giờ bỏ qua một bước**.
- Chọn thẳng `gemini`/`openai`/`anthropic` thì lỗi sẽ được báo ra, không tự chuyển sang luật.

### Quay không nói phía

180° / 360° / "một vòng" → quay trái (hai phía cùng về một hướng). Góc khác → *"Quay 90 độ sang trái hay sang
phải?"*; robot không đoán.

### Hỏi lại có ngữ cảnh

Câu hỏi nêu các bước đã hiểu (*Đã hiểu: 1) quay trái 180 độ; …*) và đúng phần chưa hiểu. Câu trả lời được đặt
vào chỗ đó rồi đọc lại cả lệnh: *"quay 90 độ rồi đi thẳng 30 cm"* → *trái hay phải?* → *"trái"*; *"đi thẳng rồi
rẽ trái"* → *'đi thẳng' bao xa?* → *"nửa mét"*. Trả lời bằng cả lệnh mới cũng được. Một cách đọc làm mất bước của
lệnh đang chờ không bao giờ được nhận. Ở LLM, lịch sử hội thoại và ghi chú câu robot vừa hỏi làm việc đó; số bạn
đưa trong câu trả lời vẫn qua được kiểm tra "số phải có trong câu".

### Kiểm ý định

Stage [2] in hướng và vị trí tích luỹ sau mỗi bước (theo lệnh). Có câu mục đích thì báo bước nào đưa robot về lại
ban đầu (*✓ 'để trở về vị trí ban đâu': sau bước 3*), hoặc cảnh báo khi không bước nào làm được; bước không bị sửa.

## Giới hạn (validator, `drive_config.py`)

| | |
|---|---|
| mỗi lệnh | tối đa 10 bước |
| đi thẳng/lùi | 1 cm … 3 m mỗi bước |
| quay | 1° … 360° mỗi bước (vòng × 360 trong code); không có góc → 90°; quay đầu không góc → 180° (sang trái) |
| số viết bằng chữ | chỉ một từ: *một…mười, nửa, one…ten, half*. Số ghép như "ba mươi" hay "một mét rưỡi" phải viết bằng chữ số |

Góc quay được chia thành các đoạn < 180°, vì SEQ quy mọi hướng về (−180°, 180°]. Nếu không chia,
lệnh "quay phải 180°" sẽ bị chạy thành quay trái, còn "quay 360°" sẽ thành không quay.

## Kết quả mỗi lần chạy — `vision/output/<YYYYmmdd_HHMMSS>/`

| File | Nội dung |
|---|---|
| `detections.mp4` | khung YOLO với khoảng cách (`1.23 m` đo bằng depth Astra, `~1.23 m` ước lượng theo sàn); sàn trống Depth Anything (xanh = trống, đỏ = vật cản); thanh dưới: % sàn trống, khoảng trống phía trước, `t+X.Xs leg N/M <lệnh UART> (bước)` |
| `detections.jsonl` | mỗi frame một dòng: thời gian, trạng thái depth, bước đang chạy, danh sách vật (`range_source`), tóm tắt sàn |
| `run.json` | câu lệnh, parser, JSON thô, các bước, plan (leg + thông số bánh), timeline, kết quả; có bản đồ thật: `start_pose` / `end_pose` (khung bản đồ); `--compensate`: `executed_legs` (mỗi leg đã gửi, bù hay chính, bị firmware cắt không, đo được), `requests`, `measured_pose`, `vo_scale` |
| `plan.txt` | file đưa cho `robot_link --run-plan` |
| `robot_link.log`, `recorder.log` | log của hai tiến trình con |
| `map_build.log` | lệnh "lập bản đồ": log của `drive_map --publish` (bản đồ nằm ở `map/`) |
| `approved_plan.json`, `console.log` | chạy từ trang realroom: kế hoạch trang đã duyệt, và toàn bộ những gì `demo_drive` in ra |

`result`: `complete` | `failed` | `aborted` (Ctrl-C khi xe đang chạy) | `cancelled` (huỷ trước khi chạy).

## An toàn

- **Ctrl-C** được gửi thẳng tới `robot_link` (cùng process group). `robot_link` huỷ plan và gửi `S`.
  Recorder chạy ở session riêng nên video vẫn được đóng đúng cách. Đã test với ESP32 giả: Ctrl-C giữa
  leg 1 → `S`, không có leg nào được gửi thêm.
- Xe chỉ chạy sau khi bạn nhấn Enter (trừ khi có `--yes`) **và** recorder đã quay được frame đầu tiên.
  Nếu recorder không lên thì xe không chạy. Muốn chạy mà không quay thì dùng `--no-vision`.
- Odometry của firmware là open-loop (tính theo số bước đã ra lệnh), nên nó không thấy trượt. `--compensate` đo
  từng leg bằng odometry thị giác (VO) và bù; hệ số trượt k học theo sàn ở `slip_profiles.json`.
- Đi tới một chỗ: đường đi bị từ chối nếu đĩa thân robot chạm ô vật cản, hoặc > 35 % sàn dọc đường chưa từng
  được nhìn thấy (`realroom/README.md`). Stage [2] in tỉ lệ đó trước khi bạn nhấn Enter.
- Lệnh dừng giữa chừng (lỗi, Ctrl-C) không cập nhật vị trí robot trong bản đồ thật, và in cảnh báo.
- Một lúc chỉ một `demo_drive` lái robot: lần chạy thật giữ khoá `vision/output/.robot.lock` (flock, tự nhả khi
  tiến trình kết thúc); lần thứ hai — ở terminal khác hay từ trang realroom — báo lỗi và không gửi gì.
- `demo_drive` luôn đặt lại SIGINT thành KeyboardInterrupt khi khởi động: được chạy từ một tiến trình nền (SIGINT bị
  bỏ qua và được thừa kế) thì Ctrl-C / nút Dừng vẫn có tác dụng.
- Ctrl-C sau khi robot đã chạy xong (lúc quay đuôi video, đóng video) chỉ được ghi nhận (`run.json`
  `stop_requested`), không làm gián đoạn: kết quả giữ nguyên, vị trí chỉ cộng một lần, `run.json` luôn được ghi, và
  lệnh "lập bản đồ" thì không dựng bản đồ. Trong lúc dựng bản đồ, Ctrl-C dừng `drive_map`.
- `--plan-json` chỉ nhận bước trong giới hạn validator; một đường tới một chỗ mang vị trí nó được lập từ và bị từ
  chối nếu bản đồ đã đổi hoặc robot đã dời (> 1 mm / 0.001 rad) kể từ lúc đó.

## File

| File | Vai trò |
|---|---|
| `demo_drive.py` | chương trình chính: hỏi lệnh (`prepare()`, dùng chung với trang realroom), in 4 tầng, chạy recorder và robot_link, ghi `run.json`, cập nhật vị trí trong bản đồ thật, dựng + publish bản đồ sau lệnh "lập bản đồ" |
| `cmd_parser.py` | parser LLM, parser luật, hỏi–đáp có ngữ cảnh, validator, `MotionStep`, kiểm ý định |
| `goto.py` | nhánh `goal`: chọn vị trí đứng (CM grounding, hoặc luật) + đường đi (`realroom/planner.py`) |
| `leg_executor.py` | `--compensate`: từng leg, đo bằng VO, bù, dừng khi kẹt / mất camera |
| `slip_model.py` | hệ số trượt k theo sàn × tiến / lùi / quay trái / quay phải (`slip_profiles.json`) |
| `robot_link_cli.py` | dòng in của `robot_link` và `--stop` trước mỗi plan |
| `motion_plan.py` | bước → primitive → dòng UART → thông số bánh qua MC; `plan.txt`; `plan_from_legs` cho đường đi |
| `drive_config.py` | hằng số: giới hạn, đường dẫn, tốc độ mặc định |
| `prompts/drive_command.json` | system prompt, luật, JSON schema, ví dụ cho LLM (v1.2: quay tự nhiên, `goal`) |
| `natural_commands.json` | bộ câu tự nhiên + kết quả mong đợi, dùng chung cho test luật và test LLM thật |
| `test_drive.py`, `test_leg_executor.py`, `test_live_parser.py` | unit test; executor trên robot giả; bộ câu qua LLM thật |

## Cần có trên Jetson

- `project/build/mc/libmc.so` (`bash project/tools/build_mc.sh`) và
  `motivation/jetson/build/robot_link` (`bash motivation/jetson/tools/build_jetson.sh`).
- Python: `python-dotenv` (CM config). Nếu dùng LLM thì cần thêm `openai` (cho gemini/openai) hoặc `anthropic`.
- Vision: xem `vision/README.md`. Trên Jetson, `ultralytics` phải được cài với `--no-deps` để không thay
  torch của NVIDIA, và `torchvision` phải được build từ source cho đúng bản torch đó.
