# communication/ — lệnh chữ → LLM → bánh xe (demo, không dùng bản đồ)

Gõ một câu như *"đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"*. Chương trình phân tích câu lệnh,
chuyển dần qua từng tầng thành thông số cho từng bánh, rồi cho robot chạy. Trong lúc đó `vision/record.py`
detect vật và quay video. Đây là nhánh `primitive` của tầng L2 trong `agent/description/robot_process_llm.md`:
lệnh chuyển động trực tiếp nên bỏ qua bản đồ, L3 và L4.

```bash
python3 communication/demo_drive.py "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
python3 communication/demo_drive.py --dry-run "..."      # chạy mọi thứ trừ UART, video vẫn quay
python3 communication/demo_drive.py                       # hỏi lệnh
python3 communication/test_drive.py                       # unit test, không cần key/camera/robot
```

Tuỳ chọn: `--provider auto|rule|gemini|openai|anthropic`, `--yes` (không chờ Enter), `--port /dev/ttyUSB0`,
`--speed 0.15` (m/s), `--yaw-rate 0.8` (rad/s), `--no-vision`, `--no-depth`, `--no-floor`, `--color-index N`, `-v`.

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

- `auto` (mặc định): dùng LLM mà CM đang cấu hình (`CM_PROVIDER`, mặc định gemini; key đặt trong
  `project/src/CM/.env`, ví dụ `GEMINI_API_KEY=...`). Nếu thiếu SDK, thiếu key hoặc mất mạng thì tự chuyển
  sang luật và **in ra cảnh báo**.
- `rule`: offline. Hiểu các mẫu *đi thẳng/tiến/đi tiếp/lùi X cm|m|mm|phân|mét*, *rẽ/quay/xoay trái|phải
  [X độ]*, *quay X độ sang trái*, *quay đầu*, và câu tiếng Anh tương ứng. Viết có dấu hay không dấu đều được.
  Nếu câu còn thừa một số hoặc một động từ di chuyển mà luật không khớp được, lệnh bị từ chối **nguyên
  câu**. Làm vậy để không bao giờ bỏ sót một bước mà không báo.
- Chọn thẳng `gemini`/`openai`/`anthropic` thì lỗi sẽ được báo ra, không tự chuyển sang luật.

Nếu LLM hỏi lại (ví dụ thiếu khoảng cách), chương trình in câu hỏi. Câu trả lời của bạn được tính là một
phần của lệnh, nên số bạn đưa trong câu trả lời vẫn qua được kiểm tra "số phải có trong câu".

## Giới hạn (validator, `drive_config.py`)

| | |
|---|---|
| mỗi lệnh | tối đa 10 bước |
| đi thẳng/lùi | 1 cm … 3 m mỗi bước |
| quay | 1° … 360° mỗi bước; không có góc → 90°; quay đầu → 180° (sang trái) |
| số viết bằng chữ | chỉ một từ: *một…mười, nửa, one…ten, half*. Số ghép như "ba mươi" hay "một mét rưỡi" phải viết bằng chữ số |

Góc quay được chia thành các đoạn < 180°, vì SEQ quy mọi hướng về (−180°, 180°]. Nếu không chia,
lệnh "quay phải 180°" sẽ bị chạy thành quay trái, còn "quay 360°" sẽ thành không quay.

## Kết quả mỗi lần chạy — `vision/output/<YYYYmmdd_HHMMSS>/`

| File | Nội dung |
|---|---|
| `detections.mp4` | khung YOLO với khoảng cách (`1.23 m` đo bằng depth Astra, `~1.23 m` ước lượng theo sàn); sàn trống Depth Anything (xanh = trống, đỏ = vật cản); thanh dưới: % sàn trống, khoảng trống phía trước, `t+X.Xs leg N/M <lệnh UART> (bước)` |
| `detections.jsonl` | mỗi frame một dòng: thời gian, trạng thái depth, bước đang chạy, danh sách vật (`range_source`), tóm tắt sàn |
| `run.json` | câu lệnh, parser, JSON thô, các bước, plan (leg + thông số bánh), timeline, kết quả |
| `plan.txt` | file đưa cho `robot_link --run-plan` |
| `robot_link.log`, `recorder.log` | log của hai tiến trình con |

`result`: `complete` | `failed` | `aborted` (Ctrl-C khi xe đang chạy) | `cancelled` (huỷ trước khi chạy).

## An toàn

- **Ctrl-C** được gửi thẳng tới `robot_link` (cùng process group). `robot_link` huỷ plan và gửi `S`.
  Recorder chạy ở session riêng nên video vẫn được đóng đúng cách. Đã test với ESP32 giả: Ctrl-C giữa
  leg 1 → `S`, không có leg nào được gửi thêm.
- Xe chỉ chạy sau khi bạn nhấn Enter (trừ khi có `--yes`) **và** recorder đã quay được frame đầu tiên.
  Nếu recorder không lên thì xe không chạy. Muốn chạy mà không quay thì dùng `--no-vision`.
- Odometry là open-loop (tính theo số bước đã ra lệnh), nên robot không biết bánh có trượt hay không.

## File

| File | Vai trò |
|---|---|
| `demo_drive.py` | chương trình chính: hỏi lệnh, in 4 tầng, chạy recorder và robot_link, ghi `run.json` |
| `cmd_parser.py` | parser LLM, parser luật, validator, `MotionStep` |
| `motion_plan.py` | bước → primitive → dòng UART → thông số bánh qua MC; `plan.txt` |
| `drive_config.py` | hằng số: giới hạn, đường dẫn, tốc độ mặc định |
| `prompts/drive_command.json` | system prompt, luật, JSON schema, ví dụ cho LLM |
| `test_drive.py` | unit test |

## Cần có trên Jetson

- `project/build/mc/libmc.so` (`bash project/tools/build_mc.sh`) và
  `motivation/jetson/build/robot_link` (`bash motivation/jetson/tools/build_jetson.sh`).
- Python: `python-dotenv` (CM config). Nếu dùng LLM thì cần thêm `openai` (cho gemini/openai) hoặc `anthropic`.
- Vision: xem `vision/README.md`. Trên Jetson, `ultralytics` phải được cài với `--no-deps` để không thay
  torch của NVIDIA, và `torchvision` phải được build từ source cho đúng bản torch đó.
