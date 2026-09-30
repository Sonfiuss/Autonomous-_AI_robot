# realroom/ — bản đồ phòng thật và lập đường đi trên nó

Bản đối ứng của `simulation/room/` cho phòng thật. `simulation/` vẫn là module riêng, không bị sửa. Hai module
dùng chung một định dạng: **Scene JSON v2.0** (`simulation/room/README.md`), cũng là định dạng CM (grounding)
và MV (lập đường) đọc. Chỉ nguồn dữ liệu khác nhau: ở đây scene đến từ `vision/drive_map.py` trên một lần chạy
thật, thay vì từ `room_generator.py`.

```bash
python3 vision/drive_map.py --run vision/output/<run> --publish   # dựng bản đồ từ một lần chạy, dùng nó từ giờ
python3 realroom/map_store.py show                                # bản đồ hiện tại + vị trí robot + màu vật
python3 realroom/map_image.py [--map <run>] [--out FILE]          # ảnh PNG để đánh giá (mặc định maps/latest.png)
python3 realroom/app.py                                           # web: http://<jetson>:5002 - xem + chat lái robot
python3 realroom/app.py --dry-run                                 # thử trang: Chạy không động tới robot / camera
python3 communication/demo_drive.py "đi tới cái ghế"              # lệnh dùng bản đồ này (từ terminal)
python3 realroom/test_realroom.py                                 # test offline
python3 realroom/test_chat.py                                     # test offline chat + chạy (~1 phút, dry-run)
```

## Dữ liệu — `realroom/maps/`

Biến môi trường `REALROOM_MAPS_DIR` thay thư mục này cho mọi tiến trình thừa kế nó (test, hoặc thử trang trên một
bản sao bản đồ).

| File | Nội dung |
|---|---|
| `map_<run>.json` | scene đúng như lúc dựng (id = tên run) |
| `map_<run>_grid.npz` | lưới "đã biết": `cells` int8 [iy, ix] — 1 vật cản, 0 trống (đã thấy, hoặc thân robot đã đi qua), −1 chưa thấy; `res_m`; `origin_m` = góc ô (0, 0) trong khung scene |
| `latest.json` | bản đồ các lệnh sau dùng; `robot` = vị trí **hiện tại**, `pose_history` = mọi lần vị trí đổi (lệnh nào, đo bằng VO hay dự đoán) |
| `map_<run>.png`, `latest.png` | ảnh của hai file trên (`map_image.py`), vẽ lại sau mỗi lần publish / mỗi lần vị trí đổi |

Scene có thêm so với simulation: `objects[].color_rgb` ("#rrggbb" hoặc null), `objects[].color_share` (tỉ lệ
pixel bỏ phiếu cho màu đó, hoặc null), `map_offset`, `map_id`, `source_run`,
`grid_file`, `pose_history`. `robot` có đủ `chassis` / `wheels` / `footprint` như simulation, nhưng là hình xe
**thật**: hình simulation xoay 180° → bánh W1 ở 0° (mũi hẹp phía trước, hướng camera), W2 / W3 ở ±120°
(simulation vẽ 60/180/300°). Bản đồ publish trước 2026-09-29 không có các trường này → vẽ xe bằng đĩa.

## Màu vật — `vision/object_color.py`

`drive_map` lấy màu cho từng vật từ chính các khung hình đã phân tích sàn: ở mỗi cột ảnh chạm vật cản, pixel vật
cản ngay trên chân vật (tối đa 60 hàng) bỏ phiếu cho ô bản đồ dưới chân đó, bằng một trong 11 tên màu cơ bản
(black, white, gray, red, orange, yellow, green, blue, purple, pink, brown — gồm cả bảng màu của simulation).
Chân vật ở đáy hộp YOLO (phần giữa 60 % bề rộng hộp) lấy lên tới đỉnh hộp, vì nhãn vật cản chỉ cao tới mép hình
thang sàn và cả hộp bị tô là vật cản (giữa chân ghế là sàn, dây, bóng). Tường, sàn, bóng không có sắc, nên một màu
có sắc chiếm ≥ 25 % pixel của vật là màu của vật; nếu không, màu nhiều nhất nếu chiếm ≥ 40 %; < 50 pixel → `unknown`.
Đây là tên màu, không phải màu sơn: cân bằng trắng của camera và đèn phòng nhuộm mọi pixel.
Bản đồ 20260928_222009: ghế green (28 % pixel — phần còn lại là tấm bảng đen phía sau, sàn), vali black, obj_4
green (chậu), obj_2 / obj_5 gray, obj_6 unknown (27 pixel). Trên bản đồ, vật được tô nhạt bằng `color_rgb` của nó.

## Nhãn vật lớn — người nằm trên giường

YOLO thấy một vật lớn từng phần (chỉ bàn chân, cả người, đầu và ngực khi ở gần), mỗi phần có đáy hộp ở một hàng
khác, nên rơi thành nhiều cụm. `drive_map` gộp các cụm cùng lớp mà một hộp đã bao cả hai (cách nhau không quá bề
rộng hộp + 15 cm) → một vật `parts` phần, `extent_m` bao các phần; hai cái chai cùng hướng nhìn vẫn tách riêng (hộp
chai rộng vài cm). Hộp có đáy cách mép dưới ảnh ≤ 8 px thì chân nằm ngoài ảnh → bỏ.
Người nằm trên giường: đáy hộp là mặt nệm, chiếu xuống sàn rơi SAU mép giường → phiếu nhãn đặt lên vật cản đầu tiên
dưới đáy hộp (mép giường). Mép giường dài ≥ 1 m trông như tường; một "tường" có ≥ 10 phiếu của một lớp là cạnh của
vật đó → vật riêng mang nhãn ấy. Nhãn của vật tính cả phiếu cách nó ≤ 2 ô (10 cm) mà gần nó hơn mọi vật khác (một phiếu chỉ tính cho một vật). Bản đồ 20260929_231612: 5 cụm person
→ 1 (+ 1 vật tối ở đầu giường, 4 lần thấy, user chấp nhận), `obj_14 person` = mép giường → "đi tới người" có đích.

Khung: khung scene = khung của run đã dựng bản đồ (x tới trước lúc run bắt đầu, y sang trái, θ ngược chiều kim
đồng hồ), dời để hộp bao mọi thứ đã thấy bắt đầu ở (0, 0) (`map_offset`: scene = map − offset). Vùng thân robot
đã đi qua được tính là trống: camera không bao giờ thấy dưới và sau xe, nên thiếu nó thì robot đứng ngoài khung
và không có đường về.

## Vị trí robot

`communication/demo_drive.py` ghi `start_pose` (lấy từ `latest.json`) vào `run.json`. Chạy xong, nó cộng chuyển
động của lệnh vào vị trí đó:
- `--compensate`: chuyển động đo bằng VO;
- chạy thường: lệnh × hệ số trượt k, gắn cờ `predicted`.

Lệnh dừng giữa chừng thì **không** cập nhật và in cảnh báo: đường truyền có thể chết trong khi ESP32 vẫn chạy
tiếp (run 2026-09-28 221843). Khi đó đặt lại robot, hoặc dựng lại bản đồ. Mỗi lần `--publish` thay bản đồ bằng
run mới nhất, và vị trí robot là chỗ run đó kết thúc — trừ khi publish lại đúng bản đồ đang dùng (dựng lại run
đó, ví dụ để có màu) trong cùng khung: khi đó vị trí hiện tại và lịch sử được giữ. Đặt robot về chỗ run kết thúc:
`python3 realroom/map_store.py publish vision/output/<run> --reset-pose`. Ghép nhiều run thành một bản đồ chung thuộc
explore-map Pha 2, chưa làm ở đây.

Lệnh *"lập bản đồ"* ở terminal (`demo_drive "lập bản đồ"` / `--map`; chat trên trang không lập bản đồ) tự làm bước publish: chạy xong,
`demo_drive` gọi `drive_map --run <run> --publish` → bản đồ mới (id = run đó) thay bản đồ cũ, robot ở chỗ run kết
thúc theo VO; trang tự tải lại. Chi tiết: `communication/README.md` mục "Lập bản đồ".

## Lập đường — `planner.py`

MV (qua `mv_client` của CM) lập đường trên các polygon vật của scene, và coi mọi chỗ khác trong hộp phòng là
trống. Ở bản đồ thật, phần lớn hộp đó chưa ai nhìn thấy, nên mỗi đường MV đưa ra được kiểm trên lưới "đã biết"
bằng cách quét đĩa thân robot (bán kính của scene) dọc đường robot thật sự chạy - các đoạn thẳng nối waypoint
của MV, không phải đường lưới dày của MV (đường đó uốn quanh góc mà các leg cắt thẳng):
- chạm ô vật cản → từ chối;
- sàn chưa thấy > `MAX_UNSEEN_SHARE` (35 %) → từ chối;
- MV dời điểm xuất phát > 5 cm (robot đứng trong lề an toàn của một vật) → từ chối.

Ngưỡng 35 % đo trên bản đồ 20260928_222009: 10 vị trí cạnh 6 vật đều đi được, sàn chưa thấy 9–29 %, cao nhất ở
các đường ngắn. Phần chưa thấy chủ yếu là vùng mù ~0.78 m ngay trước xe (camera chỉ thấy sàn từ ~0.6 m trước
ống kính). Robot không có cảm biến va chạm, nên tỉ lệ này được in ra trước khi bạn nhấn Enter.

Các leg là kế hoạch không holonomic của MV, quay – đi – quay: `("ROTATE", rad tương đối)` và `("FORWARD", m)`.

## Web — `app.py` (cổng 5002)

| Endpoint | Trả về |
|---|---|
| `GET /` | trang: bản đồ (lưới đã biết, vật + nhãn + ô màu, robot + hướng, vệt `pose_history`, đường A* + đích của kế hoạch; các vị trí cạnh vật chỉ khi tick "hiện các điểm đến") bên trái, cột chat + Chạy / Dừng bên phải (mục dưới); nút "Tải PNG" |
| `GET /api/map/latest` | `latest.json` + lưới cắt theo phòng (`rows`: chuỗi `o` vật cản / `.` trống / ` ` chưa thấy) |
| `GET /api/map/<id>` | một bản đồ đã publish, đúng như lúc dựng |
| `GET /api/map/latest.png[?spot=<id>]` | ảnh `map_image.py` của `latest.json` (+ đường tới spot đó) |
| `GET /api/map/<id>.png` | ảnh của một bản đồ đã publish, đúng như lúc dựng |
| `GET /api/spots` | các vị trí cạnh vật (CM `candidates.py`) |
| `POST /api/plan` | `{"spot": id}` hoặc `{"goal": {x, y, theta}}` → `RoomPlan` (ok, reason, legs, path, length_m, unseen_share, …) — chỉ xem |
| `GET /api/map/version` | `{version}`: mtime (ns, dạng chuỗi) của `latest.json` — đổi sau mỗi publish / mỗi lần vị trí đổi |
| `POST /api/chat` | `{"text"}` → `{type: ask \| plan \| none \| busy \| error, text, plan}` (`drive_chat.ChatSession`); `plan.target` = đích của đường (`{map_id, spot_id, object_id, name, x, y, theta}`) hoặc null với lệnh bước |
| `POST /api/chat` (tiếp) | cả câu là *"chạy / chạy đi / bắt đầu / ok / run / go"* khi có kế hoạch → chạy nó: `{type: run, text, run}` (như `/api/run`); là *"đi tiếp / tiếp tục"* → `ChatSession.go_on` tới đích lần chạy trước chưa tới |
| `POST /api/chat/go_on` | `ChatSession.go_on(RunManager.unreached())`: đường A* từ vị trí hiện tại tới đích lần chạy trước chưa tới |
| `POST /api/chat/reset` | quên lệnh đang hỏi dở và kế hoạch chưa chạy |
| `POST /api/run` | `{"compensate": true}` → chạy kế hoạch cuối của chat; 400 chưa có kế hoạch, 409 đang có run |
| `POST /api/run/stop` | SIGINT tới run (như Ctrl-C) |
| `GET /api/run/status` | `{state: idle \| running \| stopping \| finished, run_id, target, line, log, elapsed_s, exit_code, result, end_pose, map, video, message, go_on, dry_run}`; `message` = câu chat khi xong (`drive_chat.said` + `leg_report`); `go_on` = chưa tới đích mà biết robot đang ở đâu |

Tuỳ chọn: `--provider auto|rule|gemini|openai` (parser của chat), `--dry-run`, `--port`.

## Điều khiển robot từ trang — `drive_chat.py`

Khung chat dùng để **đi lại trên bản đồ đã có** (`maps/latest.json`); nó không lập bản đồ (*"lập bản đồ"* → hướng
dẫn dùng `demo_drive --map` ở terminal). Mỗi lệnh đi qua đúng `demo_drive.prepare()` của terminal:
- nói một chỗ: *"di chuyển tới bên cạnh cái ghế màu xanh"*, *"đến bên trái cái giường"*, *"đi tới người"* → parser
  (Gemini; hết quota / quá tải → model phụ; không LLM → bộ luật, chat ghi rõ) lấy cụm chỉ chỗ → `goto.ground` chọn
  vị trí cạnh vật (LLM chọn trong các spot của bản đồ; bộ luật: loại vật + màu + phía) → `planner` (A* của MV, kiểm
  trên lưới đã biết) → bot trả lời *"Đích: cạnh sau của ghế xanh lá (obj_16) / Đường A*: 0.48 m, 3 leg, khoảng 12 s"*,
  bản đồ vẽ đường nét đứt đỏ và vòng đỏ có tên ở đích. Nhiều vật hợp → bot hỏi cái nào (tiếng Việt); đã biết vật thì
  robot tự chọn phía và chỗ (`goto.realroom_prompt`: không hỏi phía / giữa hay góc; `order_spots`: giữa cạnh trước
  góc, gần robot trước; `route_first`: chỗ đầu tiên A* tới được, chỗ bị bỏ qua được nói kèm lý do); không có đường
  → nói lý do;
- lệnh bước tương đối vẫn dùng được: *"đi thẳng 50 cm"*, *"quay trái 90 độ"* (có cảnh báo `!` nếu đường quét qua
  ô vật cản hoặc > 35 % sàn chưa thấy — robot vẫn chạy nếu bạn bấm, như ở terminal);
- câu hỏi của parser / bản đồ hiện trong chat, câu gõ tiếp theo là câu trả lời (mỗi lệnh chạy trên một luồng riêng,
  chờ như `input()`; 90 s không trả lời được thì huỷ). **Huỷ lệnh** bỏ câu đang hỏi và kế hoạch. Ô nhập: Enter gửi,
  Shift+Enter xuống dòng;
- **Chạy**: nút ngay trong bong bóng kế hoạch, nút ở thanh dưới, hoặc gõ *"chạy"* (chỉ khi có kế hoạch): `demo_drive --yes --plan-json vision/output/<run>/approved_plan.json --run-dir
  … [--compensate]` trong process group riêng (`RunManager`). Kế hoạch bị tiêu ngay: bấm hai lần không chạy hai lần.
  Ô *đo bằng camera và bù lệch (VO)* (`--compensate`) mặc định bật. Trong lúc chạy, chat trả lời *"Robot đang chạy"*;
- **Dừng** (hoặc phím **Esc**): SIGINT cả group — đúng đường Ctrl-C đã kiểm (robot_link huỷ plan, gửi `S`;
  demo_drive vẫn đóng video, ghi `run.json`). Dừng khi demo_drive còn đang khởi động → `cancelled`, không gì
  được gửi tới robot; Dừng khi robot đã xong (đang đóng video) → kết quả giữ nguyên;
- chạy xong mà chưa tới (dừng giữa kế hoạch, còn cách > 15 cm): bot kể từng leg (`leg_report`, từ `run.json`
  requests của `--compensate`): *"✗ quay phải 122°: đo được -114° - bù thêm không nhích được nữa, firmware cắt ngắn
  3/3 lần / – đi thẳng 0.55 m: chưa chạy"*, vị trí hiện tại (camera đo / dự đoán), rồi tự lập **đường đi tiếp** từ
  vị trí đó tới cùng đích (`ChatSession.go_on`, có nút Chạy; không tự lái). Sau Dừng thì không tự lập — gõ
  *"đi tiếp"*. Vị trí sau một run dừng sớm là "đo" khi mọi leg đã chạy đều được camera đo; "dự đoán" khi có leg
  không đo được hoặc robot có thể đã đi thêm mà không đo (Dừng giữa leg, mất nối ESP32);
- chạy xong, bot nhắn (`drive_chat.said`): *"Đã tới cạnh sau của ghế xanh lá (obj_16). Robot ở (x, y), cách đích N cm."*
  (vị trí cuối trong `ARRIVE_TOL_M` = 15 cm quanh đích), *"Đã chạy hết đường nhưng còn cách … N cm"*, *"Chưa tới …:
  đã dừng giữa đường"*, *"Chưa tới …: lần chạy failed"*, hoặc lý do robot không chạy; vị trí dự đoán (không
  `--compensate`) có ghi chú. Nhật ký `demo_drive` xem ở mục *nhật ký demo_drive*.

Bản đồ tự làm mới: trang hỏi `/api/map/version` mỗi 2 s, đổi thì tải lại bản đồ + vị trí robot — sau mỗi lệnh (vị
trí dời), sau một bản đồ mới publish từ terminal. Reload trang cũng được.

Giới hạn: trang mở cho cả mạng LAN — ai vào được :5002 cũng lái được robot; một người điều khiển một lúc (các tab
dùng chung một hội thoại). Đường tới một chỗ được lập từ vị trí lúc chat; nếu robot đã dời hoặc bản đồ đổi trước khi
bấm Chạy, `demo_drive` từ chối (*"lập lại kế hoạch"*). Chạy song song với `demo_drive` ở terminal: lần sau bị từ chối
(khoá `vision/output/.robot.lock`). Tắt server web giữa chừng không dừng run đang chạy (nó ở session riêng) — dùng
Ctrl-C/`kill -INT` với tiến trình `demo_drive`.

Thử không robot: `python3 realroom/app.py --dry-run` (Chạy = phát lại thời gian của các leg, không camera, không
UART, vị trí không đổi; run vẫn ghi vào `vision/output/`).

Bảng màu của bản đồ cố định (trang web và PNG như nhau, chế độ sáng hay tối): sàn trống #f4f4f0, vật cản #1c1c1c,
chưa thấy #84847e gạch chéo — độ sáng L* 96 / 11 / 55, cách nhau ≥ 40. Lưới 0.5 m chỉ vẽ trên sàn trống.
