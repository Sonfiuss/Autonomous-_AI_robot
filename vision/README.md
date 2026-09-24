# vision/ — nhận diện vật, khoảng cách và dựng map (Orbbec Astra Pro)

RGB + depth của Orbbec Astra Pro → YOLO (80 lớp COCO) → khoảng cách và vị trí 3D của từng vật →
map phòng dạng lưới + scene JSON v2.0 như `simulation/room/README.md`. Đây là tầng L1 (perception)
trong `agent/description/robot_process_llm.md`.

## Dựng map (`map_builder.py`) — chiến lược tối thiểu

Robot chưa tự chạy được, nên người dùng cầm camera di chuyển rồi **báo cho tool biết đã làm gì**. Cách
này giống hệt việc robot sau này sẽ báo chặng `F`/`T` vừa chạy xong. Quy trình: di chuyển → bấm phím
tương ứng → giữ yên → bấm `space` để chụp. Chưa có scan matching, nên pose sai bao nhiêu thì map lệch bấy nhiêu.

```powershell
python vision/map_builder.py                                    # chạy trực tiếp, ghi phiên vào captures/map_<thời gian>/
python vision/map_builder.py --replay vision/captures/map_...   # dựng lại map từ phiên đã ghi, không cần camera
python vision/map_builder.py --replay DIR --cam-pitch 4         # ... thử lại với thông số gắn camera khác
python vision/test_mapping.py                                   # test trên depth giả
```

| Phím | Ý nghĩa |
|---|---|
| `a` / `d` | camera vừa xoay trái / phải `--rot-step` độ (mặc định 45) |
| `w` / `s` | camera vừa tiến / lùi `--move-step` mét (mặc định 0,5) theo hướng đang nhìn |
| `space` | chụp tại pose hiện tại: 3 frame depth và nhãn YOLO được đưa vào map |
| `f` | bật/tắt lớp tô từng pixel: sàn xanh lá, vật cản đỏ, cao hơn robot xanh dương, dải 5–8cm (bỏ qua) vàng, thấp hơn sàn tím. Mặc định bật; cũng được lưu thành `NNN_floor.png` mỗi lần chụp |
| `m` | ghi file map ngay |
| `q` / `ESC` | ghi file map rồi thoát |

**Cách chụp một phòng:**
1. Đặt camera ở giữa phòng, chụp 8 hướng: `space`, rồi `a` + xoay tay 45° + `space`, lặp lại đến khi đủ 8.
   FOV 58° nên hai ảnh liền nhau chồng lên nhau khoảng 13°.
2. Chuyển sang 1–2 vị trí khác để nhìn thấy mặt sau của các vật, rồi quét thêm vài hướng ở mỗi vị trí.

**Thông số gắn camera:** mặc định `--cam-height 0.24 --cam-pitch 0`, trong đó pitch dương là **nghiêng
xuống**. Ban đầu user cho số 5°, nhưng trên khung hình thật, khai "xuống 5°" đẩy chân đồ vật xuống dưới
sàn (tô tím), còn 0° đặt chúng đúng lên mặt sàn. User đã chốt 0° (2026-09-25). Đối chiếu trong
`captures/floor_pitch_compare.png`.
Mỗi lần chụp, tool tự fit mặt sàn rồi in ra chiều cao và góc nghiêng *đo được* (cần thấy đủ sàn). Nếu
lệch quá 2cm hoặc 1,5° thì tool gợi ý giá trị mới. Khai sai 5° thì sàn ở khoảng cách 3m lệch khoảng
0,26m, sai 10° thì lệch khoảng 0,5m. Kiểm tra nhanh nhất bằng mắt: phím `f`, chân đồ vật phải nằm ở
dải xanh lá.

**Sàn gạch bóng + camera thấp:** camera ở độ cao 24cm nhìn sàn ở góc chỉ 15–22° (khoảng cách 0,6–0,9m),
nên chip depth không khớp được lưới chấm laser trên sàn. Sàn gần như **không có depth**, trong khi map
chỉ coi ô là trống khi thấy sàn. Kết quả là trên loại sàn này, map gần như chỉ có vật cản và vùng chưa
biết. Ảnh IR `captures/ir_pattern.png` cho thấy lưới chấm vẫn in lên sàn.

**Map dựng như thế nào:**
- Lưới 5cm (bằng độ phân giải của MV), cộng dồn bằng log-odds.
- Điểm cao 8cm–0,6m là **vật cản** (`--max-obstacle-height`). Điểm cao dưới 5cm là **sàn**.
- **Ô chỉ được coi là trống khi nhìn thấy sàn ở ô đó**, không dùng dò tia. Dò tia sẽ xoá mất vật thấp
  và mọi thứ trong vùng mù dưới 0,6m. Ô chưa từng thấy sàn thì là "chưa biết".
- Tường là các đường thẳng dài ≥1m, tìm bằng Hough và gộp theo cùng một đường. Phải tìm tường trước,
  vì tường nối nhau ở góc phòng thành một vành khép kín.
- Vật là các cụm ô vật cản còn lại. Nhãn lấy theo lớp YOLO có nhiều phiếu nhất trên các ô của cụm.
  Nếu không đủ phiếu thì nhãn là `unknown`, nhưng cụm vẫn là vật cản.

**Output, trong thư mục phiên:**
- `map.png`: trắng là trống, đen là vật cản, xám là chưa biết, đường đỏ là tường.
- `map_grid.npy` + `map_meta.json`: lưới log-odds kèm gốc toạ độ và độ phân giải.
- `scene.json`: v2.0 với các trường `room`, `walls`, `robot`, và `objects` gồm id, class hoặc
  `unknown`, `confidence`, `center`, `yaw`, polygon lồi 3–6 đỉnh ngược chiều kim đồng hồ phủ trọn vật,
  `free_sides`, `near`. `doors` để trống, `color` là `"unknown"`. `map_offset` là độ dời từ khung map
  sang khung scene.

**Giới hạn đã biết:**
- Pose do người nhập, không được kiểm tra lại.
- Vật cao dưới 8cm (dây điện, ngưỡng cửa) không thấy được.
- Cạnh thẳng dài ≥1m của vật lớn như sofa hay giường có thể bị nhận thành tường.
- Chưa có cửa (door).
- Vật mới chỉ nhìn từ một phía thì polygon chỉ phủ phần đã thấy.

## Chạy

```powershell
python vision/perception.py                   # --device auto: CUDA (fp16) nếu có, không thì CPU
python vision/perception.py --device cpu
python vision/perception.py --headless        # không mở cửa sổ (Jetson qua SSH), chỉ log; Ctrl+C để dừng
python vision/test_vision.py                  # unit test, không cần camera hay model
```

Tuỳ chọn: `--conf 0.5`, `--imgsz 640`, `--fps 6`, `--color-index 0`, `--model PATH`,
`--intrinsics FILE.json`. Trên Linux không có `DISPLAY` thì tự chạy headless.

Phím tắt (khi cửa sổ đang focus): `q`/`ESC` hoặc nút X để thoát. `s` lưu RGB, depth đã registration
(PNG 16-bit, đơn vị mm), ảnh có chú thích và `detections.json` vào `captures/`. `o` bật/tắt lớp depth
phủ lên RGB để tự kiểm tra registration bằng mắt: mép vật trên depth phải trùng mép vật trên RGB.

Trên màn hình, mỗi vật hiện nhãn `class conf | X.XX m`, với khoảng cách là khoảng cách Euclid từ
camera. Nếu hiện `--` thì depth ở đó quá thưa hoặc quá vụn để tin được. Thanh trên cùng hiện trạng thái
(`OK` / `NO COLOR` / `NO DEPTH` / `OUT OF SYNC`), tần số xử lý, thời gian suy luận và device.

## Sự thật về phần cứng (đã kiểm chứng, 2026-09-25)

- **Hai thiết bị USB riêng.** Depth là `PID_0403`, đọc qua OpenNI2. RGB là "Astra Pro HD Camera"
  `PID_0501`, một webcam UVC thông thường, đọc bằng `cv2.VideoCapture`. Nếu xin color stream từ
  OpenNI2 thì bị treo hoặc segfault, vì dòng Pro không đưa màu qua OpenNI2.
- **Phải tắt mirror depth.** OpenNI2 mặc định lật ngang depth, còn UVC thì không. Nếu để mặc định,
  depth đã registration nằm ngược trái-phải so với RGB.
- **Hardware registration D2C hoạt động.** Khi tắt, depth lệch dọc khoảng 12–15px so với RGB (ở khoảng
  0,6–1m). Khi bật, mép vật khớp.
- **Không có intrinsics của nhà sản xuất.** Property `OBEXTENSION_ID_CAM_PARAMS` trả về NaN. Chương trình
  tạm dùng FOV do OpenNI2 báo (58,6°×45,6° → fx≈fy≈570). Giá trị Z đo trực tiếp nên đúng. X/Y ngang có thể
  sai vài %, vì FOV đó là của camera IR còn ảnh sau D2C theo hình học của RGB. Nếu có file hiệu chuẩn thì
  truyền vào bằng `--intrinsics` (JSON `{fx, fy, cx, cy}`).
- **Tầm đo từ 0,6 đến 8m.** Vật gần hơn 0,6m sẽ hiện `--`. Với robot, điều này có nghĩa là
  **không biết**, không có nghĩa là **không có vật**.
- **YOLO trên CPU laptop** (torch 2.1 CPU): yolo11n ở imgsz 640 mất khoảng 170–230ms, nên chỉ đạt
  khoảng 5Hz thay vì 6Hz. Trên Jetson có CUDA thì dư. yolo26n không nhanh hơn trên máy này.

## Cách chống treo và chống đo sai

- Mỗi camera được đọc trên một thread nền riêng (`frame_grabber.py`). Vòng xử lý không bao giờ gọi
  thẳng vào driver, nên USB có giật khi di chuyển camera nhanh thì cửa sổ vẫn phản hồi và `q` vẫn thoát được.
- Depth được đọc qua `wait_for_any_stream(timeout)`, vì `read_frame()` của OpenNI2 không có timeout.
  Nguyên nhân treo của bản trước chính là chỗ này.
- Nếu một nguồn không có frame nào trong 2s, grabber tự đóng rồi mở lại. Khi thoát, thread bị kẹt trong
  driver sẽ bị bỏ lại sau 2s và chương trình thoát cứng, không treo theo.
- **Ghép cặp theo thời gian.** Frame RGB mới nhất được ghép với frame depth gần nó nhất về thời gian
  (trong 6 frame gần nhất). Nếu lệch quá 40ms thì hiện `OUT OF SYNC` và không đo. Lý do: khi camera đang
  di chuyển, depth lệch vài chục ms sẽ làm bbox rơi sang nền, ra một con số tự tin nhưng sai. Đo thực tế
  trên laptop: độ lệch trung vị 0ms, 90% nằm trong khoảng −19 đến +15ms.
- **Khoảng cách tới vật** (`object_distance.py`): chỉ lấy phần lõi 50% ở giữa bbox, chia các giá trị
  depth hợp lệ thành cụm theo khoảng nhảy độ sâu, rồi lấy **cụm gần nhất có ít nhất 20% số pixel**.
  Không dùng median của cả bbox, vì với vật có lỗ như ghế, median đó rơi vào bức tường phía sau.
  Nếu dưới 30% pixel ở lõi là hợp lệ thì trả về `--`.
- **Giới hạn còn lại:** timestamp là thời điểm host nhận frame, không phải thời điểm cảm biến chụp. Nếu
  hai camera có độ trễ USB khác nhau một khoảng cố định, việc ghép cặp không phát hiện được khoảng đó.

## Cài đặt

- Python có `opencv-python`, `numpy`, `openni` (binding OpenNI2), `ultralytics`, `torch`.
- **OpenNI2 runtime** (`OpenNI2.dll` hoặc `libOpenNI2.so`, cùng thư mục `Drivers/`) không commit vào repo.
  Trỏ biến môi trường `OPENNI2_REDIST` tới thư mục chứa nó. Trên Windows có fallback về bản SDK đã giải
  nén trong Downloads của laptop dev.
- **Weights**: `models/yolo11n.pt` (gitignore). Nếu thiếu, ultralytics tự tải khi có mạng.

### Lên Jetson

1. Tải bản OpenNI2 SDK **Linux ARM64** của Orbbec (bản đang có trên laptop là Windows). Chạy script cài
   udev rules trong SDK để user thường truy cập được USB, rồi `export OPENNI2_REDIST=.../sdk/libs`.
2. Cài torch bản có CUDA dành cho JetPack. `--device auto` sẽ tự chọn CUDA và fp16. Nếu gõ
   `--device cuda` mà torch không thấy GPU, chương trình **báo lỗi**, không lặng lẽ rơi về CPU, để
   không che mất một bộ CUDA hỏng.
3. Camera RGB là một `/dev/videoN` (backend V4L2). Nếu không phải index 0 thì dùng `--color-index`.
   Sai index thì chương trình báo ngay, vì registration cần đúng 640×480.
4. (Tuỳ chọn, chưa thử) Tăng tốc bằng TensorRT: `yolo export model=yolo11n.pt format=engine half=True`,
   rồi chạy với `--model yolo11n.engine`.

## File

| File | Vai trò |
|---|---|
| `perception.py` | chương trình chính: vòng ≤6Hz, ghép cặp, detect, đo, vẽ, log, phím tắt |
| `map_builder.py` | dựng map: chụp khi đứng yên, pose theo phím, ghi phiên, `--replay` |
| `floor_geometry.py` | pixel depth → khung robot (độ cao so với sàn), fit mặt sàn để kiểm tra mount |
| `occupancy_map.py` | lưới log-odds: vật cản theo dải độ cao, ô trống khi thấy sàn, phiếu nhãn |
| `scene_export.py` | lưới → tường (Hough) + vật (polygon, nhãn/unknown, free_sides, near) → scene JSON |
| `drawing.py` | vẽ khung nhìn camera và map từ trên xuống |
| `test_mapping.py` | test map trên depth giả (dựng ảnh một căn phòng hộp) |
| `frame_grabber.py` | thread đọc nền, tự mở lại khi stall, `pair_frames()` |
| `depth_source.py` | OpenNI2: tắt mirror, bật D2C, đọc có timeout |
| `color_source.py` | RGB UVC qua OpenCV (DSHOW trên Windows, V4L2 trên Linux) |
| `detector.py` | YOLO, `select_device(auto/cpu/cuda)`, fp16 trên CUDA |
| `object_distance.py` | khoảng cách và vị trí 3D từ depth, module thuần |
| `test_vision.py` | unit test không cần phần cứng |
| `models/`, `captures/` | weights và ảnh đã lưu (gitignore) |
