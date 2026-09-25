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
python vision/map_builder.py --replay vision/captures/map_...   # dựng lại map từ phiên đã ghi, không cần camera hay model
python vision/map_builder.py --replay DIR --cam-pitch 4         # ... thử lại với thông số gắn camera khác
python vision/map_builder.py --no-da                            # không chạy Depth Anything: chỉ có vật cản, không có ô trống
python vision/test_mapping.py                                   # test trên depth giả, không cần model
```

Tuỳ chọn cho phần sàn: `--da-size 518` (cạnh ngắn ảnh đưa vào Depth Anything), `--floor-far 3.0` (tìm sàn
trống tới bao xa), `--floor-half-width 0` (bề rộng mỗi bên đường nhìn; 0 = cả khung hình).

| Phím | Ý nghĩa |
|---|---|
| `a` / `d` | camera vừa xoay trái / phải `--rot-step` độ (mặc định 45) |
| `w` / `s` | camera vừa tiến / lùi `--move-step` mét (mặc định 0,5) theo hướng đang nhìn |
| `space` | chụp tại pose hiện tại: 3 frame depth, Depth Anything trên ảnh màu và nhãn YOLO được đưa vào map |
| `f` | bật/tắt lớp tô từng pixel: sàn trống xanh lá, vật cản đỏ, cao hơn robot xanh dương, chỗ hụt tím; viền trắng là hình thang tìm sàn, chấm trắng là điểm chạm sàn của vật. Mặc định bật; cũng được lưu thành `NNN_floor.png` mỗi lần chụp. Khi xem live, phần Depth Anything chạy nền nên trễ một lần suy luận (thanh trên cùng hiện `DA x.xs`) |
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
Mỗi lần chụp, tool kiểm mount theo hai cách và in ra số *đo được*: fit mặt sàn trên depth Astra (cần
thấy đủ sàn có depth, trên gạch bóng thường không đủ) và pitch từ điểm chạm sàn của Depth Anything (xem
dưới). Nếu lệch quá 2cm hoặc 1,5° thì tool gợi ý giá trị mới. Khai sai 5° thì sàn ở khoảng cách 3m lệch
khoảng 0,26m, sai 10° thì lệch khoảng 0,5m. Kiểm tra nhanh bằng mắt: phím `f`, các chấm trắng (điểm
chạm) phải nằm đúng chân đồ vật.

**Sàn gạch bóng + camera thấp:** camera ở độ cao 24cm nhìn sàn ở góc chỉ 10–22°, nên chip depth không
khớp được lưới chấm laser trên sàn. Trên frame thật, hình thang sàn 0,6–3m chỉ có 3,6% pixel có depth
(0% ở 0,6–1m). Ảnh IR `captures/ir_pattern.png` cho thấy lưới chấm vẫn in lên sàn. Vì vậy **sàn trống lấy
từ Depth Anything V2 Small chạy trên ảnh màu** (task `2026-09-25_vision-da-floor`).

**Tìm sàn trống (`floor_segment.py`):**
- Quy tắc (user): trong một hình thang ảo trên sàn trước robot, đi lên theo từng cột ảnh thì độ sâu phải
  tăng dần; hết tăng dần là có vật. Dạng định lượng: nghịch đảo độ sâu của sàn thay đổi **đều** theo hàng
  ảnh, còn mọi mặt đứng (tường, chân ghế, mặt trước hộp) giữ nó **không đổi** theo cột. Độ dốc theo cột
  (cửa sổ 9 hàng) chia độ dốc của sàn ra ≈1 trên sàn, ≈0 trên mặt đứng.
- Output của Depth Anything không có mét, và thang/độ lệch đổi theo từng ảnh. Vì vậy mỗi frame tự fit
  một mặt phẳng sàn ngay trên output đó, trong hình thang. Pixel là sàn khi nằm sát mặt phẳng (±4%)
  **và** có độ dốc cột giống sàn. Gần hơn mặt phẳng hoặc đứng thẳng là vật cản. Xa hơn mặt phẳng là
  chỗ hụt (hố, bậc xuống), không bao giờ được tính trống. Vật cản và chỗ hụt phải kéo dài ≥3 hàng, để
  một sợi dây điện 1cm không chặn cả cột.
- Từ mép gần đi lên theo từng cột, sàn được tính là trống cho tới vật cản, chỗ hụt hoặc khung YOLO đầu
  tiên. Hàng đó là **điểm chạm sàn** của vật, và được đưa vào map như vật cản. Sàn nằm sau vật để là
  "chưa biết".
- Vị trí trên sàn tính theo hình học: độ cao camera cộng pitch. Pitch được **đo lại mỗi lần chụp** từ
  các điểm chạm nằm dưới một mặt đứng cao (≥40 hàng có depth Astra), vì chân vật nằm trên sàn nên depth
  Astra ở đó cố định tia. Không đủ điểm như vậy thì dùng `--cam-pitch`. Log in ra pitch đo được và gợi ý
  `--cam-pitch` khi lệch quá 1,5°.
- Nếu hình thang không phải phần lớn là sàn (đứng sát tường), frame đó không cho ô trống nào.

**Map dựng như thế nào:**
- Lưới 5cm (bằng độ phân giải của MV), cộng dồn bằng log-odds.
- Astra chỉ đóng góp **vật cản**: điểm cao 8cm–0,6m (`--max-obstacle-height`). Astra **không còn đánh dấu
  trống**: dải "sàn" cũ của nó thực ra là chân tủ và một ống kim loại nằm trên sàn.
- **Ô trống chỉ lấy từ sàn Depth Anything thấy được**, không dùng dò tia. Dò tia sẽ xoá mất vật thấp và
  mọi thứ trong vùng mù dưới 0,6m. Ô có vật cản Astra trong cùng lần chụp không bao giờ được tính trống.
- Mỗi lần chụp, Depth Anything chạy một lần, nên mang trọng số bằng cả lần chụp: một lần là đủ để ô
  thành trống hoặc có vật, và một lần chụp trái ngược kéo ô về "chưa biết".
- YOLO: khung vật không bao giờ là sàn trống. Nhãn được gắn ở bề mặt vật theo Astra; nếu trong khung
  không có depth Astra thì gắn ở các điểm chạm sàn dưới khung.
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
- Vật thấp chỉ thấy được qua Depth Anything (Astra cần ≥8cm). Ống 3cm bị bắt; dây điện 1cm thì thường
  bị coi là sàn.
- Vật thấp bị đặt **gần hơn tới ~13cm** so với thực tế (phía an toàn). Trên gạch bóng, ảnh phản chiếu
  của vật bị cả Depth Anything lẫn Astra coi là thân vật.
- Pitch sai 1° làm sàn ở 3m lệch khoảng ±0,3m. Pitch chỉ đo được khi có vật cao chạm sàn trong hình thang.
- Hố sâu nhìn từ 24cm thường chỉ thấy thành xa của hố, nên nó thành vật cản ở mép hố thay vì chỗ hụt.
  Cả hai đều chặn vùng trống.
- Depth Anything trên CPU laptop mất khoảng 3,5s mỗi ảnh (518px), 0,8s ở 308px nhưng kém hơn. Trên Jetson
  nên chạy GPU (fp16), sau này chuyển sang TensorRT.
- Cạnh thẳng dài ≥1m của vật lớn như sofa hay giường có thể bị nhận thành tường.
- Chưa có cửa (door).
- Vật mới chỉ nhìn từ một phía thì polygon chỉ phủ phần đã thấy.

## Quay video khi robot chạy (`record.py`)

```bash
python3 vision/record.py --out vision/output/test                      # thường do demo_drive.py gọi
python3 vision/record.py --out DIR --no-depth --no-floor --max-seconds 20
```

Trên mỗi frame video có:

- **Sàn trống (Depth Anything):** dùng cùng `floor_segment.analyze_floor()` như map_builder. Sàn trống tô
  xanh, vật cản đỏ, hố/bậc xuống tím, hình thang vùng tìm sàn viền trắng. Thanh dưới hiện % sàn trống, %
  vật cản, **clear ahead** (khoảng trống trong hành lang rộng ±0,25m trước robot, tính tới điểm chạm gần
  nhất) và độ trễ của DA. DA chạy ở thread riêng cùng phần phân tích sàn: khoảng 0,3s + 0,15s mỗi lần trên
  Orin, nên overlay trễ một lần suy luận. YOLO vẫn chạy mỗi frame.
- **Khoảng cách mỗi vật:** nếu có depth Astra thì là số đo, hiện `1.23 m`. Nếu không có depth, hoặc depth
  quá thưa, thì ước lượng theo hình học sàn, hiện `~1.23 m`: đáy bbox là chỗ vật chạm sàn, cộng với độ cao
  và pitch của camera (`--cam-height`, `--cam-pitch`). Cách này đúng với vật đứng trên sàn. Vật treo hoặc
  đặt trên bàn sẽ bị đọc xa hơn thực tế. Bbox chạm mép dưới ảnh thì hiện `--`, vì vật gần hơn khoảng 0,6m.
  Trong `detections.jsonl`, trường `range_source` là `depth` hoặc `floor`.
- Hiện Jetson chưa có SDK Orbbec nên depth Astra chưa mở được, và mọi khoảng cách đang là ước lượng `~`.

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

- Python có `opencv-python`, `numpy`, `openni` (binding OpenNI2), `ultralytics`, `torch`, `transformers`
  (đã thử 4.40.1).
- **OpenNI2 runtime** (`OpenNI2.dll` hoặc `libOpenNI2.so`, cùng thư mục `Drivers/`) không commit vào repo.
  Trỏ biến môi trường `OPENNI2_REDIST` tới thư mục chứa nó. Trên Windows có fallback về bản SDK đã giải
  nén trong Downloads của laptop dev.
- **Weights**: `models/yolo11n.pt` (gitignore). Nếu thiếu, ultralytics tự tải khi có mạng.
- **Depth Anything V2 Small** (`depth-anything/Depth-Anything-V2-Small-hf`, 99MB, license Apache-2.0): lần
  chạy đầu tải từ HuggingFace rồi lưu vào `models/depth-anything-v2-small/` (gitignore), từ lần sau chạy
  offline. Không thay bằng bản Base/Large nếu dùng thương mại, vì hai bản đó dùng license CC-BY-NC.

### Lên Jetson

1. Tải bản OpenNI2 SDK **Linux ARM64** của Orbbec (bản đang có trên laptop là Windows). Chạy script cài
   udev rules trong SDK để user thường truy cập được USB, rồi `export OPENNI2_REDIST=.../sdk/libs`.
2. Cài torch bản có CUDA dành cho JetPack. `--device auto` sẽ tự chọn CUDA và fp16. Nếu gõ
   `--device cuda` mà torch không thấy GPU, chương trình **báo lỗi**, không lặng lẽ rơi về CPU, để
   không che mất một bộ CUDA hỏng.
3. Camera RGB là một `/dev/videoN` (backend V4L2). Nếu không phải index 0 thì dùng `--color-index`.
   Sai index thì chương trình báo ngay, vì registration cần đúng 640×480.
4. **Đã làm trên Jetson Orin (JetPack 5, Python 3.8), 2026-09-25:** `pip3 install --user --no-deps ultralytics`,
   sau đó `tqdm py-cpuinfo ultralytics-thop` (cũng `--no-deps`). Nếu cài không có `--no-deps`, pip sẽ thay torch
   của NVIDIA. Bản `torchvision` 0.16.0 cài qua pip lỗi ABI (`_C.so: undefined symbol`), khiến NMS của YOLO
   crash. Cần build v0.16.1 từ source với `FORCE_CUDA=1 TORCH_CUDA_ARCH_LIST=8.7` (mất khoảng 30 phút);
   wheel đã build nằm ở `~/wheels/`. YOLO11n fp16 chạy khoảng 40ms/frame. Depth: OpenNI2 hệ thống
   (`/usr/lib`, driver PS1080) **không** thấy Astra, vẫn cần SDK Orbbec ARM64 như bước 1.
5. (Tuỳ chọn, chưa thử) Tăng tốc bằng TensorRT: `yolo export model=yolo11n.pt format=engine half=True`,
   rồi chạy với `--model yolo11n.engine`.

## File

| File | Vai trò |
|---|---|
| `perception.py` | chương trình chính: vòng ≤6Hz, ghép cặp, detect, đo, vẽ, log, phím tắt |
| `record.py` | quay video không cần màn hình cho demo `communication/demo_drive.py`: khung YOLO, khoảng cách mỗi vật, sàn trống từ Depth Anything, bước đang chạy. Ra `detections.mp4` (10fps, khớp thời gian thực) và `detections.jsonl` (xem mục bên dưới) |
| `map_builder.py` | dựng map: chụp khi đứng yên, pose theo phím, ghi phiên, `--replay` |
| `floor_geometry.py` | pixel depth → khung robot (độ cao so với sàn), fit mặt sàn để kiểm tra mount |
| `mono_depth.py` | Depth Anything V2 Small (transformers) + thread nền, frame mới nhất thắng |
| `floor_segment.py` | sàn trống từ Depth Anything: hình thang, mặt sàn mỗi frame, độ dốc theo cột, điểm chạm, đo pitch |
| `occupancy_map.py` | lưới log-odds: vật cản Astra theo dải độ cao, sàn trống + điểm chạm từ Depth Anything, phiếu nhãn |
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
