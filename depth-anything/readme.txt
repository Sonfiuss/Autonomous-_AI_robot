==============================================================
 depth-anything — Pipeline ước lượng độ sâu Depth Anything V2
==============================================================

CHỨC NĂNG
  Ước lượng độ sâu mono (1 ảnh RGB → bản đồ độ sâu) bằng Depth Anything V2.
  Là NGUỒN ĐỘ SÂU dùng chung cho deepmap (map 360°) và slam (RGB-D node).
  Ngoài ra có các tiện ích: back-project 3D, phát hiện vùng đi được,
  lưới vật cản (obstacle grid), nhận diện vật thể.

CÀI ĐẶT (Jetson)
  bash depth-anything/install_jetson.sh
  # hoặc: pip install -r depth-anything/requirements.txt
  # tải checkpoint .pth (vits/vitb/vitl) vào depth-anything/model/

CHẠY PIPELINE CỤC BỘ (có đo thời gian)
  cd depth-anything/src
  python run_local.py [options]
    --encoder     vits | vitb | vitl   (mặc định vitb)
    --input-size  518
    --assets      thư mục ảnh đầu vào (mặc định ../assets)
    --outdir      thư mục kết quả (mặc định ../output)
    --device      auto | cuda | cpu
    --half        dùng FP16 (chỉ CUDA, nhanh hơn)
    --grayscale   xuất ảnh xám

CÁC SCRIPT TIỆN ÍCH (trong src/)
  depth_to_3d.py            ảnh → độ sâu → point cloud 3D
  depth_to_3d_timed.py      bản có đo thời gian từng bước (deepmap tái dùng)
  depth_to_3d_annotated.py  bản có chú thích trực quan
  drive_area.py             phát hiện vùng đi được (cũ, dễ lỗi)
  obstacle_grid.py          lưới vật cản BEV (height-above-ground) — THAY drive_area
  object_detect.py          phát hiện vật thể
  merge_360.py              helper hình học + IO (deepmap dùng để ghép)
  view_web.py               xem kết quả qua trình duyệt

MÔ HÌNH
  vits = Small (24.8M)  → nhanh nhất
  vitb = Base  (97.5M)  → cân bằng (mặc định)
  vitl = Large (335M)   → chính xác nhất, chậm

GHI CHÚ
  Mã gốc DA-V2 nằm trong src/depth_anything_v2/ — không sửa.
  Xem src/README.md để biết chi tiết upstream.
