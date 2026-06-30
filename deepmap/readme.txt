==============================================================
 deepmap — Bộ dựng bản đồ độ sâu 360° (Python)
==============================================================

CHỨC NĂNG
  Robot tự xoay 360° tại chỗ, chụp ảnh ở từng góc, chạy Depth Anything V2
  để lấy độ sâu mỗi khung hình, back-project ra điểm 3D metric, rồi ghép
  tất cả thành một point cloud 360° (file .ply).

  LƯU Ý: cách ghép-theo-góc này là LEGACY / chỉ dùng cho quét tại 1 điểm.
  Hướng mapping diện rộng đã chuyển sang slam/ (RTAB-Map SLAM).

CÁCH NHANH NHẤT — 1 LỆNH DUY NHẤT
  python deepmap/scan360.py
    → tự xoay + chụp + dựng map  (chế độ height)
    → deepmap/output/map_360.ply + map_360_bev.png

  Tùy chọn:
    python deepmap/scan360.py --step-deg 45 --hz 8000   # xoay nhanh, ít frame
    python deepmap/scan360.py --skip-spin               # dựng lại từ shots/ có sẵn
    python deepmap/scan360.py --skip-build              # chỉ xoay + chụp
    python deepmap/scan360.py --test-round360           # offline (không robot/camera)

CHẠY THỦ CÔNG TỪNG BƯỚC
  1) Xoay + chụp:
     python deepmap/rotate_scan.py [--step-deg 30] [--omega 0.5] [--cam 0]
     → deepmap/shots/shot_NNN.jpg + angles.csv

  2) Dựng map:
     python deepmap/build_map.py [--shots deepmap/shots] [--encoder vits] [--show]
     → deepmap/output/map_360.ply

TỰ HÀNH + DỰNG MAP (robot di chuyển, không chỉ xoay tại chỗ)
  Robot tự lái về VÙNG TRỐNG LỚN NHẤT phía trước, mỗi --step-cm (30cm) dừng lại
  chụp ảnh + depthmap, rồi GHÉP vào một point cloud gốc đang lớn dần.
     python deepmap/explore_map.py [--step-cm 30] [--max-steps 12] [--voxel 0.03]
     → deepmap/output/explore_map.ply (lưu tăng dần sau mỗi bước)
  Kiểm thử offline (không robot/không camera, ảnh cố định):
     python deepmap/explore_map.py --test-explore
  Chỉ chụp+map, KHÔNG điều khiển robot (test camera):
     python deepmap/explore_map.py --dry-run
  Lưu ý: chưa có encoder/IMU → pose suy từ lệnh đã gửi (dead-reckon), chạy dài bị
  trôi; --voxel bật ICP gộp bớt. Vòng lặp KHÔNG tiến nếu ô ngay trước chưa chắc trống.
  GỘP / KHÔNG BỊ "1 VẬT THÀNH 3 BẢN SAO": depth của Depth-Anything không metric →
  mỗi ảnh scale + độ nghiêng sàn khác nhau → cùng 1 vật bị đặt vào 3 chỗ. Sửa bằng
  NEO METRIC THEO MẶT SÀN (mặc định --metric-floor): mỗi frame fit mặt sàn, xoay cho
  sàn nằm ngang + scale sao cho camera đúng --camera-height (0.32m) trên sàn → mọi
  frame cùng tỉ lệ + cùng phương ngang → các lần chụp trùng khít. Tắt bằng
  --no-metric-floor (quay lại --tilt/--scale cố định).
  Căn chỉnh cuối: --merge icp (mặc định, cần open3d) — point-to-plane ICP tinh chỉnh
  sai số dead-reckon rồi gộp; chạy 1 lần ở bước cuối. --merge all = chỉ ghép thô
  (dùng --voxel để lọc); --merge nearest = z-buffer giữ điểm gần nhất mỗi hướng.
  Lọc trùng: --voxel 0.02 (numpy) áp dụng sau mọi mode.

HIỆU CHỈNH TỈ LỆ METRIC
  python deepmap/scale_calib.py    # ước lượng hệ số k (motion parallax)

FILE CHÍNH
  scan360.py       lệnh tất-cả-trong-một (gọi rotate_scan + build_map)
  explore_map.py   tự hành: lái về vùng trống lớn nhất + map tăng dần
  rotate_scan.py   điều khiển xoay UART + chụp ảnh
  build_map.py     pipeline độ sâu + ghép 360°
  scale_calib.py   hiệu chỉnh tỉ lệ metric
  shots/           ảnh chụp runtime + angles.csv
  output/          map_360.ply kết quả
  TIMING_REFERENCE.md  số liệu thời gian tham khảo

PHỤ THUỘC
  pyserial (UART), torch + Depth Anything V2 (dùng chung depth-anything/src),
  open3d (tùy chọn, cho ICP/hiển thị).
