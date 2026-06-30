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

HIỆU CHỈNH TỈ LỆ METRIC
  python deepmap/scale_calib.py    # ước lượng hệ số k (motion parallax)

FILE CHÍNH
  scan360.py       lệnh tất-cả-trong-một (gọi rotate_scan + build_map)
  rotate_scan.py   điều khiển xoay UART + chụp ảnh
  build_map.py     pipeline độ sâu + ghép 360°
  scale_calib.py   hiệu chỉnh tỉ lệ metric
  shots/           ảnh chụp runtime + angles.csv
  output/          map_360.ply kết quả
  TIMING_REFERENCE.md  số liệu thời gian tham khảo

PHỤ THUỘC
  pyserial (UART), torch + Depth Anything V2 (dùng chung depth-anything/src),
  open3d (tùy chọn, cho ICP/hiển thị).
