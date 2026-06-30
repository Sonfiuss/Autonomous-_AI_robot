==============================================================
 area-detection — Phát hiện vùng đi được từ stereo vision (Python)
==============================================================

CHỨC NĂNG
  Từ cặp camera stereo, phát hiện vùng mặt đất KHÔNG có vật cản (đi được)
  phía trước robot và vẽ overlay màu xanh lá. Chỉ dùng OpenCV 4.2 + NumPy
  (không cần thư viện thêm trên Jetson).

PIPELINE
  1. Grab ảnh trái/phải (camera live hoặc cặp ảnh)
  2. Rectify  — ORB → fundamental matrix → stereoRectifyUncalibrated (rectify.py)
  3. Disparity — SGBM + WLS (stereo.py)
  4. Histogram U/V-disparity (disparity.py)
  5. Biên vùng trống — Viterbi decode + mặt đường Hough (boundary.py)
  6. Overlay xanh lên ảnh trái → captures/

CHẠY
  # Camera live (trái=/dev/video0, phải=/dev/video2) → captures/area.jpg
  python3 run.py
  python3 run.py --display      # + cửa sổ live (q để thoát)
  python3 run.py --once         # 1 khung live rồi thoát

  # Test offline trên ảnh mẫu KITTI (đã rectify sẵn)
  python3 run.py --left left.png --right right.png --no-rectify

FILE CHÍNH
  run.py             entry point
  rectify.py         căn chỉnh stereo không cần calib
  stereo.py          tính disparity SGBM + WLS
  disparity.py       histogram U/V-disparity
  boundary.py        decode biên vùng trống (Viterbi)
  capture_chessboard.py  chụp bàn cờ để hiệu chỉnh
  captures/          ảnh kết quả

GHI CHÚ
  Chi tiết đầy đủ (so sánh với upstream) xem README.md cùng thư mục.
  Refactor từ sajaysurya/drivable_area_detection (demo KITTI).
