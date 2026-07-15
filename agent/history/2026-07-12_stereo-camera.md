# 2026-07-12 — stereo-camera: accuracy toolchain (Stages A–D code-complete)

## Context
User gác plan free-move-merge (design lưu `agent/description/deepmap_free_move_merge.md`),
chuyển sang solution 2: tăng độ tin cậy stereo camera. Root cause xác định:
calibration (2 cặp chessboard @0.9m trên màn laptop, RMS 1.897px, distortion=0,
fx ±15%) chứ không phải thuật toán matching. Task: `agent/tasks/2026-07-11_stereo-accuracy.md`.

## Implemented (all offline-verified on synthetic ground truth)
- **`stereo-camera/tools/calib_io.py`** (NEW, shared): load calib cả 2 schema
  (K chung cũ / K_left+K_right mới), `fb_used` ưu tiên `fb_measured`;
  `find_board` = findChessboardCornersSB + fallback classic detector +
  cornerSubPix + canonical corner order. Lý do: SB một mình pixel-lock ±0.23px
  ở phase 1/4-pixel và flaky trên board to hơi blur.
- **`epipolar_check.py`** (NEW): đo |dy| p50/p90 của correspondence trên ảnh đã
  rectify (board mode ~0.1px / feature mode SIFT+F-RANSAC), kèm brightness
  delta + sharpness. Baseline calib hiện tại: **dy p50=1.55px p90=3.28px**
  (in-sample!) → golden_points tìm ±1px là trượt phần lớn match thật.
- **`depth_eval.py`** (NEW): Z vs thước dây theo session manifest; `--selftest`
  PASS (board 0.027px, roi 0.018px); `--fit-fb` + `--write-fb` (Stage C);
  `--sweep-golden` 32 config (Stage D1). Synth session: Z err ≤0.1%, fb
  recovered đúng 60.69.
- **`capture_eval.sh`** (NEW, Jetson): capture có tag + manifest.csv, fail thì
  không ghi file nào.
- **`make_checkerboard.py`** (NEW): PDF/PNG A4 9x6 inner 25mm + thanh thước
  100mm; detector nhận 18/18 tổ hợp scale×blur.
- **`stereo_calibrate_2view.py`** (REWRITE): printed-board path — per-camera K,
  k1/k2 free, pp free, per-view RMS rejection (≤2 vòng, giữ ≥12), stereoCalibrate
  FIX_INTRINSIC, alpha=-1, yml VERSIONED (từ chối ghi đè file active) +
  coverage heatmap. Synth test: fx sai 0.03%, baseline 0.05%, góc ±0.02°,
  rms 0.074px. Legacy screen path giữ sau `--legacy-screen`.
- **`sgbm_probe.py`** (NEW): SGBM trên rectify maps THẬT + fb thật (2 tool SGBM
  cũ chưa từng dùng calib), LR-check thủ công (không cần ximgproc), WLS tự bật
  nếu có contrib; so sánh cùng metric với golden points.
- **`lock_exposure.sh`** (NEW, Jetson): probe v4l2 khóa exposure/gain/WB, re-read
  verify.
- **`stereo_ruler.py`**: `load_rectify` đọc schema mới + ưu tiên `fb_measured`
  (fallback giữ nguyên); `golden_points` thêm param `dy_search`.
- **`capture_pair.sh`**: vá atomic — ghi .tmp, chỉ promote khi CẢ HAI camera OK.

## Phát hiện quan trọng
1. **left.jpg (27/06) + righ.jpg (05/07) cách nhau 8 ngày — KHÔNG phải cặp
   stereo.** Đây là cặp mặc định của stereo_ruler và mọi test offline gần đây
   (39 golden/3 inliers các ngày 09–10/07 tính trên cặp giả này). Nguyên nhân:
   capture_pair.sh ghi đè từng file, một lần fail camera trái để lại cặp lệch
   → đã vá. **Mọi kết luận từ cặp này cần đánh giá lại sau khi chụp cặp mới.**
2. Calib hiện tại lệch epipolar p90 3.3px ngay trên chính ảnh input của nó.
3. SB corner detector pixel-lock ±0.23px — mọi phép đo subpixel trong repo nên
   dùng calib_io.find_board.

## Pending (user, cần robot + thước dây)
1. In `tools/checkerboard/checkerboard_9x6_25mm.pdf` @100%, kiểm 4 ô = 100mm,
   dán bìa cứng phẳng.
2. Session eval: `capture_eval.sh d040 0.40` … 6 khoảng cách ×2 cặp (0.4/0.6/
   0.9/1.2/1.6/2.0m, đo từ mặt lens trái).
3. Session calib: 20–25 cặp `capture_eval.sh cb01` … (0.4–2m, nghiêng ±30°,
   phủ 4 góc khung hình cả 2 cam; xem coverage heatmap).
4. Copy 2 session sang PC → chạy calibrate → epipolar_check (pass p90<0.5px)
   → archive yml cũ + promote → depth_eval --fit-fb --write-fb (pass Zerr≤3%)
   → sweep-golden + sgbm_probe trên cảnh thật → replay walk shots (Stage E).

## Interface changes
- yml schema mới: K_left/K_right/dist_left/dist_right (+ giữ R1..P2, fb);
  key mới `fb_measured`/`fb_offset_px`. stereo_ruler + calib_io đọc được cả cũ/mới.
- `golden_points(..., dy_search=(0,-1,1))` — thêm param, default không đổi.
