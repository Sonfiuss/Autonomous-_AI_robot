---
id: 2026-07-13_stereo-cloud-live
status: testing
module: stereo-camera
started: 2026-07-13
---

## Task
Live stereo pair + Depth Anything V2 → metric point cloud; dùng NHIỀU điểm đo
stereo (golden points) làm mốc, nội suy cục bộ để hiệu chỉnh độ sâu cho mọi
pixel còn lại (không chỉ 1 fit affine toàn cục).

## Input
- `stereo-camera/calib/stereo_rectify.yml` (1280×720, fx=1123.89, B=5.4cm,
  fb=60.69, rms 1.9px) — đã calib từ chessboard.
- Camera KHÔNG còn cúi xuống (nhìn ngang) → bỏ đường floor-anchor/cam_h/tilt
  của stereo_ruler.
- Capture trực tiếp từ cổng camera: left=/dev/video0, right=/dev/video2.
- Tái dùng thư viện có sẵn: `stereo_ruler.py` (capture_pair, load_rectify,
  stereo_half, depth_half, fit_affine_ransac), back-projection kiểu
  `stereo_walk_map.depth_to_cloud`.

## Expected output
Script `depth-anything/src/stereo_cloud.py`:
`python3 stereo_cloud.py --camera` → capture pair 1280×720, chạy DA-V2,
fuse + nội suy residual cục bộ, xuất:
- `output/stereo_cloud/cloud.ply` (point cloud màu, đơn vị mét)
- `golden_overlay.jpg` (điểm đo stereo + khoảng cách mét)
- `depth_metric_m.npy` + `depth_vis.jpg`
- log so sánh: rms residual tại golden points TRƯỚC (affine toàn cục) vs
  SAU (nội suy cục bộ) — sau phải nhỏ hơn.
Kiểm tra thực tế: đo tay 1–2 vật, so với giá trị trên overlay.

## Plan
- [ ] 1. Viết `stereo_cloud.py`: khung CLI + capture live 1280×720 (reuse
      `capture_pair`, size lấy từ calib, fail nếu lệch); `--left/--right`
      cho chế độ offline từ ảnh đã lưu.
- [ ] 2. Stereo dày hơn: gọi `stereo_half` với grid mịn (24×14, per_cell 4,
      d_max 140) để có nhiều mốc đo trải khắp khung hình.
- [ ] 3. Fit toàn cục KHÔNG floor-anchor: `fit_affine_ransac` trên
      (disp, d_mono) golden; giữ gate span-ratio + trust-horizon (z_trust)
      từ fuse_metric nhưng bỏ nhánh cam_h/tilt.
- [ ] 4. Nội suy cục bộ (điểm mới chính): residual Δdisp = disp_stereo −
      disp_fit tại từng inlier → nội suy trường Δ(u,v) bằng
      scipy.interpolate.griddata (linear trong hull, suy giảm→0 ngoài hull,
      lọc inlier |Δ| < 3σ trước) → disp_dense_corrected = disp_fit + Δ →
      Z = fb/disp. In rms@golden trước/sau.
- [ ] 5. Back-project P2 (fx,fy,cx,cy) → cloud XYZ (KHÔNG màu — user yêu cầu
      2026-07-14), mask valid & z_trust, subsample --stride; PLY binary
      little-endian (nhanh) + depth_vis + overlay.
      Toàn pipeline GRAYSCALE: rectify 1 kênh, golden trên gray, DA-V2 nhận
      gray→3ch; tối ưu tốc độ theo yêu cầu user.
- [ ] 6. Test offline với pair đã lưu trong stereo-camera/tools/captures →
      xem outputs hợp lý; sau đó user chạy `--camera` live để xác nhận.

## Execution log
<!-- One line per event. Format: [HH:MM] step N: <action> | risk: <note> | info: <key fact> -->
[07-14] step 1-2: stereo_cloud.py viết xong, grayscale end-to-end | info: scipy 1.3.3 hỏng với numpy mới (np.int) → nội suy dùng matplotlib.tri thay griddata
[07-14] step 3-4: residual chuyển sang LOG-RATIO (nhân) — cộng px bùng nổ ở disp nhỏ; field từ TẤT CẢ điểm neighbor-consistent (k=5, gate log1.35), không chỉ inlier RANSAC | info: golden points là góc → nằm trên biên độ sâu
[07-14] step 3: thêm --z-max 8m (loại match rác disp nhỏ) + lọc điểm dính viền đen rectify (baseline rig nghiêng 27° → ảnh xoay, viền đen 2 view lệch nhau)
[07-14] step 6: FAIL trên pair lưu sẵn → phát hiện left.jpg/righ.jpg chụp CÁCH NHAU 22 PHÚT (không đồng bộ, dy thô 247px) — không phải lỗi pipeline | risk: các test offline cũ dựa pair này đều đáng ngờ
[07-14] step 6: pair LIVE (video0+2): dy median 0px sau rectify → calib hợp lệ; holdout 63.6%→15.6%, |Z err| golden mean 4.4cm; thêm near-side trust; PLY 88k điểm Z 0.30-1.30m mở được bằng open3d
[07-14] step 6b: frame reject giờ in rõ + vẫn lưu overlay | info: gate span đổi từ inlier-RANSAC sang điểm evidence của field (RANSAC nhảy cluster giữa các lần chụp → x7.4/x1.35 trên cùng cảnh); min-span-ratio hạ 2.0→1.5 (trust horizon đã chặn ngoại suy 2 đầu)
[07-14] step 6c: PASS live cảnh ghế chắn: 118/134 field points, holdout 18.4%→9.0%, |Z err| mean 1.8cm, cloud 184k điểm | risk: mở camera lần đầu mất ~19s (USB enumerate) — walk loop nên dùng PersistentStereoCam

[07-14] test: user báo overlay sai scale (~15cm baseline thay vì 5.4cm). Điều tra: output 00:50 là STALE — sinh bởi bản stereo_cloud.py trung gian trong lúc đang sửa (00:22–00:50), toàn bộ depth ×~2.8. Detection KHÔNG sai: golden trên cùng pair cho tường disp≈80px → 0.75m, xác nhận độc lập bằng ORB + NCC wide-range (0.73–0.76m, ncc 0.92–0.96). Chạy lại code hiện tại trên cùng pair → tường 0.72–0.79m, calib fb=60.69 (B=5.4cm) hợp lệ. Output dir đã được regenerate. | risk: git repo có loose object corrupt (git log fail) — bản code cũ không khôi phục được

[07-14] test 2: user đo tay tường = 2.0–2.2m (KHÔNG phải 0.75m) → phân tích 6 pair bàn cờ known-D trong stereo-camera/captures (left/right_81..180 = cm): fit disp = fB/D + offset cho fB≈67 (calib 60.69 ≈ ĐÚNG) + OFFSET disparity ≈ +44px (±8). Kết luận: fx=1124 đúng (ô bàn cờ suy ra ~2.3cm nhất quán 3 cự ly), B=5.4cm đúng, nhưng YAW giữa 2 camera đã lệch ~2.2° so với calib 02/07 → mọi disp bị cộng ~44-52px → Z=fb/disp sai PHI TUYẾN (tường 80px: 60.69/80=0.76m thay vì 60.69/(80-50)≈2.0m). "Baseline 15cm" chỉ là ảo giác của offset tại dải ~2m. | risk: offset có thể trôi nếu mount camera lỏng

[07-15] fix: stereo_cloud.py thêm --disp-offset (default 51px, đo 14/07) trừ vào mọi golden disp trước fit; --mono-gate 2.0 — DA-V2 làm bộ lọc nhiễu, loại điểm stereo lệch >2x so với mono-fit (hết nhãn rác 12m); cloud.ply đổi sang đơn vị MM (depth_metric_m.npy vẫn mét). Test offline pair tường: tường 1.7-2.3m ✓ khớp đo tay, sàn 0.7-1.5m đúng phối cảnh, PLY 156k điểm Z 461-4291mm. Kiến trúc giữ nguyên: hình học cloud từ mono, stereo chỉ chuẩn hoá scale.

[07-15] fix 2 (cloud "quá rộng"): nguyên nhân là rèm smear ở biên vật + mono ngoại suy sau tường (11% điểm nằm sau tường 2.2m, Z tới 4.3m) chứ KHÔNG phải sai tỉ lệ — lõi tường 2.0-2.3m đúng. Sửa: (1) far horizon dùng p5 disparity thay min (1 điểm golden lạc không kéo horizon), trust-beyond 1.5→1.2, near giữ nguyên 1.5×max; (2) lọc edge-smear bằng gradient log-depth >8%/px (Sobel + dilate). Kết quả: Z max 4291→2905mm, X ±1089→±850mm, điểm sau 2.5m: 10.9%→6.3%, cloud 144k điểm.

[07-15] fix 3: "không zoom out được" = view_web.py hardcode scale mét (camera z=6, far=1000 → cloud mm bán kính ~2000 bị cắt + camera nằm trong cloud). Viewer giờ auto-fit theo boundingSphere: camera/near/far/minmax-zoom/cỡ điểm/trục đều theo bán kính cloud — xem được mọi đơn vị. PLY giữ nguyên mm.

## Test result
<!-- Filled when user confirms. Pass / Fail / Partial + brief note -->
FAIL (calibration drift, không phải lỗi code) — rectify map hiện tại lệch yaw ~2.2° (disparity offset ~+44..52px). Cần: (1) đo cạnh ô bàn cờ thật (dự đoán ~2.3cm) để chốt fx, (2) RECALIBRATE stereo bằng bàn cờ in thật (nhiều tư thế) với --baseline 0.054, hoặc quick-fix trừ offset ước lượng từ các pair known-D.
