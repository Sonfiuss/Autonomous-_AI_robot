# Task: Height-above-ground obstacle + drivable-area detection

- **Module:** depth-anything (drive-area / obstacle detection)
- **Input:** RGB ảnh đơn (hoặc shot_NNN.jpg từ deepmap). Depth-Anything v2.
- **Expected output:** mask vật cản dày đặc per-pixel + lưới occupancy bird's-eye
  (free / blocked / unknown) + overlay trực quan. Ngưỡng vật cản theo **độ cao (cm)**.
- **Status:** planning

## Bối cảnh / vì sao
`drive_area.py` dùng ray-linearity: giả định depth tuyến tính dọc ray = sàn. Sai về
hình học (depth-chuẩn-hoá trên sàn phẳng ~hyperbol, không tuyến tính), bỏ sót vật
thấp/mảnh, và polygon hình-sao không khoét được vật ở giữa sàn. Tăng r2-min chỉ sinh
false-positive, không bắt thêm vật. → Chuyển sang ngưỡng theo độ cao vật lý.

Tái dùng `depth_to_3d.py`: `depth_to_points` (back-project, Y=up, tilt/camera-height),
`fit_floor_plane` (SVD plane fit), `pick_device`, CFGS.

## Plan
1. Tạo `depth-anything/src/obstacle_grid.py`:
   - infer depth → `depth_to_points()` ra point cloud 3D (camera/world frame).
   - `fit_floor_plane()` trên dải đáy ảnh (bottom-frac) → (normal, centroid).
   - Tính **signed height** mỗi point so với mặt phẳng: `h = (p - centroid)·normal`.
   - Phân loại pixel: floor `|h|<h-floor`, obstacle `h>h-obs`, còn lại unknown.
2. Per-pixel obstacle mask + overlay (xanh=free/sàn, đỏ=vật cản, xám=unknown).
3. Bird's-eye occupancy grid:
   - Bin (X,Z) của point vào cell `cell-size` trên phạm vi `grid-range`.
   - Cell = BLOCKED nếu có ≥k obstacle point; FREE nếu có floor point và không blocked;
     else UNKNOWN.
   - Flood-fill reachability từ ô robot (đáy-giữa) → vùng drivable thật sự (loại ô free
     bị vật cản chắn).
4. Scale: arg `--scale` (từ deepmap/scale_calib) để h-obs/cell tính bằng mét; không có
   scale thì chạy đơn vị tương đối + in cảnh báo.
5. Visualize: hstack [overlay | depth colormap | occupancy grid]. Lưu + optional show.
6. CLI: `--h-obs --h-floor --cell-size --grid-range --scale --bottom-frac --min-pts-cell`.

## Execution log
| Time | Step | Summary |
|------|------|---------|
| 06-29 | 1-2 | Tạo obstacle_grid.py: tái dùng depth_to_points + fit_floor_plane; signed-height + classify(floor/obstacle/unknown); overlay + banner L/C/R |
| 06-29 | 3 | Occupancy grid bird's-eye + flood-fill reachability từ ô robot |
| 06-29 | fix | Sàn xa bị tô đỏ giả (depth phi-metric → sàn tái dựng cong). info: thay "1 mặt phẳng + ngưỡng tuyệt đối" bằng ground-profile g(Z) per-range (percentile thấp), classify theo residual = height − g(Z). blocked 677→260 |
| 06-29 | fix | 0 ô đi được: hàng 0-1 sát robot là vùng mù dưới camera (tilt) → seed flood-fill từ hàng gần nhất CÓ ô free. info: right_1 → 636 drivable / 260 blocked, cone BEV đúng |
| 06-29 | test | Validate right_1 (sàn sạch → cone đi-được rõ) + right_3 (bừa → blocked, thận trọng hợp lý) |

## Status
redesign-executing

## Execution log (v2)
| Time | Step | Summary |
|------|------|---------|
| 06-29 | build | Wired height-above-ground into deepmap/build_map.py as `--classify height`: forces metric project (Z=k/disp, k=3.0034 from slam/config/scale.yaml) + per-frame floor-plane SVD fit (bottom-frac seed, 1 inlier re-fit) + signed-height classify. Added top-down BEV PNG render (map_360_bev.png). |
| 06-29 | run | Built 360° map from 12 restored shots: 1,023,840 pts. BEV = coherent green drivable disc around robot + red obstacle wedges at periphery. Floor geometry now consistent (vs broken obstacle_grid). |
| 06-29 | confirm | Red wedges showed 12-fold radial symmetry (1/frame, far range) → far-floor false positives, NOT real walls. `--h-obs 0.15 --bev-range 2.0` removes them (only far-diagonal specks remain) → confirms far-range plane-fit drift. Quick tune works but sacrifices sensitivity+range. info: next = per-range ground profile to keep low h-obs + full range. |
| 06-29 | live-run | Ran rotate_scan.py on real hardware: robot self-rotated 360° (12×30°, ~6.96s/step) + captured 12 fresh frames → deepmap/shots/. Then build_map --classify height: 1.02M pts, BEV = ~1m green drivable core + heavier red ring (591k floor/402k obstacle). Real enclosed room → more genuine near obstacles; far radial streaks still = floor drift. StepperClient "Timeout waiting for READY" per step is benign (reconnects each step, moves executed). |

## REDESIGN v2 — root cause + new plan (2026-06-29)

### Root cause: warped 3D from wrong depth conversion
`obstacle_grid.py` back-projects via `depth_to_3d.depth_to_points`, which converts
the model output as `d = (max - disp); d = d/max*5 + 0.5` — a **linear remap of an
inverse-depth signal**. But Depth-Anything v2 returns RELATIVE INVERSE depth
`disp = k/Z` (documented in `deepmap/scale_calib.py`). Correct metric is `Z = k/disp`,
NOT `max - disp`. Consequence: a flat floor does NOT reconstruct flat — it curves —
so every height-above-plane threshold fights the warp. `ground_profile_residual`
(per-Z-bin percentile) is a band-aid over this broken geometry → mediocre accuracy.

The old `drive_area.py` accidentally avoided this: it works in IMAGE space on the raw
depth profile (detects where floor depth-trend breaks), so it never depends on a metric
reconstruction. That's why it felt more accurate despite being "fragile."

### Fix strategy: correct the geometry, then height-above-plane works as designed
1. Replace the fake conversion with the project's own correct one: reuse
   `scale_calib.apply_scale` (`Z = k/disp`, clamped to D_max) + `scale_calib.project_metric`
   for back-projection. Geometry becomes correct **up to a global scale** even with k=1
   (unknown k is just a uniform scale → plane fit + relative thresholds are invariant).
   Load k from `slam/config/scale.yaml` if present; else k=1 with a "relative scale" warn.
2. With a genuinely planar floor, drop the `ground_profile_residual` band-aid. Use a clean
   **RANSAC plane fit** on the bottom-band floor seed → signed height-above-plane per point.
   (Keep the robust re-fit on inliers already in `fit_plane_robust`.)
3. Add a second, image-space cross-check for robustness on thin/low objects (the old
   method's real strength): a **ground-profile-in-row-space** test — expected floor depth
   as a function of image row v; pixels markedly NEARER than the row's floor depth are
   obstacles. Combine: obstacle = (height > h_obs) OR (depth-nearer-than-floor-row by margin).
   This catches vertical faces/thin posts that a pure height test grazes, with no metric scale.
4. Keep dense per-pixel mask + overlay + L/C/R banner + BEV occupancy + flood-fill drivable
   (these parts of the new method are good and stay). BEV X/Z now from corrected geometry.
5. Validate: run BOTH detectors on assets right_1..right_5 + test.jpg, compare overlays
   side by side; confirm v2 catches what old caught (and the interior obstacles old missed)
   without the far-floor false-positives. Tune `--h-obs`, deviation margin on real images.

### Open question for user (before coding)
- Is there a calibrated `k` available (`slam/config/scale.yaml` or a deepmap calib), or
  should v2 run scale-free (k=1, thresholds in relative units) for now?

## Old test result (v1, superseded)
Chạy: `cd depth-anything/src && python3 obstacle_grid.py --img ../assets/right_1.jpg --show`
- right_1: 636 ô đi được, 260 vật cản; overlay sàn xanh + banner L:BLOCKED C/R:CLEAR; BEV cone xanh đúng.
- Chỉ có checkpoint vits (default). Tuning: --h-obs (hạ để bắt vật thấp / nâng để bỏ qua cáp mảnh),
  --scale (từ deepmap/scale_calib để ngưỡng ra cm thật), --bottom-frac, --cell-size, --grid-range-z.
