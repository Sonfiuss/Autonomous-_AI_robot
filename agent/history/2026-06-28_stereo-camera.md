# Session summary — 2026-06-28 — depth-anything

## Tổng quan
Phiên làm việc tập trung vào module `depth-anything/` — cải thiện point cloud 3D và xây dựng
drive area detection từ ảnh đơn dùng Depth Anything V2 vitb.

---

## 1. depth_to_3d.py — các thay đổi tích lũy

### Model default đổi sang vitb
`--encoder` default: `vits` → `vitb` (user đã chuyển sang dùng `depth_anything_v2_vitb.pth`).

### Thêm --camera-height (32 cm)
Camera đặt cao 32 cm so với sàn. Thêm `--camera-height 0.32` (default), cộng vào `pts[:, 1]`
sau unproject để đặt gốc tọa độ tại mặt sàn (Y=0).

### Thêm --tilt (15°) + rotation matrix
Camera nghiêng xuống 15°. Áp dụng rotation +15° quanh trục X để chuyển camera frame → world frame:
```
Yw =  Yc·cos(t) + Zc·sin(t)
Zw = -Yc·sin(t) + Zc·cos(t)
```
Trước khi có rotation, point cloud nằm nghiêng theo trục camera.

### Floor plane fitting (--no-flatten để tắt)
Vấn đề: sàn trong PLY bị cong do DA output là relative depth (bowl effect), không phải metric.
Thêm `flatten_floor_simple()`: sample ngẫu nhiên 2000 điểm từ bottom `--floor-rows` % của ảnh,
fit plane bằng SVD (`full_matrices=False` để tránh OOM), xoay cloud bằng Rodrigues rotation
sao cho plane đó nằm ngang.

**Bug fix**: SVD ban đầu allocate ma trận N×N → OOM 24.7 GiB. Fix: random sample max 2000 điểm + `full_matrices=False`.

### Kết luận về bowl effect
Sàn vẫn còn cong nhẹ sau floor fitting — đây là giới hạn của monocular depth estimation.
Tuy nhiên không ảnh hưởng đến drive area detection vì logic đó hoạt động trên depth map 2D tương đối.

---

## 2. drive_area.py — file mới

**File**: `depth-anything/src/drive_area.py`

### Iteration 1: floor reference per column
Logic đơn giản: lấy median depth N dòng dưới làm tham chiếu, scan từng cột tìm obstacle.
Vấn đề: không fill đủ drive area.

### Iteration 2: ray-linearity từ bottom-center (góc)
Từ bottom-center bắn ray mọi hướng (mỗi 5°). Trên mỗi ray, depth tăng tuyến tính = sàn;
khi R² < threshold → boundary. Polygon nối boundary points = drive area.

### Iteration 3 (hiện tại): ray từ cạnh dưới → trung điểm cạnh trên
**Thay đổi theo yêu cầu user**: thay vì ray tỏa từ tâm, kẻ từ N điểm đều trên cạnh dưới
hướng tới trung điểm cạnh trên `(W//2, 0)`. Phù hợp hơn với vanishing point thực tế.

```python
# Vector mỗi ray: (bx, H-1) → (W//2, 0)
vx = top_cx - bx
vy = -(H - 1)
dx, dy = normalize(vx, vy)
```

**R² threshold**: user tune từ 0.80 → 0.92 → 0.99 → 0.999 để tránh drive area che object.

### Output
- Ảnh ghép 3 cột: `[gốc | depth colormap | overlay xanh]`
- Zone L/C/R với trạng thái CLEAR/BLOCKED
- Ray debug: xanh (sàn) / vàng (ranh giới) / đỏ (vật cản)
- Saved tại `depth-anything/output/drive_area/<stem>_drive.jpg`

### Params chính
| Param | Default | Ý nghĩa |
|-------|---------|---------|
| `--n-rays` | 40 | Số ray dọc cạnh dưới |
| `--ray-step` | 4 | Bước lấy mẫu (pixel) |
| `--min-lin` | 10 | Số điểm tối thiểu fit tuyến tính |
| `--r2-min` | 0.999 | Ngưỡng R² tuyến tính |
| `--blur` | 15 | Kernel Gaussian blur depth |

---

## Pending / next
- Tune `--r2-min` trên nhiều ảnh khác nhau để tìm giá trị stable.
- Có thể kết hợp drive area detection với free_space → steering signal cho robot.
- Nếu cần metric depth thật: stereo calibration → Q matrix → reprojectImageTo3D().

## Interface changes
- `depth_to_3d.py`: thêm `--encoder vitb` default, `--camera-height`, `--tilt`, `--floor-rows`, `--no-flatten`
- `drive_area.py`: file mới standalone, không phụ thuộc module khác
