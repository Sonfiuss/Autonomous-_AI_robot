# Session summary — 2026-06-28 — depth-based obstacle detection

## Tong quan
Phien lam viec tap trung vao module `depth-anything/` — loai bo YOLO, xay dung
obstacle detection thuan tuy tu depth map (Depth Anything V2).

---

## 1. Qua trinh thay doi

### Ban dau: YOLO + depth fusion
- Da tao `object_detect.py` dung YOLOv8n + depth median per bbox.
- Van de: YOLO COCO 80 class khong phu hop moi truong workshop (hop, day dien,
  cong cu khong co class tuong ung) -> bo qua phan lon vat the.
- Quyet dinh: loai bo YOLO hoan toan, dung depth-based obstacle.

### Python interpreter conflict
- `ultralytics` duoc cai trong Python 3.12, nhung DA v2 + torch chi co trong Python 3.10.
- Fix: luon dung `D:\University\Python\Python310\python.exe`.
- NumPy 2.2.6 khong tuong thich `torch 2.1.2+cpu` (compile voi NumPy 1.x).
- Fix: `pip install "numpy==1.26.4"` trong Python 3.10.

### Depth-based obstacle (hien tai)
**File**: `depth-anything/src/object_detect.py` — viet lai hoan toan.

**Thuat toan:**
1. DA v2 -> depth_norm (invert: 0=gan, 1=xa, normalize 0..1, Gaussian blur)
2. `build_floor_profile`: fit linear depth theo row tu bottom `--floor-rows`%
   -> noi suy len toan frame.
3. `detect_obstacles`: obstacle mask = pixel co depth < (floor_ref - gap_eff).
   `gap_eff` tu dong giam khi len phia tren anh (vung xa, depth contrast nho hon)
   qua param `--gap-scale`.
4. Morphological close -> `findContours` -> bbox.
5. Zone L/C/R, sort theo area.

**Params chinh:**
| Param | Default | Y nghia |
|-------|---------|---------|
| `--gap` | 0.03 | nguong depth gap co ban |
| `--gap-scale` | 0.4 | he so giam gap vung xa |
| `--min-area` | 300 | dien tich contour toi thieu (px^2) |
| `--morph` | 15 | kernel morphological close |
| `--ceil-cut` | 0.05 | cat 5% tren cung (troi/tuong xa) |
| `--floor-rows` | 0.25 | ti le dong duoi dung lam tham chieu san |

### Arm exclusion (Option C)
Camera nhin thay canh tay robot o bottom-center frame (right_3.jpg xac nhan).
Them logic loai tru vung arm:

**Params:**
| Param | Default | Y nghia |
|-------|---------|---------|
| `--arm-region` | `0.35,0.82,0.65,1.0` | vung arm (x1,y1,x2,y2 ti le) |
| `--arm-near` | 0.20 | pixel depth < gia tri nay trong vung arm bi bo qua |
| `--no-arm-mask` | flag | tat hoan toan arm exclusion |

Overlay hien thi hinh chu nhat tim nhan **ARM** de debug vi tri.

### drive_area.py — don gian hoa
- Xoa toan bo code YOLO (`load_yolo`, `run_yolo`, `overlay_detections`, `get_zone`).
- Xoa flag `--detect`, `--conf`, `--iou`.
- File giu nguyen chuc nang drive area / ray-linearity.

---

## 2. Output

### object_detect.py
- Anh 4-cot: `[goc | depth colormap | obstacle mask | overlay]`
- Saved: `depth-anything/output/object_detect/<stem>_det.jpg`

### drive_area.py
- Anh 3-cot: `[goc | depth colormap | overlay xanh]` (khong doi)
- Saved: `depth-anything/output/drive_area/<stem>_drive.jpg`

---

## 3. Cach chay

```bash
# Obstacle detection
D:\University\Python\Python310\python.exe src/object_detect.py --img assets/right_3.jpg

# Tat arm mask de so sanh
D:\University\Python\Python310\python.exe src/object_detect.py --img assets/right_3.jpg --no-arm-mask

# Drive area (khong thay doi)
D:\University\Python\Python310\python.exe src/drive_area.py --img assets/right_1.jpg
```

---

## 4. Pending / next
- Tune `--arm-region` chinh xac tren anh thuc te khi arm xuat hien.
- Ket hop output `object_detect.py` voi `drive_area.py` -> steering signal cho robot.
- Xem xet downgrade numpy: `pip install "numpy==1.26.4"` neu chua lam.

## 5. Interface changes
- `object_detect.py`: viet lai hoan toan, khong con YOLO, them arm exclusion
- `drive_area.py`: xoa YOLO code, don gian hoa
