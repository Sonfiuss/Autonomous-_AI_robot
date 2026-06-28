# Kế hoạch review source code Stereo Depth Pipeline

> Mục tiêu: nâng cấp pipeline stereo depth estimation, ưu tiên giải quyết vấn đề từ gốc (calibration) đến hiệu chỉnh ngoại vi (post-processing). Chấp nhận sai số ở vùng tối, không cố gắng đạt depth chính xác tuyệt đối.

---

## Tổng quan vấn đề hiện tại

| Vấn đề | Mức độ ảnh hưởng | Phase xử lý |
|--------|------------------|-------------|
| Dùng ORB+RANSAC affine thay cho stereo calibration | Rất cao | Phase 1 |
| `shift_y = 88.5px` — epipolar lines không nằm ngang | Rất cao | Phase 1 |
| Exposure/chất lượng 2 ảnh khác nhau | Cao | Phase 2 |
| Tham số stereo matching mặc định | Trung bình | Phase 3 |
| Không có post-processing | Trung bình | Phase 4 |
| Không có confidence map / validation | Thấp-Trung bình | Phase 5 |

---

## Phase 1 — Calibration (ƯU TIÊN CAO NHẤT)

> Quyết định ~80% chất lượng depth map. Phải làm trước, mọi nâng cấp khác phụ thuộc vào phase này.

### Điểm cần check trong code hiện tại

- [ ] Đang dùng affine 2x3 từ ORB+RANSAC để align ảnh? → đây là nguyên nhân gốc rễ
- [ ] Có file calibration thực sự (camera matrix K1, K2, distortion D1, D2, R, T) không?
- [ ] Có gọi `cv2.stereoRectify()` và `cv2.initUndistortRectifyMap()` không?
- [ ] Maps rectification có được cache (tính 1 lần, dùng lại nhiều frame) không?

### Đề xuất nâng cấp

1. Chụp **15-20 ảnh checkerboard** (9x6 hoặc 8x6) ở các góc/khoảng cách khác nhau
2. Chạy `cv2.calibrateCamera()` cho từng camera → lấy K1, D1, K2, D2
3. Chạy `cv2.stereoCalibrate()` → lấy R, T giữa 2 camera
4. Chạy `cv2.stereoRectify()` → lấy R1, R2, P1, P2, Q
5. Lưu maps: `map1x, map1y, map2x, map2y` từ `cv2.initUndistortRectifyMap()`
6. Mỗi frame: `cv2.remap(left, map1x, map1y, ...)` thay cho affine transform

### Tiêu chí pass

- Sau rectification, **vertical disparity giữa các điểm tương ứng < 1 pixel**
- Reprojection error từ `stereoCalibrate` < 0.5 pixel
- Ma trận Q dùng được để reproject thành point cloud 3D (`cv2.reprojectImageTo3D`)

---

## Phase 2 — Image Preprocessing

### Điểm cần check trong code hiện tại

- [ ] Có cân bằng exposure / histogram giữa left và right không?
- [ ] Có convert sang grayscale trước matching không?
- [ ] Có denoise / CLAHE không?
- [ ] Preprocessing áp dụng **giống hệt nhau** cho 2 ảnh?

### Đề xuất nâng cấp — pipeline chuẩn

```
Raw → Rectify → Histogram match (R sang L) → Grayscale →
Bilateral filter → CLAHE → (optional) Unsharp mask
```

### Code reference

```python
import cv2
import numpy as np
from skimage.exposure import match_histograms

def preprocess_for_stereo(img):
    if len(img.shape) == 3:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        gray = img.copy()

    # Denoise giữ edges
    denoised = cv2.bilateralFilter(gray, d=5, sigmaColor=25, sigmaSpace=25)

    # CLAHE - tăng contrast vùng tối
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    equalized = clahe.apply(denoised)

    # Gamma correction - kéo sáng vùng tối
    gamma = 0.7
    lut = np.array([((i / 255.0) ** gamma) * 255
                    for i in range(256)]).astype(np.uint8)
    brightened = cv2.LUT(equalized, lut)

    # Unsharp mask - tăng edges
    blurred = cv2.GaussianBlur(brightened, (0, 0), 2.0)
    sharpened = cv2.addWeighted(brightened, 1.5, blurred, -0.5, 0)

    return sharpened

# Cân bằng histogram trước
right_matched = match_histograms(right_img, left_img, channel_axis=-1)

left_p = preprocess_for_stereo(left_img)
right_p = preprocess_for_stereo(right_matched)
```

### Tiêu chí pass

- Histogram của left và right gần như trùng nhau
- Edge map (Canny) của 2 ảnh có mật độ edge tương đương
- Không có pixel saturated (= 0 hoặc = 255) ở vùng cần matching

---

## Phase 3 — Stereo Matching

### Điểm cần check trong code hiện tại

- [ ] Đang dùng `StereoBM` hay `StereoSGBM`? → SGBM tốt hơn nhiều cho cảnh thực tế
- [ ] Các tham số có được tune cho scene không, hay chỉ dùng default?
- [ ] `numDisparities` có phù hợp với baseline và khoảng cách object không?

### Đề xuất tham số (điểm khởi đầu)

```python
window_size = 5   # hoặc 7
min_disp = 0
num_disp = 128    # phải chia hết cho 16
                  # object gần hơn → cần num_disp lớn hơn

stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=window_size,
    P1=8 * 3 * window_size**2,
    P2=32 * 3 * window_size**2,
    disp12MaxDiff=1,        # left-right consistency check
    uniquenessRatio=10,     # 5-15
    speckleWindowSize=100,  # lọc noise nhỏ
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
```

### Tiêu chí pass

- Disparity map vùng có texture (sàn, tường có đốm) liên tục và mượt
- Vùng object gần có disparity cao, xa có disparity thấp (gradient đúng hướng)
- Tỷ lệ invalid pixels (= -1 hoặc 0) < 30% tổng diện tích

---

## Phase 4 — Post-processing

### Điểm cần check trong code hiện tại

- [ ] Có dùng WLS filter không?
- [ ] Có xử lý vùng invalid không?
- [ ] Có handle vùng tối / textureless riêng không?

### Nâng cấp 3 layer

#### Layer A — WLS filter (mặc định bật)

```python
left_matcher = cv2.StereoSGBM_create(...)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
wls.setLambda(8000)
wls.setSigmaColor(1.5)

left_disp = left_matcher.compute(left_p, right_p)
right_disp = right_matcher.compute(right_p, left_p)
filtered_disp = wls.filter(left_disp, left_p, disparity_map_right=right_disp)
```

#### Layer B — Dark region fill (chấp nhận sai số)

```python
def fix_dark_region_depth(left_img, disparity_map, dark_threshold=40):
    gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY) \
           if len(left_img.shape) == 3 else left_img

    _, dark_mask = cv2.threshold(gray, dark_threshold, 255, cv2.THRESH_BINARY_INV)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)
    dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_OPEN, kernel)

    num_labels, labels = cv2.connectedComponents(dark_mask)
    result_disp = disparity_map.copy().astype(np.float32)

    for label_id in range(1, num_labels):
        region_mask = (labels == label_id).astype(np.uint8)
        if region_mask.sum() < 500:
            continue

        # Lấy border xung quanh vùng
        dilated = cv2.dilate(region_mask, kernel, iterations=2)
        border = dilated - region_mask

        border_coords = np.where(border > 0)
        border_disps = disparity_map[border_coords]
        valid = border_disps > 0

        if valid.sum() < 20:
            continue

        valid_coords_y = border_coords[0][valid]
        valid_coords_x = border_coords[1][valid]
        valid_disps = border_disps[valid]

        # Fit mặt phẳng disparity = a*x + b*y + c
        A = np.column_stack([valid_coords_x, valid_coords_y,
                             np.ones(len(valid_disps))])
        coeffs, _, _, _ = np.linalg.lstsq(A, valid_disps, rcond=None)
        a, b, c = coeffs

        region_coords = np.where(region_mask > 0)
        predicted_disp = a * region_coords[1] + b * region_coords[0] + c
        result_disp[region_coords] = predicted_disp

    return result_disp, dark_mask
```

#### Layer C — Speckle removal & median filter

```python
cv2.filterSpeckles(disp, newVal=0, maxSpeckleSize=400, maxDiff=32)
disp_smooth = cv2.medianBlur(disp.astype(np.float32), 5)
```

### Tiêu chí pass

- Vùng tối được fill liên tục (chấp nhận sai số như đã đồng ý)
- Không còn speckle noise (chấm nhỏ disparity sai lệch)
- Border vật thể vẫn giữ được sharp (không bị blur quá)

---

## Phase 5 — Validation & Monitoring

### Cần thêm vào code

#### Confidence map — biết depth nào tin được

```python
# WLS có sẵn confidence map
confidence = wls.getConfidenceMap()
# Hoặc tự tính: vùng có texture cao + uniqueness cao = confidence cao
```

#### Logging metrics mỗi frame

- % invalid pixels
- Mean / std disparity
- Số connected dark regions được fill
- Thời gian xử lý từng stage

#### Visualization debug

- Side-by-side: rectified left | rectified right | disparity | confidence
- Overlay dark mask lên disparity để verify

---

## Thứ tự review code đề xuất

| Bước | File / function cần xem | Quyết định |
|------|-------------------------|------------|
| 1 | Code load / align ảnh (ORB+RANSAC) | **Thay** bằng stereo rectification |
| 2 | Code preprocessing | Bổ sung CLAHE + histogram match |
| 3 | Code stereo matching | Đổi sang SGBM với params tuned |
| 4 | Code post-processing | Thêm WLS + dark region fill |
| 5 | Code output / usage | Thêm confidence map cho downstream |

---

## Checklist trước khi bắt đầu review

- [ ] Share được code stereo pipeline hiện tại (file chính + cách gọi)
- [ ] Đã có sẵn ảnh checkerboard chưa, hay cần plan cả việc chụp calibration
- [ ] Output cuối cùng là gì: disparity map / depth map (mét) / point cloud 3D
- [ ] Mục đích sử dụng: navigation, scene reconstruction, đo đạc, hay khác
- [ ] Hardware constraint: real-time hay offline, GPU available không

---

## Ghi chú quan trọng

1. **Không bỏ qua Phase 1.** Mọi nỗ lực ở Phase 2-5 sẽ không hiệu quả nếu calibration sai. ORB+RANSAC affine không thể thay thế stereo rectification chuẩn.
2. **Chấp nhận giới hạn vật lý.** Vùng tối hoàn toàn không có texture sẽ không bao giờ có depth chính xác từ passive stereo, dù preprocessing tốt đến đâu. Plan này chấp nhận và xử lý giả định đó.
3. **Tune theo scene.** Các tham số SGBM cần điều chỉnh theo từng scene cụ thể. Số trong plan là điểm khởi đầu, không phải giá trị cuối cùng.
4. **Đo lường trước khi tối ưu.** Phase 5 (validation) nên triển khai sớm để có metric so sánh trước/sau mỗi nâng cấp.

---

## Kết quả review & áp dụng (2026-06-26) — engine thực tế là AD-Census, KHÔNG phải SGBM

> Pipeline dùng `stereo-camera-AD-Census/adcensus_depth.exe` (đọc ảnh COLOR,
> Census illumination-invariant + AD color). AD-Census thường TỐT HƠN SGBM →
> một số phase của plan không áp dụng / sẽ là bước lùi.

| Phase | Quyết định | Lý do |
|-------|-----------|-------|
| 1 — Calibration | **Hoãn** | Đòn bẩy lớn nhất nhưng cần 15-20 cặp checkerboard chưa có. `_room_align.yml` (ORB+RANSAC affine) vẫn là giới hạn gốc. |
| 2 — Preprocessing | **ĐÃ ÁP DỤNG** | hist-match R→L (CDF tự cài, không cần skimage) + CLAHE trên kênh L (giữ màu cho AD). Bỏ gamma/unsharp/bilateral (Census đã illumination-robust; unsharp tạo edge giả). Cờ `--no-preprocess --clahe-clip`. |
| 3 — SGBM | **Bỏ** | Đổi sang SGBM = bước lùi so với AD-Census. |
| 4A — WLS filter | **Bỏ** | WLS cần left+right matcher của SGBM; exe chỉ xuất left disparity → không chạy được nếu không sửa C++/chạy exe 2 lần. |
| 4B — Plane-fit dark fill | Chưa làm (user chọn bỏ qua đợt này) | Tốt hơn inpaint cho lỗ phẳng; để dành. |
| 4C — Speckle + median | **ĐÃ CÓ** | `denoise_disparity()` (filterSpeckles + median). |
| 5 — Validation/metrics | Chưa làm (user bỏ qua) | Có thể thêm %invalid, mean/std, timing sau. |

Phụ: thêm `OPENBLAS_NUM_THREADS=1`/`OMP_NUM_THREADS=2` vào env của exe để né
"OpenBLAS Memory allocation failed" trên máy ít RAM (<1.5GB free).