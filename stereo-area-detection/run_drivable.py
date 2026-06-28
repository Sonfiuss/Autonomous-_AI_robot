#!/usr/bin/env python3
"""
run_drivable.py  –  AD-Census stereo depth (disparity + disp_color).

Pipeline:
  left.jpg ─────────────────────────────────────┐
  right.jpg → warpAffine(_room_align.yml, tx=0) ─┴► AD-Census .exe → disparity
                                                       │
                            tái tạo disparity (int) từ PNG đã chuẩn hoá
                                                       │
                  khử biên không hợp lệ (occlusion trái + viền warp) → blank đen
                                                       │
                                          <prefix>_disp_color.png

GHI CHÚ: Logic drivable-area (u/v-disparity + HoughLine + HMM Viterbi) đã được
GỠ BỎ trong phiên này theo yêu cầu — chỉ còn depth thuần (disparity/disp_color).

LƯU Ý stereo-quan-trọng:
  _room_align.yml gồm rotation + dịch dọc (ty) + dịch ngang (tx). Để stereo depth
  ĐÚNG ta chỉ khử rotation + ty (làm epipolar nằm ngang) và GIỮ tx (chính là
  tín hiệu độ sâu). Dùng --full-warp nếu muốn áp cả tx.

Khử boundary artifact (trái + đáy/đỉnh ảnh):
  - viền warp: warpAffine fill đen vùng trống → mask vùng có nội dung thật.
  - dải trái rộng maxd cột: không có pixel tương ứng bên phải → vô hiệu.
  Các pixel vô hiệu được đặt disparity=0 và hiển thị ĐEN trong disp_color.

Usage:
  python run_drivable.py                       # cặp mặc định trong captures/
  python run_drivable.py --left L.jpg --right R.jpg --out-prefix captures/run1
  python run_drivable.py --full-warp           # áp cả tx (thường SAI cho depth)
  python run_drivable.py --no-align            # bỏ qua bước align
"""

import argparse
import os
import re
import subprocess
import sys

import cv2
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
_ADCENSUS_DIR = os.path.join(_ROOT, "stereo-camera-AD-Census")
# Config alignment CHÍNH THỨC cho cặp camera này: đo trên cảnh phòng đại diện
# (350 inliers) — khử lệch dọc 88.5px + xoay 1.13°, giữ horizontal disparity.
# KHÔNG dùng alignment.yml từ bàn cờ (210px) — sai cho standardize stereo.
_DEFAULT_ALIGN = os.path.join(_HERE, "captures", "_room_align.yml")


# ─── args ────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Drivable-area từ AD-Census stereo")
    p.add_argument("--left",  default=os.path.join(_HERE, "captures", "left.jpg"))
    p.add_argument("--right", default=os.path.join(_HERE, "captures", "right.jpg"))
    p.add_argument("--out-prefix", default=os.path.join(_HERE, "captures", "drive"))
    p.add_argument("--align",  default=_DEFAULT_ALIGN,
                   help="File alignment.yml (warp right→left)")
    p.add_argument("--no-align", action="store_true",
                   help="Bỏ qua bước align (dùng right gốc)")
    p.add_argument("--full-warp", action="store_true",
                   help="Áp cả tx (dịch ngang) — thường làm hỏng depth stereo")
    p.add_argument("--adcensus-bin", default=None,
                   help="Đường dẫn adcensus_depth(.exe). Mặc định tự dò.")
    p.add_argument("--mind", type=int, default=0)
    p.add_argument("--maxd", type=int, default=64)
    p.add_argument("--adc-width", type=int, default=640,
                   help="Downscale ảnh trước khi đưa vào AD-Census (RAM/tốc độ)")
    p.add_argument("--speckle-size", type=int, default=600,
                   help="filterSpeckles maxSpeckleSize (0=tắt); blob nhỏ hơn → khử")
    p.add_argument("--speckle-diff", type=float, default=1.5,
                   help="filterSpeckles maxDiff (chênh disparity coi là cùng vùng)")
    p.add_argument("--median", type=int, default=7,
                   help="Kernel median làm mượt sàn (0/1=tắt, lẻ)")
    p.add_argument("--no-fill", action="store_true",
                   help="Không lấp lỗ nội bộ (giữ mọi pixel disp==0 đen)")
    p.add_argument("--no-preprocess", action="store_true",
                   help="Bỏ tiền xử lý (histogram-match R→L + CLAHE)")
    p.add_argument("--clahe-clip", type=float, default=3.0,
                   help="clipLimit CLAHE (tăng tương phản vùng tối)")
    p.add_argument("--skip-adcensus", action="store_true",
                   help="Bỏ qua chạy exe; đọc <out-prefix>_disp.png có sẵn")
    return p.parse_args()


# ─── stage 0: preprocessing (Phase 2 — hợp với AD-Census, giữ màu) ───────────

def match_histograms_color(src, ref):
    """Khớp histogram src→ref theo từng kênh (CDF mapping), giữ ảnh màu.

    Sửa lệch exposure/màu giữa 2 camera → giúp số hạng AD (color) của AD-Census.
    Tự cài (không phụ thuộc skimage).
    """
    matched = np.empty_like(src)
    for c in range(src.shape[2]):
        s = src[:, :, c].ravel()
        r = ref[:, :, c].ravel()
        s_vals, bin_idx, s_counts = np.unique(s, return_inverse=True,
                                              return_counts=True)
        r_vals, r_counts = np.unique(r, return_counts=True)
        s_q = np.cumsum(s_counts).astype(np.float64); s_q /= s_q[-1]
        r_q = np.cumsum(r_counts).astype(np.float64); r_q /= r_q[-1]
        interp = np.interp(s_q, r_q, r_vals)
        matched[:, :, c] = interp[bin_idx].reshape(src[:, :, c].shape)
    return matched.astype(np.uint8)


def clahe_color(img, clip):
    """CLAHE trên kênh L của Lab → tăng tương phản vùng tối, GIỮ màu cho AD."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    cl = cv2.createCLAHE(clipLimit=float(clip), tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)


def preprocess_pair(left, right, clip):
    """match R→L (trên ảnh GỐC, trước warp để né viền đen) rồi CLAHE cả hai."""
    right_m = match_histograms_color(right, left)
    return clahe_color(left, clip), clahe_color(right_m, clip)


# ─── stage 1: alignment ──────────────────────────────────────────────────────

def load_align_matrix(path, full_warp, img_w, img_h):
    """Trả ma trận warp 2x3 (đã scale theo độ phân giải, tx tuỳ chọn)."""
    if not os.path.isfile(path):
        print(f"[align] Không thấy {path} → bỏ qua align.")
        return None
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    M = fs.getNode("M").mat()
    src_w = fs.getNode("image_width").real() or img_w
    src_h = fs.getNode("image_height").real() or img_h
    fs.release()
    if M is None:
        print("[align] alignment.yml không có 'M' → bỏ qua.")
        return None
    M = M.astype(np.float64).copy()
    # scale translation theo độ phân giải nếu ảnh khác kích thước hiệu chuẩn
    if abs(src_w - img_w) > 1 or abs(src_h - img_h) > 1:
        M[0, 2] *= img_w / src_w
        M[1, 2] *= img_h / src_h
        print(f"[align] scale tx,ty theo res {int(src_w)}x{int(src_h)}→{img_w}x{img_h}")
    if not full_warp:
        # giữ horizontal disparity = tín hiệu depth → bỏ tx
        M[0, 2] = 0.0
        print(f"[align] giữ horizontal disparity: tx=0, ty={M[1,2]:.1f}px, "
              f"rot={np.degrees(np.arctan2(M[1,0], M[0,0])):.2f}°")
    else:
        print(f"[align] FULL warp: tx={M[0,2]:.1f}, ty={M[1,2]:.1f}px")
    return M


# ─── stage 2: AD-Census ──────────────────────────────────────────────────────

def find_adcensus_bin(explicit):
    cands = [explicit] if explicit else []
    cands += [os.path.join(_ADCENSUS_DIR, n) for n in
              ("adcensus_depth.exe", "adcensus_depth",
               "build/adcensus_depth.exe", "build/adcensus_depth")]
    for c in cands:
        if c and os.path.isfile(c):
            return c
    return None


def run_adcensus(binp, left_path, right_path, out_prefix, mind, maxd):
    """Chạy exe, trả (disp_png_path, dmin, dmax)."""
    # exe chạy với cwd = thư mục AD-Census → path tương đối sẽ sai. Ép tuyệt đối.
    left_path  = os.path.abspath(left_path)
    right_path = os.path.abspath(right_path)
    out_prefix = os.path.abspath(out_prefix)
    cmd = [binp, left_path, right_path, out_prefix, str(mind), str(maxd)]
    print(f"[adcensus] {' '.join(cmd)}")
    # exe build bằng MinGW → cần OpenCV DLL của msys64 trên PATH
    env = os.environ.copy()
    mingw = r"C:\msys64\mingw64\bin"
    if os.path.isdir(mingw):
        env["PATH"] = mingw + os.pathsep + env.get("PATH", "")
    # OpenBLAS cấp buffer theo số thread → máy ít RAM (<1.5GB free) bị
    # "Memory allocation failed". Giới hạn thread để giảm RAM dự phòng.
    env.setdefault("OPENBLAS_NUM_THREADS", "1")
    env.setdefault("OMP_NUM_THREADS", "2")
    proc = subprocess.run(cmd, cwd=_ADCENSUS_DIR, capture_output=True, text=True,
                          env=env)
    print(proc.stdout[-800:] if proc.stdout else "")
    if proc.returncode != 0:
        print(proc.stderr[-800:])
        raise RuntimeError(f"AD-Census thất bại (exit {proc.returncode})")
    # parse "Disparity range: [min, max] px"
    dmin, dmax = float(mind), float(maxd)
    m = re.search(r"Disparity range:\s*\[([-\d.]+),\s*([-\d.]+)\]", proc.stdout or "")
    if m:
        dmin, dmax = float(m.group(1)), float(m.group(2))
    disp_png = out_prefix + "_disp.png"
    if not os.path.isfile(disp_png):
        # exe dùng cwd của nó; nếu out_prefix là tương đối thì nằm trong _ADCENSUS_DIR
        alt = os.path.join(_ADCENSUS_DIR, os.path.basename(out_prefix) + "_disp.png")
        disp_png = disp_png if os.path.isfile(disp_png) else alt
    return disp_png, dmin, dmax


def reconstruct_disparity(disp_png, dmin, dmax):
    """PNG chuẩn hoá 0-255 → disparity thực [dmin,dmax]; 0=invalid giữ là 0."""
    norm = cv2.imread(disp_png, cv2.IMREAD_GRAYSCALE)
    if norm is None:
        raise FileNotFoundError(f"Không đọc được disparity PNG: {disp_png}")
    disp = norm.astype(np.float32) / 255.0 * (dmax - dmin) + dmin
    disp[norm == 0] = 0.0          # invalid/min → 0 (xa/sàn)
    disp[disp < 0] = 0.0
    return disp


# ─── khử boundary artifact ───────────────────────────────────────────────────

def invalidate_borders(disp, valid_mask, maxd):
    """Đặt disparity=0 ở các vùng KHÔNG hợp lệ về mặt hình học:

      - valid_mask==0: viền đen do warpAffine (right_aligned không có nội dung).
      - dải trái rộng `maxd` cột: pixel bên trái không có cặp tương ứng trong
        ảnh phải (search chạy ra ngoài khung) → disparity bịa.
    """
    h, w = disp.shape
    if valid_mask is not None:
        if valid_mask.shape != disp.shape:
            valid_mask = cv2.resize(valid_mask, (w, h),
                                    interpolation=cv2.INTER_NEAREST)
        disp[valid_mask == 0] = 0.0
    left_cols = int(max(0, min(maxd, w)))
    if left_cols:
        disp[:, :left_cols] = 0.0
    return disp


def denoise_disparity(disp, speckle_size, speckle_diff, median_k):
    """Khử nhiễu speckle trên sàn granite (texture lặp → AD-Census match sai).

      - filterSpeckles: zero các blob disparity nhỏ, lệch hẳn vùng xung quanh.
      - median: làm mượt sàn (giữ biên vật thể tốt hơn blur thường).
    Pixel bị khử = 0 (invalid → đen trong disp_color).
    """
    d16 = disp.astype(np.int16)
    if speckle_size and speckle_size > 0:
        cv2.filterSpeckles(d16, 0, int(speckle_size), float(speckle_diff))
    out = d16.astype(np.float32)
    if median_k and median_k >= 3:
        k = median_k if median_k % 2 == 1 else median_k + 1
        med = cv2.medianBlur(d16.astype(np.uint8), k).astype(np.float32)
        keep = out > 0                 # không lấp pixel đã invalid
        out = np.where(keep, med, 0.0)
    return out


def fill_holes(disp, dmax, valid_mask, left_cols):
    """Lấp các LỖ NỘI BỘ (disp==0 nằm trong vùng hợp lệ) bằng inpaint từ lân cận.

    GIỮ đen các vùng biên geometry (ngoài valid_mask + dải trái occlusion) vì đó
    thực sự không có dữ liệu stereo — chỉ lấp lỗ rải rác giữa sàn/vật thể.
    """
    h, w = disp.shape
    region = np.ones((h, w), bool) if valid_mask is None else (valid_mask > 0)
    if left_cols:
        region[:, :int(left_cols)] = False
    holes = region & (disp <= 0)
    if not holes.any():
        return disp
    disp8 = np.clip(disp / max(dmax, 1e-6) * 255.0, 0, 255).astype(np.uint8)
    filled8 = cv2.inpaint(disp8, holes.astype(np.uint8) * 255, 3, cv2.INPAINT_TELEA)
    out = disp.copy()
    out[holes] = filled8[holes].astype(np.float32) / 255.0 * dmax
    print(f"[fill] lấp {int(holes.sum())} px lỗ nội bộ (biên geometry giữ đen)")
    return out


def colorize(disp_int, ncols):
    """JET colormap; pixel vô hiệu (disp==0) hiển thị ĐEN, không phải xanh JET."""
    vis = (disp_int.astype(np.float32) / max(ncols - 1, 1) * 255).astype(np.uint8)
    color = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
    color[disp_int == 0] = (0, 0, 0)
    return color


# ─── main ────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()
    for f in (args.left, args.right):
        if not os.path.isfile(f):
            print(f"[err] không thấy ảnh: {f}"); sys.exit(1)

    left = cv2.imread(args.left)
    right = cv2.imread(args.right)
    H, W = left.shape[:2]
    print(f"[info] ảnh {W}x{H}")

    # ── Stage 0: preprocess (hist-match R→L + CLAHE; trước warp né viền đen) ─
    if not args.no_preprocess:
        left, right = preprocess_pair(left, right, args.clahe_clip)
        print(f"[pre] histogram-match R→L + CLAHE(clip={args.clahe_clip})")
    else:
        print("[pre] --no-preprocess: dùng ảnh gốc")

    # ── Stage 1: align right (+ valid mask để khử viền warp) ──────────────
    valid_full = np.full((H, W), 255, np.uint8)
    if args.no_align:
        right_al = right
        print("[align] --no-align: dùng right gốc")
    else:
        M = load_align_matrix(args.align, args.full_warp, W, H)
        if M is not None:
            right_al = cv2.warpAffine(right, M, (W, H), flags=cv2.INTER_LINEAR)
            # warp cùng ma trận lên ảnh trắng → 255 = vùng có nội dung thật
            valid_full = cv2.warpAffine(valid_full, M, (W, H),
                                        flags=cv2.INTER_NEAREST,
                                        borderValue=0)
        else:
            right_al = right
    right_al_path = args.out_prefix + "_right_aligned.jpg"
    os.makedirs(os.path.dirname(os.path.abspath(args.out_prefix)), exist_ok=True)
    cv2.imwrite(right_al_path, right_al)
    print(f"[out] {right_al_path}")

    # ── Stage 2: AD-Census depth (chạy ở độ phân giải nhỏ cho nhẹ RAM) ────
    # AD-Census ngốn RAM ~O(W*H*ndisp); 1280x720 → bad_alloc. Downscale trước.
    adc_w = min(args.adc_width, W)
    adc_h = int(round(H * adc_w / W))
    adc_maxd = max(16, int(round(args.maxd * adc_w / W)))
    left_s  = cv2.resize(left,     (adc_w, adc_h), interpolation=cv2.INTER_AREA)
    right_s = cv2.resize(right_al, (adc_w, adc_h), interpolation=cv2.INTER_AREA)
    left_s_path  = args.out_prefix + "_adcL.jpg"
    right_s_path = args.out_prefix + "_adcR.jpg"
    cv2.imwrite(left_s_path, left_s); cv2.imwrite(right_s_path, right_s)

    disp_png = args.out_prefix + "_disp.png"
    dmin, dmax = float(args.mind), float(adc_maxd)
    if not args.skip_adcensus:
        binp = find_adcensus_bin(args.adcensus_bin)
        if binp is None:
            print("[err] không tìm thấy adcensus_depth(.exe). Dùng --adcensus-bin "
                  "hoặc --skip-adcensus."); sys.exit(1)
        print(f"[adcensus] chạy ở {adc_w}x{adc_h}, maxd={adc_maxd}")
        disp_png, dmin, dmax = run_adcensus(
            binp, left_s_path, right_s_path, args.out_prefix, args.mind, adc_maxd)
    print(f"[adcensus] disparity range = [{dmin:.1f}, {dmax:.1f}] px")

    disp = reconstruct_disparity(disp_png, dmin, dmax)   # ở độ phân giải adc
    work_w, work_h = disp.shape[1], disp.shape[0]

    # ── khử boundary artifact (viền warp trái/đáy/đỉnh + dải occlusion trái) ─
    valid_adc = cv2.resize(valid_full, (work_w, work_h),
                           interpolation=cv2.INTER_NEAREST)
    n_before = int((disp > 0).sum())
    disp = invalidate_borders(disp, valid_adc, adc_maxd)
    n_after = int((disp > 0).sum())
    print(f"[border] khử {n_before - n_after} px viền (trái {adc_maxd} cột + warp)")

    # ── khử speckle noise trên sàn texture (granite) ──────────────────────
    n_pre = int((disp > 0).sum())
    disp = denoise_disparity(disp, args.speckle_size, args.speckle_diff, args.median)
    print(f"[denoise] speckle={args.speckle_size} median={args.median} → "
          f"khử thêm {n_pre - int((disp > 0).sum())} px nhiễu")

    # ── lấp lỗ nội bộ (giữ đen biên geometry) ─────────────────────────────
    if not args.no_fill:
        disp = fill_holes(disp, dmax, valid_adc, adc_maxd)

    ncols = int(np.ceil(dmax)) + 1
    disp_int = np.clip(np.round(disp), 0, ncols - 1).astype(np.int32)

    # ── save (depth thuần — không còn drivable overlay) ───────────────────
    out = args.out_prefix
    cv2.imwrite(out + "_disp_color.png", colorize(disp_int, ncols))
    cv2.imwrite(out + "_disp.png", (disp_int.astype(np.float32) /
                                    max(ncols - 1, 1) * 255).astype(np.uint8))
    print(f"\n[out] {out}_disp_color.png  ← KẾT QUẢ (AD-Census disparity, viền đã khử)")
    print(f"[out] {out}_disp.png        (disparity 8-bit chuẩn hoá)")


if __name__ == "__main__":
    main()
