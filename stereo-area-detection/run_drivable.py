#!/usr/bin/env python3
"""
run_drivable.py  –  Drivable / free-space detection từ AD-Census stereo depth.

Pipeline:
  left.jpg ─────────────────────────────────────┐
  right.jpg → warpAffine(alignment.yml, tx=0) ───┴► AD-Census .exe → disparity
                                                       │
                            tái tạo disparity (int) từ PNG đã chuẩn hoá
                                                       │
                    u/v-disparity + HoughLine (mặt sàn) + HMM (biên free-space)
                                                       │
                                            <prefix>_drivable.jpg

Thuật toán free-space (u/v-disparity + HMM Viterbi) mượn từ
  github.com/sajaysurya/drivable_area_detection  (sajaysurya)
nhưng nguồn depth là AD-Census (chất lượng hơn SGBM) và đã vá:
  - bỏ np.float (gỡ ở numpy>=1.20)
  - không popup matplotlib — lưu thẳng ảnh
  - onehot tiết kiệm RAM (uint16, có downscale)
  - HoughLines có fallback khi cảnh không phải đường phẳng KITTI

LƯU Ý stereo-quan-trọng:
  alignment.yml gồm rotation + dịch dọc (ty) + dịch ngang (tx). Để stereo depth
  ĐÚNG ta chỉ khử rotation + ty (làm epipolar nằm ngang) và GIỮ tx (chính là
  tín hiệu độ sâu). Dùng --full-warp nếu muốn áp cả tx.

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
from scipy.sparse import diags
from hmmlearn import base as hmm_base

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
_DEFAULT_ALIGN = os.path.join(_HERE, "calib", "alignment.yml")


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
    p.add_argument("--work-width", type=int, default=640,
                   help="Bề rộng xử lý u/v-disparity (downscale cho nhẹ RAM)")
    p.add_argument("--obstacle-disp", type=int, default=12,
                   help="Ngưỡng disparity coi là vật cản trong u-disparity")
    p.add_argument("--skip-adcensus", action="store_true",
                   help="Bỏ qua chạy exe; đọc <out-prefix>_disp.png có sẵn")
    return p.parse_args()


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


# ─── stage 3: free-space (u/v-disparity + HMM) ───────────────────────────────
# Mượn thuật toán từ sajaysurya/drivable_area_detection, vá cho numpy mới + RAM.

class _HMM(hmm_base._BaseHMM):
    """HMM Viterbi cho biên free-space (theo bản gốc)."""
    obstacle_disp = 12  # gán động trước khi decode

    def _compute_log_likelihood(self, disp):
        loglike = np.log((disp > self.obstacle_disp).astype(np.float64) + 1e-32)
        loglike += np.tile(-np.flip(np.arange(disp.shape[1])),
                           (disp.shape[0], 1)) * 0.1
        return loglike


def onehot_uv(disp_int, ncols):
    """u/v-disparity tiết kiệm RAM (không bung onehot int64 của bản gốc).

    v_disparity[r, d] = số pixel ở hàng r có disparity d
    u_disparity[d, c] = số pixel ở cột c có disparity d
    """
    h, w = disp_int.shape
    v_disparity = np.zeros((h, ncols), np.int32)
    u_disparity = np.zeros((ncols, w), np.int32)
    for d in range(1, ncols):           # bỏ d=0 (invalid/sàn xa)
        mask = (disp_int == d)
        if not mask.any():
            continue
        v_disparity[:, d] = mask.sum(axis=1)
        u_disparity[d, :] = mask.sum(axis=0)
    return v_disparity, u_disparity


def ground_projection(v_disparity):
    """Tìm mặt sàn từ v-disparity bằng HoughLines; fallback nếu cảnh không phẳng.

    Trả lambda: disparity → row index (biên sàn ở mỗi mức disparity).
    """
    v_thresh = (v_disparity > max(50, v_disparity.max() // 8)).astype(np.uint8)
    lines = cv2.HoughLines(v_thresh, 1, np.pi / 180, 60)
    if lines is not None:
        lines = np.squeeze(lines, axis=1) if lines.ndim == 3 else lines
        sel = lines[(lines[:, 1] > 1.5) & (lines[:, 1] < 3.0)]
        line = sel[0] if sel.size else lines[0]
        rho, theta = float(line[0]), float(line[1])
        if abs(np.sin(theta)) > 1e-3:
            print(f"[ground] HoughLine rho={rho:.1f} theta={np.degrees(theta):.1f}°")
            return lambda x: -np.cos(theta) / np.sin(theta) * x + rho / np.sin(theta)
    # fallback: ánh xạ tuyến tính disparity→row (gần=disparity cao=hàng dưới)
    h = v_disparity.shape[0]; nd = v_disparity.shape[1]
    print("[ground] Không thấy đường sàn rõ — fallback tuyến tính.")
    return lambda x: h - (np.asarray(x) / max(nd - 1, 1)) * h


def free_bound_hmm(u_disparity, obstacle_disp):
    """HMM Viterbi → với mỗi cột, mức disparity của vật cản gần nhất (biên free)."""
    num_states = u_disparity.shape[0]
    model = _HMM(num_states)
    model.obstacle_disp = obstacle_disp
    model.startprob_ = np.ones(num_states) / num_states
    band = [1, 2, 3, 5, 7, 9, 11, 15, 11, 9, 7, 5, 3, 2, 1]
    posi = [-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7]
    mat = diags(band, posi, shape=(num_states, num_states)).toarray() + 0.5
    mat = (mat / np.sum(mat, axis=0)).T
    model.transmat_ = mat
    _, states = model.decode(u_disparity.T)
    return states          # độ dài W: với mỗi cột → disparity biên


# ─── overlay ─────────────────────────────────────────────────────────────────

def render_overlay(left_bgr, free_bound, project, scale_x, scale_y):
    """Tô xanh vùng drivable lên ảnh trái (toạ độ work → full res)."""
    H, W = left_bgr.shape[:2]
    overlay = left_bgr.copy()
    # biên sàn theo từng cột (work-space) → row, rồi map lên full res
    rows_work = project(free_bound)                       # độ dài W_work
    rows_work = np.clip(rows_work, 0, None)
    cols_full = np.round(np.arange(len(free_bound)) * scale_x).astype(int)
    rows_full = np.clip(np.round(rows_work * scale_y), 0, H - 1).astype(int)
    mask = np.zeros((H, W), np.uint8)
    for cf, rf in zip(cols_full, rows_full):
        if 0 <= cf < W:
            mask[rf:H, cf] = 1
    # mượt biên + tô
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    green = np.zeros_like(overlay); green[:] = (0, 200, 0)
    overlay = np.where(mask[..., None] == 1,
                       cv2.addWeighted(overlay, 0.5, green, 0.5, 0), overlay)
    # vẽ đường biên
    ys = rows_full
    for cf, rf in zip(cols_full, ys):
        if 0 <= cf < W:
            cv2.circle(overlay, (cf, rf), 1, (0, 255, 255), -1)
    return overlay, mask


def colorize(disp_int, ncols):
    vis = (disp_int.astype(np.float32) / max(ncols - 1, 1) * 255).astype(np.uint8)
    return cv2.applyColorMap(vis, cv2.COLORMAP_JET)


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

    # ── Stage 1: align right ──────────────────────────────────────────────
    if args.no_align:
        right_al = right
        print("[align] --no-align: dùng right gốc")
    else:
        M = load_align_matrix(args.align, args.full_warp, W, H)
        right_al = (cv2.warpAffine(right, M, (W, H), flags=cv2.INTER_LINEAR)
                    if M is not None else right)
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

    # ── Stage 3: free-space (làm việc thẳng ở độ phân giải AD-Census) ──────
    work_w, work_h = disp.shape[1], disp.shape[0]
    ncols = int(np.ceil(dmax)) + 1
    disp_int = np.clip(np.round(disp), 0, ncols - 1).astype(np.int32)

    print(f"[freespace] work {work_w}x{work_h}, ncols={ncols}")
    v_disp, u_disp = onehot_uv(disp_int, ncols)
    project = ground_projection(v_disp)
    free_bound = free_bound_hmm(u_disp, min(args.obstacle_disp, ncols - 2))

    # ── render & save ─────────────────────────────────────────────────────
    overlay, mask = render_overlay(left, free_bound, project,
                                   scale_x=W / work_w, scale_y=H / work_h)

    out = args.out_prefix
    cv2.imwrite(out + "_disp_color.png", colorize(disp_int, ncols))
    cv2.imwrite(out + "_udisparity.png",
                colorize((u_disp.astype(np.float32) /
                          max(u_disp.max(), 1) * (ncols - 1)).astype(np.int32), ncols))
    cv2.imwrite(out + "_vdisparity.png",
                colorize((v_disp.astype(np.float32) /
                          max(v_disp.max(), 1) * (ncols - 1)).astype(np.int32), ncols))
    cv2.imwrite(out + "_drivable.jpg", overlay)
    print(f"\n[out] {out}_drivable.jpg   ← KẾT QUẢ chính (vùng xanh = đi được)")
    print(f"[out] {out}_disp_color.png  (AD-Census disparity)")
    print(f"[out] {out}_udisparity.png / _vdisparity.png  (debug)")


if __name__ == "__main__":
    main()
