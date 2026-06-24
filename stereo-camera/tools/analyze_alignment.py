#!/usr/bin/env python3
"""
analyze_alignment.py  –  Đo độ lệch hình học giữa 2 camera stereo.

Không cần nhiều ảnh — chỉ cần 1 cặp left/right.
Ưu tiên checkerboard corners (chính xác nhất), fallback sang ORB feature matching
nếu board không detect được (ảnh góc lớn, màn hình, v.v.).

Usage:
  python3 tools/analyze_alignment.py --left IMG_L --right IMG_R [opts]
  python3 tools/analyze_alignment.py --dir captures/checkerboard/<session> [opts]

Options:
  --left   PATH   Ảnh left trực tiếp
  --right  PATH   Ảnh right trực tiếp
  --dir    PATH   Thư mục session (tự tìm cặp tốt nhất)
  --rows   INT    Inner corners dọc   (default 7)
  --cols   INT    Inner corners ngang (default 7)
  --square FLOAT  Kích thước ô, mm   (default 18.0)
  --out    PATH   Lưu ma trận warp   (default calib/alignment.yml)
  --save-vis PATH Lưu ảnh visualize  (default calib/alignment_vis.jpg)
  --method auto|checker|orb   Phương pháp detect (default auto)

Output:
  Console: shift_x, shift_y, rotation, scale
  calib/alignment.yml   – ma trận affine 2x3 để warpAffine right → left
  calib/alignment_vis.jpg  – ảnh so sánh trước/sau
"""

import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# ─── helpers ────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="Đo độ lệch hình học stereo")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--dir",   help="Thư mục session (tự chọn cặp tốt nhất)")
    g.add_argument("--left",  help="Ảnh left trực tiếp")
    p.add_argument("--right", help="Ảnh right (bắt buộc nếu dùng --left)")
    p.add_argument("--rows",  type=int,   default=7)
    p.add_argument("--cols",  type=int,   default=7)
    p.add_argument("--square",type=float, default=18.0)
    p.add_argument("--out",   default="calib/alignment.yml")
    p.add_argument("--save-vis", default="calib/alignment_vis.jpg")
    p.add_argument("--method", default="auto",
                   choices=["auto", "checker", "orb"])
    return p.parse_args()


def find_best_pair(folder):
    """Tìm cặp left/right trong folder, trả (left_path, right_path)[]."""
    lefts, rights = {}, {}
    for path in glob.glob(os.path.join(folder, "*.jpg")) + \
                glob.glob(os.path.join(folder, "*.png")):
        name = os.path.basename(path)
        m = re.match(r"(left|right)[_-]?(\d+)\.", name, re.I)
        if m:
            side, idx = m.group(1).lower(), int(m.group(2))
            (lefts if side == "left" else rights)[idx] = path
    common = sorted(set(lefts) & set(rights))
    return [(lefts[i], rights[i]) for i in common]


def load_gray(path):
    img = cv2.imread(path)
    if img is None:
        raise FileNotFoundError(f"Không đọc được: {path}")
    return img, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


# ─── method 1: checkerboard corners ─────────────────────────────────────────

def try_checkerboard(gray_l, gray_r, rows, cols):
    pattern = (cols, rows)
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)

    ok_l, c_l = cv2.findChessboardCorners(gray_l, pattern, flags)
    ok_r, c_r = cv2.findChessboardCorners(gray_r, pattern, flags)

    if not (ok_l and ok_r):
        found = []
        if ok_l: found.append("L")
        if ok_r: found.append("R")
        print(f"[checker] Detect thất bại — chỉ thấy: {found if found else 'none'}")
        return None, None

    c_l = cv2.cornerSubPix(gray_l, c_l, (11,11), (-1,-1), _SUBPIX_CRIT)
    c_r = cv2.cornerSubPix(gray_r, c_r, (11,11), (-1,-1), _SUBPIX_CRIT)
    print(f"[checker] OK — {rows*cols} corners/cam")
    return c_l.reshape(-1, 2), c_r.reshape(-1, 2)


# ─── method 2: ORB feature matching ─────────────────────────────────────────

def try_orb(gray_l, gray_r, min_matches=30):
    orb = cv2.ORB_create(nfeatures=3000)
    kp_l, des_l = orb.detectAndCompute(gray_l, None)
    kp_r, des_r = orb.detectAndCompute(gray_r, None)

    if des_l is None or des_r is None or len(kp_l) < 10 or len(kp_r) < 10:
        print(f"[orb] Không đủ keypoints: L={len(kp_l) if kp_l else 0} R={len(kp_r) if kp_r else 0}")
        return None, None, []

    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
    raw = bf.knnMatch(des_r, des_l, k=2)
    good = [m for m, n in raw if m.distance < 0.75 * n.distance]

    print(f"[orb] Keypoints L={len(kp_l)} R={len(kp_r)} | good matches={len(good)}")
    if len(good) < min_matches:
        print(f"[orb] Không đủ matches (cần {min_matches})")
        return None, None, []

    pts_r = np.float32([kp_r[m.queryIdx].pt for m in good])
    pts_l = np.float32([kp_l[m.trainIdx].pt for m in good])
    return pts_l, pts_r, good


# ─── estimate affine ─────────────────────────────────────────────────────────

def estimate_affine(pts_dst, pts_src):
    """Tính ma trận affine warp src (right) → dst (left)."""
    M, inliers = cv2.estimateAffinePartial2D(
        pts_src, pts_dst,
        method=cv2.RANSAC, ransacReprojThreshold=3.0,
        maxIters=2000, confidence=0.995)
    if M is None:
        return None, 0
    n_inliers = int(inliers.sum()) if inliers is not None else 0
    return M, n_inliers


def decompose_affine(M):
    """Trích shift_x, shift_y, rotation (deg), scale từ ma trận 2x3."""
    cos_a = M[0, 0]; sin_a = M[1, 0]
    scale = float(np.sqrt(cos_a**2 + sin_a**2))
    angle_deg = float(np.degrees(np.arctan2(sin_a, cos_a)))
    shift_x = float(M[0, 2])
    shift_y = float(M[1, 2])
    return shift_x, shift_y, angle_deg, scale


# ─── visualizations ──────────────────────────────────────────────────────────

def draw_match_lines(img_l, img_r, pts_l, pts_r, max_draw=40):
    """Vẽ đường nối các điểm tương ứng side-by-side."""
    h = max(img_l.shape[0], img_r.shape[0])
    w_l = img_l.shape[1]; w_r = img_r.shape[1]
    canvas = np.zeros((h, w_l + w_r, 3), np.uint8)
    canvas[:img_l.shape[0], :w_l] = img_l
    canvas[:img_r.shape[0], w_l:] = img_r

    step = max(1, len(pts_l) // max_draw)
    for i in range(0, len(pts_l), step):
        pl = (int(pts_l[i][0]), int(pts_l[i][1]))
        pr = (int(pts_r[i][0]) + w_l, int(pts_r[i][1]))
        color = (0, int(255 * i / len(pts_l)), int(255 * (1 - i/len(pts_l))))
        cv2.line(canvas, pl, pr, color, 1)
        cv2.circle(canvas, pl, 3, (0, 255, 0), -1)
        cv2.circle(canvas, pr, 3, (0, 200, 255), -1)
    return canvas


def draw_overlay(img_l, img_warped):
    """Overlay left (blue) + warped right (red) để thấy lệch màu."""
    h = min(img_l.shape[0], img_warped.shape[0])
    w = min(img_l.shape[1], img_warped.shape[1])
    out = np.zeros((h, w, 3), np.uint8)
    out[:, :, 0] = cv2.cvtColor(img_l[:h, :w], cv2.COLOR_BGR2GRAY)        # B = left
    out[:, :, 2] = cv2.cvtColor(img_warped[:h, :w], cv2.COLOR_BGR2GRAY)   # R = right warped
    return out


def draw_epipolar(img_l, img_warped, step=40):
    """Đường ngang epipolar — nếu aligned đúng, nội dung 2 ảnh nằm cùng hàng."""
    h = min(img_l.shape[0], img_warped.shape[0])
    w = img_l.shape[1]
    combined = np.hstack([img_l[:h], img_warped[:h]])
    for y in range(0, h, step):
        cv2.line(combined, (0, y), (combined.shape[1], y), (0, 255, 0), 1)
    return combined


def make_vis(img_l, img_r, img_warped, pts_l, pts_r,
             shift_x, shift_y, angle, scale, method_name, inliers):
    """Tổng hợp ảnh kết quả gồm 3 hàng."""
    font  = cv2.FONT_HERSHEY_SIMPLEX
    H, W  = img_l.shape[:2]

    # ── Row 1: Before (raw pair side-by-side) ──────────────────────────────
    before = np.hstack([img_l.copy(), img_r.copy()])
    cv2.putText(before, "LEFT (raw)", (12, 34), font, 0.9, (60,220,220), 2)
    cv2.putText(before, "RIGHT (raw)", (W+12, 34), font, 0.9, (60,220,220), 2)
    cv2.putText(before, "BEFORE alignment", (12, before.shape[0]-16),
                font, 0.7, (200,200,200), 2)

    # ── Row 2: Match lines ─────────────────────────────────────────────────
    matches_vis = draw_match_lines(img_l, img_r, pts_l, pts_r)
    cv2.putText(matches_vis, f"Matches ({method_name})  inliers={inliers}",
                (12, 34), font, 0.9, (0, 220, 255), 2)

    # ── Row 3: After (epipolar check) ──────────────────────────────────────
    after_ep = draw_epipolar(img_l, img_warped)
    cv2.putText(after_ep, "LEFT", (12, 34), font, 0.9, (60,220,220), 2)
    cv2.putText(after_ep, "RIGHT warped → LEFT", (W+12, 34), font, 0.9, (60,220,220), 2)
    cv2.putText(after_ep, "AFTER alignment (epipolar lines)", (12, after_ep.shape[0]-16),
                font, 0.7, (0, 220, 0), 2)

    # ── Row 4: Overlay (color diff) ────────────────────────────────────────
    overlay = draw_overlay(img_l, img_warped)
    # scale to same width
    overlay = cv2.resize(overlay, (W*2, H))
    cv2.putText(overlay, "OVERLAY: Blue=LEFT  Red=RIGHT warped  (cyan=aligned)",
                (12, 34), font, 0.7, (200,200,200), 2)

    # ── Stats banner ───────────────────────────────────────────────────────
    stats_h = 80
    stats = np.zeros((stats_h, W*2, 3), np.uint8)
    dir_x = "RIGHT" if shift_x > 0 else "LEFT"
    dir_y = "DOWN"  if shift_y > 0 else "UP"
    dir_r = "CW"    if angle  > 0 else "CCW"
    line1 = (f"shift_x={shift_x:+.1f}px ({dir_x})   "
             f"shift_y={shift_y:+.1f}px ({dir_y})   "
             f"rotation={angle:+.2f}deg ({dir_r})   "
             f"scale={scale:.4f}")
    line2 = (f"Method: {method_name}  |  Inliers: {inliers}  |  "
             f"Interpretation: right cam is {dir_x} {abs(shift_x):.0f}px  "
             f"{dir_y} {abs(shift_y):.0f}px  rotated {dir_r} {abs(angle):.2f}deg vs left")
    cv2.putText(stats, line1, (12, 26), font, 0.6, (0, 255, 200), 2)
    cv2.putText(stats, line2, (12, 58), font, 0.5, (180,180,180), 1)

    # ── Stack rows ─────────────────────────────────────────────────────────
    target_w = W * 2
    rows_vis = []
    for r in [before, matches_vis, after_ep, overlay]:
        if r.shape[1] != target_w:
            r = cv2.resize(r, (target_w, int(r.shape[0] * target_w / r.shape[1])))
        rows_vis.append(r)
    rows_vis.append(stats)

    full = np.vstack(rows_vis)
    # Scale down if too tall
    max_h = 2400
    if full.shape[0] > max_h:
        s = max_h / full.shape[0]
        full = cv2.resize(full, None, fx=s, fy=s)
    return full


# ─── main ────────────────────────────────────────────────────────────────────

def main():
    args = parse_args()

    # Resolve image paths
    if args.dir:
        pairs = find_best_pair(args.dir)
        if not pairs:
            print(f"[err] Không tìm thấy cặp left/right trong: {args.dir}")
            sys.exit(1)
        left_path, right_path = pairs[0]
        print(f"[info] Dùng cặp: {os.path.basename(left_path)} / {os.path.basename(right_path)}")
    elif args.left and args.right:
        left_path, right_path = args.left, args.right
    else:
        print("[err] Cần --dir HOẶC --left + --right")
        sys.exit(1)

    img_l, gray_l = load_gray(left_path)
    img_r, gray_r = load_gray(right_path)
    H, W = img_l.shape[:2]
    print(f"[info] Ảnh: {W}x{H}")

    # ── Detect points ───────────────────────────────────────────────────────
    pts_l = pts_r = None
    method_name = ""
    orb_matches = []

    if args.method in ("auto", "checker"):
        print("[step 1] Thử detect checkerboard corners ...")
        pts_l, pts_r = try_checkerboard(gray_l, gray_r, args.rows, args.cols)
        if pts_l is not None:
            method_name = "checkerboard"

    if pts_l is None and args.method in ("auto", "orb"):
        print("[step 2] Fallback → ORB feature matching ...")
        pts_l, pts_r, orb_matches = try_orb(gray_l, gray_r)
        if pts_l is not None:
            method_name = "ORB+RANSAC"

    if pts_l is None:
        print("[err] Không thể xác định điểm tương ứng giữa 2 ảnh.")
        print("      Gợi ý: chụp lại bàn cờ IN RA GIẤY (không dùng màn hình), đặt thẳng góc.")
        sys.exit(2)

    # ── Estimate affine ─────────────────────────────────────────────────────
    print(f"\n[step 3] Tính affine transform ({method_name}) ...")
    M, inliers = estimate_affine(pts_l, pts_r)
    if M is None:
        print("[err] estimateAffinePartial2D thất bại (quá ít inliers).")
        sys.exit(3)

    shift_x, shift_y, angle, scale = decompose_affine(M)

    # ── Report ───────────────────────────────────────────────────────────────
    dir_x = "RIGHT" if shift_x > 0 else "LEFT"
    dir_y = "DOWN"  if shift_y > 0 else "UP"
    dir_r = "CW"    if angle   > 0 else "CCW"
    print(f"\n{'='*55}")
    print(f"  CAMERA ALIGNMENT MEASUREMENT  ({method_name})")
    print(f"{'='*55}")
    print(f"  shift_x   = {shift_x:+8.2f} px   → right cam lệch {dir_x} {abs(shift_x):.1f}px")
    print(f"  shift_y   = {shift_y:+8.2f} px   → right cam lệch {dir_y} {abs(shift_y):.1f}px")
    print(f"  rotation  = {angle:+8.3f} °    → right cam xoay {dir_r} {abs(angle):.3f}°")
    print(f"  scale     = {scale:+.5f}      → {'phóng to' if scale>1 else 'thu nhỏ'} {abs(scale-1)*100:.2f}%")
    print(f"  inliers   = {inliers}")
    print(f"{'='*55}")
    print(f"\n  Để pre-process: warpAffine(right_frame, M, (W,H))")
    print(f"  Ma trận M (2x3):\n{M}\n")

    # ── Apply warp & save ────────────────────────────────────────────────────
    img_warped = cv2.warpAffine(img_r, M, (W, H), flags=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    # Save alignment yml
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fs = cv2.FileStorage(args.out, cv2.FILE_STORAGE_WRITE)
    fs.write("M", M)
    fs.write("shift_x", shift_x);  fs.write("shift_y", shift_y)
    fs.write("rotation_deg", angle); fs.write("scale", scale)
    fs.write("inliers", inliers); fs.write("method", method_name)
    fs.write("image_width", W); fs.write("image_height", H)
    fs.release()
    print(f"[out] Ma trận warp → {args.out}")

    # Save visualisation
    vis = make_vis(img_l, img_r, img_warped, pts_l, pts_r,
                   shift_x, shift_y, angle, scale, method_name, inliers)
    os.makedirs(os.path.dirname(os.path.abspath(args.save_vis)), exist_ok=True)
    cv2.imwrite(args.save_vis, vis)
    print(f"[out] Ảnh visualize → {args.save_vis}")
    print(f"\nKiểm tra: mở {args.save_vis}")
    print("  Row 1: Raw pair (trước)")
    print("  Row 2: Feature matches")
    print("  Row 3: Sau alignment (đường ngang epipolar — nếu OK thì nội dung cùng hàng)")
    print("  Row 4: Overlay Blue=left Red=right-warped (cyan = aligned hoàn hảo)")


if __name__ == "__main__":
    main()
