#!/usr/bin/env python3
"""
calibrate_from_images.py  –  Định nghĩa CONFIG stereo từ các cặp ảnh đã LƯU.

Khác capture_checkerboard.py: tool đó calibrate LIVE (cần 2 cam cắm vào lúc chạy).
Tool NÀY đọc ảnh left_NN.jpg / right_NN.jpg đã lưu trên đĩa rồi tính toàn bộ
"config" của cặp camera — kể cả khi 2 cam KHÔNG thẳng hàng (sai lệch nằm trong R,T).

Usage:
  python3 tools/calibrate_from_images.py --dir captures/checkerboard/<session> [opts]

Options:
  --dir    PATH   Thư mục chứa left_*.jpg / right_*.jpg   (BẮT BUỘC)
  --rows   INT    Inner corners theo chiều dọc   (default 7)   ← KHÔNG phải số ô!
  --cols   INT    Inner corners theo chiều ngang (default 7)
  --square FLOAT  Kích thước 1 ô vuông, mm        (default 18.0)
  --out    PATH   File config xuất ra             (default calib/stereo.yml)
  --preview-dir PATH  Nơi lưu ảnh debug           (default <dir>/_calib)
  --fix-intrinsic / --no-fix-intrinsic  Cố định nội tham số khi stereoCalibrate
                                        (default: tự — fix nếu >= 12 cặp tốt)
  --expected-dist FLOAT  Khoảng cách thật cam→bàn cờ, mm (chỉ để sanity-check)

LƯU Ý quan trọng về "8x8":
  OpenCV cần số GÓC TRONG (inner corners), KHÔNG phải số ô.
  Bàn cờ 8x8 Ô  →  7x7 GÓC TRONG  →  dùng --rows 7 --cols 7 (mặc định).
  Bàn cờ 9x6 góc trong (mặc định của capture tool) → --rows 6 --cols 9.

Output (tương thích StereoCamera::loadCalibration):
  M1,D1  M2,D2  R,T  R1,R2,P1,P2,Q  baseline  image_width/height
Ảnh debug:
  <preview-dir>/detect_NN.jpg   – mỗi cặp: corners vẽ lên, hoặc lý do FAIL
  <preview-dir>/rectified.jpg   – cặp đã nắn thẳng + đường epipolar ngang (kiểm tra)
"""

import argparse
import glob
import os
import re
import sys

import cv2
import numpy as np

# Console Windows mặc định cp1258 → in tiếng Việt crash. Ép UTF-8.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def parse_args():
    p = argparse.ArgumentParser(description="Calibrate stereo từ ảnh đã lưu")
    p.add_argument("--dir", required=True, help="Thư mục chứa left_*/right_* .jpg")
    p.add_argument("--rows", type=int, default=7, help="Inner corners dọc")
    p.add_argument("--cols", type=int, default=7, help="Inner corners ngang")
    p.add_argument("--square", type=float, default=18.0, help="Kích thước ô, mm")
    p.add_argument("--out", default="calib/stereo.yml")
    p.add_argument("--preview-dir", default=None)
    p.add_argument("--fix-intrinsic", dest="fix_intrinsic",
                   action="store_true", default=None)
    p.add_argument("--no-fix-intrinsic", dest="fix_intrinsic",
                   action="store_false")
    p.add_argument("--expected-dist", type=float, default=None,
                   help="Khoảng cách thật cam→bàn cờ, mm (chỉ sanity-check)")
    return p.parse_args()


def find_pairs(folder):
    """Ghép left_NN.* với right_NN.* theo số NN. Trả [(idx, left_path, right_path)]."""
    lefts = {}
    rights = {}
    for path in glob.glob(os.path.join(folder, "*")):
        name = os.path.basename(path)
        m = re.match(r"(left|right)[_-]?(\d+)\.(jpg|jpeg|png|bmp)$", name, re.I)
        if not m:
            continue
        side, num = m.group(1).lower(), int(m.group(2))
        (lefts if side == "left" else rights)[num] = path
    pairs = []
    for idx in sorted(set(lefts) & set(rights)):
        pairs.append((idx, lefts[idx], rights[idx]))
    orphans = (set(lefts) ^ set(rights))
    return pairs, sorted(orphans)


def make_objp(rows, cols, square_mm):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm / 1000.0   # mm → m
    return objp


def find_corners(gray, pattern):
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
             | cv2.CALIB_CB_FAST_CHECK)
    ret, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ret:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRIT)


def save_detect_preview(out_path, img_l, img_r, c_l, c_r, pattern, note):
    d_l = img_l.copy(); d_r = img_r.copy()
    cv2.drawChessboardCorners(d_l, pattern, c_l, c_l is not None)
    cv2.drawChessboardCorners(d_r, pattern, c_r, c_r is not None)
    font = cv2.FONT_HERSHEY_SIMPLEX
    okc = (0, 220, 0); bad = (0, 0, 255)
    cv2.putText(d_l, "L " + ("OK" if c_l is not None else "FAIL"),
                (12, 34), font, 0.9, okc if c_l is not None else bad, 2)
    cv2.putText(d_r, "R " + ("OK" if c_r is not None else "FAIL"),
                (12, 34), font, 0.9, okc if c_r is not None else bad, 2)
    if d_l.shape != d_r.shape:
        d_r = cv2.resize(d_r, (d_l.shape[1], d_l.shape[0]))
    combined = np.hstack([d_l, d_r])
    cv2.putText(combined, note, (12, combined.shape[0] - 16),
                font, 0.7, (60, 220, 220), 2)
    if combined.shape[1] > 1800:
        s = 1800.0 / combined.shape[1]
        combined = cv2.resize(combined, None, fx=s, fy=s)
    cv2.imwrite(out_path, combined)


def save_rectified_preview(out_path, img_l, img_r, M1, D1, M2, D2,
                           R1, R2, P1, P2, size):
    map1x, map1y = cv2.initUndistortRectifyMap(M1, D1, R1, P1, size, cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(M2, D2, R2, P2, size, cv2.CV_32FC1)
    rl = cv2.remap(img_l, map1x, map1y, cv2.INTER_LINEAR)
    rr = cv2.remap(img_r, map2x, map2y, cv2.INTER_LINEAR)
    if rl.shape != rr.shape:
        rr = cv2.resize(rr, (rl.shape[1], rl.shape[0]))
    combined = np.hstack([rl, rr])
    # Đường ngang: nếu nắn đúng, cùng 1 điểm vật thể nằm trên cùng 1 hàng ở 2 ảnh.
    for y in range(0, combined.shape[0], 40):
        cv2.line(combined, (0, y), (combined.shape[1], y), (0, 255, 0), 1)
    if combined.shape[1] > 1800:
        s = 1800.0 / combined.shape[1]
        combined = cv2.resize(combined, None, fx=s, fy=s)
    cv2.imwrite(out_path, combined)


def main():
    args = parse_args()
    folder = args.dir
    if not os.path.isdir(folder):
        print(f"[err] Không thấy thư mục: {folder}")
        sys.exit(1)

    pattern = (args.cols, args.rows)   # OpenCV: (width=cols, height=rows)
    preview_dir = args.preview_dir or os.path.join(folder, "_calib")
    os.makedirs(preview_dir, exist_ok=True)

    pairs, orphans = find_pairs(folder)
    if orphans:
        print(f"[warn] Bỏ qua ảnh lẻ (không đủ cặp) chỉ số: {orphans}")
    if not pairs:
        print(f"[err] Không tìm thấy cặp left_*/right_* nào trong {folder}")
        sys.exit(1)

    print(f"[calib] {len(pairs)} cặp ảnh. Pattern inner corners = "
          f"{args.cols}x{args.rows}, square={args.square}mm")
    print(f"[calib] Ảnh debug → {preview_dir}\n")

    objp = make_objp(args.rows, args.cols, args.square)
    obj_pts, img_l_pts, img_r_pts = [], [], []
    img_size = None
    n_ok = 0

    for idx, lp, rp in pairs:
        img_l = cv2.imread(lp)
        img_r = cv2.imread(rp)
        if img_l is None or img_r is None:
            print(f"[pair {idx:02d}] FAIL đọc ảnh")
            continue
        gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            img_size = (gray_l.shape[1], gray_l.shape[0])
        c_l = find_corners(gray_l, pattern)
        c_r = find_corners(gray_r, pattern)
        ok = (c_l is not None) and (c_r is not None)
        if ok:
            obj_pts.append(objp)
            img_l_pts.append(c_l)
            img_r_pts.append(c_r)
            n_ok += 1
        which = ("OK" if ok else
                 "FAIL(" + ("R" if c_l is not None else
                            "L" if c_r is not None else "L+R") + ")")
        print(f"[pair {idx:02d}] {which}")
        save_detect_preview(os.path.join(preview_dir, f"detect_{idx:02d}.jpg"),
                            img_l, img_r, c_l, c_r, pattern, f"pair {idx:02d} {which}")

    print(f"\n[calib] {n_ok}/{len(pairs)} cặp phát hiện được bàn cờ ở CẢ 2 ảnh.")
    if n_ok < 5:
        print("[err] Cần >= ~5 cặp tốt (khuyến nghị 15-25, đa dạng góc/khoảng cách).")
        print("      Xem detect_*.jpg trong thư mục debug để biết ảnh nào fail & vì sao")
        print("      (bàn cờ bị che/cắt mép/sai số góc trong --rows/--cols).")
        sys.exit(2)

    fix = args.fix_intrinsic
    if fix is None:
        fix = n_ok >= 12   # đủ nhiều cặp → fix intrinsic cho stereo ổn định hơn

    print(f"\n[calib] Calibrate nội tham số từng cam ...")
    rms_l, M1, D1, _, _ = cv2.calibrateCamera(obj_pts, img_l_pts, img_size, None, None)
    rms_r, M2, D2, _, _ = cv2.calibrateCamera(obj_pts, img_r_pts, img_size, None, None)
    print(f"[calib] Left RMS={rms_l:.4f}px  Right RMS={rms_r:.4f}px")

    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    flags = cv2.CALIB_FIX_INTRINSIC if fix else 0
    print(f"[calib] stereoCalibrate (fix_intrinsic={bool(fix)}) ...")
    rms, M1, D1, M2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, img_l_pts, img_r_pts, M1, D1, M2, D2, img_size,
        criteria=crit, flags=flags)
    print(f"[calib] Stereo RMS={rms:.4f}px  (tốt: < ~1.0, lý tưởng < 0.5)")

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        M1, D1, M2, D2, img_size, R, T, alpha=0, newImageSize=img_size)

    baseline_m = float(np.linalg.norm(T))
    # Góc xoay giữa 2 cam (mức độ "không thẳng")
    rvec, _ = cv2.Rodrigues(R)
    tilt_deg = float(np.degrees(np.linalg.norm(rvec)))

    print(f"\n[calib] === CONFIG cặp camera ===")
    print(f"[calib] baseline = {baseline_m*1000:.2f} mm")
    print(f"[calib] góc lệch giữa 2 cam = {tilt_deg:.2f}°  "
          f"(T = [{T[0,0]*1000:.1f}, {T[1,0]*1000:.1f}, {T[2,0]*1000:.1f}] mm)")
    print(f"[calib] Left  fx={M1[0,0]:.1f} fy={M1[1,1]:.1f} "
          f"cx={M1[0,2]:.1f} cy={M1[1,2]:.1f}")
    print(f"[calib] Right fx={M2[0,0]:.1f} fy={M2[1,1]:.1f} "
          f"cx={M2[0,2]:.1f} cy={M2[1,2]:.1f}")

    if args.expected_dist:
        print(f"[calib] (sanity) Khoảng cách kỳ vọng cam→bàn cờ = "
              f"{args.expected_dist:.0f}mm — kiểm chứng bằng depth sau khi nắn.")

    # Ảnh rectified để mắt thường kiểm tra việc nắn thẳng
    img_l0 = cv2.imread(pairs[0][1])
    img_r0 = cv2.imread(pairs[0][2])
    save_rectified_preview(os.path.join(preview_dir, "rectified.jpg"),
                           img_l0, img_r0, M1, D1, M2, D2, R1, R2, P1, P2, img_size)
    print(f"[calib] Ảnh nắn thẳng + đường epipolar → "
          f"{os.path.join(preview_dir, 'rectified.jpg')}")
    print("        (Nếu nắn ĐÚNG: các góc bàn cờ ở 2 nửa nằm trên CÙNG đường ngang.)")

    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fs = cv2.FileStorage(out, cv2.FILE_STORAGE_WRITE)
    fs.write("rms", rms);          fs.write("baseline", baseline_m)
    fs.write("M1", M1); fs.write("D1", D1)
    fs.write("M2", M2); fs.write("D2", D2)
    fs.write("R", R);   fs.write("T", T)
    fs.write("R1", R1); fs.write("R2", R2)
    fs.write("P1", P1); fs.write("P2", P2)
    fs.write("Q", Q)
    fs.write("image_width",  img_size[0])
    fs.write("image_height", img_size[1])
    fs.release()
    print(f"\n[calib] Đã ghi config → {out}")


if __name__ == "__main__":
    main()
