#!/usr/bin/env python3
"""
capture_checkerboard.py  –  Chụp cặp ảnh bàn cờ (stereo) + calibrate ngay (CLI).

Khác stereo_calibrate.py: tool này LƯU ẢNH GỐC left/right ra đĩa (tool cũ chỉ
giữ corners trong RAM, mất khi tắt). Dùng cửa sổ cv2.imshow local, không cần browser.

Usage:
  python3 tools/capture_checkerboard.py [options]

Options:
  --rows   INT    Inner corners dọc        (default 6)
  --cols   INT    Inner corners ngang      (default 9)
  --square FLOAT  Kích thước ô vuông, mm   (default 25.0)
  --left   INT    Left  camera index       (default 0)
  --right  INT    Right camera index       (default 2)
  --width  INT    Capture width            (default 1280)
  --height INT    Capture height           (default 960)
  --min-pairs INT Số cặp tối thiểu để calibrate (default 15)
  --img-dir PATH  Thư mục lưu ảnh gốc      (default captures/checkerboard)
  --out    PATH   File calib xuất ra       (default calib/stereo.yml)

Phím trong cửa sổ:
  q (hoặc SPACE) : chụp cặp hiện tại (chỉ khi cả 2 cam thấy bàn cờ & ảnh nét)
  c              : calibrate với các cặp đã chụp (>= min-pairs)
  r              : xoá tất cả cặp đã chụp trong session
  ESC            : thoát

Quy trình:
  1. In checkerboard (vd 9x6 inner corners = 10x7 ô). Đặt --square đúng kích thước ô.
  2. Đưa bàn cờ trước 2 cam → border XANH = cả 2 detect được → bấm SPACE.
  3. Đổi góc/khoảng cách/vị trí mỗi lần, thu 15-25 tư thế.
  4. Bấm c để calibrate → file lưu vào calib/stereo.yml.

Output (tương thích StereoCamera::loadCalibration):
  M1,D1  M2,D2  R,T  R1,R2,P1,P2,Q  baseline  image_width/height
Ảnh gốc: <img-dir>/<YYYYMMDD_HHMMSS>/left_NN.jpg, right_NN.jpg
"""

import argparse
import datetime
import os
import sys

import cv2
import numpy as np

# Console Windows mặc định cp1258 → in tiếng Việt sẽ crash. Ép UTF-8.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
_MIN_SHARPNESS = 60.0   # Laplacian variance; dưới ngưỡng = quá mờ


def parse_args():
    p = argparse.ArgumentParser(description="Chụp bàn cờ stereo + calibrate (CLI)")
    p.add_argument("--rows",   type=int,   default=6)
    p.add_argument("--cols",   type=int,   default=9)
    p.add_argument("--square", type=float, default=25.0, help="Kích thước ô, mm")
    p.add_argument("--left",   type=int,   default=0)
    p.add_argument("--right",  type=int,   default=2)
    p.add_argument("--width",  type=int,   default=1280)
    p.add_argument("--height", type=int,   default=960)
    p.add_argument("--min-pairs", type=int, default=15)
    p.add_argument("--img-dir", default="captures/checkerboard")
    p.add_argument("--out",     default="calib/stereo.yml")
    p.add_argument("--backend", default="auto",
                   choices=["auto", "dshow", "msmf", "v4l2", "any"],
                   help="Backend camera. Windows nên thử 'dshow' nếu 2 cam không "
                        "mở đồng thời được.")
    p.add_argument("--test", action="store_true",
                   help="Chế độ kiểm tra: đọc từng cam rồi cả 2, không cần bàn cờ.")
    return p.parse_args()


# Thứ tự backend thử khi --backend auto (dshow trước vì cho phép nhiều webcam
# chạy song song tốt hơn msmf/ffmpeg trên Windows).
_BACKENDS = {
    "dshow": cv2.CAP_DSHOW,
    "msmf":  cv2.CAP_MSMF,
    "v4l2":  cv2.CAP_V4L2,
    "any":   cv2.CAP_ANY,
}


def _backend_order(name):
    if name == "auto":
        if sys.platform.startswith("win"):
            return [("dshow", cv2.CAP_DSHOW), ("msmf", cv2.CAP_MSMF),
                    ("any", cv2.CAP_ANY)]
        return [("v4l2", cv2.CAP_V4L2), ("any", cv2.CAP_ANY)]
    return [(name, _BACKENDS[name])]


def open_camera(idx, w, h, backend="auto"):
    """Mở camera, thử lần lượt các backend cho tới khi đọc được 1 frame."""
    for bname, bflag in _backend_order(backend):
        cap = cv2.VideoCapture(idx, bflag)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, 15)
            ok, _ = cap.read()
            if ok:
                print(f"[cap] camera {idx} mở OK qua backend '{bname}' "
                      f"({int(cap.get(3))}x{int(cap.get(4))})")
                return cap
            cap.release()
            print(f"[cap] camera {idx}: backend '{bname}' mở nhưng không đọc được frame.")
        # backend này fail → thử backend kế
    print(f"[cap] ERROR: không mở được camera index {idx} (đã thử mọi backend)")
    return None


def run_camera_test(left, right, w, h, backend):
    """Đọc từng camera riêng rồi cả 2 cùng lúc — báo cáo + hiện cửa sổ live."""
    print("\n=== CAMERA TEST ===")
    for name, idx in (("LEFT", left), ("RIGHT", right)):
        cap = open_camera(idx, w, h, backend)
        if cap is None:
            print(f"[test] {name} (idx {idx}): FAIL mở")
            continue
        ok, frame = cap.read()
        shp = frame.shape if ok and frame is not None else None
        print(f"[test] {name} (idx {idx}): read={ok} shape={shp}")
        cap.release()

    print("[test] Mở CẢ HAI cùng lúc...")
    cap_l = open_camera(left,  w, h, backend)
    cap_r = open_camera(right, w, h, backend)
    if cap_l is None or cap_r is None:
        print("[test] ✗ Không mở được đồng thời 2 cam. "
              "Thử --backend dshow, hoặc cắm 2 cam vào 2 cổng USB khác controller.")
        if cap_l: cap_l.release()
        if cap_r: cap_r.release()
        return
    print("[test] ✓ Cả 2 cam mở đồng thời OK. Hiện cửa sổ live — bấm ESC để thoát.")
    win = "CAMERA TEST  left | right   [ESC] thoat"
    while True:
        okl, fl = cap_l.read()
        okr, fr = cap_r.read()
        if not (okl and okr):
            print(f"[test] read L={okl} R={okr} (mất frame)")
            continue
        if fl.shape != fr.shape:
            fr = cv2.resize(fr, (fl.shape[1], fl.shape[0]))
        combined = np.hstack([fl, fr])
        if combined.shape[1] > 1600:
            s = 1600.0 / combined.shape[1]
            combined = cv2.resize(combined, None, fx=s, fy=s)
        cv2.imshow(win, combined)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break
    cap_l.release(); cap_r.release()
    cv2.destroyAllWindows()
    print("[test] Kết thúc test.\n")


def make_objp(rows, cols, square_mm):
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_mm / 1000.0   # mm → m
    return objp


def find_corners(gray, pattern):
    ret, corners = cv2.findChessboardCorners(
        gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ret:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRIT)


def sharpness(gray):
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def calibrate(pairs, img_size, rows, cols, square, outpath):
    if len(pairs) < 1:
        print("[calib] Không có cặp nào.")
        return
    objp    = make_objp(rows, cols, square)
    obj_pts = [objp] * len(pairs)
    img_l   = [p[0] for p in pairs]
    img_r   = [p[1] for p in pairs]

    print(f"\n[calib] Calibrating với {len(pairs)} cặp ...")
    rms_l, M1, D1, _, _ = cv2.calibrateCamera(obj_pts, img_l, img_size, None, None)
    rms_r, M2, D2, _, _ = cv2.calibrateCamera(obj_pts, img_r, img_size, None, None)
    print(f"[calib] Left RMS={rms_l:.4f}px  Right RMS={rms_r:.4f}px")

    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-6)
    rms, M1, D1, M2, D2, R, T, E, F = cv2.stereoCalibrate(
        obj_pts, img_l, img_r, M1, D1, M2, D2, img_size,
        criteria=crit, flags=cv2.CALIB_FIX_INTRINSIC)
    print(f"[calib] Stereo RMS={rms:.4f}px")

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        M1, D1, M2, D2, img_size, R, T, alpha=0, newImageSize=img_size)

    baseline_m = float(np.linalg.norm(T))
    print(f"[calib] baseline={baseline_m*1000:.2f}mm")
    print(f"[calib] Left  fx={M1[0,0]:.1f} fy={M1[1,1]:.1f} "
          f"cx={M1[0,2]:.1f} cy={M1[1,2]:.1f}")
    print(f"[calib] Right fx={M2[0,0]:.1f} fy={M2[1,1]:.1f}")

    os.makedirs(os.path.dirname(os.path.abspath(outpath)), exist_ok=True)
    fs = cv2.FileStorage(outpath, cv2.FILE_STORAGE_WRITE)
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
    print(f"[calib] Đã lưu → {outpath}\n")


def main():
    args = parse_args()
    pattern = (args.cols, args.rows)   # OpenCV: (width=cols, height=rows)
    img_size = (args.width, args.height)

    if args.test:
        run_camera_test(args.left, args.right, args.width, args.height, args.backend)
        return

    cap_l = open_camera(args.left,  args.width, args.height, args.backend)
    cap_r = open_camera(args.right, args.width, args.height, args.backend)
    if cap_l is None or cap_r is None:
        if cap_l: cap_l.release()
        if cap_r: cap_r.release()
        print("[cap] Gợi ý: chạy '--test' để kiểm tra từng cam, "
              "hoặc thử '--backend dshow'.")
        sys.exit(1)

    session = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir = os.path.join(args.img_dir, session)
    os.makedirs(save_dir, exist_ok=True)

    print(f"[cap] Board {args.cols}x{args.rows} inner corners, square={args.square}mm")
    print(f"[cap] Capture {args.width}x{args.height}  cam L={args.left} R={args.right}")
    print(f"[cap] Ảnh lưu vào: {save_dir}")
    print(f"[cap] q=chụp  c=calibrate  r=reset  ESC=thoát\n")

    pairs = []          # (corners_full_l, corners_full_r) — chỉ để calibrate nếu muốn
    saved = 0           # tổng số cặp ảnh đã lưu (kể cả chưa thấy bàn cờ)
    font = cv2.FONT_HERSHEY_SIMPLEX
    win = "Checkerboard capture  [q]chup [c]calibrate [r]reset [ESC]thoat"

    while True:
        ok_l = cap_l.grab(); ok_r = cap_r.grab()
        if not (ok_l and ok_r):
            continue
        _, frame_l = cap_l.retrieve()
        _, frame_r = cap_r.retrieve()
        if frame_l is None or frame_r is None:
            continue

        gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)
        c_l = find_corners(gray_l, pattern)
        c_r = find_corners(gray_r, pattern)
        found = (c_l is not None) and (c_r is not None)
        sharp = min(sharpness(gray_l), sharpness(gray_r))

        disp_l = frame_l.copy(); disp_r = frame_r.copy()
        cv2.drawChessboardCorners(disp_l, pattern, c_l, c_l is not None)
        cv2.drawChessboardCorners(disp_r, pattern, c_r, c_r is not None)

        # Border chỉ để gợi ý: xanh = thấy bàn cờ rõ. Bấm q luôn chụp được.
        if not found:
            bcolor, status = (0, 165, 255), "chua thay ban co (van chup duoc)"
        elif sharp < _MIN_SHARPNESS:
            bcolor, status = (0, 200, 255), "thay ban co - hoi mo"
        else:
            bcolor, status = (0, 220, 0), "thay ban co - tot"

        for d in (disp_l, disp_r):
            cv2.rectangle(d, (0, 0), (d.shape[1]-1, d.shape[0]-1), bcolor, 6)
        cv2.putText(disp_l, f"L {status}", (12, 34), font, 0.7, bcolor, 2)
        cv2.putText(disp_r, f"R sharp={sharp:.0f}", (12, 34), font, 0.8, bcolor, 2)
        cv2.putText(disp_l, f"da chup: {saved}   [q]chup [ESC]thoat", (12, 70),
                    font, 0.7, (60, 220, 220), 2)

        combined = np.hstack([disp_l, disp_r])
        # Thu nhỏ để vừa màn hình nếu native res lớn
        if combined.shape[1] > 1600:
            scale = 1600.0 / combined.shape[1]
            combined = cv2.resize(combined, None, fx=scale, fy=scale)
        cv2.imshow(win, combined)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:               # ESC = thoát
            break
        elif key in (ord('q'), ord(' ')):   # q (hoặc SPACE) = chụp
            # Chỉ chụp lấy ảnh — LUÔN lưu, không bắt buộc thấy bàn cờ / không chặn mờ.
            n = saved
            pl = os.path.join(save_dir, f"left_{n:02d}.jpg")
            pr = os.path.join(save_dir, f"right_{n:02d}.jpg")
            cv2.imwrite(pl, frame_l)
            cv2.imwrite(pr, frame_r)
            saved += 1
            warn = "" if found else "  (CHUA thay ban co!)"
            if sharp < _MIN_SHARPNESS:
                warn += "  (anh hoi mo)"
            if found:                       # giữ corners để calibrate nếu muốn
                pairs.append((c_l, c_r))
            print(f"[cap] Chụp #{saved} → {pl} , {pr}{warn}")
        elif key == ord('r'):
            pairs.clear()
            print("[cap] Đã xoá danh sách cặp (ảnh đã lưu vẫn còn trên đĩa).")
        elif key == ord('c'):
            if len(pairs) < args.min_pairs:
                print(f"[cap] Cần >= {args.min_pairs} cặp (hiện {len(pairs)}).")
            else:
                calibrate(pairs, img_size, args.rows, args.cols,
                          args.square, args.out)

    cap_l.release(); cap_r.release()
    cv2.destroyAllWindows()
    print(f"[cap] Kết thúc. Đã lưu {saved} cặp ảnh ở {save_dir}")


if __name__ == "__main__":
    main()
