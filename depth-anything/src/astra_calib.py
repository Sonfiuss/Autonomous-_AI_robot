#!/usr/bin/env python3
"""astra_calib.py -- depth<->color extrinsic calibration for the Astra Pro
(RGB "mode B": pixel-accurate colour registration for astra_slam).

The Astra Pro exposes depth+IR through OpenNI2 (SAME physical sensor, so the
self-calibrated depth intrinsics ARE the IR intrinsics) and colour through a
separate UVC webcam (cv2). There is no hardware registration between them, so
we solve it once here with a checkerboard seen by both cameras:

    1. --capture  : save paired IR + colour checkerboard images
    2. --calibrate: cv2.calibrateCamera (colour intrinsics) +
                    cv2.stereoCalibrate (fixed intrinsics both sides)
                    -> astra_calib.npz  (K_color, dist_color, R, T depth->color)

astra_rgbd.py consumes astra_calib.npz to colour each depth pixel exactly.

IMPORTANT during --capture: cover the Astra's laser projector (small round
window) with opaque tape, otherwise the IR speckle pattern is painted over the
checkerboard and corner detection fails. Light the board with a lamp instead
(incandescent/halogen shows up brightest in IR).

------------------------------------------------------------------- usage -----
  python astra_calib.py --capture calib/ --rgb-index 0        # SPACE saves a pair
  python astra_calib.py --calibrate calib/ --cols 9 --rows 6 --square-mm 25
  python astra_calib.py --calibrate calib/ --out ../output/astra_calib.npz
"""
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# Reuse the OpenNI2 redist path + depth self-calibration (no modification).
from astra_cloud import OPENNI2_REDIST, calibrate_intrinsics, read_depth_mm

_DEF_CALIB = Path(__file__).resolve().parent.parent / "output" / "astra_calib.npz"

DEPTH_INTR_FILE = "depth_intr.npz"  # written by --capture, read by --calibrate
MIN_PAIRS = 5           # fewer usable checkerboard pairs than this -> abort
RMS_WARN_PX = 0.5       # stereo reprojection error above this -> warn user


# ------------------------------------------------------------------ OpenNI2 ----
def open_device():
    """Initialise OpenNI2 and return (openni2, device). Caller unloads."""
    if hasattr(os, "add_dll_directory") and os.path.isdir(OPENNI2_REDIST):
        os.add_dll_directory(OPENNI2_REDIST)
    from openni import openni2

    openni2.initialize(OPENNI2_REDIST)
    return openni2, openni2.Device.open_any()


def read_ir_u8(ir_stream):
    """One IR frame (Gray16) normalised to uint8 for display + corner detection."""
    frame = ir_stream.read_frame()
    h, w = frame.height, frame.width
    buf = frame.get_buffer_as_uint16()
    ir = np.frombuffer(buf, dtype=np.uint16).reshape(h, w).astype(np.float32)
    hi = max(float(np.percentile(ir, 99)), 1.0)
    return np.clip(ir / hi * 255.0, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------ capture ----
def capture(out_dir, rgb_index, n_pairs):
    """Interactive capture: IR (OpenNI2) + colour (cv2) side by side.
    SPACE saves a pair, q quits early. Depth intrinsics are self-calibrated
    FIRST (depth stream), then the sensor is switched to IR (they cannot run
    simultaneously on the Astra)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    openni2, dev = open_device()
    ir = None
    cap = None
    try:
        # 1. depth intrinsics == IR intrinsics (same sensor behind both streams)
        depth = dev.create_depth_stream()
        depth.start()
        first = read_depth_mm(depth)
        h, w = first.shape
        fx, fy, cx, cy = calibrate_intrinsics(openni2, depth, w, h)
        depth.stop()
        np.savez(out / DEPTH_INTR_FILE, fx=fx, fy=fy, cx=cx, cy=cy, w=w, h=h)
        print(f"[calib] depth/IR intrinsics fx={fx:.1f} fy={fy:.1f} "
              f"cx={cx:.1f} cy={cy:.1f} -> {out / DEPTH_INTR_FILE}")

        # 2. switch the sensor to IR
        ir = dev.create_ir_stream()
        ir.start()

        # 3. colour via UVC
        cap = cv2.VideoCapture(rgb_index, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        ok, _ = cap.read()
        if not ok:
            raise RuntimeError(f"colour index {rgb_index} gave no frame -- "
                               "try a different --rgb-index")

        print(f"[calib] SPACE = save pair ({n_pairs} wanted), q = quit. "
              "COVER THE LASER PROJECTOR with tape!")
        saved = 0
        while saved < n_pairs:
            ir_u8 = read_ir_u8(ir)
            ok, col = cap.read()
            if not ok:
                print("[calib] WARN colour frame dropped")
                continue
            view = np.hstack([cv2.cvtColor(ir_u8, cv2.COLOR_GRAY2BGR),
                              cv2.resize(col, (ir_u8.shape[1], ir_u8.shape[0]))])
            cv2.putText(view, f"pair {saved}/{n_pairs}  SPACE=save q=quit",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("astra_calib capture (IR | colour)", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                ok_ir = cv2.imwrite(str(out / f"ir_{saved:02d}.png"), ir_u8)
                ok_col = cv2.imwrite(str(out / f"color_{saved:02d}.png"), col)
                if not (ok_ir and ok_col):
                    print("[calib] WARN failed to write pair -- check disk/"
                          "permissions, pair not counted")
                    continue
                saved += 1
                print(f"[calib] saved pair {saved}/{n_pairs}")
        cv2.destroyAllWindows()
        print(f"[calib] done -> {saved} pairs in {out}")
    finally:
        if cap is not None:
            cap.release()
        if ir is not None:
            ir.stop()
        openni2.unload()


# ---------------------------------------------------------------- calibrate ----
def find_corners(img_gray, cols, rows):
    """Checkerboard inner-corner detection + subpixel refine. None if not found."""
    flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    found, corners = cv2.findChessboardCorners(img_gray, (cols, rows), flags)
    if not found:
        return None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    return cv2.cornerSubPix(img_gray, corners, (11, 11), (-1, -1), term)


def calibrate(in_dir, out_path, cols, rows, square_mm):
    """Solve colour intrinsics + depth->colour extrinsics from captured pairs.
    T is stored in MILLIMETRES (same unit as the depth stream)."""
    in_dir = Path(in_dir)
    if not (in_dir / DEPTH_INTR_FILE).exists():
        raise RuntimeError(f"{in_dir / DEPTH_INTR_FILE} missing -- run "
                           "--capture first (it saves the depth intrinsics)")
    intr = np.load(in_dir / DEPTH_INTR_FILE)
    K_ir = np.array([[float(intr["fx"]), 0, float(intr["cx"])],
                     [0, float(intr["fy"]), float(intr["cy"])],
                     [0, 0, 1]], dtype=np.float64)
    dist_ir = np.zeros(5)  # OpenNI2's converter is already an undistorted pinhole

    # One checkerboard model in mm -> stereoCalibrate T comes out in mm.
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    obj_pts, ir_pts, col_pts = [], [], []
    col_size = None
    ir_size = (int(intr["w"]), int(intr["h"]))
    for ir_file in sorted(in_dir.glob("ir_*.png")):
        col_file = in_dir / ir_file.name.replace("ir_", "color_")
        if not col_file.exists():
            continue
        ir_img = cv2.imread(str(ir_file), cv2.IMREAD_GRAYSCALE)
        col_img = cv2.imread(str(col_file))
        col_gray = cv2.cvtColor(col_img, cv2.COLOR_BGR2GRAY)
        c_ir = find_corners(ir_img, cols, rows)
        c_col = find_corners(col_gray, cols, rows)
        status = ("OK" if c_ir is not None and c_col is not None else
                  f"skip (ir={'y' if c_ir is not None else 'N'} "
                  f"col={'y' if c_col is not None else 'N'})")
        print(f"[calib] {ir_file.name}: {status}")
        if c_ir is None or c_col is None:
            continue
        obj_pts.append(obj)
        ir_pts.append(c_ir)
        col_pts.append(c_col)
        col_size = (col_img.shape[1], col_img.shape[0])

    if len(obj_pts) < MIN_PAIRS:
        raise RuntimeError(
            f"only {len(obj_pts)} usable pairs (< {MIN_PAIRS}). Recapture "
            "with the projector covered + better lighting.")

    # Colour camera intrinsics on its own images.
    rms_c, K_c, dist_c, _, _ = cv2.calibrateCamera(
        obj_pts, col_pts, col_size, None, None)
    print(f"[calib] colour intrinsics rms={rms_c:.3f}px  "
          f"fx={K_c[0,0]:.1f} fy={K_c[1,1]:.1f} "
          f"cx={K_c[0,2]:.1f} cy={K_c[1,2]:.1f}")

    # Extrinsics only: both intrinsics fixed. R,T map a point in the IR/depth
    # frame into the colour camera frame: p_c = R @ p_d + T   (T in mm).
    rms_s, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        obj_pts, ir_pts, col_pts, K_ir, dist_ir, K_c, dist_c, ir_size,
        flags=cv2.CALIB_FIX_INTRINSIC)
    print(f"[calib] stereo rms={rms_s:.3f}px  baseline={np.linalg.norm(T):.1f}mm"
          f"  T(mm)={T.ravel().round(1)}")
    if rms_s > RMS_WARN_PX:
        print(f"[calib] WARN stereo rms > {RMS_WARN_PX}px target -- colours "
              "may bleed at edges; recapture with more/better pairs if visible.")

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out_path,
             fx_d=float(intr["fx"]), fy_d=float(intr["fy"]),
             cx_d=float(intr["cx"]), cy_d=float(intr["cy"]),
             K_c=K_c, dist_c=dist_c, R=R, T_mm=T,
             rms_color=rms_c, rms_stereo=rms_s,
             depth_size=ir_size, color_size=col_size)
    print(f"[calib] -> {out_path}")


# -------------------------------------------------------------------- main -----
def main():
    p = argparse.ArgumentParser(
        description="Astra Pro depth<->colour calibration (RGB mode B).")
    p.add_argument("--capture", metavar="DIR",
                   help="save paired IR+colour checkerboard images to DIR")
    p.add_argument("--calibrate", metavar="DIR",
                   help="solve calibration from pairs previously saved in DIR")
    p.add_argument("--out", default=str(_DEF_CALIB),
                   help="output .npz for --calibrate")
    p.add_argument("--rgb-index", type=int, default=0,
                   help="cv2 camera index of the Astra colour stream")
    p.add_argument("--pairs", type=int, default=20,
                   help="number of pairs to capture")
    p.add_argument("--cols", type=int, default=9,
                   help="checkerboard INNER corners per row")
    p.add_argument("--rows", type=int, default=6,
                   help="checkerboard INNER corners per column")
    p.add_argument("--square-mm", type=float, default=25.0,
                   help="checkerboard square size in mm")
    args = p.parse_args()

    if args.capture:
        capture(args.capture, args.rgb_index, args.pairs)
    elif args.calibrate:
        calibrate(args.calibrate, args.out, args.cols, args.rows, args.square_mm)
    else:
        p.error("need --capture DIR or --calibrate DIR")


if __name__ == "__main__":
    sys.exit(main())
