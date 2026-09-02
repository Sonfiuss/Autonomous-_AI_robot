#!/usr/bin/env python3
"""astra_rgbd.py -- pixel-accurate colour registration for Astra Pro depth
(RGB "mode B", replaces the same-pixel approximation "mode A" in astra_slam).

Consumes `astra_calib.npz` produced by astra_calib.py and, per frame, colours
every depth pixel by projecting it into the colour camera:

    depth pixel (u,v,z) -> 3D point in depth frame -> R,T -> colour frame
    -> project with K_color -> sample the (undistorted) colour image.

Everything is vectorised numpy; no Open3D dependency so astra_slam can import
this next to the OpenNI2 front-end.

Convention: camera frame is x right, y DOWN, z forward (standard CV) -- note
astra_cloud.backproject flips Y up for its PLY output; this module does its own
back-projection and never mixes with that flip.

------------------------------------------------------------------- usage -----
  python astra_rgbd.py --preview                     # live alignment check
  python astra_rgbd.py --preview --calib my.npz --rgb-index 1
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

_DEF_CALIB = Path(__file__).resolve().parent.parent / "output" / "astra_calib.npz"

MIN_Z_MM = 1.0          # points closer than this to the colour cam are invalid
BLEND_COLOR = 0.6       # preview overlay: colour weight (depth gets the rest)


# -------------------------------------------------------------------- calib ----
def load_calib(path):
    """Load astra_calib.npz -> dict of plain numpy values."""
    d = np.load(path)
    return {
        "fx_d": float(d["fx_d"]), "fy_d": float(d["fy_d"]),
        "cx_d": float(d["cx_d"]), "cy_d": float(d["cy_d"]),
        "K_c": d["K_c"].astype(np.float64),
        "dist_c": d["dist_c"].astype(np.float64),
        "R": d["R"].astype(np.float64),
        "T_mm": d["T_mm"].reshape(3).astype(np.float64),
    }


# ---------------------------------------------------------------- alignment ----
def _undistort_cached(color_bgr, calib):
    """Undistort via precomputed remap tables. cv2.undistort would rebuild the
    tables EVERY frame; building them once (per image size) keeps the per-frame
    cost to a single remap. Tables are cached inside the calib dict."""
    key = color_bgr.shape[:2]
    maps = calib.setdefault("_undistort_maps", {})
    if key not in maps:
        h, w = key
        maps[key] = cv2.initUndistortRectifyMap(
            calib["K_c"], calib["dist_c"], None, calib["K_c"], (w, h),
            cv2.CV_16SC2)
    m1, m2 = maps[key]
    return cv2.remap(color_bgr, m1, m2, cv2.INTER_LINEAR)


def align_color_to_depth(depth_mm, color_bgr, calib):
    """Colour image re-sampled onto the depth grid (HxWx3 uint8).

    Pixels with no depth, or that project outside the colour image, stay black.
    The colour frame is undistorted first so the projection is pure pinhole.
    Occlusion (depth point hidden from the colour camera) is ignored -- the
    depth<->colour baseline is a few cm, the error is not visible in practice."""
    h, w = depth_mm.shape
    und = _undistort_cached(color_bgr, calib)
    ch, cw = und.shape[:2]
    aligned = np.zeros((h, w, 3), np.uint8)

    valid = depth_mm > 0
    if not np.any(valid):
        return aligned
    vs, us = np.nonzero(valid)
    z = depth_mm[vs, us].astype(np.float64)               # mm
    x = (us - calib["cx_d"]) * z / calib["fx_d"]          # y DOWN convention
    y = (vs - calib["cy_d"]) * z / calib["fy_d"]
    pts = np.stack([x, y, z], axis=1) @ calib["R"].T + calib["T_mm"]

    zc = pts[:, 2]
    front = zc > MIN_Z_MM                                  # behind-cam guard
    K = calib["K_c"]
    uc = np.round(K[0, 0] * pts[:, 0] / zc + K[0, 2]).astype(np.int64)
    vc = np.round(K[1, 1] * pts[:, 1] / zc + K[1, 2]).astype(np.int64)
    inb = front & (uc >= 0) & (uc < cw) & (vc >= 0) & (vc < ch)
    aligned[vs[inb], us[inb]] = und[vc[inb], uc[inb]]
    return aligned


# ------------------------------------------------------------------ preview ----
def preview(calib_path, rgb_index):
    """Live visual check: aligned colour blended over the depth colormap.
    Edges of objects must coincide; q quits."""
    from astra_cloud import (open_depth_stream, read_depth_mm, depth_colormap)

    calib = load_calib(calib_path)
    openni2, depth = open_depth_stream()
    cap = cv2.VideoCapture(rgb_index, cv2.CAP_DSHOW)
    ok, _ = cap.read()
    if not ok:
        raise RuntimeError(f"colour index {rgb_index} gave no frame")
    try:
        while True:
            d = read_depth_mm(depth)
            ok, col = cap.read()
            if not ok:
                continue
            aligned = align_color_to_depth(d, col, calib)
            blend = cv2.addWeighted(aligned, BLEND_COLOR,
                                    depth_colormap(d), 1.0 - BLEND_COLOR, 0)
            cv2.imshow("astra_rgbd preview (aligned colour + depth, q quits)",
                       blend)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
        cv2.destroyAllWindows()
    finally:
        cap.release()
        depth.stop()
        openni2.unload()


# -------------------------------------------------------------------- main -----
def main():
    p = argparse.ArgumentParser(
        description="Pixel-accurate Astra colour<->depth registration (mode B).")
    p.add_argument("--calib", default=str(_DEF_CALIB),
                   help="astra_calib.npz from astra_calib.py")
    p.add_argument("--preview", action="store_true",
                   help="live blended alignment check window")
    p.add_argument("--rgb-index", type=int, default=0,
                   help="cv2 camera index of the Astra colour stream")
    args = p.parse_args()

    if args.preview:
        preview(args.calib, args.rgb_index)
    else:
        p.error("nothing to do -- use --preview (astra_slam imports the rest)")


if __name__ == "__main__":
    sys.exit(main())
