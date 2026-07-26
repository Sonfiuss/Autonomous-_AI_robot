#!/usr/bin/env python3
"""astra_cloud.py -- XYZ point cloud from an Orbbec Astra Pro depth sensor.

Replaces the manual stereo-matching + DA-V2 mono-scale pipeline
(stereo_cloud.py / stereo_ruler.py) as the PRIMARY depth source. Those files
are kept for reference (legacy) but are no longer the main pipeline.

No colour: Astra Pro does NOT expose RGB through OpenNI2 (the color stream hangs
in read_frame, and hardware depth->color registration returns ONI_STATUS_ERROR).
The cloud is depth-only XYZ, back-projected with a pinhole model whose intrinsics
are self-calibrated at startup from OpenNI2's own
oniCoordinateConverterDepthToWorld (verified correct), so we never call the C
converter 307200x per frame.

-------------------------------------------------------------------- setup ----
Windows (verified 2026-07-23):
  * Driver: obdrv4 v4.3.0.22  (github.com/orbbec/OpenNI_SDK release
    openni-device-windows-driver-4.3.0.22.zip). Already installed.
  * OpenNI2 SDK Windows: unzip OpenNI_2.3.0.86_..._beta6_windows_x64.zip into
    OPENNI2_REDIST below (folder that contains OpenNI2.dll + OpenNI2/ drivers).
  * VC++ 2013 runtime: vcredist_x64.exe (fixes missing MSVCR120.dll).
  * pip install openni       (import: from openni import openni2)
  * MUST call os.add_dll_directory(OPENNI2_REDIST) BEFORE openni2.initialize()
    or LoadLibrary fails on a missing dependency.

Jetson port: use the linux_x64 / arm64 build from the SAME github.com/orbbec/
OpenNI_SDK release. No vcredist needed (Windows-only). Point OPENNI2_REDIST at
the extracted Redist folder (or set the OPENNI2_REDIST env var).

------------------------------------------------------------------- usage -----
  python astra_cloud.py --out cloud.ply              # one frame -> PLY (mm)
  python astra_cloud.py --out cloud.ply --meters     # PLY in metres (like stereo_cloud)
  python astra_cloud.py --display                     # live depth colormap, q to quit
  python astra_cloud.py --display --out cloud.ply     # preview + SPACE saves a frame
"""
import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np

# Folder that holds OpenNI2.dll (+ the OpenNI2/ driver subfolder). Override with
# the OPENNI2_REDIST env var when porting to Jetson.
OPENNI2_REDIST = os.environ.get(
    "OPENNI2_REDIST",
    r"C:\Users\admin\Downloads\openni_sdk\extracted"
    r"\OpenNI_2.3.0.86_202210111950_4c8f5aa4_beta6_windows"
    r"\Win64-Release\sdk\libs",
)

_DEF_OUT = Path(__file__).resolve().parent.parent / "output" / "astra_cloud.ply"


# ------------------------------------------------------------------ OpenNI2 ----
def open_depth_stream():
    """Initialise OpenNI2 and return (openni2, depth_stream). Caller must
    openni2.unload() when done."""
    if hasattr(os, "add_dll_directory") and os.path.isdir(OPENNI2_REDIST):
        os.add_dll_directory(OPENNI2_REDIST)
    from openni import openni2  # imported here so --help works without the SDK

    openni2.initialize(OPENNI2_REDIST)
    dev = openni2.Device.open_any()
    depth = dev.create_depth_stream()
    depth.start()
    return openni2, depth


def read_depth_mm(depth_stream):
    """Grab one depth frame as an HxW uint16 array in millimetres."""
    frame = depth_stream.read_frame()
    h, w = frame.height, frame.width
    buf = frame.get_buffer_as_uint16()
    return np.frombuffer(buf, dtype=np.uint16).reshape(h, w)


# ---------------------------------------------------------- self-calibration ---
def calibrate_intrinsics(openni2, depth_stream, w, h):
    """Solve pinhole fx, fy, cx, cy from OpenNI2's own depth->world converter.

    For a pinhole sensor  X = (u - cx) * z / fx , so at a fixed z the world X is
    linear in u:  X(u) = a*u + b  with a = z/fx, b = -cx*z/fx. Two samples per
    axis pin a, b -> fx, cx (and fy, cy). Verified against the sampled points
    (320,240)->(0,0), (0,0)->(-561,421), (639,479)->(559,-419)."""
    z0 = 1000.0  # mm; any positive depth works, it cancels out of cx/cy
    d2w = openni2.convert_depth_to_world

    def world(u, v):
        return d2w(depth_stream, float(u), float(v), z0)

    x0, _, _ = world(0, h // 2)
    x1, _, _ = world(w - 1, h // 2)
    _, y0, _ = world(w // 2, 0)
    _, y1, _ = world(w // 2, h - 1)

    ax = (x1 - x0) / (w - 1)          # = z0 / fx
    ay = (y1 - y0) / (h - 1)          # = -z0 / fy  (image v grows down, world Y up)
    fx = z0 / ax
    fy = -z0 / ay
    cx = -x0 / ax                      # u where X == 0
    cy = -y0 / ay                      # v where Y == 0
    return fx, fy, cx, cy


def backproject(depth_mm, fx, fy, cx, cy):
    """Vectorised pinhole back-projection. Returns Nx3 float32 XYZ in mm for
    every pixel with depth > 0."""
    h, w = depth_mm.shape
    z = depth_mm.astype(np.float32)
    us, vs = np.meshgrid(np.arange(w, dtype=np.float32),
                         np.arange(h, dtype=np.float32))
    x = (us - cx) * z / fx
    y = -(vs - cy) * z / fy           # flip so world Y points up
    valid = z > 0
    return np.stack([x[valid], y[valid], z[valid]], axis=1)


# -------------------------------------------------------------------- PLY ------
def save_ply_xyz(path, pts):
    """Binary little-endian PLY, xyz only. Mirrors stereo_cloud.save_ply_xyz."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "end_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


def depth_colormap(depth_mm):
    """uint16 mm depth -> BGR colormap for preview."""
    d = depth_mm.astype(np.float32)
    hi = np.percentile(d[d > 0], 99) if np.any(d > 0) else 1.0
    norm = np.clip(d / max(hi, 1.0) * 255.0, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


# -------------------------------------------------------------------- main -----
def main():
    p = argparse.ArgumentParser(
        description="Orbbec Astra Pro depth -> XYZ point cloud (OpenNI2).")
    p.add_argument("--out", default=str(_DEF_OUT),
                   help="output .ply path (xyz only)")
    p.add_argument("--meters", action="store_true",
                   help="write PLY in metres (default mm) to match stereo_cloud")
    p.add_argument("--display", action="store_true",
                   help="live depth colormap window; SPACE saves a frame, q quits")
    args = p.parse_args()

    openni2, depth = open_depth_stream()
    try:
        first = read_depth_mm(depth)
        h, w = first.shape
        fx, fy, cx, cy = calibrate_intrinsics(openni2, depth, w, h)
        print(f"[astra] depth {w}x{h}  intrinsics "
              f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        def dump(depth_mm):
            pts = backproject(depth_mm, fx, fy, cx, cy)
            if args.meters:
                pts = pts / 1000.0
            save_ply_xyz(args.out, pts)
            print(f"[astra] {len(pts)} pts -> {args.out}"
                  f"  ({'m' if args.meters else 'mm'})")

        if not args.display:
            dump(first)
            return

        while True:
            depth_mm = read_depth_mm(depth)
            cv2.imshow("astra depth (SPACE save, q quit)",
                       depth_colormap(depth_mm))
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                dump(depth_mm)
        cv2.destroyAllWindows()
    finally:
        depth.stop()
        openni2.unload()


if __name__ == "__main__":
    sys.exit(main())
