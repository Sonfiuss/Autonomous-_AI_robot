#!/usr/bin/env python3
"""astra_probe.py -- click-to-measure depth points on the Astra Pro live view.

Left-click a pixel on the live depth colormap to measure its depth (median of
a 5x5 window, zero/hole pixels ignored) and pin a `Pn: <mm>` label there. The
3D position is also computed with the same self-calibrated pinhole model as
astra_cloud, so saved points can be compared against exported point clouds.

Keys:
  left-click  measure + label a point
  SPACE       freeze / unfreeze the live frame (click precisely on a still)
  s           save annotated PNG + JSON to depth-anything/output/probe/
  c           clear all points
  q           quit

Output JSON per save: intrinsics + list of points
  {id, u, v, depth_mm, x_mm, y_mm, z_mm}   (y up, same frame as astra_cloud)
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from astra_cloud import (calibrate_intrinsics, depth_colormap,
                         open_depth_stream, read_depth_mm)

_DEF_OUT_DIR = Path(__file__).resolve().parent.parent / "output" / "probe"
PROBE_WIN = 5           # side of the median window around a click, pixels
WINDOW_NAME = "astra probe (click=measure, SPACE=freeze, s=save, c=clear, q=quit)"
LABEL_OFFSET_PX = 8     # label anchor offset from the marker dot
LABEL_MARGIN_PX = 130   # right margin so the label never runs off-image
LABEL_TOP_PX = 16       # minimum v so the label stays below the image top


# ------------------------------------------------------------- measurement ----
def measure_depth_mm(depth_mm, u, v, win=PROBE_WIN):
    """Median depth (mm) of the win x win window centred on (u, v), ignoring
    zero (hole) pixels. Returns 0 when the whole window is holes."""
    h, w = depth_mm.shape
    r = win // 2
    patch = depth_mm[max(0, v - r):min(h, v + r + 1),
                     max(0, u - r):min(w, u + r + 1)]
    valid = patch[patch > 0]
    if valid.size == 0:
        return 0
    return int(np.median(valid))


def pixel_to_xyz_mm(u, v, z_mm, fx, fy, cx, cy):
    """Back-project one pixel to XYZ mm (y up), same model as astra_cloud."""
    x = (u - cx) * z_mm / fx
    y = -(v - cy) * z_mm / fy
    return float(x), float(y), float(z_mm)


# --------------------------------------------------------------- rendering ----
def draw_points(img, points):
    """Draw a marker + `Pn: <mm>` label for every measured point on img."""
    for pt in points:
        u, v = pt["u"], pt["v"]
        cv2.circle(img, (u, v), 4, (255, 255, 255), -1)
        cv2.circle(img, (u, v), 5, (0, 0, 0), 1)
        label = f"P{pt['id']}: {pt['depth_mm']}mm"
        tu = min(u + LABEL_OFFSET_PX, img.shape[1] - LABEL_MARGIN_PX)
        tv = max(v - LABEL_OFFSET_PX, LABEL_TOP_PX)
        cv2.putText(img, label, (tu, tv), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, label, (tu, tv), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ------------------------------------------------------------------ saving ----
def save_outputs(out_dir, annotated_bgr, points, intrinsics, resolution):
    """Write probe_<ts>.png (annotated view) + probe_<ts>.json (point data)."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    png_path = out_dir / f"probe_{ts}.png"
    json_path = out_dir / f"probe_{ts}.json"

    if not cv2.imwrite(str(png_path), annotated_bgr):
        print(f"[probe] ERROR: failed to write {png_path}")
    fx, fy, cx, cy = intrinsics
    payload = {
        "timestamp": ts,
        "resolution": {"width": resolution[0], "height": resolution[1]},
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "units": "mm",
        "points": points,
    }
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    print(f"[probe] saved {len(points)} pts -> {png_path.name} + {json_path.name}")


# -------------------------------------------------------------------- main ----
def main():
    p = argparse.ArgumentParser(
        description="Click-to-measure depth points on the Astra Pro live view.")
    p.add_argument("--out-dir", default=str(_DEF_OUT_DIR),
                   help="folder for probe_<ts>.png/.json saves")
    args = p.parse_args()

    openni2, depth = open_depth_stream()
    try:
        first = read_depth_mm(depth)
        h, w = first.shape
        intrinsics = calibrate_intrinsics(openni2, depth, w, h)
        fx, fy, cx, cy = intrinsics
        print(f"[probe] depth {w}x{h}  intrinsics "
              f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}")

        state = {"depth_mm": first, "base": depth_colormap(first),
                 "points": [], "frozen": False}

        def on_mouse(event, u, v, flags, _param):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            z = measure_depth_mm(state["depth_mm"], u, v)
            if z == 0:
                print(f"[probe] ({u},{v}): no depth (hole), point skipped")
                return
            x, y, _ = pixel_to_xyz_mm(u, v, z, fx, fy, cx, cy)
            pt = {"id": len(state["points"]) + 1, "u": u, "v": v,
                  "depth_mm": z, "x_mm": round(x, 1), "y_mm": round(y, 1),
                  "z_mm": z}
            state["points"].append(pt)
            print(f"[probe] P{pt['id']} ({u},{v}) depth={z}mm "
                  f"xyz=({pt['x_mm']}, {pt['y_mm']}, {z})mm")

        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, on_mouse)

        while True:
            if not state["frozen"]:
                state["depth_mm"] = read_depth_mm(depth)
                state["base"] = depth_colormap(state["depth_mm"])
            view = draw_points(state["base"].copy(), state["points"])
            if state["frozen"]:
                cv2.putText(view, "FROZEN", (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW_NAME, view)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord(" "):
                state["frozen"] = not state["frozen"]
            elif k == ord("c"):
                state["points"] = []
                print("[probe] points cleared")
            elif k == ord("s"):
                if state["points"]:
                    # re-annotate a clean base so the FROZEN overlay never
                    # ends up inside the saved image
                    annotated = draw_points(state["base"].copy(),
                                            state["points"])
                    save_outputs(args.out_dir, annotated, state["points"],
                                 intrinsics, (w, h))
                else:
                    print("[probe] nothing to save, click a point first")
        cv2.destroyAllWindows()
    finally:
        depth.stop()
        openni2.unload()


if __name__ == "__main__":
    sys.exit(main())
