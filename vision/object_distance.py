"""Distance and 3D position of a detected object from registered depth. Pure: no camera, no model.

A bbox is a rectangle, so it always contains more than the object: background through a chair's
back, the floor under a cup, wall beside a bottle. Taking the depth at the bbox center, or the
median over the whole bbox, lands on that background often enough to be useless. Instead:
1. keep only the central CORE_FRACTION of the box (edges are where the background is);
2. split its valid depths into clusters at depth gaps;
3. take the NEAREST cluster that holds at least MIN_CLUSTER_FRACTION of the valid pixels.
The object is normally the nearest substantial surface inside its own box. The support
threshold keeps a few speckle pixels in front of it from being mistaken for it.

Camera frame: +x right, +y down, +z forward along the optical axis (OpenCV convention), meters.
"""
import collections
import json
import math
import os

import numpy as np

CORE_FRACTION = 0.5            # central share of bbox width/height that is sampled
MIN_VALID_FRACTION = 0.3       # below this share of valid core pixels -> no reading
MIN_CLUSTER_FRACTION = 0.2     # a cluster needs this share of the valid pixels to count as a surface
CLUSTER_GAP_MIN_MM = 100       # a jump larger than max(this, CLUSTER_GAP_REL * depth) splits clusters
CLUSTER_GAP_REL = 0.05
OBJECT_BAND_MIN_MM = 150       # object_mask: within max(this, OBJECT_BAND_REL * depth) of the surface
OBJECT_BAND_REL = 0.10
RECORD_DIGITS = 3              # result_records: metres and confidences (mm, 0.001)
BOX_DIGITS = 1                 # result_records: bbox pixels
RANGE_FROM_DEPTH = "depth"
RANGE_FROM_FLOOR = "floor"
# Written by calib_floor.py when its marks pin fx down; the default --intrinsics of the recorder.
CAMERA_RGB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "camera_rgb.json")
# RGB intrinsics when nothing better is known: the FOV OpenNI2 reported for this camera on the dev
# laptop (vision/README.md, fx ~ fy ~ 570 at 640x480).
FALLBACK_FOV_DEG = (58.59, 45.64)

Intrinsics = collections.namedtuple("Intrinsics", "fx fy cx cy")
# range_m: Euclidean camera->object distance. z_m: depth along the optical axis.
ObjectRange = collections.namedtuple("ObjectRange", "range_m z_m xyz valid_fraction")


def intrinsics_from_fov(width, height, hfov_rad, vfov_rad):
    return Intrinsics(fx=(width / 2.0) / math.tan(hfov_rad / 2.0),
                      fy=(height / 2.0) / math.tan(vfov_rad / 2.0),
                      cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)


def fallback_intrinsics(width, height):
    """Intrinsics from FALLBACK_FOV_DEG, principal point at the image centre."""
    return intrinsics_from_fov(width, height, *(math.radians(a) for a in FALLBACK_FOV_DEG))


def load_intrinsics(path):
    """JSON {"fx": .., "fy": .., "cx": .., "cy": ..} for the RGB camera at 640x480."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return Intrinsics(float(data["fx"]), float(data["fy"]), float(data["cx"]), float(data["cy"]))


def default_intrinsics_file():
    """CAMERA_RGB_FILE when calib_floor.py has written it, else None."""
    return CAMERA_RGB_FILE if os.path.exists(CAMERA_RGB_FILE) else None


def _core_bounds(lo, hi, limit):
    """Pixel range [a, b) of the central CORE_FRACTION of [lo, hi], clipped, at least 1 px wide
    when the box is inside the image at all."""
    center = (lo + hi) / 2.0
    half = (hi - lo) * CORE_FRACTION / 2.0
    a = max(int(math.floor(center - half)), 0)
    b = min(int(math.ceil(center + half)), limit)
    if b <= a and 0 <= center < limit:
        a, b = int(center), int(center) + 1
    return a, b


def nearest_surface_mm(values_mm):
    """Median depth of the nearest cluster with enough support, or None when none qualifies."""
    v = np.sort(values_mm.astype(np.float64))
    gaps = np.diff(v)
    split = np.nonzero(gaps > np.maximum(CLUSTER_GAP_MIN_MM, CLUSTER_GAP_REL * v[:-1]))[0] + 1
    starts = np.concatenate(([0], split))
    ends = np.concatenate((split, [v.size]))
    need = MIN_CLUSTER_FRACTION * v.size
    for s, e in zip(starts, ends):   # ascending depth, so the first qualifying cluster is nearest
        if e - s >= need:
            return float(np.median(v[s:e]))
    return None


def measure_object(depth_mm, box, intrinsics):
    """ObjectRange for a bbox (x1, y1, x2, y2) in the registered depth image, or None when the
    depth there is too sparse or too fragmented to trust."""
    h, w = depth_mm.shape
    x1, y1, x2, y2 = box
    u0, u1 = _core_bounds(x1, x2, w)
    v0, v1 = _core_bounds(y1, y2, h)
    if u1 <= u0 or v1 <= v0:
        return None
    core = depth_mm[v0:v1, u0:u1]
    valid = core[core > 0]
    valid_fraction = valid.size / core.size
    if valid_fraction < MIN_VALID_FRACTION:
        return None
    surface_mm = nearest_surface_mm(valid)
    if surface_mm is None:
        return None
    z = surface_mm / 1000.0
    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    x = (u - intrinsics.cx) * z / intrinsics.fx
    y = (v - intrinsics.cy) * z / intrinsics.fy
    return ObjectRange(math.sqrt(x * x + y * y + z * z), z, (x, y, z), valid_fraction)


def object_mask(depth_mm, box, surface_z_m):
    """Pixels of the WHOLE bbox whose depth lies on the object's measured surface: the points that
    belong to the object. The rest of the box is background or floor."""
    h, w = depth_mm.shape
    x1, y1, x2, y2 = box
    u0, u1 = max(int(math.floor(x1)), 0), min(int(math.ceil(x2)), w)
    v0, v1 = max(int(math.floor(y1)), 0), min(int(math.ceil(y2)), h)
    mask = np.zeros(depth_mm.shape, dtype=bool)
    if u1 <= u0 or v1 <= v0:
        return mask
    surface_mm = surface_z_m * 1000.0
    band = max(OBJECT_BAND_MIN_MM, OBJECT_BAND_REL * surface_mm)
    roi = depth_mm[v0:v1, u0:u1].astype(np.float64)
    mask[v0:v1, u0:u1] = (roi > 0) & (np.abs(roi - surface_mm) <= band)
    return mask


def result_records(results):
    """JSON-ready rows for (Detection, ObjectRange | None) pairs. range_source: "depth" (measured),
    "floor" (estimated from where the box meets the floor: valid_fraction None) or None."""
    return [{
        "class": d.name, "class_id": d.class_id, "confidence": round(d.conf, RECORD_DIGITS),
        "box": [round(v, BOX_DIGITS) for v in d.box],
        "range_m": round(r.range_m, RECORD_DIGITS) if r else None,
        "z_m": round(r.z_m, RECORD_DIGITS) if r else None,
        "xyz_cam": [round(v, RECORD_DIGITS) for v in r.xyz] if r else None,
        "valid_fraction": round(r.valid_fraction, RECORD_DIGITS) if r and r.valid_fraction is not None else None,
        "range_source": None if r is None else (RANGE_FROM_FLOOR if r.valid_fraction is None else RANGE_FROM_DEPTH),
    } for d, r in results]
