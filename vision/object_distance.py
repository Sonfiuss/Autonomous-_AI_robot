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

import numpy as np

CORE_FRACTION = 0.5            # central share of bbox width/height that is sampled
MIN_VALID_FRACTION = 0.3       # below this share of valid core pixels -> no reading
MIN_CLUSTER_FRACTION = 0.2     # a cluster needs this share of the valid pixels to count as a surface
CLUSTER_GAP_MIN_MM = 100       # a jump larger than max(this, CLUSTER_GAP_REL * depth) splits clusters
CLUSTER_GAP_REL = 0.05
OBJECT_BAND_MIN_MM = 150       # object_mask: within max(this, OBJECT_BAND_REL * depth) of the surface
OBJECT_BAND_REL = 0.10

Intrinsics = collections.namedtuple("Intrinsics", "fx fy cx cy")
# range_m: Euclidean camera->object distance. z_m: depth along the optical axis.
ObjectRange = collections.namedtuple("ObjectRange", "range_m z_m xyz valid_fraction")


def intrinsics_from_fov(width, height, hfov_rad, vfov_rad):
    return Intrinsics(fx=(width / 2.0) / math.tan(hfov_rad / 2.0),
                      fy=(height / 2.0) / math.tan(vfov_rad / 2.0),
                      cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)


def load_intrinsics(path):
    """JSON {"fx": .., "fy": .., "cx": .., "cy": ..} for the RGB camera at 640x480."""
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return Intrinsics(float(data["fx"]), float(data["fy"]), float(data["cx"]), float(data["cy"]))


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
