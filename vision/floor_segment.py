"""Free floor ahead of the robot from Depth Anything disparity, checked column by column.

The rule (user, 2026-09-25): inside a virtual trapezoid on the floor ahead, depth keeps growing up
each image column; where it stops growing, something stands there. Made quantitative:
- Disparity (~1/z) of a floor plane is linear in the image row, while any VERTICAL surface - wall,
  chair leg, the front of a box - holds it constant up the column. The local column slope divided by
  the floor's own slope is ~1 on the floor and ~0 on a vertical face.
- The floor's plane is fitted to the disparity of the same frame (RANSAC inside the trapezoid).
  Depth Anything's scale and shift are unknown, and not even consistent within one image: scaled on
  the walls through the Astra, the real floor came out at 0.44 of its true slope.
- Floor = on that plane (within PLANE_TOL) with a floor-like column slope. Closer than the plane, or
  vertical = obstacle. Farther than the plane = a drop (stairs down, a hole, a reflection) - never free.
- Walking up each column from the robot, the floor is free up to the first obstacle, drop or YOLO
  box. That first obstacle row is the CONTACT, where the object meets the floor. Floor seen beyond
  it stays unknown: behind something, the column is no longer what the rule talks about.

Depth Anything has no metres, so positions come from floor geometry: the ray through a floor pixel
meets the floor at a distance fixed by the camera height and pitch. 2 deg of pitch error moves the
floor at 3 m to 2.1 or 5.3 m, so the pitch is MEASURED per capture from the contacts the Astra has
depth for: an object's foot is on the floor, so its depth pins that ray.

Pure: numpy + OpenCV filters, no model, no camera.
"""
import collections
import math

import cv2
import numpy as np

from floor_geometry import MAX_RANGE_M
from object_distance import ObjectRange
from occupancy_map import PIXEL_DROP, PIXEL_FREE, PIXEL_NONE, PIXEL_OBSTACLE

NEAR_M = 0.6                  # Astra minimum range; the bottom image row sees ~0.57 m (0.24 m high, level)
BOX_BOTTOM_MARGIN_PX = 3      # a box this close to the image's bottom edge is cut off: its foot is not in view
BOX_MAX_RANGE_M = 8.0         # beyond this a one-row error at the box bottom swings the estimate by metres
DEFAULT_FAR_M = 3.0
PLANE_TOL = 0.04              # |disparity - floor plane| / floor plane
PLANE_SAMPLE = 4000           # trapezoid pixels kept for RANSAC
PLANE_TRIALS = 200
MIN_FLOOR_FRACTION = 0.3      # of the trapezoid on the fitted plane, or the floor is not in view
MIN_PLANE_SPAN = 0.2          # plane must rise by this share from the far to the near edge: a wall filling
                              # the view fits a plane too, a flat one
PLANE_TRUST_FRACTION = 0.1    # the plane is used only where it predicts this share of its near-edge value.
                              # Depth Anything's shift can be negative: on the real floor its plane reached 0
                              # ~20 rows below the true horizon, and a relative tolerance means nothing there
SLOPE_WINDOW_ROWS = 9
MIN_STOP_ROWS = 3             # an obstacle / drop must hold this many rows up its column. A floor cable dipped
                              # the ratio to 0.48 for one row; 3 cm at 3 m still spans ~6 rows
FLOOR_RATIO_MIN = 0.5         # column slope / floor slope: 0 on a vertical face ...
FLOOR_RATIO_MAX = 1.6         # ... h / (h - top) on a box top: above 1.6 once the top is 9+ cm high
TALL_CONTACT_ROWS = 40        # pitch uses only contacts under a vertical face this tall in the Astra depth.
                              # On the real glossy floor a 3 cm tube's contacts sat ~14 rows below where a floor
                              # point at its depth would be, a cabinet's within 1 row: the tube's reflection in
                              # the tiles reads as more tube to Depth Anything AND the Astra (15 rows of constant
                              # depth, 3 cm "under" the floor). Low objects are thus placed up to ~15 cm nearer
                              # than they are - the safe side - but would pull the pitch by ~1 deg.
MIN_TALL_SAMPLES = 28         # of those rows with depth
TALL_SPREAD_REL = 0.03        # depth may vary this much up the face: a vertical surface keeps it constant
MIN_PITCH_CONTACTS = 30       # columns
MAX_PITCH_SPREAD_DEG = 2.0    # interquartile range of the per-column estimates
MAX_PITCH_CORRECTION_DEG = 10.0
MIN_PROJECT_DEPTH_M = 1e-9    # floor_project: a point this close to the camera plane (or behind) has no pixel
FREE_SUBSTEPS = 4             # floor points per row step between vertically adjacent free pixels: one row
                              # spans 6.6 cm of floor at 3 m (12 cm at 4 m) - more than a 5 cm map cell, and
                              # sampled per pixel the far free floor came out striped

# The trapezoid: floor NEAR_M..far_m ahead; half_width_m None = as wide as the view (user, 2026-09-25).
FloorConfig = collections.namedtuple("FloorConfig", "far_m half_width_m")
DEFAULT_FLOOR = FloorConfig(far_m=DEFAULT_FAR_M, half_width_m=None)

# labels: occupancy_map.PIXEL_* per pixel. contacts: (N, 2) int (u, v), one per trapezoid column that
# meets an obstacle or a YOLO box. da_contact: (N,) bool, the contact is Depth Anything's own (a box
# bottom is only roughly an object's foot). pitch_deg: measured from the contacts, None if unmeasured.
# plane: the fitted floor plane (a, b, c), None when the trapezoid was not mostly floor.
FloorView = collections.namedtuple("FloorView", "labels trapezoid contacts da_contact pitch_deg plane")


def floor_hit(us, vs, intrinsics, mount):
    """(forward, left) body-frame metres where the rays through pixels (us, vs) meet the floor, and
    the depth z along the optical axis there. NaN at and above the horizon."""
    pitch = math.radians(mount.pitch_deg)
    dx = (np.asarray(us, np.float64) - intrinsics.cx) / intrinsics.fx
    dy = (np.asarray(vs, np.float64) - intrinsics.cy) / intrinsics.fy
    down = math.sin(pitch) + dy * math.cos(pitch)   # ray's downward component per unit z
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(down > 1e-9, mount.height_m / down, np.nan)
    return z * (math.cos(pitch) - dy * math.sin(pitch)) + mount.forward_m, mount.left_m - z * dx, z


def floor_project(forward, left, intrinsics, mount):
    """(u, v) pixels of body-frame floor points - the exact inverse of floor_hit. NaN for points at or
    behind the camera plane."""
    pitch = math.radians(mount.pitch_deg)
    ahead = np.asarray(forward, np.float64) - mount.forward_m
    z = ahead * math.cos(pitch) + mount.height_m * math.sin(pitch)      # depth along the optical axis
    y = mount.height_m * math.cos(pitch) - ahead * math.sin(pitch)      # camera +y, down
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(z > MIN_PROJECT_DEPTH_M, z, np.nan)
        return (intrinsics.cx - intrinsics.fx * (np.asarray(left, np.float64) - mount.left_m) / z,
                intrinsics.cy + intrinsics.fy * y / z)


def floor_xy(us, vs, intrinsics, mount):
    """(N, 2) body-frame floor points under the given pixels; pixels at or above the horizon dropped."""
    forward, left, _ = floor_hit(us, vs, intrinsics, mount)
    keep = np.isfinite(forward)
    return np.column_stack((forward[keep], left[keep]))


def box_floor_range(box, image_h, intrinsics, mount):
    """ObjectRange of the point where a YOLO box meets the floor (bottom centre of the box), from the
    camera height and pitch alone - no depth. valid_fraction None marks it as that estimate. None when
    the box is cut off by the image's bottom edge (the object is closer than the bottom row sees,
    ~NEAR_M), sits above the horizon, or lands beyond BOX_MAX_RANGE_M. Right for things standing on
    the floor; a box whose bottom is not a foot (a hanging object, a table top) reads too far."""
    x1, _, x2, y2 = box
    if y2 >= image_h - BOX_BOTTOM_MARGIN_PX:
        return None
    u = 0.5 * (x1 + x2)
    _, left, z = (float(a[0]) for a in floor_hit([u], [y2], intrinsics, mount))
    if not math.isfinite(z) or z <= 0.0:
        return None
    xyz = (mount.left_m - left, z * (y2 - intrinsics.cy) / intrinsics.fy, z)   # camera frame: +x right, +y down
    rng = math.sqrt(xyz[0] ** 2 + xyz[1] ** 2 + xyz[2] ** 2)
    if rng > BOX_MAX_RANGE_M:
        return None
    return ObjectRange(range_m=rng, z_m=z, xyz=xyz, valid_fraction=None)


def free_floor_xy(labels, intrinsics, mount, col_stride=1):
    """(N, 2) body-frame points covering the free floor of a label image: each free pixel on every
    col_stride-th column, plus FREE_SUBSTEPS - 1 points between it and the free pixel above it."""
    free = labels[:, ::col_stride] == PIXEL_FREE
    rows, cols = np.nonzero(free)
    above = rows > 0
    above[above] = free[rows[above] - 1, cols[above]]
    us = [cols * col_stride] + [cols[above] * col_stride] * (FREE_SUBSTEPS - 1)
    vs = [rows] + [rows[above] - k / FREE_SUBSTEPS for k in range(1, FREE_SUBSTEPS)]
    return floor_xy(np.concatenate(us), np.concatenate(vs), intrinsics, mount)


def floor_trapezoid(shape, intrinsics, mount, far_m=DEFAULT_FAR_M, half_width_m=None):
    """Pixels whose ray meets the floor NEAR_M..far_m ahead of the camera and, if half_width_m is
    given, within that much of the camera's line of sight: a floor rectangle seen in perspective."""
    rows, cols = np.mgrid[0:shape[0], 0:shape[1]]
    forward, left, _ = floor_hit(cols, rows, intrinsics, mount)
    ahead = forward - mount.forward_m
    with np.errstate(invalid="ignore"):   # NaN above the horizon compares False
        keep = (ahead >= NEAR_M) & (ahead <= far_m)
        if half_width_m is not None:
            keep &= np.abs(left - mount.left_m) <= half_width_m
    return keep


def fit_floor_plane(disparity, trapezoid):
    """(a, b, c) of disparity = a * row + b * col + c on the floor, or None when the trapezoid is not
    mostly floor - facing a wall up close, or an object filling the view."""
    rows, cols = np.nonzero(trapezoid)
    if rows.size < 3:
        return None
    rng = np.random.default_rng(0)   # fixed seed: the same frame always gives the same answer
    if rows.size > PLANE_SAMPLE:
        pick = rng.choice(rows.size, PLANE_SAMPLE, replace=False)
        rows, cols = rows[pick], cols[pick]
    design = np.column_stack((rows, cols, np.ones(rows.size)))
    d = disparity[rows, cols].astype(np.float64)
    triples = rng.integers(0, rows.size, size=(PLANE_TRIALS, 3))
    solvable = np.abs(np.linalg.det(design[triples])) > 1e-6
    planes = np.linalg.solve(design[triples[solvable]], d[triples[solvable]])
    planes = planes[planes[:, 0] > 0]   # the floor gets closer going down the image
    if planes.shape[0] == 0:
        return None
    predicted = design @ planes.T
    counts = (np.abs(d[:, None] - predicted) < PLANE_TOL * np.abs(predicted)).sum(axis=0)
    best = predicted[:, int(np.argmax(counts))]
    inliers = np.abs(d - best) < PLANE_TOL * np.abs(best)
    plane = np.linalg.lstsq(design[inliers], d[inliers], rcond=None)[0]

    rows, cols = np.nonzero(trapezoid)
    predicted = plane[0] * rows + plane[1] * cols + plane[2]
    on_plane = np.abs(disparity[rows, cols] - predicted) < PLANE_TOL * np.abs(predicted)
    near, far = _plane_near(plane, trapezoid), predicted[rows == rows.min()].mean()
    if plane[0] <= 0 or near <= 0 or on_plane.mean() < MIN_FLOOR_FRACTION or (near - max(far, 0.0)) / near < MIN_PLANE_SPAN:
        return None
    return plane


def _plane_near(plane, trapezoid):
    """Mean plane value along the trapezoid's nearest (lowest) row."""
    rows, cols = np.nonzero(trapezoid)
    bottom = rows == rows.max()
    return float(np.mean(plane[0] * rows[bottom] + plane[1] * cols[bottom] + plane[2]))


def column_slope_ratio(disparity, floor_slope):
    """Least-squares slope of disparity against the row over SLOPE_WINDOW_ROWS rows centered on each
    pixel, divided by the floor's slope: ~1 on the floor, ~0 on anything vertical, < 0 leaning in."""
    rows = np.broadcast_to(np.arange(disparity.shape[0], dtype=np.float64)[:, None], disparity.shape)
    d = disparity.astype(np.float64)

    def mean(a):
        return cv2.boxFilter(np.ascontiguousarray(a), -1, (1, SLOPE_WINDOW_ROWS), borderType=cv2.BORDER_REPLICATE)

    mean_v, mean_d = mean(rows), mean(d)
    variance = np.maximum(mean(rows * rows) - mean_v * mean_v, 1e-9)
    return (mean(rows * d) - mean_v * mean_d) / variance / floor_slope


def trusted_trapezoid(plane, trapezoid):
    """The part of the trapezoid where the fitted plane is usable (PLANE_TRUST_FRACTION)."""
    rows, cols = np.mgrid[0:trapezoid.shape[0], 0:trapezoid.shape[1]]
    return trapezoid & (plane[0] * rows + plane[1] * cols + plane[2] > PLANE_TRUST_FRACTION * _plane_near(plane, trapezoid))


def segment_floor(disparity, trapezoid, plane, boxes=()):
    """(labels, contacts, da_contact) - see FloorView - for a fitted floor plane, inside a trusted
    trapezoid."""
    h, w = disparity.shape
    rows, cols = np.mgrid[0:h, 0:w]
    floor_plane = plane[0] * rows + plane[1] * cols + plane[2]
    with np.errstate(divide="ignore", invalid="ignore"):
        residual = (disparity - floor_plane) / floor_plane   # > 0: closer than the floor
    ratio = column_slope_ratio(disparity, plane[0])
    floor = (np.abs(residual) < PLANE_TOL) & (ratio > FLOOR_RATIO_MIN) & (ratio < FLOOR_RATIO_MAX)
    column = np.ones((MIN_STOP_ROWS, 1), np.uint8)
    obstacle = trapezoid & cv2.morphologyEx(((residual >= PLANE_TOL) | (ratio <= FLOOR_RATIO_MIN)).astype(np.uint8),
                                            cv2.MORPH_OPEN, column).astype(bool)
    drop = trapezoid & cv2.morphologyEx((residual <= -PLANE_TOL).astype(np.uint8), cv2.MORPH_OPEN,
                                        column).astype(bool) & ~obstacle
    in_box = np.zeros((h, w), bool)
    for x1, y1, x2, y2 in boxes:
        in_box[max(int(y1), 0):max(int(math.ceil(y2)), 0), max(int(x1), 0):max(int(math.ceil(x2)), 0)] = True
    in_box &= trapezoid

    stop = obstacle | drop | in_box
    has_stop = stop.any(axis=0)
    first = np.where(has_stop, h - 1 - np.argmax(stop[::-1], axis=0), -1)   # lowest stop row per column
    labels = np.full((h, w), PIXEL_NONE, np.uint8)
    labels[drop] = PIXEL_DROP
    labels[obstacle | in_box] = PIXEL_OBSTACLE
    labels[trapezoid & floor & (rows > first[None, :])] = PIXEL_FREE

    u = np.nonzero(has_stop)[0]
    v = first[u]
    keep = ~drop[v, u] | in_box[v, u]   # a drop ends the free run but is no object's foot
    u, v = u[keep], v[keep]
    return labels, np.column_stack((u, v)), obstacle[v, u]


def measure_pitch(contacts, depth_mm, intrinsics, mount):
    """Camera pitch (deg, + down) that puts the Astra depth at the contacts on the floor, or None when
    too few contacts stand under a tall vertical face with depth, or they disagree.

    Per contact, the object's depth z at row v must satisfy z = h / (sin p + dy cos p), dy = (v - cy)/fy,
    so p = asin(h / (z * sqrt(1 + dy^2))) - atan(dy). The median over the columns is the answer.
    """
    if contacts.shape[0] < MIN_PITCH_CONTACTS:
        return None
    u, v = contacts[:, 0], contacts[:, 1]
    sample_rows = np.clip(v[:, None] - np.arange(TALL_CONTACT_ROWS)[None, :], 0, depth_mm.shape[0] - 1)
    samples = depth_mm[sample_rows, u[:, None]].astype(np.float64)
    samples[samples == 0] = np.nan
    enough = np.isfinite(samples).sum(axis=1) >= MIN_TALL_SAMPLES
    if enough.sum() < MIN_PITCH_CONTACTS:
        return None
    samples, v = samples[enough], v[enough]
    z = np.nanmedian(samples, axis=1)
    tall = (np.nanmax(samples, axis=1) - np.nanmin(samples, axis=1)) <= TALL_SPREAD_REL * z
    z, v = z[tall] / 1000.0, v[tall]
    dy = (v - intrinsics.cy) / intrinsics.fy
    sine = mount.height_m / (z * np.sqrt(1.0 + dy * dy))
    usable = (z >= NEAR_M) & (z <= MAX_RANGE_M) & (np.abs(sine) <= 1.0)
    if usable.sum() < MIN_PITCH_CONTACTS:
        return None
    pitch = np.degrees(np.arcsin(sine[usable]) - np.arctan(dy[usable]))
    q1, median, q3 = np.percentile(pitch, [25, 50, 75])
    if q3 - q1 > MAX_PITCH_SPREAD_DEG or abs(median - mount.pitch_deg) > MAX_PITCH_CORRECTION_DEG:
        return None
    return float(median)


def analyze_floor(disparity, depth_mm, boxes, intrinsics, mount, floor_cfg=DEFAULT_FLOOR):
    """FloorView of one frame. boxes: YOLO (x1, y1, x2, y2), never free. depth_mm (registered Astra,
    may be None) only serves the pitch measurement."""
    trapezoid = floor_trapezoid(disparity.shape, intrinsics, mount, floor_cfg.far_m, floor_cfg.half_width_m)
    plane = fit_floor_plane(disparity, trapezoid)
    if plane is None:
        return FloorView(np.full(disparity.shape, PIXEL_NONE, np.uint8), trapezoid,
                         np.zeros((0, 2), np.int64), np.zeros(0, bool), None, None)
    trapezoid = trusted_trapezoid(plane, trapezoid)
    labels, contacts, da_contact = segment_floor(disparity, trapezoid, plane, boxes)
    pitch = measure_pitch(contacts[da_contact], depth_mm, intrinsics, mount) if depth_mm is not None else None
    return FloorView(labels, trapezoid, contacts, da_contact, pitch, tuple(float(p) for p in plane))
