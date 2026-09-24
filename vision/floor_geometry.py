"""Camera geometry for mapping: depth pixels -> robot body frame, with height above the floor.

Frames:
  camera: +x right, +y down, +z forward along the optical axis (OpenCV), meters.
  body:   +x forward, +y left, +z up, origin on the floor under the robot center.
The camera sits `height_m` above the floor and `forward_m` ahead of the robot center, pitched DOWN
by `pitch_deg` (negative = up). No roll, no yaw - a pan servo would add a yaw term here.
"""
import collections
import math

import numpy as np

CameraMount = collections.namedtuple("CameraMount", "height_m pitch_deg forward_m")
# 24 cm high (user). Pitch 0: first given as "5 deg", but on a real frame 5 deg down put the feet of
# furniture below the floor and 0 put them on it; user set 0 (2026-09-25). forward_m not measured yet.
DEFAULT_MOUNT = CameraMount(height_m=0.24, pitch_deg=0.0, forward_m=0.0)
MAX_RANGE_M = 3.5            # Astra depth noise grows ~quadratically; farther points blur floor vs obstacle

FLOOR_FIT_BAND_M = 0.35      # floor candidates: this close to the ASSUMED floor
FLOOR_FIT_MAX_DIST_M = 2.5   # ... and within this distance ahead of the camera
FLOOR_FIT_SAMPLE = 5000      # candidates kept for RANSAC
FLOOR_FIT_TRIALS = 300
FLOOR_FIT_MAX_SLOPE = math.tan(math.radians(20))   # a pitch error beyond 20 deg is not the floor
FLOOR_FIT_MIN_PAIR_GAP_M = 0.2
FLOOR_FIT_INLIER_M = 0.02
FLOOR_FIT_MIN_POINTS = 300
FLOOR_FIT_MIN_SPREAD_M = 0.5 # inliers must span this much distance, or the slope means nothing
FLOOR_FIT_CELL_M = 0.05      # floor points sharing such a cell with anything standing ...
FLOOR_FIT_STANDING_M = 0.05  # ... this far above the RANSAC floor line are dropped (feet of walls)

FloorFit = collections.namedtuple("FloorFit", "height_m pitch_deg points")


def backproject(depth_mm, intrinsics, stride=1, mask=None):
    """(N, 3) camera-frame points of the valid pixels (0 < depth <= MAX_RANGE_M) on every
    `stride`-th row and column, optionally restricted to a boolean `mask` of the image."""
    rows, cols = np.mgrid[0:depth_mm.shape[0]:stride, 0:depth_mm.shape[1]:stride]
    depth = depth_mm[::stride, ::stride]
    keep = (depth > 0) & (depth <= MAX_RANGE_M * 1000.0)
    if mask is not None:
        keep &= mask[::stride, ::stride]
    z = depth[keep] / 1000.0
    x = (cols[keep] - intrinsics.cx) * z / intrinsics.fx
    y = (rows[keep] - intrinsics.cy) * z / intrinsics.fy
    return np.column_stack((x, y, z))


def pixel_heights(depth_mm, intrinsics, mount):
    """(H, W) height above the floor of every pixel, NaN where there is no depth or it is beyond
    MAX_RANGE_M - the per-pixel view of what backproject + camera_to_body feed the map."""
    rows, cols = np.mgrid[0:depth_mm.shape[0], 0:depth_mm.shape[1]]
    z = depth_mm / 1000.0
    y = (rows - intrinsics.cy) * z / intrinsics.fy
    pitch = math.radians(mount.pitch_deg)
    height = mount.height_m - z * math.sin(pitch) - y * math.cos(pitch)
    return np.where((depth_mm > 0) & (z <= MAX_RANGE_M), height, np.nan)


def camera_to_body(points_cam, mount):
    """(N, 3) camera-frame points -> (N, 3) body frame: forward, left, height above the floor."""
    pitch = math.radians(mount.pitch_deg)
    x, y, z = points_cam[:, 0], points_cam[:, 1], points_cam[:, 2]
    forward = z * math.cos(pitch) + y * math.sin(pitch) + mount.forward_m
    height = mount.height_m - z * math.sin(pitch) - y * math.cos(pitch)
    return np.column_stack((forward, -x, height))


def fit_floor(points_body, mount):
    """Camera height and pitch MEASURED from the floor in one frame, or None if too little floor.

    Transformed with the assumed mount, real floor points should sit at height 0 at every
    distance. A pitch error phi tilts them into a line, height = c - d * tan(phi), d = distance
    ahead of the camera; a height error shifts c. Fitting that line measures both - which is how
    a flipped pitch sign shows up: 5 deg up read as 5 deg down gives a 10 deg slope.

    RANSAC, not least squares: the band around the assumed floor also catches the foot of every
    wall and obstacle, and a plain fit is dragged onto them (measured on a synthetic room: 0.29-0.43 m
    and 9-16 deg reported for a true 0.24 m / 5 deg). Seen as height against distance, a wall is a
    near-vertical streak, which the slope limit rules out. The last few cm of a wall's foot still
    pass as floor, so before the final fit every cell holding something that stands above the
    RANSAC line is dropped - relative to that line, so it works with a wrong pitch too.
    """
    dist = points_body[:, 0] - mount.forward_m
    height = points_body[:, 2]
    ahead = (dist > 0) & (dist < FLOOR_FIT_MAX_DIST_M)
    dist, height, left = dist[ahead], height[ahead], points_body[ahead, 1]
    keep = np.abs(height) < FLOOR_FIT_BAND_M
    if keep.sum() < FLOOR_FIT_MIN_POINTS:
        return None
    d, h = dist[keep], height[keep]
    rng = np.random.default_rng(0)   # fixed seed: the same frame always gives the same answer
    if d.size > FLOOR_FIT_SAMPLE:
        pick = rng.choice(d.size, FLOOR_FIT_SAMPLE, replace=False)
        d, h = d[pick], h[pick]
    i, j = rng.integers(0, d.size, size=(2, FLOOR_FIT_TRIALS))
    gap = d[j] - d[i]
    usable = np.abs(gap) > FLOOR_FIT_MIN_PAIR_GAP_M
    slopes = (h[j] - h[i])[usable] / gap[usable]
    usable_slopes = np.abs(slopes) <= FLOOR_FIT_MAX_SLOPE
    slopes = slopes[usable_slopes]
    intercepts = h[i][usable][usable_slopes] - slopes * d[i][usable][usable_slopes]
    if slopes.size == 0:
        return None
    counts = (np.abs(slopes[:, None] * d[None, :] + intercepts[:, None] - h[None, :]) < FLOOR_FIT_INLIER_M).sum(axis=1)
    best = int(np.argmax(counts))
    residual = height - (slopes[best] * dist + intercepts[best])
    cells = (np.floor(dist / FLOOR_FIT_CELL_M).astype(np.int64) * 1_000_000
             + np.floor(left / FLOOR_FIT_CELL_M).astype(np.int64))
    standing = np.isin(cells, np.unique(cells[residual > FLOOR_FIT_STANDING_M]))
    inliers = (np.abs(residual) < FLOOR_FIT_INLIER_M) & ~standing
    if inliers.sum() < FLOOR_FIT_MIN_POINTS or np.ptp(dist[inliers]) < FLOOR_FIT_MIN_SPREAD_M:
        return None
    slope, intercept = np.polyfit(dist[inliers], height[inliers], 1)
    phi = -math.atan(slope)                  # assumed pitch minus true pitch
    # Exact for a rotated plane: intercept = h_assumed - h_true * cos(2 phi) / cos(phi).
    true_height = (mount.height_m - intercept) * math.cos(phi) / math.cos(2 * phi)
    return FloorFit(true_height, mount.pitch_deg - math.degrees(phi), int(inliers.sum()))
