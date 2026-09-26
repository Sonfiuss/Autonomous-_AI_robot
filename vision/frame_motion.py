"""Robot motion between two camera frames, measured on the floor.

Corners on the floor are tracked from one raw frame to the next (pyramidal Lucas-Kanade, checked
forwards and backwards). Through the camera height and pitch every tracked pixel is a floor point in
the robot's body frame, in both frames, so the pair pins down the planar motion (dx, dy, dtheta) of
the robot centre between them. The camera sits `forward_m` ahead of that centre (and `left_m` to
its left), so an in-place turn swings it along an arc; modelling the floor points in the body frame accounts for that, and an
in-place turn should come out with dx, dy near 0.

Not everything below the horizon is floor: the feet and faces of furniture, and on the glossy tiles
the reflections of lamps and windows, move differently. RANSAC over 2-point rigid fits separates
them; the winner is refined on the pixel error of its inliers, which weights near floor (1 px ~ 1 cm)
over far floor (1 px ~ 6 cm at 3 m) the way the image actually measures them.

Motion (dx, dy, dtheta): the pose of the body at the second frame, in the body frame of the first:
b_first = R(dtheta) b_second + (dx, dy).
"""
import collections
import logging
import math

import cv2
import numpy as np

from floor_segment import floor_hit, floor_project

FLOOR_MAX_RANGE_M = 2.5       # farther floor: 1 px of row is > 4 cm of range
FLOOR_EDGE_PX = 8             # corners this close to the image edge are dropped
MAX_CORNERS = 300
CORNER_QUALITY = 0.01
CORNER_MIN_DIST_PX = 8
LK_WIN_PX = 21
LK_LEVELS = 3
LK_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
FB_MAX_PX = 0.7               # forward-backward disagreement of a track
RANSAC_TRIALS = 150
RANSAC_SAMPLE = 2             # point pairs that fix a planar rigid motion
INLIER_PX = 1.5
MIN_INLIERS = 12
MIN_PAIR_GAP_M = 0.15         # the two points of a RANSAC sample: closer than this pin the angle badly
REFINE_ITERS = 10
REFINE_STEP = (1e-5, 1e-5, 1e-6)   # numeric Jacobian steps for dx, dy (m) and dtheta (rad)

# Motion: see the module docstring; inliers / tracks: point counts; rms_px over the inliers.
Motion = collections.namedtuple("Motion", "dx dy dtheta inliers tracks rms_px")

logger = logging.getLogger(__name__)


def floor_rows(intrinsics, mount, image_h, max_range_m=FLOOR_MAX_RANGE_M):
    """(first, last) image rows that see floor within max_range_m straight ahead."""
    _, v = floor_project([mount.forward_m + max_range_m], [mount.left_m], intrinsics, mount)
    return int(math.ceil(float(v[0]))), image_h - FLOOR_EDGE_PX


def track(gray0, gray1, mask):
    """(N, 2) corners of gray0 inside mask and (N, 2) where they went in gray1, forward-backward checked."""
    p0 = cv2.goodFeaturesToTrack(gray0, MAX_CORNERS, CORNER_QUALITY, CORNER_MIN_DIST_PX, mask=mask)
    if p0 is None:
        return np.empty((0, 2)), np.empty((0, 2))
    lk = dict(winSize=(LK_WIN_PX, LK_WIN_PX), maxLevel=LK_LEVELS, criteria=LK_CRITERIA)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(gray0, gray1, p0, None, **lk)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(gray1, gray0, p1, None, **lk)
    good = (st1.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((back - p0).reshape(-1, 2), axis=1) < FB_MAX_PX)
    return p0.reshape(-1, 2)[good].astype(np.float64), p1.reshape(-1, 2)[good].astype(np.float64)


def _predict(params, b0, intrinsics, mount):
    """Pixels in the second frame of body-frame floor points b0 of the first, for motion params."""
    dx, dy, dtheta = params
    c, s = math.cos(dtheta), math.sin(dtheta)
    rel = b0 - (dx, dy)
    u, v = floor_project(c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1], intrinsics, mount)
    return np.column_stack((u, v))


def _rigid_from_pair(b0, b1):
    """(dx, dy, dtheta) with b0 = R b1 + t from two point pairs, or None when they are too close."""
    d0, d1 = b0[1] - b0[0], b1[1] - b1[0]
    if np.linalg.norm(d0) < MIN_PAIR_GAP_M or np.linalg.norm(d1) < MIN_PAIR_GAP_M:
        return None
    dtheta = math.atan2(d0[1], d0[0]) - math.atan2(d1[1], d1[0])
    c, s = math.cos(dtheta), math.sin(dtheta)
    t = b0[0] - (c * b1[0, 0] - s * b1[0, 1], s * b1[0, 0] + c * b1[0, 1])
    return float(t[0]), float(t[1]), math.atan2(math.sin(dtheta), math.cos(dtheta))


def estimate_motion(p0, p1, intrinsics, mount, rng):
    """Motion from tracked pixels p0 -> p1, or None when too few tracks agree."""
    if len(p0) < MIN_INLIERS:
        return None
    f0, l0, _ = floor_hit(p0[:, 0], p0[:, 1], intrinsics, mount)
    f1, l1, _ = floor_hit(p1[:, 0], p1[:, 1], intrinsics, mount)
    b0, b1 = np.column_stack((f0, l0)), np.column_stack((f1, l1))
    usable = np.isfinite(b0).all(axis=1) & np.isfinite(b1).all(axis=1)
    b0, b1, p1 = b0[usable], b1[usable], p1[usable]
    if len(b0) < MIN_INLIERS:
        return None
    best, best_count = None, 0
    for _ in range(RANSAC_TRIALS):
        pick = rng.choice(len(b0), RANSAC_SAMPLE, replace=False)
        params = _rigid_from_pair(b0[pick], b1[pick])
        if params is None:
            continue
        err = np.linalg.norm(_predict(params, b0, intrinsics, mount) - p1, axis=1)
        count = int((err < INLIER_PX).sum())
        if count > best_count:
            best, best_count = params, count
    if best is None or best_count < MIN_INLIERS:
        return None
    params = np.array(best)
    for _ in range(REFINE_ITERS):          # Gauss-Newton on the inliers' pixel error
        inl = np.linalg.norm(_predict(params, b0, intrinsics, mount) - p1, axis=1) < INLIER_PX
        if inl.sum() < MIN_INLIERS:        # the step lost the consensus: too few rows to solve for 3
            break
        res = (_predict(params, b0[inl], intrinsics, mount) - p1[inl]).ravel()
        jac = np.column_stack([((_predict(params + np.eye(3)[k] * REFINE_STEP[k], b0[inl], intrinsics, mount)
                                 - p1[inl]).ravel() - res) / REFINE_STEP[k] for k in range(3)])
        step, *_ = np.linalg.lstsq(jac, -res, rcond=None)
        params = params + step
        if np.all(np.abs(step) < REFINE_STEP):
            break
    err = np.linalg.norm(_predict(params, b0, intrinsics, mount) - p1, axis=1)
    inl = err < INLIER_PX
    logger.debug("motion dx %.4f dy %.4f dtheta %.5f: %d / %d inliers (RANSAC %d)",
                 params[0], params[1], params[2], int(inl.sum()), len(b0), best_count)
    if inl.sum() < MIN_INLIERS:
        return None
    return Motion(float(params[0]), float(params[1]), float(params[2]), int(inl.sum()), len(b0),
                  float(np.sqrt(np.mean(err[inl] ** 2))))
