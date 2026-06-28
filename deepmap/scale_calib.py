#!/usr/bin/env python3
"""
Step A4 (plan: rtabmap-slam) ported to the deepmap pipeline.

Motion-parallax scale recovery: Depth Anything V2 returns a RELATIVE inverse
depth `d = k / Z` (k unknown). We recover the metric constant `k` once by
driving the robot forward a known distance `a` and comparing matched feature
depths between the two frames:

    k_i = a * d1 * d2 / (d2 - d1)        (only valid where d2 > d1)

`k` is the robust median over inlier matches. Metric depth is then:

    Z_metric = k / d_relative     (clamped to [0, D_max])

Test_round360 mode
------------------
The real 30 cm forward move is SKIPPED. The step is still executed
structurally: we synthesise a placeholder `k`, build a metric depth map from
the relative depth, and CONFIRM the outputs have the desired TYPE/SHAPE
(value is irrelevant in test mode). See `confirm_output_type`.
"""

import time

import cv2
import numpy as np

# Placeholder metric constant used only in Test_round360 (value irrelevant).
TEST_K = 1.0


# ── Core conversions ──────────────────────────────────────────────────────────

def apply_scale(rel_depth, k, d_max):
    """Relative inverse depth -> metric depth (metres), clamped to [0, D_max].

    Returns float32 array, same H×W as `rel_depth`.
    """
    rel = np.asarray(rel_depth, dtype=np.float32)
    metric = k / np.maximum(rel, 1e-6)
    np.clip(metric, 0.0, d_max, out=metric)
    return metric.astype(np.float32)


def project_metric(metric_depth, color_bgr, fov_deg, stride, near_clip=0.26,
                   return_keep=False):
    """Back-project METRIC depth to 3D (camera at origin, looking -Z).

    Unlike depth_to_3d_timed.step_project (which fakes a 0.5..5.5 m range from
    relative depth), this uses the true metric Z directly. Near-clip removes the
    floor-contact band (plan Filter 1). Returns (pts float32 N×3, cols uint8 N×3).

    If `return_keep=True`, also returns the boolean keep-mask (length = full
    sampled grid) so per-point labels computed on the same grid stay aligned.
    """
    H, W = metric_depth.shape
    fx = fy = (W / 2.0) / np.tan(np.deg2rad(fov_deg) / 2.0)
    cx, cy = W / 2.0, H / 2.0

    ys, xs = np.mgrid[0:H:stride, 0:W:stride]
    z = metric_depth[ys, xs].astype(np.float32)

    X = (xs - cx) * z / fx
    Y = -(ys - cy) * z / fy
    Z = -z

    pts = np.stack([X, Y, Z], axis=-1).reshape(-1, 3).astype(np.float32)
    cols = color_bgr[ys, xs][:, :, ::-1].reshape(-1, 3).astype(np.uint8)  # BGR->RGB

    keep = z.reshape(-1) >= near_clip
    if return_keep:
        return pts[keep], cols[keep], keep
    return pts[keep], cols[keep]


# ── Movable-space classification (drive area -> per-point label) ───────────────

CLASS_DRIVABLE = (60, 200, 60)   # green (RGB) — floor / movable space
CLASS_OBSTACLE = (210, 50, 50)   # red   (RGB) — obstacle


def drivable_labels(depth_shape, polygon, stride, keep=None):
    """Per-point boolean: True = inside the drive-area polygon (movable floor).

    Sampled on the SAME grid as the projection so labels align with `pts`.
    `keep` is project_metric's keep-mask (None when the projector kept all points).
    """
    H, W = depth_shape
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    ys, xs = np.mgrid[0:H:stride, 0:W:stride]
    drv = (mask[ys, xs] > 0).reshape(-1)
    return drv if keep is None else drv[keep]


def colorize_by_class(cols, drivable, alpha=0.55):
    """Blend each point's colour toward green (drivable) or red (obstacle).

    alpha=blend strength; keeps some texture while clearly dividing the classes.
    """
    out = cols.astype(np.float32)
    g = np.asarray(CLASS_DRIVABLE, dtype=np.float32)
    r = np.asarray(CLASS_OBSTACLE, dtype=np.float32)
    out[drivable]  = out[drivable]  * (1.0 - alpha) + g * alpha
    out[~drivable] = out[~drivable] * (1.0 - alpha) + r * alpha
    return out.astype(np.uint8)


# ── Real motion-parallax recovery (used when NOT in test mode) ─────────────────

def _capture_relative_depth(cap, model, infer, prep, input_size):
    """Grab a fresh frame and return (relative_depth float32, bgr_image)."""
    for _ in range(3):           # flush stale V4L2 buffers
        cap.read()
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError('camera read failed during scale calibration')
    tensor, hw = prep(model, frame, input_size)
    return infer(model, tensor, hw).astype(np.float32), frame


def recover_scale_motion_parallax(cap, ser, model, prep, infer, input_size,
                                  a_cm=30.0, spd_ms=0.1, move_timeout=30.0,
                                  timing=None):
    """Drive forward `a_cm`, match ORB features across the two frames, return k.

    `ser` must already be connected (serial bridge / ESP32). `prep`/`infer` are
    depth_to_3d_timed.step_prep / step_infer. Returns float k.

    If `timing` is a dict, it is filled with the phase latencies (ms):
    `scale_capture_ms` (D1+D2 capture+infer), `move_30cm_ms` (F command -> K ack),
    `scale_compute_ms` (ORB detect+match + k median).
    """
    a_m = a_cm / 100.0

    # P1
    t = time.perf_counter()
    d1, img1 = _capture_relative_depth(cap, model, infer, prep, input_size)
    cap_ms = (time.perf_counter() - t) * 1000.0

    # Drive forward a known distance and wait for the motion-complete ack (K).
    t = time.perf_counter()
    ser.write(f'F {a_m:.4f} {spd_ms:.4f}\n'.encode())
    deadline = time.time() + move_timeout
    buf = b''
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b'K' in buf:
                break
    move_ms = (time.perf_counter() - t) * 1000.0

    # P2
    t = time.perf_counter()
    d2, img2 = _capture_relative_depth(cap, model, infer, prep, input_size)
    cap_ms += (time.perf_counter() - t) * 1000.0

    # ORB match between the two frames.
    t = time.perf_counter()
    g1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    orb = cv2.ORB_create(2000)
    k1, des1 = orb.detectAndCompute(g1, None)
    k2, des2 = orb.detectAndCompute(g2, None)
    if des1 is None or des2 is None:
        raise RuntimeError('no ORB features for scale calibration')
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1, des2)

    ks = []
    for m in matches:
        p1 = k1[m.queryIdx].pt
        p2 = k2[m.trainIdx].pt
        dv1 = float(d1[int(p1[1]), int(p1[0])])
        dv2 = float(d2[int(p2[1]), int(p2[0])])
        if dv2 <= dv1:           # robot moved toward point -> denom must be > 0
            continue
        ki = a_m * dv1 * dv2 / (dv2 - dv1)
        if ki > 0:
            ks.append(ki)

    if not ks:
        raise RuntimeError('no valid feature pairs for scale recovery')

    ks = np.asarray(ks, dtype=np.float64)
    q1, q3 = np.percentile(ks, [25, 75])           # IQR outlier reject
    iqr = q3 - q1
    inliers = ks[(ks >= q1 - 1.5 * iqr) & (ks <= q3 + 1.5 * iqr)]
    k = float(np.median(inliers if len(inliers) else ks))

    if timing is not None:
        timing['scale_capture_ms'] = round(cap_ms, 1)
        timing['move_30cm_ms'] = round(move_ms, 1)
        timing['scale_compute_ms'] = round((time.perf_counter() - t) * 1000.0, 1)
    return k


def write_calib_log(path, model_load_ms, timing):
    """Write the one-shot calibration timings to calib_log.csv."""
    import csv
    import os
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rows = [('model_load_ms', model_load_ms)]
    rows += [(key, timing.get(key, '')) for key in
             ('scale_capture_ms', 'move_30cm_ms', 'scale_compute_ms')]
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['stage', 'ms'])
        w.writerows(rows)
    return path


# ── Type confirmation (Test_round360) ─────────────────────────────────────────

def confirm_output_type(k, metric_depth, rel_depth):
    """Assert the calibration step produced the DESIRED TYPES, ignoring values.

    Returns True and prints a PASS line, or raises AssertionError on mismatch.
    """
    assert isinstance(k, float), f'k must be float, got {type(k)}'
    assert isinstance(metric_depth, np.ndarray), 'metric_depth must be ndarray'
    assert metric_depth.dtype == np.float32, \
        f'metric_depth dtype must be float32, got {metric_depth.dtype}'
    assert metric_depth.shape == rel_depth.shape, \
        f'metric_depth shape {metric_depth.shape} != rel {rel_depth.shape}'
    print('  [Test_round360] scale step output TYPE OK: '
          f'k=float, metric_depth=float32{metric_depth.shape}  -> PASS')
    return True


def calibrate(rel_depth, d_max, test_round360, **real_kwargs):
    """Run the scale step. Returns (k, metric_depth).

    test_round360=True  -> SKIP the 30 cm move; use TEST_K, build metric depth,
                           confirm output type (value irrelevant).
    test_round360=False -> run recover_scale_motion_parallax(**real_kwargs).
    """
    if test_round360:
        k = TEST_K
        metric_depth = apply_scale(rel_depth, k, d_max)
        confirm_output_type(k, metric_depth, rel_depth)
        return k, metric_depth

    k = recover_scale_motion_parallax(**real_kwargs)  # pass timing=<dict> to log
    metric_depth = apply_scale(rel_depth, k, d_max)
    return k, metric_depth
