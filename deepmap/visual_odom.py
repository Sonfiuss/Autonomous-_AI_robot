#!/usr/bin/env python3
"""
visual_odom — image-based motion between walk stops (VO for stereo_walk_map).

Dead-reckoning from commanded steps cannot see wheel slip. But the images
can: driving forward makes the scene "zoom" and turning shifts it sideways —
and because every stop already has a METRIC depth map, that zoom/shift can be
measured as real metres/degrees instead of heuristically:

  1. ORB features on the right-RECTIFIED image of two consecutive stops
     (the depth map lives in that frame), ratio-test matched.
  2. Each matched keypoint is lifted to metric 3D with its own stop's depth,
     then moved into the SAME leveled frame the cloud uses (floor at Y=0),
     so the relative motion is planar: yaw + translation in XZ.
  3. RANSAC over 2-point minimal samples fits p_prev ≈ R2(yaw)·p_cur + t;
     inliers refit by closed-form 2D Kabsch.

The same physical point seen twice must also keep its height above the floor
and its distances to other points (rigid scene) — both are used as filters,
and the inter-point distance ratio doubles as a SCALE cross-check: a stop
whose metric fit collapsed (the walk-scale-fit failure mode) shows up as a
ratio far from 1 before it can poison the map.

Frame conventions match stereo_walk_map/explore_map: X right, Y up, Z forward;
2D rotation on (x, z) uses R2(yaw) = [[c, s], [-s, c]], the same matrix as
to_world(), so composed poses drop straight into Pose(x, z, yaw).
"""
import cv2
import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Pose algebra on plain (x, z, yaw) tuples — callers wrap into explore_map.Pose
# ─────────────────────────────────────────────────────────────────────────────
def _r2(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, s], [-s, c]])


def compose(base, yaw_rel, t_rel):
    """World pose of the current stop given the previous stop's world pose and
    the relative transform (cur-camera coords → prev-camera coords)."""
    x, z, yaw = base
    tx, tz = _r2(yaw) @ t_rel
    return (x + tx, z + tz, yaw + yaw_rel)


def relative(a, b):
    """(yaw_rel, t_rel) with compose(a, yaw_rel, t_rel) == b."""
    dx, dz = b[0] - a[0], b[1] - a[1]
    t = _r2(-a[2]) @ np.array([dx, dz])
    return b[2] - a[2], t


# ─────────────────────────────────────────────────────────────────────────────
# Geometry — keypoints → leveled metric 3D, planar rigid fit
# ─────────────────────────────────────────────────────────────────────────────
def lift_keypoints(kp_uv, Z, valid, calib, z_min, z_max, lvl=None):
    """Rectified pixel keypoints → 3D in the SAME frame the cloud uses.

    lvl = (R, floor_y) from level_to_floor, or None when leveling failed
    (points stay in the tilted camera frame — the planar fit then under-reads
    distance by ~cos(tilt); acceptable because the dead-reckon sanity bound
    still gates the result). Returns (pts3d Nx3, keep mask over kp_uv).
    """
    h, w = Z.shape
    u = np.clip(np.round(kp_uv[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.round(kp_uv[:, 1]).astype(int), 0, h - 1)
    z = Z[v, u]
    keep = valid[v, u] & (z > z_min) & (z < z_max)
    z, u, v = z[keep], u[keep], v[keep]
    x = (u - calib.cx) * z / calib.fx
    y = -(v - calib.cy) * z / calib.fy
    pts = np.stack([x, y, z], axis=1).astype(np.float64)
    if lvl is not None:
        R, floor_y = lvl
        pts = pts @ np.asarray(R, np.float64).T
        pts[:, 1] -= floor_y
    return pts, keep


def _fit_planar(P, Q):
    """Closed-form 2D Kabsch on XZ: p ≈ R2(yaw)·q + t. P, Q are Nx3."""
    Pxz, Qxz = P[:, [0, 2]], Q[:, [0, 2]]
    Pm, Qm = Pxz.mean(0), Qxz.mean(0)
    Pc, Qc = Pxz - Pm, Qxz - Qm
    a = (Pc * Qc).sum()                                   # Σ px·qx + pz·qz
    b = (Pc[:, 0] * Qc[:, 1] - Pc[:, 1] * Qc[:, 0]).sum()  # Σ px·qz − pz·qx
    yaw = float(np.arctan2(b, a))
    t = Pm - _r2(yaw) @ Qm
    return yaw, t


def _residuals(P, Q, yaw, t):
    pred = Q[:, [0, 2]] @ _r2(yaw).T + t
    return np.linalg.norm(pred - P[:, [0, 2]], axis=1)


def estimate_motion(P, Q, min_inliers=12, resid_m=0.03, height_m=0.08,
                    iters=300, seed=0):
    """Matched metric 3D pairs (P = prev stop, Q = cur stop, same leveled
    frame) → planar relative motion. Returns dict or None when unreliable.

    dict: yaw (rad), t (2,), n_matches, n_inl, rms (m), scale — scale is the
    median inter-point distance ratio prev/cur (rigid scene ⇒ 1.0; a metric
    fit collapsed on either stop pushes it away from 1).
    """
    same_height = np.abs(P[:, 1] - Q[:, 1]) < height_m
    P, Q = P[same_height], Q[same_height]
    n = len(P)
    if n < max(6, min_inliers // 2):
        return None

    rng = np.random.default_rng(seed)
    i = rng.integers(0, n, size=min(500, n * 2))
    j = rng.integers(0, n, size=i.size)
    dp = np.linalg.norm(P[i] - P[j], axis=1)
    dq = np.linalg.norm(Q[i] - Q[j], axis=1)
    ok = dq > 0.10                       # short segments: ratio too noisy
    scale = float(np.median(dp[ok] / dq[ok])) if ok.sum() >= 8 else 1.0

    best_inl, best = None, -1
    for _ in range(iters):
        a, b = rng.integers(0, n, size=2)
        if np.linalg.norm(Q[a, [0, 2]] - Q[b, [0, 2]]) < 0.05:
            continue                     # degenerate minimal sample
        yaw, t = _fit_planar(P[[a, b]], Q[[a, b]])
        inl = _residuals(P, Q, yaw, t) < resid_m
        if inl.sum() > best:
            best, best_inl = inl.sum(), inl
    if best < min_inliers:
        return None
    yaw, t = _fit_planar(P[best_inl], Q[best_inl])         # refine on inliers
    inl = _residuals(P, Q, yaw, t) < resid_m
    if inl.sum() < min_inliers:
        return None
    yaw, t = _fit_planar(P[inl], Q[inl])
    rms = float(np.sqrt((_residuals(P, Q, yaw, t)[inl] ** 2).mean()))
    return {'yaw': yaw, 't': t, 'n_matches': n, 'n_inl': int(inl.sum()),
            'rms': rms, 'scale': scale}


# ─────────────────────────────────────────────────────────────────────────────
# Tracker — one per walk; owns the previous stop's features and poses
# ─────────────────────────────────────────────────────────────────────────────
class VOTracker:
    """Feeds on each fused stop; returns the pose the cloud should be placed
    at. VO replaces dead-reckoning only when it is confident AND agrees with
    the commanded motion to within dist_bounds — otherwise the commanded
    (dead-reckoned) RELATIVE step is chained onto the last assigned pose, so
    one bad stop never makes the map jump."""

    def __init__(self, calib, z_min=0.2, z_max=5.0, nfeatures=2000,
                 min_inliers=12, rms_max=0.04, dist_bounds=(0.3, 2.5),
                 yaw_max_deg=20.0, scale_warn=0.15):
        self.calib = calib
        self.z_min, self.z_max = z_min, z_max
        self.min_inliers, self.rms_max = min_inliers, rms_max
        self.dist_bounds, self.yaw_max_deg = dist_bounds, yaw_max_deg
        self.scale_warn = scale_warn
        self.orb = cv2.ORB_create(nfeatures=nfeatures, fastThreshold=12)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.prev = None          # (desc, pts3d, assigned_xzyaw, dr_xzyaw)

    def _features(self, right_rect, Z, valid, lvl):
        gray = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        kps, desc = self.orb.detectAndCompute(gray, None)
        if not kps:
            return None, None
        uv = np.array([k.pt for k in kps])
        pts, keep = lift_keypoints(uv, Z, valid, self.calib,
                                   self.z_min, self.z_max, lvl)
        if keep.sum() < 6:
            return None, None
        return desc[keep], pts

    def _match(self, d_prev, d_cur, p_prev, p_cur):
        pairs = self.bf.knnMatch(d_prev, d_cur, k=2)
        idx = [(p[0].queryIdx, p[0].trainIdx) for p in pairs
               if len(p) == 2 and p[0].distance < 0.75 * p[1].distance]
        if not idx:
            return None, None
        qi, ti = np.array(idx).T
        return p_prev[qi], p_cur[ti]

    def update(self, right_rect, Z, valid, lvl, dr_xzyaw):
        """→ (xzyaw to place this stop at, one-line note for the stop log)."""
        feats = self._features(right_rect, Z, valid, lvl)
        prev = self.prev
        # fallback pose: chain the commanded relative step onto the last
        # assigned pose (never the absolute dead-reckon, which may have
        # diverged from earlier VO corrections)
        if prev is None:
            pose = dr_xzyaw
            note = 'vo: first stop'
        else:
            yaw_dr, t_dr = relative(prev[3], dr_xzyaw)
            pose = compose(prev[2], yaw_dr, t_dr)
            note = 'vo: no features -> dead-reckon'
            if feats[0] is not None and prev[0] is not None:
                pose, note = self._vo_pose(prev, feats, pose,
                                           float(np.linalg.norm(t_dr)))
        if feats[0] is not None:
            self.prev = (feats[0], feats[1], pose, dr_xzyaw)
        else:                       # keep chaining dr steps through bad stops
            self.prev = (None, None, pose, dr_xzyaw)
        return pose, note

    def _vo_pose(self, prev, feats, fallback, dr_dist):
        P, Q = self._match(prev[0], feats[0], prev[1], feats[1])
        if P is None:
            return fallback, 'vo: 0 matches -> dead-reckon'
        m = estimate_motion(P, Q, min_inliers=self.min_inliers)
        if m is None:
            return fallback, (f'vo: REJECTED (matches={len(P)}, no confident '
                              f'fit) -> dead-reckon')
        d = float(np.linalg.norm(m['t']))
        yaw_deg = np.degrees(m['yaw'])
        tag = ''
        if abs(m['scale'] - 1.0) > self.scale_warn:
            tag = f'  SCALE? x{m["scale"]:.2f}'
        lo, hi = self.dist_bounds
        # scale off 1 is a HARD reject: the metric fit of one of the two stops
        # is compressed/stretched, so the VO translation is off by that factor
        sane = (m['rms'] <= self.rms_max and abs(yaw_deg) <= self.yaw_max_deg
                and abs(m['scale'] - 1.0) <= self.scale_warn
                and (d <= 0.15 if dr_dist < 0.05
                     else lo * dr_dist <= d <= hi * dr_dist))
        detail = (f'd={d:.3f}m (cmd {dr_dist:.2f}) yaw={yaw_deg:+.1f}deg '
                  f'inl={m["n_inl"]}/{m["n_matches"]} rms={m["rms"]:.3f}')
        if not sane:
            return fallback, f'vo: REJECTED ({detail}) -> dead-reckon{tag}'
        return compose(prev[2], m['yaw'], m['t']), f'vo: {detail}{tag}'
