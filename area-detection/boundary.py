#!/usr/bin/env python3
"""
boundary.py — free-space boundary from U/V-disparity maps.

This is the heart of the original sajaysurya/drivable_area_detection, refactored to
drop the hmmlearn + scikit-learn dependency (not installed on this Jetson). The
banded-transition HMM + Viterbi decode is reimplemented in pure NumPy — same math,
no fragile ARM build.

Two stages, both identical in spirit to upstream:
  1. free_boundary(u_disparity): for every image column, decode the disparity of the
     nearest obstacle. Emission favours columns where the U-disparity accumulation
     exceeds an obstacle-height threshold and prefers closer (higher-disparity)
     obstacles; a banded transition matrix enforces left-right spatial smoothness.
  2. road_plane(v_disparity): Hough-fit the ground line in the V-disparity map, giving
     project(disparity)->image_row so the per-column obstacle disparity becomes a pixel
     row = the free-space boundary in that column.
"""
import numpy as np
import cv2


# ── Banded transition matrix (replicates the scipy.sparse.diags construction) ──
_BAND = [1, 2, 3, 5, 7, 9, 11, 15, 11, 9, 7, 5, 3, 2, 1]
_POSI = [-7, -6, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7]


def _log_transmat(n):
    """Banded matrix + 0.5, column-normalised, transposed — then log. Shape (n, n)."""
    mat = np.zeros((n, n), dtype=np.float64)
    for val, off in zip(_BAND, _POSI):
        if off >= 0:
            i = np.arange(n - off)
            mat[i, i + off] = val
        else:
            i = np.arange(n + off)
            mat[i - off, i] = val
    mat += 0.5
    mat /= mat.sum(axis=0, keepdims=True)
    mat = mat.T
    return np.log(mat)


def _viterbi(log_emission, log_trans, log_start):
    """Standard Viterbi. log_emission: (T, N). Returns most-likely state per step."""
    t_len, n = log_emission.shape
    delta = np.empty((t_len, n), dtype=np.float64)
    psi = np.empty((t_len, n), dtype=np.int64)
    delta[0] = log_start + log_emission[0]
    for t in range(1, t_len):
        scores = delta[t - 1][:, None] + log_trans      # (N from, N to)
        psi[t] = np.argmax(scores, axis=0)
        delta[t] = scores[psi[t], np.arange(n)] + log_emission[t]
    states = np.empty(t_len, dtype=np.int64)
    states[-1] = int(np.argmax(delta[-1]))
    for t in range(t_len - 2, -1, -1):
        states[t] = psi[t + 1, states[t + 1]]
    return states


def free_boundary(u_disparity, obstacle_height=25):
    """
    u_disparity : (Dmax+1, W) histogram from disparity.u_v_disparity.
    obstacle_height : min U-disparity accumulation (px) to count as an obstacle.
    Returns int array of length W: the obstacle disparity chosen for each column.
    """
    num_states, _ = u_disparity.shape          # = Dmax+1
    obs = u_disparity.T.astype(np.float64)      # (W, Dmax+1) = (T, N)

    # Emission: high where an obstacle is present, biased toward closer obstacles.
    log_emission = np.log((obs > obstacle_height).astype(np.float64) + 1e-32)
    closer_bias = (np.arange(num_states) - (num_states - 1)) * 0.1   # prefer high disparity
    log_emission += closer_bias[None, :]

    log_start = np.full(num_states, -np.log(num_states))
    log_trans = _log_transmat(num_states)
    return _viterbi(log_emission, log_trans, log_start)


def road_plane(v_disparity, v_thresh=50, hough_thresh=50):
    """
    Fit the ground line in the V-disparity map.
    Returns project(disparity)->image_row, or None if no plausible line is found.
    """
    mask = (v_disparity > v_thresh).astype(np.uint8)
    lines = cv2.HoughLines(mask, 1, np.pi / 180, hough_thresh)
    if lines is None:
        return None
    lines = lines.reshape(-1, 2)

    # Prefer lines whose angle matches a forward-looking ground plane.
    sel = lines[(lines[:, 1] > 1.5) & (lines[:, 1] < 3.0)]
    line = sel[0] if sel.size else lines[0]
    rho, theta = float(line[0]), float(line[1])
    if abs(np.sin(theta)) < 1e-6:
        return None
    return lambda d: -np.cos(theta) / np.sin(theta) * d + rho / np.sin(theta)
