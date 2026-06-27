#!/usr/bin/env python3
"""
disparity.py — U-disparity and V-disparity maps from an integer depth map.

Upstream (utilities.onehot_initialization + freespace.get_disparity) built a dense
(H, W, Dmax+1) one-hot cube and summed it. For a 480x640 frame with Dmax~96 that is
~238 MB of int64 per frame — far too heavy for the Jetson. This computes the exact
same two histograms with np.bincount, allocating only the small output maps.

Definitions (identical to upstream):
  u_disparity[d, c] = number of pixels in image column c whose disparity == d   -> (Dmax+1, W)
  v_disparity[r, d] = number of pixels in image row    r whose disparity == d   -> (H, Dmax+1)
"""
import numpy as np


def u_v_disparity(depth_map, dmax):
    """
    depth_map : int ndarray (H, W), values in [0, dmax].
    dmax      : maximum disparity (defines histogram width).
    Returns (v_disparity, u_disparity) matching upstream get_disparity() order.
    """
    h, w = depth_map.shape
    nbins = int(dmax) + 1
    depth = depth_map.astype(np.int64, copy=False)

    # V-disparity: per-row histogram. Offset each row into its own bincount block.
    row_idx = np.arange(h, dtype=np.int64)[:, None]
    v_flat = np.bincount((depth + nbins * row_idx).ravel(), minlength=h * nbins)
    v_disparity = v_flat[: h * nbins].reshape(h, nbins)

    # U-disparity: per-column histogram, then transpose to (Dmax+1, W).
    col_idx = np.arange(w, dtype=np.int64)[None, :]
    u_flat = np.bincount((depth + nbins * col_idx).ravel(), minlength=w * nbins)
    u_disparity = u_flat[: w * nbins].reshape(w, nbins).T

    return v_disparity, u_disparity
