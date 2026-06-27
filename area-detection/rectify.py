#!/usr/bin/env python3
"""
rectify.py — uncalibrated stereo rectification for the robot's USB cameras.

The original repo assumed KITTI's pre-rectified pairs. These USB cameras are not
rectified and there is no calibration file, so we recover rectifying homographies
from feature matches (ORB -> fundamental matrix -> stereoRectifyUncalibrated).

Ported from stereo-camera/tools/freespace.py. Homographies are computed once on the
first frame and cached (cameras are rigidly mounted); call reset_cache() after a
physical bump. Run a proper stereo calibration later for cleaner disparity.
"""
import numpy as np
import cv2

_cached_H = None   # (H1, H2)


def reset_cache():
    global _cached_H
    _cached_H = None


def rectify_pair(left_bgr, right_bgr, min_inliers=20):
    """
    left_bgr, right_bgr : raw stereo frames (BGR).
    Returns (left_bgr_rect, left_gray_rect, right_gray_rect, ok).
    On failure (ok=False) the inputs are returned unchanged so the caller can degrade.
    """
    global _cached_H
    gray_l = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    h, w = gray_l.shape

    if _cached_H is not None:
        h1, h2 = _cached_H
        return (cv2.warpPerspective(left_bgr, h1, (w, h)),
                cv2.warpPerspective(gray_l, h1, (w, h)),
                cv2.warpPerspective(gray_r, h2, (w, h)), True)

    print("[rectify] Computing rectification (ORB, first frame only)...", flush=True)
    orb = cv2.ORB_create(nfeatures=2000)
    kp1, d1 = orb.detectAndCompute(gray_l, None)
    kp2, d2 = orb.detectAndCompute(gray_r, None)
    if d1 is None or d2 is None or len(kp1) < 8 or len(kp2) < 8:
        return left_bgr, gray_l, gray_r, False

    raw = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(d1, d2, k=2)
    good = [m for m, n in raw if m.distance < 0.85 * n.distance]
    if len(good) < min_inliers:
        return left_bgr, gray_l, gray_r, False

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1.0, 0.99)
    if F is None or mask is None:
        return left_bgr, gray_l, gray_r, False

    sel = mask.ravel().astype(bool)
    if sel.sum() < min_inliers:
        return left_bgr, gray_l, gray_r, False

    _, h1, h2 = cv2.stereoRectifyUncalibrated(pts1[sel], pts2[sel], F, (w, h))
    _cached_H = (h1, h2)
    print(f"[rectify] Cached homographies ({int(sel.sum())} inliers).", flush=True)
    return (cv2.warpPerspective(left_bgr, h1, (w, h)),
            cv2.warpPerspective(gray_l, h1, (w, h)),
            cv2.warpPerspective(gray_r, h2, (w, h)), True)
