#!/usr/bin/env python3
"""
stereo.py — dense disparity (depth) map from a rectified stereo pair.

Refactored from sajaysurya/drivable_area_detection (stereo.py) to:
  - parameterise the KITTI-tuned magic numbers (this robot uses 640x480, not 1242x375),
  - expose a small Config so run.py can tune from the CLI,
  - keep the OpenCV SGBM + ximgproc WLS pipeline (available in OpenCV 4.2 here).

The returned map is INTEGER pixel disparity in the range [0, num_disparities],
which is exactly what disparity.py expects for its U/V-disparity histograms.
"""
import numpy as np
import cv2


class StereoConfig:
    """SGBM + WLS parameters. Defaults adapted for 640x480 USB cameras."""
    # Upstream used 64 for KITTI. For 640x480 short-baseline USB cams a wider
    # range captures close obstacles; keep a multiple of 16 as OpenCV requires.
    num_disparities = 96
    block_size      = 3
    uniqueness      = 25
    # WLS filter (edge-preserving disparity smoothing)
    wls_lambda      = 8000.0
    wls_sigma       = 1.5
    # Stereo geometry — used to turn disparity into a real distance.
    baseline_mm     = 54.0    # physical separation of the two camera optical centres
    # Rectified focal length in pixels. This is an ESTIMATE for a ~70deg-HFOV 640px
    # camera (focal_px = (width/2) / tan(HFOV/2)); replace with the calibrated value
    # (P1[0,0] from a stereo.yml) for accurate distances.
    focal_px        = 457.0


def _build_matchers(cfg):
    win = cfg.block_size
    left = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=cfg.num_disparities,
        blockSize=win,
        P1=8 * 3 * win ** 2,
        P2=32 * 3 * win ** 2,
        uniquenessRatio=cfg.uniqueness,
    )
    right = cv2.ximgproc.createRightMatcher(left)
    wls = cv2.ximgproc.createDisparityWLSFilter(left)
    wls.setLambda(cfg.wls_lambda)
    wls.setSigmaColor(cfg.wls_sigma)
    return left, right, wls


def get_depth_map(l_image, r_image, cfg=StereoConfig):
    """
    Compute a dense, WLS-smoothed integer disparity map.

    l_image, r_image : rectified left/right frames (BGR or gray, same size).
    cfg              : StereoConfig (class or instance).

    Returns an int ndarray of per-pixel disparity in [0, num_disparities].
    (OpenCV returns disparity * 16; we add 16 then divide by 16 so the minimum
    valid disparity maps to 1 and stays strictly positive, matching upstream.)
    """
    left, right, wls = _build_matchers(cfg)
    l_map = left.compute(l_image, r_image)
    r_map = right.compute(r_image, l_image)
    depth = wls.filter(l_map, l_image, None, r_map)
    depth = ((depth + 16) / 16.0).astype(int)
    # Clamp into the valid histogram range so disparity.py never indexes past Dmax.
    np.clip(depth, 0, cfg.num_disparities, out=depth)
    return depth


def disparity_to_distance_mm(disparity, cfg=StereoConfig):
    """
    Convert pixel disparity to metric distance: dist = focal_px * baseline_mm / disparity.
    Accepts a scalar or ndarray; disparity <= 0 (no match) maps to +inf.
    """
    disp = np.asarray(disparity, dtype=np.float64)
    dist = np.full(disp.shape, np.inf, dtype=np.float64)
    valid = disp > 0
    dist[valid] = cfg.focal_px * cfg.baseline_mm / disp[valid]
    return dist


def main(l_path, r_path):
    import matplotlib.pyplot as plt
    l_image = cv2.imread(l_path)
    r_image = cv2.imread(r_path)
    depth = get_depth_map(l_image, r_image)
    plt.imshow(depth, vmin=0, vmax=StereoConfig.num_disparities)
    plt.colorbar()
    plt.show()


if __name__ == "__main__":
    import sys
    main(sys.argv[1], sys.argv[2])
