#!/usr/bin/env python3
"""
sgbm_probe — semi-dense SGBM experiment on the CALIBRATED rectification.

The repo's existing SGBM tools were never evidence against dense stereo:
`area-detection/stereo.py` uses a guessed focal (457 px) and `freespace.py`
rectifies uncalibrated. This probe wires SGBM to the real rectify maps and the
real fb for the first time, filters to high-confidence pixels, and scores it
with the SAME metrics as golden points so sparse-vs-semi-dense is decided by
numbers (Stage D2 decision rule: adopt only if outlier rate ≤ golden's AND
Z error within 1% absolute at all targets).

Confidence filtering:
  - left-right consistency: disparity recomputed with the cameras swapped
    (horizontally flipped) must agree within --lr-tol px (this replaces the
    ximgproc WLS confidence when opencv-contrib is not installed; WLS is used
    additionally when available)
  - speckle removal (cv2.filterSpeckles)
  - texture gate: reject low-contrast blocks (matcher hallucination guard)

Usage:
  python sgbm_probe.py --session captures/eval_20260711            # vs targets
  python sgbm_probe.py --left scene_left.jpg --right scene_right.jpg --show
Output: table + sgbm_report.csv (+ disparity vis jpgs next to the inputs).
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent.parent / "depth-anything" / "src"))
import calib_io                              # noqa: E402
from stereo_ruler import golden_points       # noqa: E402
from depth_eval import _board_bbox           # noqa: E402

_DEF_RECTIFY = _HERE.parent / "calib" / "stereo_rectify.yml"


def make_sgbm(num_disp=160, block=5):
    return cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=num_disp, blockSize=block,
        P1=8 * 3 * block ** 2, P2=32 * 3 * block ** 2,
        disp12MaxDiff=1, preFilterCap=63, uniquenessRatio=10,
        speckleWindowSize=100, speckleRange=32,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def semi_dense_disparity(gray_l, gray_r, num_disp=160, block=5, lr_tol=1.0):
    """→ (disp float32 px, valid bool). High-confidence subset of SGBM.

    LR check without ximgproc: the right-reference disparity equals SGBM run
    on the horizontally flipped pair; a pixel survives when both directions
    agree on the same surface."""
    sgbm = make_sgbm(num_disp, block)
    disp_l = sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
    disp_rf = sgbm.compute(gray_r[:, ::-1], gray_l[:, ::-1]) \
        .astype(np.float32) / 16.0
    disp_r = disp_rf[:, ::-1]                     # right-reference disparity

    h, w = gray_l.shape
    xs = np.arange(w)[None, :].repeat(h, axis=0)
    valid = disp_l > 0.5
    xr = np.clip((xs - disp_l).round().astype(int), 0, w - 1)
    dr = disp_r[np.arange(h)[:, None].repeat(w, 1), xr]
    valid &= (dr > 0.5) & (np.abs(disp_l - dr) <= lr_tol)

    # texture gate: SGBM fills flat regions by smoothness alone
    grad = cv2.Sobel(gray_l, cv2.CV_32F, 1, 0, ksize=3)
    tex = cv2.boxFilter(np.abs(grad), -1, (9, 9))
    valid &= tex > 4.0

    disp16 = (disp_l * 16).astype(np.int16)
    cv2.filterSpeckles(disp16, 0, 100, 32 * 16)
    valid &= disp16 > 0

    # optional WLS refinement when opencv-contrib is present
    try:
        import cv2.ximgproc as xi
        left_m = make_sgbm(num_disp, block)
        wls = xi.createDisparityWLSFilter(left_m)
        wls.setLambda(8000.0)
        wls.setSigmaColor(1.5)
        right_m = xi.createRightMatcher(left_m)
        dl16 = left_m.compute(gray_l, gray_r)
        dr16 = right_m.compute(gray_r, gray_l)
        filt = wls.filter(dl16, gray_l, disparity_map_right=dr16)
        conf = wls.getConfidenceMap()
        disp_l = filt.astype(np.float32) / 16.0
        valid &= conf > 200
        print("  [sgbm] ximgproc WLS active")
    except ImportError:
        pass
    return disp_l, valid


def score_roi(disp, valid, roi, fb, z_true):
    """→ (coverage%, Zerr_med%, outlier%>10%, n) inside the target ROI."""
    x, y, w, h = roi
    d = disp[y:y + h, x:x + w]
    v = valid[y:y + h, x:x + w]
    n = int(v.sum())
    if n == 0:
        return 0.0, float("nan"), float("nan"), 0
    z = fb / d[v]
    err = np.abs(z - z_true) / z_true
    return (100.0 * n / v.size, 100 * float(np.median(err)),
            100 * float((err > 0.10).mean()), n)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Semi-dense SGBM vs golden points, calibrated rectify",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--session"), ap.add_argument("--left"), ap.add_argument("--right")
    ap.add_argument("--rectify", default=str(_DEF_RECTIFY))
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--roi", default=None, help="x,y,w,h left-rectified")
    ap.add_argument("--num-disp", type=int, default=160)
    ap.add_argument("--block", type=int, default=5)
    ap.add_argument("--lr-tol", type=float, default=1.0)
    ap.add_argument("--vis", action="store_true", help="save disparity jpgs")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)
    if not args.session and not (args.left and args.right):
        ap.error("need --session DIR or --left/--right")

    calib = calib_io.load_calib(args.rectify)
    fb = calib.fb_used
    c, r = args.pattern.lower().split("x")
    pattern = (int(c), int(r))
    roi_arg = tuple(int(v) for v in args.roi.split(",")) if args.roi else None
    print(f"calib={Path(args.rectify).name}  fb={fb:.3f}  "
          f"numDisp={args.num_disp} block={args.block} lrTol={args.lr_tol}")

    header = ["tag", "z_true_m", "src", "n", "coverage_pct", "Zerr_med_pct",
              "outlier_pct", "ms"]
    rows = []
    print(f"  {'tag':<12}{'z_true':>7} | {'src':<7}{'n':>8}{'cov%':>7}"
          f"{'Zerr%':>8}{'outl%':>7}{'ms':>7}")
    for tag, lp, rp, z_true in calib_io.iter_pairs(args.session, args.left,
                                                   args.right):
        left, right = calib_io.read_pair(lp, rp)
        calib_io.check_size(left, calib, tag)
        lr, rr = calib_io.rectify_pair(calib, left, right)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        roi = roi_arg or (_board_bbox(gl, pattern) if z_true else None)

        t0 = time.time()
        disp, valid = semi_dense_disparity(gl, gr, args.num_disp, args.block,
                                           args.lr_tol)
        ms_s = 1000 * (time.time() - t0)
        t0 = time.time()
        pts = golden_points(gl, gr, d_max=args.num_disp)
        ms_g = 1000 * (time.time() - t0)

        if z_true and roi:
            cov, zmed, outl, n = score_roi(disp, valid, roi, fb, z_true)
            print(f"  {tag:<12}{z_true:>7.2f} | {'sgbm':<7}{n:>8}{cov:>7.1f}"
                  f"{zmed:>8.2f}{outl:>7.1f}{ms_s:>7.0f}")
            rows.append([tag, z_true, "sgbm", n, f"{cov:.1f}", f"{zmed:.2f}",
                         f"{outl:.1f}", f"{ms_s:.0f}"])
            x, y, w, h = roi
            gd = [g["disp"] for g in pts
                  if x <= g["x"] < x + w and y <= g["y"] < y + h
                  and g["disp"] > 0.5]
            if gd:
                z = fb / np.array(gd)
                err = np.abs(z - z_true) / z_true
                print(f"  {'':<12}{'':>7} | {'golden':<7}{len(gd):>8}{'-':>7}"
                      f"{100 * float(np.median(err)):>8.2f}"
                      f"{100 * float((err > 0.10).mean()):>7.1f}{ms_g:>7.0f}")
                rows.append([tag, z_true, "golden", len(gd), "",
                             f"{100 * float(np.median(err)):.2f}",
                             f"{100 * float((err > 0.10).mean()):.1f}",
                             f"{ms_g:.0f}"])
        else:
            n = int(valid.sum())
            print(f"  {tag:<12}{'-':>7} | {'sgbm':<7}{n:>8}"
                  f"{100.0 * n / valid.size:>7.1f}{'-':>8}{'-':>7}{ms_s:>7.0f}")
            print(f"  {'':<12}{'':>7} | {'golden':<7}{len(pts):>8}")
            rows.append([tag, "", "sgbm", n,
                         f"{100.0 * n / valid.size:.1f}", "", "", f"{ms_s:.0f}"])
            rows.append([tag, "", "golden", len(pts), "", "", "", f"{ms_g:.0f}"])

        if args.vis:
            vis = np.zeros_like(disp)
            vis[valid] = disp[valid]
            vis = (255 * np.clip(vis / args.num_disp, 0, 1)).astype(np.uint8)
            vp = Path(lp).with_name(Path(lp).stem + "_sgbm.jpg")
            cv2.imwrite(str(vp), cv2.applyColorMap(vis, cv2.COLORMAP_TURBO))
            print(f"    vis -> {vp}")

    out = args.out or str(Path(args.session or Path(args.left).parent)
                          / "sgbm_report.csv")
    calib_io.write_csv(out, header, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
