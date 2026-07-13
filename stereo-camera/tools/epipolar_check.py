#!/usr/bin/env python3
"""
epipolar_check — measure rectification quality directly.

After correct rectification, a true left/right correspondence lies on the SAME
image row: dy = y_left − y_right = 0. The |dy| distribution is therefore a
direct gauge of the calibration, independent of any downstream matcher:
  |dy| p90 < 0.5 px  → matchers can search a single row (golden points, SGBM)
  |dy| p90 ≈ 2 px    → every row-search matcher hunts on the wrong line (the
                       current 2-pair screen calibration is expected here)

Two gauges, both reported when available:
  chessboard mode — full printed board visible in both images: per-corner dy
                    at ~0.1 px noise (the precise gauge, Stage B pass metric)
  feature mode    — any scene: SIFT/ORB + ratio test + fundamental-RANSAC,
                    ~0.5 px matching noise floor (the coarse gauge)

Also reports L/R brightness delta and Laplacian sharpness per image (capture-
hygiene inputs for Stage D3).

Usage (Windows or Jetson, no torch needed):
  python epipolar_check.py --session captures/eval_20260711
  python epipolar_check.py --left captures/left.jpg --right captures/righ.jpg
  python epipolar_check.py --session captures --rectify ../calib/stereo_rectify_20260712.yml
Output: printed table + epipolar_report.csv next to the input images.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import calib_io  # noqa: E402

_DEF_RECTIFY = _HERE.parent / "calib" / "stereo_rectify.yml"


def _detector():
    """SIFT when the build has it (better subpixel localization), else ORB."""
    if hasattr(cv2, "SIFT_create"):
        return cv2.SIFT_create(nfeatures=4000), cv2.NORM_L2, "sift"
    return cv2.ORB_create(nfeatures=4000, fastThreshold=12), \
        cv2.NORM_HAMMING, "orb"


def feature_dy(gray_l, gray_r, dy_window=5.0, ratio=0.75):
    """Matched-feature dy stats on a rectified pair.

    dy_window bounds the |dy| a match may have and still be counted — wide
    (±5 px) on purpose, so a bad calibration's residual is MEASURED instead of
    clipped; the fundamental-RANSAC pass kills false matches, not large dy.
    Returns (dy array, n_raw_matches) — dy possibly empty.
    """
    det, norm, _ = _detector()
    kl, dl = det.detectAndCompute(gray_l, None)
    kr, dr = det.detectAndCompute(gray_r, None)
    if not kl or not kr:
        return np.array([]), 0
    matches = cv2.BFMatcher(norm).knnMatch(dl, dr, k=2)
    good = [m for m, n in (p for p in matches if len(p) == 2)
            if m.distance < ratio * n.distance]
    if len(good) < 12:
        return np.array([]), len(good)
    pl = np.float32([kl[m.queryIdx].pt for m in good])
    pr = np.float32([kr[m.trainIdx].pt for m in good])
    # geometric outlier rejection (fundamental RANSAC) — removes false matches
    # without constraining dy itself
    _, inl = cv2.findFundamentalMat(pl, pr, cv2.FM_RANSAC, 1.5, 0.999)
    if inl is not None:
        keep = inl.ravel().astype(bool)
        pl, pr = pl[keep], pr[keep]
    dy = pl[:, 1] - pr[:, 1]
    dy = dy[np.abs(dy) <= dy_window]
    return dy, len(good)


def board_dy(gray_l, gray_r, pattern):
    """Per-corner dy on a full chessboard seen by both cameras, or None."""
    cl = calib_io.find_board(gray_l, pattern)
    cr = calib_io.find_board(gray_r, pattern)
    if cl is None or cr is None:
        return None
    # SB returns corners in a consistent board order for both views as long as
    # the full board is found — corresponding rows ARE correspondences.
    return cl[:, 1] - cr[:, 1]


def _stats(dy):
    a = np.abs(dy)
    return (float(dy.mean()), float(np.median(a)),
            float(np.percentile(a, 90)), float(a.max()))


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Rectification quality: |dy| of true correspondences",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--session", help="dir with manifest.csv or *left*/right* pairs")
    ap.add_argument("--left"), ap.add_argument("--right")
    ap.add_argument("--rectify", default=str(_DEF_RECTIFY))
    ap.add_argument("--pattern", default="9x6",
                    help="chessboard inner corners CxR; '' disables board mode")
    ap.add_argument("--dy-window", type=float, default=5.0,
                    help="max |dy| a feature match may have and be counted")
    ap.add_argument("--out", default=None, help="csv path (default: session dir)")
    args = ap.parse_args(argv)
    if not args.session and not (args.left and args.right):
        ap.error("need --session DIR or --left/--right")

    calib = calib_io.load_calib(args.rectify)
    pattern = None
    if args.pattern:
        c, r = args.pattern.lower().split("x")
        pattern = (int(c), int(r))
    print(f"calib={Path(args.rectify).name} ({calib.schema})  "
          f"size={calib.size[0]}x{calib.size[1]}  fb={calib.fb_used:.3f}")

    header = ["tag", "mode", "n", "dy_mean", "dy_p50", "dy_p90", "dy_max",
              "bright_L", "bright_R", "bright_delta", "sharp_L", "sharp_R"]
    rows, all_feat, all_board = [], [], []
    for tag, lp, rp, _z in calib_io.iter_pairs(args.session, args.left, args.right):
        left, right = calib_io.read_pair(lp, rp)
        calib_io.check_size(left, calib, tag)
        lr, rr = calib_io.rectify_pair(calib, left, right)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        bl, br = calib_io.brightness(gl), calib_io.brightness(gr)
        sl, sr = calib_io.sharpness(gl), calib_io.sharpness(gr)

        results = []
        if pattern is not None:
            dyb = board_dy(gl, gr, pattern)
            if dyb is not None:
                results.append(("board", dyb))
                all_board.append(dyb)
        dyf, _nraw = feature_dy(gl, gr, args.dy_window)
        if dyf.size:
            results.append(("feature", dyf))
            all_feat.append(dyf)
        if not results:
            print(f"  {tag:<14} NO measurements (no board, <12 feature matches)")
            rows.append([tag, "none", 0, "", "", "", "",
                         f"{bl:.1f}", f"{br:.1f}", f"{br - bl:+.1f}",
                         f"{sl:.0f}", f"{sr:.0f}"])
            continue
        for mode, dy in results:
            mean, p50, p90, mx = _stats(dy)
            print(f"  {tag:<14} {mode:<8} n={dy.size:<5} "
                  f"dy mean={mean:+.3f}  |dy| p50={p50:.3f} p90={p90:.3f} "
                  f"max={mx:.3f} px   dB={br - bl:+.1f}  sharp={sl:.0f}/{sr:.0f}")
            rows.append([tag, mode, dy.size, f"{mean:.4f}", f"{p50:.4f}",
                         f"{p90:.4f}", f"{mx:.4f}", f"{bl:.1f}", f"{br:.1f}",
                         f"{br - bl:+.1f}", f"{sl:.0f}", f"{sr:.0f}"])

    for name, chunks in (("BOARD", all_board), ("FEATURE", all_feat)):
        if chunks:
            dy = np.concatenate(chunks)
            mean, p50, p90, mx = _stats(dy)
            print(f"\n  ALL-{name}: n={dy.size}  dy mean={mean:+.3f}  "
                  f"|dy| p50={p50:.3f}  p90={p90:.3f}  max={mx:.3f} px")
            rows.append([f"ALL-{name.lower()}", name.lower(), dy.size,
                         f"{mean:.4f}", f"{p50:.4f}", f"{p90:.4f}", f"{mx:.4f}",
                         "", "", "", "", ""])

    out = args.out or str(Path(args.session or Path(args.left).parent)
                          / "epipolar_report.csv")
    calib_io.write_csv(out, header, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
