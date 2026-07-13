#!/usr/bin/env python3
"""
depth_eval — stereo Z accuracy against a tape measure.

The instrument for Stages A/C/D: given pairs of a target captured at known
(tape-measured) distances, report measured disparity vs expected, Z error in %,
and the implied fb_i = d_med · z_true per target. `--fit-fb` turns those into
the empirical fb correction (Stage C) that kills the calibration's systematic
scale error regardless of the focal-length estimate.

Session folder protocol (captured on the Jetson with capture_eval.sh):
  captures/eval_YYYYMMDD/
    manifest.csv        tag, z_true_m, left, right[, notes]
    d040_left.jpg d040_right.jpg ...
Distance = LEFT camera lens front → target plane, along the optical axis.

Disparity measurement per target (median over many correspondences):
  board mode  printed 9×6 chessboard as target → 54 subpixel corners (~0.1 px)
  roi mode    any textured target + --roi x,y,w,h (in the LEFT RECTIFIED
              image) → golden_points() restricted to the box

Usage:
  python depth_eval.py --selftest                      # trust the tool first
  python depth_eval.py --session captures/eval_20260711
  python depth_eval.py --session ... --fit-fb          # Stage C fb fit
  python depth_eval.py --session ... --roi 500,300,300,200
No torch needed (golden_points is pure cv2/numpy).
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
import calib_io                                  # noqa: E402
from stereo_ruler import golden_points           # noqa: E402

_DEF_RECTIFY = _HERE.parent / "calib" / "stereo_rectify.yml"


# ------------------------------------------------------------- measurement ---
def board_disparity(gray_l, gray_r, pattern):
    """Full-board corners in both rectified images → per-corner disparity.
    Returns (d_med, n, dy_p90) or None. Disparity = xL − xR (positive)."""
    cl = calib_io.find_board(gray_l, pattern)
    cr = calib_io.find_board(gray_r, pattern)
    if cl is None or cr is None:
        return None
    disp = cl[:, 0] - cr[:, 0]
    dy = np.abs(cl[:, 1] - cr[:, 1])
    return float(np.median(disp)), len(disp), float(np.percentile(dy, 90))


def roi_disparity(gray_l, gray_r, roi, d_max=160):
    """golden_points filtered to a left-rectified ROI box.
    Returns (d_med, n, None) or None."""
    x, y, w, h = roi
    pts = golden_points(gray_l, gray_r, d_max=d_max)
    disp = [g["disp"] for g in pts
            if x <= g["x"] < x + w and y <= g["y"] < y + h and g["disp"] > 0.5]
    if len(disp) < 5:
        return None
    return float(np.median(disp)), len(disp), None


def measure(gray_l, gray_r, pattern, roi, mode):
    """→ (mode_used, d_med, n, dy_p90_or_None) or (None, ...) when nothing hit."""
    if mode in ("auto", "board") and pattern is not None:
        r = board_disparity(gray_l, gray_r, pattern)
        if r is not None:
            return ("board",) + r
        if mode == "board":
            return None, None, None, None
    if roi is not None:
        r = roi_disparity(gray_l, gray_r, roi)
        if r is not None:
            return ("roi",) + r
    return None, None, None, None


# ------------------------------------------------------------------ fb fit ---
def fit_fb(rows, z_weight_max=1.6):
    """rows = [(tag, z_true, d_med), ...] → dict with fb_median, fb_fit, d0.

    fb_median: median of d·z over well-conditioned targets (z ≤ z_weight_max —
    at 2 m one row is worth <1 px, its d·z is noisier).
    fb_fit/d0:  least squares  d = fb·(1/z) + d0 ; a |d0| > ~0.3 px reveals a
    residual disparity bias (principal point / toe-in) worth storing.
    """
    z = np.array([r[1] for r in rows], float)
    d = np.array([r[2] for r in rows], float)
    near = z <= z_weight_max
    fb_med = float(np.median((d * z)[near])) if near.any() else float(np.median(d * z))
    A = np.stack([1.0 / z, np.ones_like(z)], axis=1)
    (fb_fit, d0), *_ = np.linalg.lstsq(A, d, rcond=None)
    return {"fb_median": fb_med, "fb_fit": float(fb_fit), "d0_px": float(d0),
            "n": len(rows), "spread": float(np.ptp(d * z))}


def write_fb(yml_path, fb_measured, d0_px, targets):
    """Append the empirical keys to the ACTIVE yml — after archiving a copy."""
    p = Path(yml_path)
    arch = p.parent / "archive"
    arch.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup = arch / f"{p.stem}_pre_fb_{stamp}{p.suffix}"
    backup.write_bytes(p.read_bytes())
    with open(p, "a", encoding="utf-8") as f:
        f.write(f"fb_measured: {fb_measured:.6f}\n")
        if abs(d0_px) > 0.3:
            f.write(f"fb_offset_px: {d0_px:.4f}\n")
        f.write(f'fb_measured_date: "{time.strftime("%Y-%m-%d")}"\n')
        f.write(f"fb_measured_targets: {targets}\n")
    print(f"[depth_eval] fb_measured={fb_measured:.4f} written to {p}")
    print(f"[depth_eval] backup of previous yml -> {backup}")


# ------------------------------------------------------------- golden sweep --
SWEEP_GRID = {
    "d_max": (140, 160),
    "dy_search": ((0,), (0, -1, 1)),
    "grid": ((16, 10), (20, 12)),
    "per_cell": (3, 5),
    "ncc_min": (0.70, 0.75),
}


def _board_bbox(gray_l, pattern, margin=12):
    """Auto target ROI = bounding box of the detected board (+margin)."""
    c = calib_io.find_board(gray_l, pattern)
    if c is None:
        return None
    x0, y0 = c.min(axis=0) - margin
    x1, y1 = c.max(axis=0) + margin
    return int(x0), int(y0), int(x1 - x0), int(y1 - y0)


def sweep_golden(pairs, calib, pattern, roi_arg, out_csv):
    """Grid-sweep golden_points configs over the session pairs (Stage D1).

    Per config: total yield, and on targets with z_true: median |Z err| % and
    per-point outlier rate (>10% Z error) inside the target ROI (auto = board
    bbox). Prints configs ranked by (Z err, then yield)."""
    import itertools
    import time as _time
    fb = calib.fb_used
    keys = list(SWEEP_GRID)
    combos = list(itertools.product(*SWEEP_GRID.values()))
    print(f"[sweep] {len(combos)} configs x {len(pairs)} pairs")

    # pre-rectify once
    prep = []
    for tag, lp, rp, z_true in pairs:
        left, right = calib_io.read_pair(lp, rp)
        lr, rr = calib_io.rectify_pair(calib, left, right)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        roi = roi_arg or (_board_bbox(gl, pattern) if z_true else None)
        prep.append((tag, gl, gr, z_true, roi))

    rows = []
    for combo in combos:
        cfg = dict(zip(keys, combo))
        n_total, errs, n_out, n_pts_tgt = 0, [], 0, 0
        t0 = _time.time()
        for tag, gl, gr, z_true, roi in prep:
            pts = golden_points(gl, gr, **cfg)
            n_total += len(pts)
            if z_true and roi:
                x, y, w, h = roi
                tgt = [g["disp"] for g in pts
                       if x <= g["x"] < x + w and y <= g["y"] < y + h
                       and g["disp"] > 0.5]
                for d in tgt:
                    e = abs(fb / d - z_true) / z_true
                    errs.append(e)
                    n_out += e > 0.10
                n_pts_tgt += len(tgt)
        med = 100 * float(np.median(errs)) if errs else float("nan")
        outr = 100.0 * n_out / max(n_pts_tgt, 1)
        rows.append({**{k: str(v) for k, v in cfg.items()},
                     "yield": n_total, "tgt_pts": n_pts_tgt,
                     "Zerr_med_pct": med, "outlier_pct": outr,
                     "ms": 1000 * (_time.time() - t0) / max(len(prep), 1)})

    rows.sort(key=lambda r: (r["Zerr_med_pct"] if r["Zerr_med_pct"] ==
                             r["Zerr_med_pct"] else 1e9, -r["yield"]))
    print(f"  {'d_max':>6}{'dy':>10}{'grid':>10}{'cell':>5}{'ncc':>5}"
          f"{'yield':>7}{'tgtpts':>7}{'Zerr%':>7}{'outl%':>7}{'ms/pair':>8}")
    for r in rows[:12]:
        print(f"  {r['d_max']:>6}{r['dy_search']:>10}{r['grid']:>10}"
              f"{r['per_cell']:>5}{r['ncc_min']:>5}{r['yield']:>7}"
              f"{r['tgt_pts']:>7}{r['Zerr_med_pct']:>7.2f}"
              f"{r['outlier_pct']:>7.1f}{r['ms']:>8.0f}")
    calib_io.write_csv(out_csv, list(rows[0].keys()),
                       [list(r.values()) for r in rows])


# ---------------------------------------------------------------- selftest ---
def _synth_board_pair(pattern=(9, 6), sq=60, margin=80, disp=20.25, ss=4):
    """Left/right synthetic chessboard views with an EXACT subpixel disparity.

    Both images are rendered independently at ss× supersampling with the board
    horizontally offset by disp px (disp·ss must be integer), then INTER_AREA
    downsampled — band-limited edges like a real camera, no warp interpolation
    bias on the corner localization."""
    assert abs(disp * ss - round(disp * ss)) < 1e-9, "disp must be k/ss"
    cols, rows = pattern[0] + 1, pattern[1] + 1
    W = (cols * sq + 2 * margin), (rows * sq + 2 * margin)

    def render(x_off_ss):
        img = np.full((W[1] * ss, W[0] * ss), 255, np.uint8)
        for r in range(rows):
            for c in range(cols):
                if (r + c) % 2 == 0:
                    y0 = (margin + r * sq) * ss
                    x0 = (margin + c * sq) * ss + x_off_ss
                    img[y0:y0 + sq * ss, x0:x0 + sq * ss] = 20
        img = cv2.resize(img, W, interpolation=cv2.INTER_AREA)
        # lens-PSF-like blur: subpixel corner refinement pixel-locks on the
        # 1px-hard edges of a downsampled binary board; real optics never
        # deliver such edges
        return cv2.GaussianBlur(img, (5, 5), 1.0)

    # right view: board appears disp px to the LEFT (disparity = xL − xR > 0)
    return render(0), render(-int(round(disp * ss)))


def _shift(img, dx):
    """Shift image content LEFT by dx px (disparity = xL − xR = +dx)."""
    M = np.float32([[1, 0, -dx], [0, 1, 0]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]),
                          flags=cv2.INTER_LINEAR, borderValue=255)


def selftest():
    """The instrument must read a KNOWN synthetic shift before it is trusted."""
    ok = True
    true_d = 20.25
    pattern = (9, 6)

    left, right = _synth_board_pair(pattern, disp=true_d)
    r = board_disparity(left, right, pattern)
    err = abs(r[0] - true_d) if r else float("inf")
    print(f"  board mode : d_true={true_d}  d_med={r[0] if r else 'FAIL':.3f}  "
          f"n={r[1] if r else 0}  err={err:.3f}px  "
          f"{'PASS' if err <= 0.10 else 'FAIL'}")
    ok &= err <= 0.10

    true_d = 20.37                                   # warp is fine on texture
    rng = np.random.default_rng(7)
    tex = rng.integers(0, 255, (720, 1280), np.uint8)
    tex = cv2.GaussianBlur(tex, (5, 5), 1.2)         # correlated texture
    right = _shift(tex, true_d)
    r = roi_disparity(tex, right, (200, 100, 880, 520), d_max=160)
    err = abs(r[0] - true_d) if r else float("inf")
    print(f"  roi mode   : d_true={true_d}  d_med={r[0] if r else 'FAIL':.3f}  "
          f"n={r[1] if r else 0}  err={err:.3f}px  "
          f"{'PASS' if err <= 0.15 else 'FAIL'}")
    ok &= err <= 0.15

    print(f"selftest: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


# -------------------------------------------------------------------- main ---
def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Stereo Z accuracy vs tape measure",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--session", help="dir with manifest.csv (tag,z_true_m,left,right)")
    ap.add_argument("--left"), ap.add_argument("--right")
    ap.add_argument("--z-true", type=float, default=None,
                    help="tape distance for a single --left/--right pair (m)")
    ap.add_argument("--rectify", default=str(_DEF_RECTIFY))
    ap.add_argument("--pattern", default="9x6")
    ap.add_argument("--mode", choices=["auto", "board", "roi"], default="auto")
    ap.add_argument("--roi", default=None, help="x,y,w,h in LEFT RECTIFIED image")
    ap.add_argument("--sweep-golden", action="store_true",
                    help="grid-sweep golden_points configs over the session "
                         "(Stage D1); ranks by Z error then yield")
    ap.add_argument("--fit-fb", action="store_true",
                    help="fit empirical fb over the session targets (Stage C)")
    ap.add_argument("--write-fb", action="store_true",
                    help="with --fit-fb: append fb_measured to the active yml "
                         "(archives a backup copy first)")
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    if args.selftest:
        return selftest()
    if not args.session and not (args.left and args.right):
        ap.error("need --session DIR or --left/--right (or --selftest)")

    calib = calib_io.load_calib(args.rectify)
    fb = calib.fb_used
    pattern = None
    if args.pattern:
        c, r = args.pattern.lower().split("x")
        pattern = (int(c), int(r))
    roi = tuple(int(v) for v in args.roi.split(",")) if args.roi else None
    print(f"calib={Path(args.rectify).name} ({calib.schema})  fb={fb:.3f}"
          f"{' (fb_measured)' if calib.fb_measured else ''}  "
          f"offset={calib.fb_offset_px:+.2f}px")

    if args.sweep_golden:
        pairs = list(calib_io.iter_pairs(args.session, args.left, args.right))
        out = args.out or str(Path(args.session or Path(args.left).parent)
                              / "golden_sweep.csv")
        sweep_golden(pairs, calib, pattern, roi, out)
        return 0

    header = ["tag", "mode", "n", "z_true_m", "d_med_px", "d_expected_px",
              "d_err_px", "Z_meas_m", "Z_err_pct", "fb_i", "dy_p90"]
    rows_csv, rows_fb = [], []
    print(f"  {'tag':<12}{'mode':<7}{'n':>4}{'z_true':>8}{'d_med':>9}"
          f"{'d_exp':>8}{'d_err':>7}{'Z_meas':>8}{'Z_err%':>8}{'fb_i':>8}")
    for tag, lp, rp, z_true in calib_io.iter_pairs(args.session, args.left,
                                                   args.right):
        if z_true is None:
            z_true = args.z_true
        left, right = calib_io.read_pair(lp, rp)
        calib_io.check_size(left, calib, tag)
        lr, rr = calib_io.rectify_pair(calib, left, right)
        gl = cv2.cvtColor(lr, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rr, cv2.COLOR_BGR2GRAY)
        mode, d_med, n, dy90 = measure(gl, gr, pattern, roi, args.mode)
        if mode is None:
            print(f"  {tag:<12} NO measurement (no board / <5 golden in ROI)")
            rows_csv.append([tag, "none", 0, z_true or "", "", "", "", "", "",
                             "", ""])
            continue
        d_eff = d_med - calib.fb_offset_px          # bias-corrected disparity
        z_meas = fb / d_eff if d_eff > 0.1 else float("inf")
        if z_true:
            d_exp = fb / z_true + calib.fb_offset_px
            zerr = 100.0 * (z_meas - z_true) / z_true
            fb_i = d_eff * z_true
            rows_fb.append((tag, z_true, d_eff))
            print(f"  {tag:<12}{mode:<7}{n:>4}{z_true:>8.3f}{d_med:>9.2f}"
                  f"{d_exp:>8.2f}{d_med - d_exp:>7.2f}{z_meas:>8.3f}"
                  f"{zerr:>+8.1f}{fb_i:>8.2f}")
            rows_csv.append([tag, mode, n, z_true, f"{d_med:.3f}",
                             f"{d_exp:.3f}", f"{d_med - d_exp:.3f}",
                             f"{z_meas:.4f}", f"{zerr:.2f}", f"{fb_i:.3f}",
                             f"{dy90:.3f}" if dy90 is not None else ""])
        else:
            print(f"  {tag:<12}{mode:<7}{n:>4}{'-':>8}{d_med:>9.2f}"
                  f"{'':>8}{'':>7}{z_meas:>8.3f}")
            rows_csv.append([tag, mode, n, "", f"{d_med:.3f}", "", "",
                             f"{z_meas:.4f}", "", "",
                             f"{dy90:.3f}" if dy90 is not None else ""])

    if args.fit_fb:
        if len(rows_fb) < 3:
            print("[depth_eval] --fit-fb needs >=3 targets with z_true")
            return 1
        r = fit_fb(rows_fb)
        print(f"\n  fb fit over {r['n']} targets: fb_median={r['fb_median']:.3f}"
              f"  fb_lstsq={r['fb_fit']:.3f}  d0={r['d0_px']:+.3f}px"
              f"  spread(d*z)={r['spread']:.3f}")
        print(f"  (calib fb={calib.fb:.3f} -> scale correction "
              f"x{r['fb_median'] / calib.fb:.4f})")
        if args.write_fb:
            # consumers compute Z = fb/(d - fb_offset_px); when the offset is
            # stored (|d0|>0.3) the matching scale is the lstsq fb, not the
            # zero-offset median — mixing them biases Z by d0/d
            fb_w = r["fb_fit"] if abs(r["d0_px"]) > 0.3 else r["fb_median"]
            write_fb(args.rectify, fb_w, r["d0_px"], r["n"])

    out = args.out or str(Path(args.session or Path(args.left).parent)
                          / "depth_eval_report.csv")
    calib_io.write_csv(out, header, rows_csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
