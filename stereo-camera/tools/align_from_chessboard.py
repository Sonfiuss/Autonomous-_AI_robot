#!/usr/bin/env python3
"""
align_from_chessboard.py — compute a right→left alignment warp from chessboard pairs.

Reads chessboard_left*.jpg / chessboard_right*.jpg pairs (board shown on a laptop
screen is fine — the board may be cut off at the edges), estimates a partial-affine
warp (dx, dy, rotation, scale) that maps the RIGHT camera image onto the LEFT camera
frame, and saves it to calib/alignment.yml. Use the warp as a pre-process step to
normalize the right camera before any stereo processing.

IMPORTANT depth caveat (learned 2026-06-23, see claude_action_log.md): the translation
part of the warp is only valid at the depth where the board was shown; rotation and
scale are depth-independent. The tool therefore estimates each pair separately and
warns when pairs taken at different distances disagree on translation.

Detection: findChessboardCornersSBWithMeta + CALIB_CB_LARGER, so a partially visible
board still yields a corner grid. Because the board is cropped differently per camera,
the left/right grids may be offset by whole squares — and since the board is planar,
ANY integer-square shift fits an affine model equally well. The ambiguity is resolved
with an ORB feature prior on the full scene (laptop bezel, background objects).

Usage:
  python3 align_from_chessboard.py                      # compute from tools/captures
  python3 align_from_chessboard.py --captures DIR --config PATH
  python3 align_from_chessboard.py --apply right.jpg    # normalize an image with the config
"""
import argparse
import sys
from datetime import date
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent
_DEFAULT_CAPTURES = _HERE / "captures"
_DEFAULT_CONFIG = _HERE.parent / "calib" / "alignment.yml"

_SB_FLAGS = (cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
             | cv2.CALIB_CB_LARGER)
_SEED_PATTERN = (9, 6)          # seed for CALIB_CB_LARGER; real grid may be bigger
_MAX_GRID_SHIFT = 6             # search ±N squares for L/R grid correspondence
_RANSAC_THRESH_PX = 3.0
_TRANSLATION_AGREE_PX = 15.0    # pairs differing more than this → depth-dependent
_ANGLE_AGREE_DEG = 0.3


def _detect_grid(gray):
    """Detect a (possibly cropped) chessboard. Returns (corners[rows,cols,2], ok)."""
    ret, corners, meta = cv2.findChessboardCornersSBWithMeta(
        gray, _SEED_PATTERN, _SB_FLAGS)
    if not ret:
        return None
    rows, cols = meta.shape[:2]
    return corners.reshape(rows, cols, 2).astype(np.float64)


def _orb_prior(left, right):
    """Rough right→left displacement at image center from scene features.

    Needed only to pick the correct integer grid shift, so ±(square/2) accuracy
    is enough. Returns (dx, dy) or None.
    """
    orb = cv2.ORB_create(nfeatures=4000)
    kp_l, des_l = orb.detectAndCompute(left, None)
    kp_r, des_r = orb.detectAndCompute(right, None)
    if des_l is None or des_r is None:
        return None
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = matcher.knnMatch(des_r, des_l, k=2)
    good = [m for m, n in (p for p in knn if len(p) == 2)
            if m.distance < 0.75 * n.distance]
    if len(good) < 12:
        return None
    src = np.float32([kp_r[m.queryIdx].pt for m in good])
    dst = np.float32([kp_l[m.trainIdx].pt for m in good])
    M, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC,
                                         ransacReprojThreshold=5.0)
    if M is None or inl.sum() < 12:
        return None
    h, w = left.shape[:2]
    c = np.array([w / 2.0, h / 2.0, 1.0])
    dx, dy = (M @ c) - c[:2]
    return float(dx), float(dy), int(inl.sum())


def _overlap_points(grid_l, grid_r, di, dj):
    """Corner pairs assuming left(i,j) ↔ right(i+di, j+dj)."""
    rl, cl = grid_l.shape[:2]
    rr, cr = grid_r.shape[:2]
    i0, i1 = max(0, -di), min(rl, rr - di)
    j0, j1 = max(0, -dj), min(cl, cr - dj)
    if i1 - i0 < 3 or j1 - j0 < 3:
        return None, None
    pts_l = grid_l[i0:i1, j0:j1].reshape(-1, 2)
    pts_r = grid_r[i0 + di:i1 + di, j0 + dj:j1 + dj].reshape(-1, 2)
    return pts_l.astype(np.float32), pts_r.astype(np.float32)


def _fit(pts_r, pts_l):
    """Partial affine right→left. Returns (M, rms, n_inliers) or None."""
    M, inl = cv2.estimateAffinePartial2D(pts_r, pts_l, method=cv2.RANSAC,
                                         ransacReprojThreshold=_RANSAC_THRESH_PX)
    if M is None:
        return None
    inl = inl.ravel().astype(bool)
    if inl.sum() < 8:
        return None
    proj = pts_r[inl] @ M[:, :2].T + M[:, 2]
    rms = float(np.sqrt(np.mean(np.sum((proj - pts_l[inl]) ** 2, axis=1))))
    return M, rms, int(inl.sum())


def _center_shift(M, shape):
    h, w = shape[:2]
    c = np.array([w / 2.0, h / 2.0, 1.0])
    dx, dy = (M @ c) - c[:2]
    return float(dx), float(dy)


def _decompose(M):
    scale = float(np.hypot(M[0, 0], M[1, 0]))
    angle = float(np.degrees(np.arctan2(M[1, 0], M[0, 0])))
    return scale, angle


def _align_pair(left, right, prior):
    """Full pipeline for one pair. Returns result dict or None."""
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    grid_l = _detect_grid(gray_l)
    grid_r = _detect_grid(gray_r)
    if grid_l is None or grid_r is None:
        return None

    # Try every integer grid shift (and a 180° board flip); each fits the planar
    # board equally well, so rank by distance of the implied center shift to the
    # scene-feature prior, then by RMS.
    candidates = []
    for flipped in (False, True):
        g_r = grid_r[::-1, ::-1] if flipped else grid_r
        for di in range(-_MAX_GRID_SHIFT, _MAX_GRID_SHIFT + 1):
            for dj in range(-_MAX_GRID_SHIFT, _MAX_GRID_SHIFT + 1):
                pts_l, pts_r = _overlap_points(grid_l, g_r, di, dj)
                if pts_l is None:
                    continue
                fit = _fit(pts_r, pts_l)
                if fit is None:
                    continue
                M, rms, n_inl = fit
                dx, dy = _center_shift(M, left.shape)
                if prior is not None:
                    d_prior = np.hypot(dx - prior[0], dy - prior[1])
                else:
                    d_prior = np.hypot(dx, dy)   # no prior: assume smallest motion
                candidates.append((d_prior, rms, M, n_inl, (di, dj, flipped)))
    if not candidates:
        return None
    candidates.sort(key=lambda c: (c[0], c[1]))
    d_prior, rms, M, n_inl, shift = candidates[0]
    dx, dy = _center_shift(M, left.shape)
    scale, angle = _decompose(M)
    return {"M": M, "rms": rms, "inliers": n_inl, "dx": dx, "dy": dy,
            "scale": scale, "angle": angle, "grid_shift": shift,
            "prior_dist": d_prior,
            "corners_l": int(grid_l.shape[0] * grid_l.shape[1]),
            "corners_r": int(grid_r.shape[0] * grid_r.shape[1])}


def _anaglyph(left, right):
    """Left=green, right=red; aligned regions look yellow/gray."""
    g_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    g_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    return cv2.merge([np.zeros_like(g_l), g_l, g_r])


def _find_pairs(cap_dir):
    pairs = []
    for lp in sorted(cap_dir.glob("chessboard_left*.*")):
        rp = lp.with_name(lp.name.replace("left", "right"))
        if rp.exists():
            pairs.append((lp, rp))
    return pairs


def compute(args):
    cap_dir = Path(args.captures)
    pairs = _find_pairs(cap_dir)
    if not pairs:
        sys.exit(f"[align] No chessboard_left*/right* pairs in {cap_dir}")
    print(f"[align] {len(pairs)} pair(s) in {cap_dir}")

    results = []
    for lp, rp in pairs:
        left = cv2.imread(str(lp))
        right = cv2.imread(str(rp))
        if left is None or right is None:
            print(f"[align] {lp.name}: cannot read pair, skipped")
            continue
        prior = _orb_prior(left, right)
        if prior is not None:
            print(f"[align] {lp.name}: ORB prior dx={prior[0]:+.1f} "
                  f"dy={prior[1]:+.1f} ({prior[2]} inliers)")
        else:
            print(f"[align] {lp.name}: WARNING no ORB prior — grid shift may be "
                  f"off by whole squares")
        res = _align_pair(left, right, prior)
        if res is None:
            print(f"[align] {lp.name}: chessboard not detected, skipped")
            continue
        res.update(left=lp, right=rp, shape=left.shape)
        results.append(res)
        di, dj, flip = res["grid_shift"]
        print(f"[align] {lp.name}: dx={res['dx']:+.1f}px dy={res['dy']:+.1f}px "
              f"rot={res['angle']:+.2f}deg scale={res['scale']:.4f} "
              f"rms={res['rms']:.2f}px inliers={res['inliers']} "
              f"(grid shift di={di} dj={dj} flip={flip})")

        # before/after anaglyph for eyeball check
        warped = cv2.warpAffine(right, res["M"], (left.shape[1], left.shape[0]))
        check = np.hstack([_anaglyph(left, right), _anaglyph(left, warped)])
        cv2.putText(check, "BEFORE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (255, 255, 255), 2)
        cv2.putText(check, "AFTER", (left.shape[1] + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        out = cap_dir / f"align_check_{lp.stem.replace('chessboard_left', 'pair')}.jpg"
        cv2.imwrite(str(out), check)
        print(f"[align]   check image -> {out.name}")

    if not results:
        sys.exit("[align] No usable pair.")

    # Cross-pair consistency: translation is depth-dependent, rotation/scale not.
    consistent = True
    if len(results) > 1:
        dxs = [r["dx"] for r in results]
        dys = [r["dy"] for r in results]
        angs = [r["angle"] for r in results]
        t_spread = max(np.ptp(dxs), np.ptp(dys))
        a_spread = np.ptp(angs)
        if t_spread > _TRANSLATION_AGREE_PX:
            consistent = False
            print(f"[align] WARNING translation differs {t_spread:.1f}px between "
                  f"pairs -> depth-dependent. The saved warp is only valid near "
                  f"the board distance; rotation/scale remain trustworthy.")
        if a_spread > _ANGLE_AGREE_DEG:
            consistent = False
            print(f"[align] WARNING rotation differs {a_spread:.2f}deg between "
                  f"pairs -> check captures (blur / detection error).")

    M_avg = np.mean([r["M"] for r in results], axis=0)
    dx, dy = _center_shift(M_avg, results[0]["shape"])
    scale, angle = _decompose(M_avg)
    print(f"[align] AVERAGE: dx={dx:+.1f}px dy={dy:+.1f}px rot={angle:+.2f}deg "
          f"scale={scale:.4f} consistent={consistent}")

    cfg_path = Path(args.config)
    cfg_path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(cfg_path), cv2.FILE_STORAGE_WRITE)
    fs.write("direction", "warp maps RIGHT image into LEFT frame (cv2.warpAffine)")
    fs.write("date", date.today().isoformat())
    fs.write("image_width", results[0]["shape"][1])
    fs.write("image_height", results[0]["shape"][0])
    fs.write("warp", M_avg)
    # Stereo-depth variant: keep the horizontal offset (it IS the disparity /
    # depth signal — see memory project-stereo-alignment), correct only the
    # vertical offset + rotation + scale.
    M_stereo = M_avg.copy()
    M_stereo[0, 2] -= dx
    fs.write("warp_stereo", M_stereo)
    fs.write("dx_center_px", dx)
    fs.write("dy_center_px", dy)
    fs.write("rotation_deg", angle)
    fs.write("scale", scale)
    fs.write("translation_consistent_across_depth", int(consistent))
    fs.write("num_pairs", len(results))
    for i, r in enumerate(results, 1):
        fs.write(f"pair{i}_images", f"{r['left'].name} | {r['right'].name}")
        fs.write(f"pair{i}_warp", r["M"])
        fs.write(f"pair{i}_dx", r["dx"])
        fs.write(f"pair{i}_dy", r["dy"])
        fs.write(f"pair{i}_rotation_deg", r["angle"])
        fs.write(f"pair{i}_scale", r["scale"])
        fs.write(f"pair{i}_rms_px", r["rms"])
        fs.write(f"pair{i}_inliers", r["inliers"])
    fs.release()
    print(f"[align] config saved -> {cfg_path}")


def apply(args):
    cfg_path = Path(args.config)
    if not cfg_path.exists():
        sys.exit(f"[align] config not found: {cfg_path} (run compute mode first)")
    fs = cv2.FileStorage(str(cfg_path), cv2.FILE_STORAGE_READ)
    key = "warp_stereo" if args.stereo else "warp"
    M = fs.getNode(key).mat()
    fs.release()
    if M is None:
        sys.exit(f"[align] '{key}' not in {cfg_path} — re-run compute mode")
    img = cv2.imread(args.apply)
    if img is None:
        sys.exit(f"[align] cannot read {args.apply}")
    warped = cv2.warpAffine(img, M, (img.shape[1], img.shape[0]))
    out = args.out or str(Path(args.apply).with_name(Path(args.apply).stem
                                                     + "_aligned.jpg"))
    cv2.imwrite(out, warped)
    print(f"[align] normalized image -> {out}")


def main():
    p = argparse.ArgumentParser(
        description="Right→left camera alignment from chessboard pairs.")
    p.add_argument("--captures", default=str(_DEFAULT_CAPTURES),
                   help="folder with chessboard_left*/right* pairs")
    p.add_argument("--config", default=str(_DEFAULT_CONFIG),
                   help="alignment YAML to write (compute) or read (--apply)")
    p.add_argument("--apply", metavar="IMG",
                   help="apply saved config to a right-camera image instead")
    p.add_argument("--out", help="output path for --apply")
    p.add_argument("--stereo", action="store_true",
                   help="with --apply: use warp_stereo (keeps horizontal "
                        "disparity for stereo depth; fixes only dy/rotation)")
    args = p.parse_args()
    if args.apply:
        apply(args)
    else:
        compute(args)


if __name__ == "__main__":
    main()
