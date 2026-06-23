#!/usr/bin/env python3
"""
segment2d.py — pure 2D object segmentation (no stereo / no depth).

Goal: outline the objects in a single (left) camera image; whatever is left after
removing the objects is the FLOOR / background. The floor here is speckled
granite/terrazzo under uneven lighting, so a plain colour threshold fails. We use
two illumination-robust cues that fail differently:

  • COLOUR distance in Lab a/b from an auto-sampled floor colour model
        (catches coloured objects: brown table legs, red/yellow wires, ...).
  • LOCAL TEXTURE (variance of L): the floor speckle is high-frequency, while
        smooth surfaces (cables, the metal arm, table legs) have low local
        variance — so grey/white objects that share the floor's colour still pop
        out by being too smooth.

floor_like = colour_like * texture_like   (both in [0,1])
object     = floor_like < --floor-thresh

Each stage writes an image so the result can be eyeballed and tuned.

Usage (from stereo-camera-AD-Census/):
    C:\\msys64\\mingw64\\bin\\python tools/segment2d.py \
        --image captures/left_20260610_001156.jpg \
        --out-prefix captures/seg

Stage outputs (prefix `captures/seg`):
    _01_input.png  _02_illum.png  _03_floorseed.png  _04_floorprob.png
    _05_objmask_raw.png  _06_objmask.png  _07_objects.png  _08_background.png
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


# ── stage 2: illumination normalization ────────────────────────────────────────
def illum_normalize(bgr, mode="divide"):
    """Flatten uneven lighting before colour/texture analysis."""
    if mode == "none":
        return bgr.copy()
    if mode == "clahe":
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
        return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    # divide (flat-field): remove the low-frequency lighting gradient by dividing
    # by a heavily blurred copy, then restore the global level.
    f = bgr.astype(np.float32) + 1.0
    sigma = max(bgr.shape[:2]) / 8.0
    blur = cv2.GaussianBlur(f, (0, 0), sigma)
    out = f / (blur + 1e-6) * float(blur.mean())
    return np.clip(out, 0, 255).astype(np.uint8)


# ── stage 3: auto-sample the floor colour model ────────────────────────────────
def floor_model(lab, seed_radius=12.0, min_seed_frac=0.05):
    """Floor is the dominant colour: find the peak of the 2D a/b histogram and
    take the pixels near it as the floor seed. Returns (mu, inv_cov, seed_mask)."""
    a = lab[..., 1].astype(np.float32)
    b = lab[..., 2].astype(np.float32)
    h, w = a.shape

    hist = cv2.calcHist([lab], [1, 2], None, [256, 256], [0, 256, 0, 256])
    hist = cv2.GaussianBlur(hist, (0, 0), 2.0)
    _, _, _, peak = cv2.minMaxLoc(hist)          # peak = (a_peak, b_peak)
    a_peak, b_peak = float(peak[0]), float(peak[1])

    seed = ((a - a_peak) ** 2 + (b - b_peak) ** 2) <= seed_radius ** 2
    if seed.sum() < min_seed_frac * h * w:       # widen if the peak is too tight
        seed = ((a - a_peak) ** 2 + (b - b_peak) ** 2) <= (seed_radius * 2) ** 2

    ab = np.stack([a[seed], b[seed]], axis=1)
    mu = ab.mean(axis=0)
    cov = np.cov(ab, rowvar=False) + np.eye(2) * 1.0   # regularize
    inv_cov = np.linalg.inv(cov)
    return mu, inv_cov, seed


# ── stage 4: floor likelihood (colour + texture) ───────────────────────────────
def local_std(gray, win=7):
    f = gray.astype(np.float32)
    k = (win, win)
    mean = cv2.boxFilter(f, -1, k)
    sqmean = cv2.boxFilter(f * f, -1, k)
    return np.sqrt(np.maximum(sqmean - mean * mean, 0.0))


def floor_likelihood(lab, gray, mu, inv_cov, seed, floor_dist=2.5, tex_win=7):
    """Return floor_like in [0,1] (high = floor) and the texture map."""
    a = lab[..., 1].astype(np.float32)
    b = lab[..., 2].astype(np.float32)
    diff = np.stack([a - mu[0], b - mu[1]], axis=-1)          # H,W,2
    d2 = np.einsum("...i,ij,...j->...", diff, inv_cov, diff)  # squared Mahalanobis
    maha = np.sqrt(np.maximum(d2, 0.0))
    colour_like = np.exp(-0.5 * (maha / max(floor_dist, 1e-3)) ** 2)

    tex = local_std(gray, tex_win)
    floor_tex = float(np.median(tex[seed])) if seed.any() else float(np.median(tex))
    tex_like = np.clip(tex / (floor_tex + 1e-6), 0.0, 1.0)    # floor speckle ->1

    floor_like = colour_like * tex_like
    return floor_like.astype(np.float32), tex


# ── stage 6: clean the object mask ─────────────────────────────────────────────
def clean_mask(obj_raw, morph=5, min_area_frac=0.004, fill=True):
    h, w = obj_raw.shape
    m = (obj_raw > 0).astype(np.uint8) * 255
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (morph, morph))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)

    # drop small blobs
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    min_area = int(min_area_frac * h * w)
    out = np.zeros_like(m)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255

    if fill:  # fill interior holes per object
        cnts, _ = cv2.findContours(out, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = np.zeros_like(out)
        cv2.drawContours(out, cnts, -1, 255, cv2.FILLED)
    return out


# ── visualisation helpers ──────────────────────────────────────────────────────
def overlay_mask(bgr, mask, color, alpha=0.45):
    ov = bgr.copy()
    m = mask > 0
    tint = np.zeros_like(bgr)
    tint[:] = color
    ov[m] = (bgr[m] * (1 - alpha) + tint[m] * alpha).astype(np.uint8)
    return ov


def draw_objects(bgr, mask):
    out = bgr.copy()
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    n = 0
    for c in cnts:
        x, y, w, h = cv2.boundingRect(c)
        cv2.drawContours(out, [c], -1, (0, 255, 255), 2)
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 140, 255), 1)
        n += 1
    cv2.putText(out, f"objects: {n}", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2, cv2.LINE_AA)
    return out, n


def main():
    ap = argparse.ArgumentParser(description="Pure 2D object/floor segmentation")
    ap.add_argument("--image", required=True, help="input image (e.g. left capture)")
    ap.add_argument("--out-prefix", default="captures/seg")
    ap.add_argument("--illum", choices=("divide", "clahe", "none"), default="divide",
                    help="illumination normalization (default divide)")
    ap.add_argument("--floor-dist", type=float, default=2.5,
                    help="colour Mahalanobis scale; larger = more counts as floor")
    ap.add_argument("--floor-thresh", type=float, default=0.40,
                    help="floor_like below this becomes object (0..1)")
    ap.add_argument("--tex-win", type=int, default=7, help="local-variance window")
    ap.add_argument("--morph", type=int, default=5, help="morphology kernel size")
    ap.add_argument("--min-obj-area", type=float, default=0.004,
                    help="min object area as fraction of frame")
    ap.add_argument("--no-fill", action="store_true", help="don't fill object holes")
    args = ap.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        sys.exit(f"[ERROR] cannot read image: {args.image}")
    h, w = img.shape[:2]
    print(f"[seg] {args.image}  {w}x{h}")

    pre = Path(args.out_prefix)
    pre.parent.mkdir(parents=True, exist_ok=True)

    def save(suffix, im):
        p = f"{pre}{suffix}.png"
        cv2.imwrite(p, im)
        print(f"[seg] -> {p}")

    # 1. input
    save("_01_input", img)

    # 2. illumination normalization
    norm = illum_normalize(img, args.illum)
    save("_02_illum", norm)

    lab = cv2.cvtColor(norm, cv2.COLOR_BGR2LAB)
    gray = cv2.cvtColor(norm, cv2.COLOR_BGR2GRAY)

    # 3. floor colour model + seed overlay
    mu, inv_cov, seed = floor_model(lab)
    seed_u8 = (seed.astype(np.uint8)) * 255
    print(f"[seg] floor seed = {100.0*seed.mean():.1f}% of frame, "
          f"mu(a,b)=({mu[0]:.0f},{mu[1]:.0f})")
    save("_03_floorseed", overlay_mask(img, seed_u8, (0, 255, 0)))

    # 4. floor likelihood (colour * texture)
    floor_like, tex = floor_likelihood(lab, gray, mu, inv_cov, seed,
                                       floor_dist=args.floor_dist, tex_win=args.tex_win)
    heat = cv2.applyColorMap((floor_like * 255).astype(np.uint8), cv2.COLORMAP_JET)
    save("_04_floorprob", heat)

    # 5. raw object mask = NOT floor
    obj_raw = ((floor_like < args.floor_thresh).astype(np.uint8)) * 255
    save("_05_objmask_raw", obj_raw)

    # 6. cleaned object mask
    obj = clean_mask(obj_raw, morph=args.morph, min_area_frac=args.min_obj_area,
                     fill=not args.no_fill)
    obj_cov = 100.0 * (obj > 0).mean()
    print(f"[seg] object mask = {obj_cov:.1f}% of frame")
    save("_06_objmask", obj)

    # 7. object outlines + bbox
    objects_vis, n_obj = draw_objects(img, obj)
    print(f"[seg] outlined {n_obj} object region(s)")
    save("_07_objects", objects_vis)

    # 8. background = floor (objects removed) — the priority output
    floor_mask = cv2.bitwise_not(obj)
    bg = img.copy()
    bg[obj > 0] = (0, 0, 0)          # blank objects -> what remains is the floor
    floor_cov = 100.0 * (floor_mask > 0).mean()
    cv2.putText(bg, f"FLOOR {floor_cov:.0f}%", (6, 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2, cv2.LINE_AA)
    save("_08_background", bg)

    print("[done] prefix:", pre)


if __name__ == "__main__":
    main()
