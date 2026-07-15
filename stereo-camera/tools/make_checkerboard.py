#!/usr/bin/env python3
"""
make_checkerboard — printable calibration board at EXACT physical scale.

Pattern: 9×6 INNER corners (10×7 squares), 25 mm squares → 250×175 mm board,
fits A4 landscape with margins. At fx≈1124 px the 25 mm square spans ~70 px at
0.4 m and ~14 px at 2.0 m — detectable across the whole 0.3–2 m working range.

Outputs (default stereo-camera/tools/checkerboard/):
  checkerboard_9x6_25mm.pdf   ← PRINT THIS ONE (vector-placed raster, A4,
                                 exact scale as long as you print at 100%)
  checkerboard_9x6_25mm.png   backup (304.8 DPI, same physical scale)

PRINT INSTRUCTIONS (also rendered on the page):
  1. Print at 100% / "Actual size" — NEVER "fit to page".
  2. VERIFY with a ruler: 4 squares = 100.0 mm, and the ruler bar = 100 mm.
     If it differs, measure one square to ±0.1 mm and pass that value to
     stereo_calibrate_2view.py --square-mm  (printer scale error goes straight
     into depth scale: 2% square error = 2% distance error).
  3. Glue/tape FLAT onto rigid board (clipboard / cardboard) — no waves.

Usage:
  python make_checkerboard.py            # 9x6 inner, 25 mm, A4 landscape
  python make_checkerboard.py --square-mm 30 --pattern 9x6
"""
import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

_HERE = Path(__file__).resolve().parent
PX_PER_MM = 12                      # 12 px/mm = 304.8 DPI, integer square edges
A4_LANDSCAPE_MM = (297, 210)


def render(pattern=(9, 6), square_mm=25.0):
    """→ PIL Image of the full A4 page at PX_PER_MM."""
    cols, rows = pattern[0] + 1, pattern[1] + 1        # squares
    bw_mm, bh_mm = cols * square_mm, rows * square_mm
    pw_mm, ph_mm = A4_LANDSCAPE_MM
    if bw_mm > pw_mm - 20 or bh_mm > ph_mm - 25:
        raise SystemExit(f"board {bw_mm:.0f}x{bh_mm:.0f}mm does not fit A4 "
                         f"landscape with margins — reduce --square-mm")
    W, H = int(pw_mm * PX_PER_MM), int(ph_mm * PX_PER_MM)
    sq = square_mm * PX_PER_MM
    x0 = (W - cols * sq) / 2.0
    y0 = (H - rows * sq) / 2.0

    img = Image.new("L", (W, H), 255)
    d = ImageDraw.Draw(img)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                d.rectangle([round(x0 + c * sq), round(y0 + r * sq),
                             round(x0 + (c + 1) * sq) - 1,
                             round(y0 + (r + 1) * sq) - 1], fill=0)

    # 100 mm ruler bar in the bottom margin, 10 mm ticks
    bar_y = round(y0 + rows * sq + 2.5 * PX_PER_MM)
    bar_x = round(x0)
    bar_len = 100 * PX_PER_MM
    d.rectangle([bar_x, bar_y, bar_x + bar_len, bar_y + PX_PER_MM], fill=0)
    for t in range(0, 101, 10):
        tx = bar_x + t * PX_PER_MM
        d.rectangle([tx, bar_y - PX_PER_MM, tx + 2, bar_y], fill=0)

    try:
        font = ImageFont.truetype("arial.ttf", int(3.2 * PX_PER_MM))
    except OSError:
        font = ImageFont.load_default()
    d.text((bar_x + bar_len + 4 * PX_PER_MM, bar_y - PX_PER_MM),
           "= 100.0 mm  |  4 squares = 100.0 mm  — VERIFY WITH A RULER",
           font=font, fill=0)
    d.text((round(x0), round(y0 - 5.5 * PX_PER_MM)),
           f"stereo calib board {pattern[0]}x{pattern[1]} inner corners, "
           f"{square_mm:g} mm squares — print at 100% / Actual size, "
           f"NO fit-to-page. Mount FLAT on rigid board.",
           font=font, fill=0)
    return img


def main():
    ap = argparse.ArgumentParser(description="Printable calibration checkerboard")
    ap.add_argument("--pattern", default="9x6", help="inner corners CxR")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--out-dir", default=str(_HERE / "checkerboard"))
    args = ap.parse_args()
    c, r = args.pattern.lower().split("x")
    pattern = (int(c), int(r))
    if (pattern[0] + pattern[1]) % 2 == 0:
        # odd+even inner corners → asymmetric board, no 180° flip ambiguity
        print(f"[board] WARNING: {args.pattern} is 180-deg symmetric; "
              f"9x6 is recommended")

    img = render(pattern, args.square_mm)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    base = f"checkerboard_{pattern[0]}x{pattern[1]}_{args.square_mm:g}mm"
    dpi = PX_PER_MM * 25.4
    img.save(out / f"{base}.png", dpi=(dpi, dpi))
    img.save(out / f"{base}.pdf", resolution=dpi)
    print(f"[board] {out / (base + '.pdf')}  <- print this at 100%")
    print(f"[board] {out / (base + '.png')}")
    print(f"[board] board {(pattern[0]+1)*args.square_mm:.0f} x "
          f"{(pattern[1]+1)*args.square_mm:.0f} mm on A4 landscape")
    print("[board] after printing: ruler-check 4 squares = 100.0 mm; pass the")
    print("[board] measured square to stereo_calibrate_2view.py --square-mm")


if __name__ == "__main__":
    main()
