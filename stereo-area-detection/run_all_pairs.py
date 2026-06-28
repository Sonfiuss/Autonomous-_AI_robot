#!/usr/bin/env python3
"""
run_all_pairs.py  –  Chạy AD-Census depth cho CẢ 5 cặp left_N/right_N trong captures/.

Gọi lại nguyên pipeline của run_drivable.py (preprocess → align → AD-Census →
khử biên → denoise → fill → colorize) cho từng cặp, ghi output riêng:
    captures/pairN_disp.png
    captures/pairN_disp_color.png
    captures/pairN_right_aligned.jpg ...

Mục đích: so sánh nhanh depth của 5 cảnh để đánh giá _room_align.yml.

Usage:
  python run_all_pairs.py                 # cặp 1..5 mặc định
  python run_all_pairs.py --pairs 1 3 5   # chỉ vài cặp
  python run_all_pairs.py --full-warp     # truyền cờ xuống run_drivable
Mọi cờ khác (--maxd, --adc-width, --no-preprocess, ...) đều được chuyển tiếp.
"""

import argparse
import os
import sys

# Tái dùng trực tiếp các hàm của run_drivable để không lặp logic.
import run_drivable as rd

_HERE = os.path.dirname(os.path.abspath(__file__))
_CAP = os.path.join(_HERE, "captures")


def _process_one(left, right, out_prefix, extra_argv):
    """Chạy pipeline run_drivable.main() cho 1 cặp bằng cách dựng argv tạm."""
    argv_bak = sys.argv
    sys.argv = [
        "run_drivable.py",
        "--left", left,
        "--right", right,
        "--out-prefix", out_prefix,
    ] + extra_argv
    try:
        rd.main()
    finally:
        sys.argv = argv_bak


def main():
    ap = argparse.ArgumentParser(description="Chạy depth cho cả 5 cặp left/right")
    ap.add_argument("--pairs", type=int, nargs="+", default=[1, 2, 3, 4, 5],
                    help="Chỉ số cặp cần chạy (mặc định 1..5)")
    ap.add_argument("--captures", default=_CAP, help="Thư mục chứa left_N/right_N")
    # Mọi cờ không nhận diện được chuyển tiếp xuống run_drivable.
    args, extra_argv = ap.parse_known_args()

    summary = []
    for n in args.pairs:
        left = os.path.join(args.captures, f"left_{n}.jpg")
        right = os.path.join(args.captures, f"right_{n}.jpg")
        if not (os.path.isfile(left) and os.path.isfile(right)):
            print(f"[skip] cặp {n}: thiếu {os.path.basename(left)} / "
                  f"{os.path.basename(right)}")
            summary.append((n, "MISSING"))
            continue
        out_prefix = os.path.join(args.captures, f"pair{n}")
        print(f"\n{'='*60}\n[pair {n}] {os.path.basename(left)} + "
              f"{os.path.basename(right)}\n{'='*60}")
        try:
            _process_one(left, right, out_prefix, extra_argv)
            summary.append((n, f"{out_prefix}_disp_color.png"))
        except SystemExit as e:           # run_drivable gọi sys.exit khi lỗi
            summary.append((n, f"FAILED (exit {e.code})"))
        except Exception as e:
            summary.append((n, f"FAILED ({e})"))

    print(f"\n{'='*60}\nTỔNG KẾT {len(summary)} cặp:")
    for n, res in summary:
        print(f"  pair {n}: {res}")


if __name__ == "__main__":
    main()
