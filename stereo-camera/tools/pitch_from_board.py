#!/usr/bin/env python3
"""
pitch_from_board — camera pitch/yaw/roll vs a wall-mounted chessboard, plus
per-pose rectification quality. Built for the SERVO PITCH SWEEP experiment:
does the inter-camera R,T survive the tilt servo moving?

The board (19x10 inner corners, ruler-measured square) hangs plumb on a
vertical wall, so its column direction is gravity: solvePnP on the RAW image
gives each camera's absolute pitch (+ = looking DOWN), yaw and roll relative
to the wall. A fixed board-mounting error cancels in the DELTAS, which are
what the sweep reads:

  d_pitch vs first tag   -> the servo's real angle step (servo cmd -> deg map)
  pitch_L - pitch_R      -> relative tilt BETWEEN the cameras; if the mount is
                            rigid this stays constant across servo angles
  dy p50/p90 (board)     -> rectification residual with the given yml at that
                            angle; rising with angle = mount flex, one calib
                            per angle band (or stiffen the mount)

Usage (Windows or Jetson, no torch):
  python pitch_from_board.py --selftest
  python pitch_from_board.py --session captures/sweep_20260713 \
      --rectify ../calib/stereo_rectify_20260712.yml
  python pitch_from_board.py --left p00_left.jpg --right p00_right.jpg \
      --rectify ../calib/stereo_rectify_20260712.yml
Capture one pair per servo angle (robot static, board static):
  ./capture_eval.sh p00 "" captures/sweep_20260713   # servo 0 deg
  ./capture_eval.sh p10 "" captures/sweep_20260713   # servo 10 deg down ...
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import calib_io          # noqa: E402
import epipolar_check    # noqa: E402  (board_dy — same dy convention)

_DEF_RECTIFY = _HERE.parent / "calib" / "stereo_rectify.yml"


def load_raw_intrinsics(yml_path):
    """(K_left, dist_left, K_right, dist_right) from either yml schema —
    the RAW-image intrinsics, not the rectified P1/P2."""
    fs = cv2.FileStorage(str(yml_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"cannot open calib yml: {yml_path}")

    def mat(name):
        n = fs.getNode(name)
        return None if n.empty() else n.mat()

    K_l, K_r = mat("K_left"), mat("K_right")
    if K_l is not None and K_r is not None:
        d_l, d_r = mat("dist_left"), mat("dist_right")
    else:
        K_l = K_r = mat("K")
        d_l = d_r = mat("dist")
    fs.release()
    if K_l is None:
        raise ValueError(f"{yml_path}: no K_left/K_right or K node")
    z5 = np.zeros((1, 5))
    return K_l, (d_l if d_l is not None else z5), \
        K_r, (d_r if d_r is not None else z5)


def obj_points(pattern, square_m, square_y_m=None):
    """Board grid; square_y_m for boards whose cells are NOT square (the
    user's wall board measured 10% taller than wide on 2026-07-13)."""
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    obj[:, 0] = grid[:, 0] * square_m
    obj[:, 1] = grid[:, 1] * (square_y_m or square_m)
    return obj


def pose_angles(corners, K, dist, pattern, square_m, square_y_m=None):
    """Camera attitude vs a plumb wall board, from RAW-image corners.

    Board frame: X along corner rows (horizontal on the wall), Y along
    columns (gravity, downward), Z out of the wall. solvePnP R maps
    board->camera, so R's columns are the board axes in camera coords:
      pitch = atan2(R21, R11)   angle of gravity in the camera Y-Z plane;
                                 0 = level, + = camera looking DOWN
      yaw   = atan2(R02, R22)   wall normal vs optical axis; + = panned so the
                                 board sits toward the image RIGHT
      roll  = atan2(R10, R00)   horizontal board axis vs image rows
    Returns dict(pitch, yaw, roll [deg], z_axis [m along optical axis],
    rms [px reprojection]).
    """
    obj = obj_points(pattern, square_m, square_y_m)
    img = corners.reshape(-1, 1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum(
        (proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    return {
        "pitch": float(np.degrees(np.arctan2(R[2, 1], R[1, 1]))),
        "yaw": float(np.degrees(np.arctan2(R[0, 2], R[2, 2]))),
        "roll": float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))),
        "z_axis": float(tvec.ravel()[2]),
        "rms": rms,
    }


# ------------------------------------------------------------------ selftest -
def _selftest():
    """Synthetic camera pitched by known angles at a plumb wall board:
    recovered pitch/yaw must match to <0.05 deg."""
    pattern, sq, D = (19, 10), 0.025, 1.0
    fx, w, h = 1000.0, 1280, 720
    K = np.array([[fx, 0, w / 2], [0, fx, h / 2], [0, 0, 1]])
    dist = np.zeros((1, 5))
    obj = obj_points(pattern, sq)
    # centre the board on the (pitched) optical axis so it stays in frame
    worst = 0.0
    for pitch_deg in (0.0, 10.0, 20.0, 30.0, -10.0):
        t = np.radians(pitch_deg)
        # world: X right, Y down, Z forward; camera pitched DOWN by t about X
        R_wc = np.array([[1, 0, 0],
                         [0, np.cos(t), -np.sin(t)],
                         [0, np.sin(t), np.cos(t)]])
        # board on the wall plane Z=D, shifted so its centre sits where the
        # pitched axis crosses the wall (y = D*tan(t))
        off = np.array([-sq * (pattern[0] - 1) / 2,
                        D * np.tan(t) - sq * (pattern[1] - 1) / 2, D])
        P_w = obj + off
        P_c = (R_wc @ P_w.T).T
        uv = (K @ P_c.T).T
        uv = (uv[:, :2] / uv[:, 2:3]).astype(np.float64)
        r = pose_angles(uv, K, dist, pattern, sq)
        err = abs(r["pitch"] - pitch_deg)
        worst = max(worst, err, abs(r["yaw"]), abs(r["roll"]))
        print(f"  [selftest] true pitch {pitch_deg:+6.1f} -> est "
              f"{r['pitch']:+7.3f}  yaw {r['yaw']:+6.3f}  roll "
              f"{r['roll']:+6.3f}  Z {r['z_axis']:.3f}  rms {r['rms']:.4f}px")
        assert err < 0.05, f"pitch error {err:.3f} deg"
    print(f"  [selftest] PASS (worst angle error {worst:.4f} deg)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Camera pitch vs wall chessboard + per-angle rectify check",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--session", help="dir with manifest.csv or *left*/right*")
    ap.add_argument("--left"), ap.add_argument("--right")
    ap.add_argument("--rectify", default=str(_DEF_RECTIFY),
                    help="yml for raw K/dist AND the dy rectify check")
    ap.add_argument("--pattern", default="19x10", help="inner corners CxR")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="RULER-MEASURED cell width, along corner rows (mm)")
    ap.add_argument("--square-y-mm", type=float, default=None,
                    help="cell height if cells are not square (wall board: "
                    "~10%% taller than wide); default = --square-mm")
    ap.add_argument("--out", default=None, help="csv path (default: session dir)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args(argv)
    if args.selftest:
        return _selftest()
    if not args.session and not (args.left and args.right):
        ap.error("need --session DIR or --left/--right (or --selftest)")

    c, r = args.pattern.lower().split("x")
    pattern = (int(c), int(r))
    sq = args.square_mm / 1000.0
    sq_y = args.square_y_mm / 1000.0 if args.square_y_mm else None
    K_l, d_l, K_r, d_r = load_raw_intrinsics(args.rectify)
    calib = calib_io.load_calib(args.rectify)
    print(f"calib={Path(args.rectify).name}  board {args.pattern} "
          f"@ {args.square_mm:g}mm  (+pitch = camera DOWN)")
    print(f"  {'tag':<8}{'pitchL':>8}{'pitchR':>8}{'L-R':>7}{'dPitch':>8}"
          f"{'yawL':>7}{'rollL':>7}{'Z_m':>7}{'dy_p50':>8}{'dy_p90':>8}")

    header = ["tag", "pitch_l_deg", "pitch_r_deg", "rel_tilt_lr_deg",
              "dpitch_vs_first_deg", "yaw_l_deg", "roll_l_deg", "z_axis_m",
              "pnp_rms_l_px", "dy_p50_px", "dy_p90_px"]
    rows, first_pitch = [], None
    for tag, lp, rp, _z in calib_io.iter_pairs(args.session, args.left,
                                               args.right):
        left, right = calib_io.read_pair(lp, rp)
        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        cl = calib_io.find_board(gl, pattern)
        cr = calib_io.find_board(gr, pattern)
        if cl is None or cr is None:
            miss = "left" if cl is None else "right"
            print(f"  {tag:<8} FULL board not found in {miss} image — skipped")
            rows.append([tag] + [""] * (len(header) - 1))
            continue
        pl = pose_angles(cl, K_l, d_l, pattern, sq, sq_y)
        pr = pose_angles(cr, K_r, d_r, pattern, sq, sq_y)

        calib_io.check_size(left, calib, tag)
        lrec, rrec = calib_io.rectify_pair(calib, left, right)
        dyb = epipolar_check.board_dy(cv2.cvtColor(lrec, cv2.COLOR_BGR2GRAY),
                                      cv2.cvtColor(rrec, cv2.COLOR_BGR2GRAY),
                                      pattern)
        if dyb is not None:
            a = np.abs(dyb)
            dy50, dy90 = float(np.median(a)), float(np.percentile(a, 90))
            dy_s = f"{dy50:>8.2f}{dy90:>8.2f}"
        else:
            dy50 = dy90 = None
            dy_s = f"{'-':>8}{'-':>8}"

        if first_pitch is None:
            first_pitch = pl["pitch"]
        dpitch = pl["pitch"] - first_pitch
        rel = pl["pitch"] - pr["pitch"]
        print(f"  {tag:<8}{pl['pitch']:>8.2f}{pr['pitch']:>8.2f}{rel:>7.2f}"
              f"{dpitch:>8.2f}{pl['yaw']:>7.2f}{pl['roll']:>7.2f}"
              f"{pl['z_axis']:>7.2f}{dy_s}")
        rows.append([tag, f"{pl['pitch']:.3f}", f"{pr['pitch']:.3f}",
                     f"{rel:.3f}", f"{dpitch:.3f}", f"{pl['yaw']:.3f}",
                     f"{pl['roll']:.3f}", f"{pl['z_axis']:.3f}",
                     f"{pl['rms']:.3f}",
                     "" if dy50 is None else f"{dy50:.3f}",
                     "" if dy90 is None else f"{dy90:.3f}"])

    print("\n  read-out: rigid mount = 'L-R' and dy columns FLAT across the "
          "sweep;\n  'dPitch' column = the servo's real step sizes "
          "(cmd -> deg map).")
    out = args.out or str(Path(args.session or Path(args.left).parent)
                          / "pitch_sweep_report.csv")
    calib_io.write_csv(out, header, rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
