"""Static calibration of the RGB camera on the robot from marks on the floor: pitch, and fx (= fy) with
the camera's yaw (as cx) when the marks allow it. The robot does not move, so neither wheel slip nor
the wheel radius is in the answer.

Marks are taped on the floor and measured from O, the floor point under the RGB lens (plumb line):
x ahead along the robot's +x, y to the LEFT (calib_marks.json holds the default layout). A mark with
"y": null is distance-only - an object whose distance STRAIGHT AHEAD (along +x, not the diagonal) was
measured - and constrains its image row alone; the diagonal to a thing 0.4 m aside at 1.6 m is 3 %
longer, as much as the whole PASS_REL_ERR.
"check": true holds a mark out of the fit; its error is an independent test of the result.

  DISPLAY=:0 python3 vision/calib_floor.py --height 0.24           # grab frames, click the marks
  python3 vision/calib_floor.py --image OUT/frame.png --points 320,410 321,355 - ...   # offline

What is fitted follows from the marks: the pitch always; fx and cx only when at least two fitted
marks with a known y lie MIN_LATERAL_SPREAD_M apart sideways (u is what carries fx), otherwise fx
stays at --fx. The height is the measured one; a refit with the height free is printed beside it,
only as a check. cy stays at the image centre: the pitch absorbs it.

Writes OUT/frame.png, OUT/calib.json (marks, pixels, fit, errors, the --points to redo it) and
OUT/check.png. When the fit passes - mean floor error of the fitted marks within PASS_REL_ERR of
their range, the plan's "under 3 cm at 1.2 m" - or with --force, also vision/camera_mount.json and,
when fx was fitted, vision/camera_rgb.json; record.py reads both by default.
"""
import argparse
import collections
import json
import logging
import math
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np

from color_source import HEIGHT, WIDTH, ColorSource
from floor_geometry import CAMERA_MOUNT_FILE, CameraMount, default_mount
from floor_segment import floor_hit, floor_project
from object_distance import CAMERA_RGB_FILE, default_intrinsics_file, fallback_intrinsics, load_intrinsics

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MARKS = os.path.join(HERE, "calib_marks.json")
FRAME_NAME, CHECK_NAME, CALIB_NAME = "frame.png", "check.png", "calib.json"   # in --out
DEFAULT_FRAMES = 10                # averaged: pixel noise down ~3x
WARMUP_FRAMES = 60                 # left to auto-exposure, as light_check
GRAB_TIMEOUT_S = 15.0
GRAB_POLL_S = 0.05
MIN_LATERAL_SPREAD_M = 0.3         # below this the u coordinates cannot tell fx from cx
PASS_REL_ERR = 0.025               # mean |floor error| / range of the fitted marks: 3 cm at 1.2 m
# Levenberg-Marquardt: damping starts small, shrinks after a step that lowers the cost, grows after
# one that does not; past DAMPING_MAX no step helps any more and the fit stops.
MAX_ITER = 60
DAMPING_START, DAMPING_DOWN, DAMPING_UP, DAMPING_MAX = 1e-3, 0.3, 10.0, 1e9
DIAG_FLOOR = 1e-12                 # keeps the damped normal matrix invertible for a flat parameter
DIFF_STEP = {"pitch_deg": 1e-4, "fx": 1e-3, "cx": 1e-3, "height_m": 1e-6}   # numeric Jacobian, also the "converged" step
BEHIND_RESIDUAL_PX = 1e3           # a mark the parameters put behind the camera
WINDOW_NAME = "calib_floor"
DISPLAY_SCALE = 2                  # the image is shown this much larger to click half pixels
ZOOM, ZOOM_HALF = 8, 12            # magnifier: a (2 * ZOOM_HALF + 1)^2 px patch shown ZOOM times
BAR_H = 28                         # instruction bar above the image, px
BAR_TEXT_X, BAR_TEXT_BASELINE, BAR_FONT_SCALE = 6, 9, 0.5
CROSS_ARM_PX, LABEL_OFFSET_PX, LABEL_FONT_SCALE = 8, 6, 0.45
PREDICT_RADIUS_PX, HORIZON_TEXT_UP_PX, HORIZON_FONT_SCALE = 5, 4, 0.4
KEY_ENTER, KEY_LF, KEY_ESC, KEY_BACKSPACE, KEY_UNDO, KEY_QUIT = 13, 10, 27, 8, ord("u"), ord("q")
WAIT_KEY_MS = 20
TEXT_COLOR, BAR_COLOR = (255, 255, 255), (0, 0, 0)
CLICK_COLOR, PREDICT_COLOR, CHECK_COLOR, HORIZON_COLOR = (0, 0, 255), (0, 220, 0), (255, 0, 255), (0, 200, 255)
PX_DIGITS, M_DIGITS, DEG_DIGITS = 2, 4, 3
CM_PER_M = 100.0
PCT = 100.0

Mark = collections.namedtuple("Mark", "name x y check")
# Fit: the parameters, which of them were free, RMS pixel residual over the fitted marks.
Fit = collections.namedtuple("Fit", "intrinsics mount free rms_px")

logger = logging.getLogger(__name__)


def load_marks(path):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    marks = [Mark(str(m["name"]), float(m["x"]), None if m.get("y") is None else float(m["y"]),
                  bool(m.get("check", False))) for m in data["marks"]]
    if not marks:
        raise ValueError(f"{path}: no marks")
    return marks


def grab_average(index, frames):
    """Mean of `frames` camera frames after WARMUP_FRAMES, as uint8 BGR."""
    got = []
    lock = threading.Lock()

    def listener(frame):
        if frame.seq >= WARMUP_FRAMES:
            with lock:
                if len(got) < frames:
                    got.append(frame.data.astype(np.float32))

    source = ColorSource(index)
    source.listener = listener
    source.start()
    deadline = time.monotonic() + GRAB_TIMEOUT_S
    try:
        while time.monotonic() < deadline:
            with lock:
                if len(got) >= frames:
                    break
            time.sleep(GRAB_POLL_S)
    finally:
        if not source.close():
            logger.warning("camera thread stuck in its driver; the process may take a while to exit")
    if len(got) < frames:
        raise RuntimeError(f"camera {index}: {len(got)}/{frames} frames in {GRAB_TIMEOUT_S:.0f} s")
    return np.clip(np.mean(got, axis=0) + 0.5, 0, 255).astype(np.uint8)


def parse_points(texts, marks):
    """--points: one 'u,v' or '-' (not visible) per mark, in the marks file's order."""
    if len(texts) != len(marks):
        raise ValueError(f"--points needs {len(marks)} entries (one per mark, '-' to skip), got {len(texts)}")
    points = []
    for text in texts:
        if text == "-":
            points.append(None)
            continue
        try:
            u, v = (float(t) for t in text.split(","))
        except ValueError:
            raise ValueError(f"--points entry {text!r} is not 'u,v' or '-'") from None
        points.append((u, v))
    return points


def points_arg(points):
    """The --points text that reproduces these clicks."""
    return " ".join("-" if p is None else f"{p[0]:.1f},{p[1]:.1f}" for p in points)


def display_to_image(x, y, scale):
    """Image pixel (u, v) under pixel (x, y) of the click window: BAR_H rows of bar on top, then the
    image zoomed `scale` times. Pixel centres map onto pixel centres."""
    return (x + 0.5) / scale - 0.5, (y - BAR_H + 0.5) / scale - 0.5


def image_to_display(u, v, scale):
    """The inverse of display_to_image."""
    return (u + 0.5) * scale - 0.5, (v + 0.5) * scale - 0.5 + BAR_H


def click_points(image, marks, scale=DISPLAY_SCALE):
    """Shows the image and returns one (u, v) or None per mark, in order; None when the user quits."""
    points = []
    cursor = [0, 0]

    def on_mouse(event, x, y, _flags, _param):
        cursor[0], cursor[1] = x, y
        if len(points) >= len(marks):
            return
        if event == cv2.EVENT_LBUTTONDOWN:
            points.append(display_to_image(x, y, scale))
        elif event == cv2.EVENT_RBUTTONDOWN:
            points.append(None)

    big = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    try:
        while True:
            canvas = cv2.copyMakeBorder(big, BAR_H, 0, 0, 0, cv2.BORDER_CONSTANT, value=BAR_COLOR)
            for mark, p in zip(marks, points):
                if p is not None:
                    _draw_cross(canvas, image_to_display(p[0], p[1], scale), CLICK_COLOR, mark.name)
            if len(points) < len(marks):
                m = marks[len(points)]
                side = "distance only" if m.y is None else f"y {m.y:+.2f}"
                text = f"click mark {m.name} (x {m.x:.2f}, {side})  |  right click: not visible  |  u: undo  |  q: quit"
            else:
                text = "all marks placed  |  Enter: fit  |  u: undo  |  q: quit"
            cv2.putText(canvas, text, (BAR_TEXT_X, BAR_H - BAR_TEXT_BASELINE), cv2.FONT_HERSHEY_SIMPLEX,
                        BAR_FONT_SCALE, TEXT_COLOR, 1)
            _draw_magnifier(canvas, image, cursor, scale)
            cv2.imshow(WINDOW_NAME, canvas)
            key = cv2.waitKey(WAIT_KEY_MS) & 0xFF
            if key in (KEY_UNDO, KEY_BACKSPACE) and points:
                points.pop()
            elif key in (KEY_ENTER, KEY_LF) and len(points) == len(marks):
                return points
            elif key in (KEY_QUIT, KEY_ESC):
                return None
    finally:
        cv2.destroyWindow(WINDOW_NAME)


def _draw_cross(img, center, color, label=None):
    x, y = int(round(center[0])), int(round(center[1]))
    cv2.line(img, (x - CROSS_ARM_PX, y), (x + CROSS_ARM_PX, y), color, 1)
    cv2.line(img, (x, y - CROSS_ARM_PX), (x, y + CROSS_ARM_PX), color, 1)
    if label:
        cv2.putText(img, label, (x + LABEL_OFFSET_PX, y - LABEL_OFFSET_PX), cv2.FONT_HERSHEY_SIMPLEX,
                    LABEL_FONT_SCALE, color, 1)


def _draw_magnifier(canvas, image, cursor, scale):
    """The patch around the mouse, ZOOM times, in the top-right corner, with a crosshair on its centre."""
    u, v = (int(round(c)) for c in display_to_image(cursor[0], cursor[1], scale))
    padded = cv2.copyMakeBorder(image, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, ZOOM_HALF, cv2.BORDER_CONSTANT)
    u, v = min(max(u, 0), image.shape[1] - 1), min(max(v, 0), image.shape[0] - 1)
    patch = padded[v:v + 2 * ZOOM_HALF + 1, u:u + 2 * ZOOM_HALF + 1]
    patch = cv2.resize(patch, None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_NEAREST)
    c = ZOOM_HALF * ZOOM + ZOOM // 2
    cv2.line(patch, (c, 0), (c, patch.shape[0] - 1), CLICK_COLOR, 1)
    cv2.line(patch, (0, c), (patch.shape[1] - 1, c), CLICK_COLOR, 1)
    x0 = canvas.shape[1] - patch.shape[1]
    canvas[BAR_H:BAR_H + patch.shape[0], x0:] = patch


def lens_is_observable(marks, points):
    """fx and cx need u residuals from marks spread sideways."""
    ys = [m.y for m, p in zip(marks, points) if p is not None and not m.check and m.y is not None]
    return len(ys) >= 2 and max(ys) - min(ys) >= MIN_LATERAL_SPREAD_M


def _unpack(params, free, intrinsics, mount):
    values = dict(zip(free, params))
    k = intrinsics._replace(fx=values.get("fx", intrinsics.fx), fy=values.get("fx", intrinsics.fy),
                            cx=values.get("cx", intrinsics.cx))
    m = mount._replace(pitch_deg=values.get("pitch_deg", mount.pitch_deg),
                       height_m=values.get("height_m", mount.height_m))
    return k, m


def _residuals(params, free, marks, points, intrinsics, mount):
    """Predicted minus clicked pixels of the fitted marks: v for every one, u where y is known."""
    k, m = _unpack(params, free, intrinsics, mount)
    out = []
    for mark, p in zip(marks, points):
        if p is None or mark.check:
            continue
        u, v = floor_project([mark.x], [0.0 if mark.y is None else mark.y], k, m)
        du, dv = float(u[0]) - p[0], float(v[0]) - p[1]
        if not math.isfinite(dv):
            du = dv = BEHIND_RESIDUAL_PX
        out.append(dv)
        if mark.y is not None:
            out.append(du)
    return np.array(out)


def fit(marks, points, intrinsics, mount, fit_lens, fit_height=False):
    """Levenberg-Marquardt on the pixel residuals, numeric Jacobian. intrinsics / mount give the fixed
    values and the start. None when fewer residuals than free parameters."""
    free = ["pitch_deg"] + (["fx", "cx"] if fit_lens else []) + (["height_m"] if fit_height else [])
    start = {"pitch_deg": mount.pitch_deg, "fx": intrinsics.fx, "cx": intrinsics.cx, "height_m": mount.height_m}
    x = np.array([start[name] for name in free])
    steps = np.array([DIFF_STEP[name] for name in free])
    r = _residuals(x, free, marks, points, intrinsics, mount)
    if r.size < len(free):
        return None
    cost = float(r @ r)
    damping = DAMPING_START
    iterations = 0
    for iterations in range(1, MAX_ITER + 1):
        jac = np.column_stack([(_residuals(x + np.eye(len(free))[i] * steps[i], free, marks, points, intrinsics, mount)
                                - r) / steps[i] for i in range(len(free))])
        normal, gradient = jac.T @ jac, jac.T @ r
        step = None
        while damping < DAMPING_MAX:
            trial = np.linalg.solve(normal + damping * np.diag(np.diag(normal) + DIAG_FLOOR), -gradient)
            r_new = _residuals(x + trial, free, marks, points, intrinsics, mount)
            if float(r_new @ r_new) < cost:
                step, x, r, cost = trial, x + trial, r_new, float(r_new @ r_new)
                damping *= DAMPING_DOWN
                break
            damping *= DAMPING_UP
        if step is None or np.all(np.abs(step) < steps):
            break
    k, m = _unpack(x, free, intrinsics, mount)
    logger.debug("fit %s: %d iterations, cost %.4g px^2, damping %.1e", free, iterations, cost, damping)
    return Fit(k, m, free, math.sqrt(cost / r.size))


def mark_errors(marks, points, intrinsics, mount):
    """Per clicked mark: pixel error (predicted - clicked) and floor error (where the clicked pixel lands
    minus where the mark is; forward only for a distance-only mark)."""
    rows = []
    for mark, p in zip(marks, points):
        if p is None:
            continue
        u, v = floor_project([mark.x], [0.0 if mark.y is None else mark.y], intrinsics, mount)
        fwd, left, _ = floor_hit([p[0]], [p[1]], intrinsics, mount)
        dx = float(fwd[0]) - mark.x
        dy = None if mark.y is None else float(left[0]) - mark.y
        floor = abs(dx) if dy is None else math.hypot(dx, dy)
        rows.append({"name": mark.name, "check": mark.check, "x": mark.x, "y": mark.y,
                     "clicked": [round(p[0], PX_DIGITS), round(p[1], PX_DIGITS)],
                     "du_px": None if mark.y is None else round(float(u[0]) - p[0], PX_DIGITS),
                     "dv_px": round(float(v[0]) - p[1], PX_DIGITS),
                     "dx_m": round(dx, M_DIGITS), "dy_m": None if dy is None else round(dy, M_DIGITS),
                     "floor_err_m": round(floor, M_DIGITS),
                     "rel_err": round(floor / math.hypot(mark.x, mark.y or 0.0), M_DIGITS)})
    return rows


def passes(rows):
    """Mean relative floor error of the fitted (non-check) marks within PASS_REL_ERR."""
    fitted = [r["rel_err"] for r in rows if not r["check"]]
    return bool(fitted) and float(np.mean(fitted)) <= PASS_REL_ERR


def draw_check(image, marks, points, intrinsics, mount):
    """Clicked (red cross), predicted (green circle; magenta for a check mark), and the horizon."""
    out = image.copy()
    horizon = int(round(intrinsics.cy - intrinsics.fy * math.tan(math.radians(mount.pitch_deg))))
    cv2.line(out, (0, horizon), (out.shape[1] - 1, horizon), HORIZON_COLOR, 1)
    cv2.putText(out, f"horizon, pitch {mount.pitch_deg:+.2f} deg", (BAR_TEXT_X, horizon - HORIZON_TEXT_UP_PX),
                cv2.FONT_HERSHEY_SIMPLEX, HORIZON_FONT_SCALE, HORIZON_COLOR, 1)
    for mark, p in zip(marks, points):
        u, v = floor_project([mark.x], [0.0 if mark.y is None else mark.y], intrinsics, mount)
        if math.isfinite(float(v[0])):
            center = (int(round(float(u[0]))), int(round(float(v[0]))))
            if mark.y is None and p is not None:
                center = (int(round(p[0])), center[1])     # distance-only: only the row is predicted
            cv2.circle(out, center, PREDICT_RADIUS_PX, CHECK_COLOR if mark.check else PREDICT_COLOR, 1)
        if p is not None:
            _draw_cross(out, p, CLICK_COLOR, mark.name)
    return out


def _report(title, fit_result, rows):
    k, m = fit_result.intrinsics, fit_result.mount
    print(f"\n{title}: free {', '.join(fit_result.free)} | pitch {m.pitch_deg:+.3f} deg, fx {k.fx:.1f}, "
          f"cx {k.cx:.1f} (yaw {math.degrees(math.atan((k.cx - (WIDTH - 1) / 2.0) / k.fx)):+.2f} deg), "
          f"h {m.height_m:.4f} m | RMS {fit_result.rms_px:.2f} px")
    if rows is None:
        return
    print(f"  {'mark':6s} {'x':>5s} {'y':>6s} {'du px':>6s} {'dv px':>6s} {'floor cm':>9s} {'%':>5s}")
    for r in rows:
        y = "  --" if r["y"] is None else f"{r['y']:+.2f}"
        du = "   --" if r["du_px"] is None else f"{r['du_px']:+.1f}"
        tag = "  (check)" if r["check"] else ""
        print(f"  {r['name']:6s} {r['x']:5.2f} {y:>6s} {du:>6s} {r['dv_px']:+6.1f} "
              f"{CM_PER_M * r['floor_err_m']:9.1f} {PCT * r['rel_err']:5.1f}{tag}")


def _write_image(path, image):
    if not cv2.imwrite(path, image):
        raise OSError(f"cannot write {path}")


def _write_json(path, data):
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    os.replace(path + ".tmp", path)


def _parse_args():
    mount = default_mount()
    parser = argparse.ArgumentParser(description="Calibrate the RGB camera's pitch (and fx, yaw) from floor marks.")
    parser.add_argument("--marks", default=DEFAULT_MARKS, help="marks JSON (default: %(default)s)")
    parser.add_argument("--height", type=float, default=mount.height_m,
                        help="MEASURED lens height above the floor, m (default: %(default)s)")
    parser.add_argument("--cam-forward", type=float, default=mount.forward_m,
                        help="lens ahead of the robot center, m - written to camera_mount.json (default: %(default)s)")
    parser.add_argument("--cam-left", type=float, default=mount.left_m,
                        help="lens left of the robot center, m - written to camera_mount.json (default: %(default)s)")
    parser.add_argument("--fx", type=float, help="fx when the marks cannot fit it (default: camera_rgb.json, else the FOV fallback)")
    parser.add_argument("--image", help="calibrate this image instead of grabbing the camera")
    parser.add_argument("--points", nargs="+", help="one 'u,v' or '-' per mark instead of clicking")
    parser.add_argument("--color-index", type=int, default=0)
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES, help="camera frames averaged")
    parser.add_argument("--scale", type=int, default=DISPLAY_SCALE, help="display zoom for clicking")
    parser.add_argument("--out", help="output folder (default: vision/output/calib_<time>)")
    parser.add_argument("--force", action="store_true", help="write camera_*.json even if the fit fails the check")
    parser.add_argument("--no-save", action="store_true", help="never write camera_*.json")
    return parser.parse_args()


def main():
    args = _parse_args()
    marks = load_marks(args.marks)
    out = args.out or os.path.join(HERE, "output", "calib_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)

    image = cv2.imread(args.image) if args.image else grab_average(args.color_index, args.frames)
    if image is None or image.shape[:2] != (HEIGHT, WIDTH):
        raise SystemExit(f"need a {WIDTH}x{HEIGHT} image, got {None if image is None else image.shape}")
    _write_image(os.path.join(out, FRAME_NAME), image)
    if args.points:
        points = parse_points(args.points, marks)
    elif os.environ.get("DISPLAY"):
        points = click_points(image, marks, args.scale)
        if points is None:
            raise SystemExit("quit before fitting")
    else:
        raise SystemExit(f"no DISPLAY to click on: set DISPLAY=:0, or pass --points (frame saved to {out}/frame.png)")

    lens_file = default_intrinsics_file()
    lens0 = load_intrinsics(lens_file) if lens_file else fallback_intrinsics(WIDTH, HEIGHT)
    if args.fx:
        lens0 = lens0._replace(fx=args.fx, fy=args.fx)
    mount0 = CameraMount(args.height, default_mount().pitch_deg, 0.0)   # marks are measured from O
    fit_lens = lens_is_observable(marks, points)
    best = fit(marks, points, lens0, mount0, fit_lens)
    if best is None:
        raise SystemExit("too few clicked marks to fit even the pitch")
    rows = mark_errors(marks, points, best.intrinsics, best.mount)
    _report("FIT (measured height)", best, rows)
    if not fit_lens:
        print(f"  fx not fitted - needs two marks with known y at least {MIN_LATERAL_SPREAD_M} m apart sideways; kept {lens0.fx:.1f}")
    free_h = fit(marks, points, lens0, mount0, fit_lens, fit_height=True)
    if free_h is not None:
        _report("check: height free too", free_h, None)
    ok = passes(rows)
    print(f"\n{'PASS' if ok else 'FAIL'}: mean floor error {PCT * np.mean([r['rel_err'] for r in rows if not r['check']]):.1f} % "
          f"of range (limit {PCT * PASS_REL_ERR:.1f} %)")
    print(f"redo offline: python3 vision/calib_floor.py --image {os.path.join(out, FRAME_NAME)} --height {args.height} "
          f"--marks {args.marks} --points {points_arg(points)}")

    _write_image(os.path.join(out, CHECK_NAME), draw_check(image, marks, points, best.intrinsics, best.mount))
    mount = CameraMount(best.mount.height_m, round(best.mount.pitch_deg, DEG_DIGITS), args.cam_forward, args.cam_left)
    lens = {name: round(value, PX_DIGITS) for name, value in best.intrinsics._asdict().items()}
    stamp = {"source": out, "created": datetime.now().isoformat(timespec="seconds")}
    _write_json(os.path.join(out, CALIB_NAME), {
        "marks_file": args.marks, "points": points_arg(points), "fitted": best.free, "rms_px": round(best.rms_px, PX_DIGITS),
        "mount": mount._asdict(), "intrinsics": lens, "pass": ok, "marks": rows,
        "height_free": None if free_h is None else {"height_m": round(free_h.mount.height_m, M_DIGITS),
                                                     "pitch_deg": round(free_h.mount.pitch_deg, DEG_DIGITS)}})
    if args.no_save or not (ok or args.force):
        print(f"camera_*.json NOT written ({'--no-save' if args.no_save else 'failed; --force to write anyway'})")
        return 0 if ok else 1
    _write_json(CAMERA_MOUNT_FILE, dict(mount._asdict(), **stamp))
    print(f"wrote {CAMERA_MOUNT_FILE}")
    if fit_lens:
        _write_json(CAMERA_RGB_FILE, dict(lens, **stamp))
        print(f"wrote {CAMERA_RGB_FILE}")
    return 0 if ok else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sys.exit(main())
