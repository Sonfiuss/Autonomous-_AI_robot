"""Offline tests of the drive-map chain - no camera, no robot, no model. Run: python test_drive_map.py

drive_timeline (the commanded profile and aligning it to measured motion), frame_motion (visual
odometry on a synthetic textured floor seen from known poses), drive_map's object landmarks, turn
axis and video, and stream_bench's rule for the largest good odometry step.
"""
import json
import math
import os
import sys
import tempfile

import cv2
import numpy as np

import drive_map
import drive_timeline
import stream_bench
from drawing import view_window
from floor_geometry import CameraMount
from floor_segment import floor_hit, floor_project
from frame_motion import estimate_motion, floor_rows, track
from object_distance import Intrinsics
from occupancy_map import PIXEL_FREE, OccupancyMap

H, W = 480, 640
INTR = Intrinsics(fx=570.0, fy=570.0, cx=319.5, cy=239.5)
MOUNT = CameraMount(height_m=0.24, pitch_deg=-0.92, forward_m=0.185, left_m=0.062)
TEXTURE_RES_M = 0.004              # floor texture: 4 mm per texel
TEXTURE_HALF_M = 4.0
# Blur of the texture, texels. Band-limited, as a lens delivers the real floor: at 2 texels the far
# floor aliases in the rendering and its moire, fixed to the pixel grid, pulled a 0.8 deg turn to 0.74.
TEXTURE_BLUR = 8.0
SKY = 90                           # grey above the horizon


def _leg(kind, amount, duration, speed):
    t, s = drive_timeline.trapezoid_profile(amount, speed, duration)
    return drive_timeline.Leg(0, kind, amount, duration, 0.0, t, s)


def check_trapezoid():
    """The fallback profile covers the leg in its duration, monotonically, never above cruise speed."""
    errors = []
    for amount, speed, duration in ((math.pi / 2, 0.3, 5.82), (-0.3, 0.15, 3.2), (0.02, 0.15, 0.5)):
        t, s = drive_timeline.trapezoid_profile(amount, speed, duration)
        v = np.diff(s) / np.diff(t)
        if abs(s[-1] - amount) > 1e-6 or np.any(np.sign(amount) * np.diff(s) < -1e-12):
            errors.append(f"{amount} in {duration} s: ends at {s[-1]:.5f}, monotonic {np.all(np.sign(amount) * np.diff(s) >= 0)}")
        if np.abs(v).max() > speed + 1e-6:
            errors.append(f"{amount}: peak {np.abs(v).max():.4f} over the cruise speed {speed}")
    return errors


def check_align():
    """A leg really started 2.2 s after its status line and went 0.8 of the command, measured at 30 fps
    with noise: align_leg must find the start within 50 ms."""
    errors = []
    rng = np.random.default_rng(3)
    for kind, amount, speed in ((drive_timeline.ROTATE, math.pi / 2, 0.3), (drive_timeline.FORWARD, 0.6, 0.15)):
        leg = _leg(kind, amount, 5.82 if kind == drive_timeline.ROTATE else 5.2, speed)
        times = np.arange(0.0, 12.0, 1 / 30.0)
        truth = 2.2
        moved = 0.8 * drive_timeline.displacement(leg, truth, times)
        measured = np.diff(moved, prepend=0.0) / (1 / 30.0) + rng.normal(0.0, 0.02 * speed, times.size)
        start, score = drive_timeline.align_leg(leg, times, measured, 0.0, 10.0)
        if abs(start - truth) > 0.05 or score < 0.9:
            errors.append(f"{kind}: start {start:.3f} (truth {truth}), score {score:.3f}")
    return errors


def _texture(seed=7):
    rng = np.random.default_rng(seed)
    n = int(2 * TEXTURE_HALF_M / TEXTURE_RES_M)
    return cv2.normalize(cv2.GaussianBlur(rng.integers(0, 255, (n, n)).astype(np.uint8), (0, 0), TEXTURE_BLUR),
                         None, 0, 255, cv2.NORM_MINMAX)


def _render_floor(texture, pose, mount=MOUNT):
    """The camera frame of a textured floor, robot centre at pose = (x, y, theta) in the texture's world."""
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    fwd, left, _ = floor_hit(u.ravel(), v.ravel(), INTR, mount)
    x, y, th = pose
    wx = x + math.cos(th) * fwd - math.sin(th) * left
    wy = y + math.sin(th) * fwd + math.cos(th) * left
    map_x = ((wx + TEXTURE_HALF_M) / TEXTURE_RES_M).reshape(H, W).astype(np.float32)
    map_y = ((wy + TEXTURE_HALF_M) / TEXTURE_RES_M).reshape(H, W).astype(np.float32)
    image = cv2.remap(texture, map_x, map_y, cv2.INTER_LINEAR, borderValue=SKY)
    image[~np.isfinite(fwd.reshape(H, W))] = SKY
    return image


def check_floor_odometry():
    """Two views of a textured floor from known poses - an in-place turn (the camera swings on its
    arc) and a turn with a sideways slip: estimate_motion must return the true relative motion."""
    errors = []
    texture = _texture()
    top, bottom = floor_rows(INTR, MOUNT, H)
    mask = np.zeros((H, W), np.uint8)
    mask[top:bottom] = 255
    for second in ((0.0, 0.0, math.radians(0.8)), (0.0, 0.0, math.radians(2.5)), (0.004, -0.006, math.radians(-1.5))):
        g0, g1 = _render_floor(texture, (0.0, 0.0, 0.0)), _render_floor(texture, second)
        m = estimate_motion(*track(g0, g1, mask), INTR, MOUNT, np.random.default_rng(0))
        if m is None:
            errors.append(f"motion {second}: not measured")
            continue
        if abs(m.dtheta - second[2]) > math.radians(0.02) or abs(m.dx - second[0]) > 0.003 or abs(m.dy - second[1]) > 0.003:
            errors.append(f"motion {second}: got dx {m.dx:.4f} dy {m.dy:.4f} dtheta {math.degrees(m.dtheta):.3f} deg")
    return errors


def check_landmarks():
    """A bottle standing still, seen from 5 poses of a turn, lands as ONE landmark where it is; a class
    seen twice is not a landmark."""
    errors = []
    world = (2.0, 0.5)
    poses = {seq: (0.0, 0.0, math.radians(a)) for seq, a in zip(range(10, 15), (0, 5, 10, 15, 20))}
    rows = []
    for seq, (x, y, th) in poses.items():
        c, s = math.cos(th), math.sin(th)
        bx, by = c * world[0] + s * world[1], -s * world[0] + c * world[1]     # world -> body
        u, v = floor_project([bx], [by], INTR, MOUNT)
        box = [float(u[0]) - 10, float(v[0]) - 60, float(u[0]) + 10, float(v[0])]
        rows.append({"seq": seq, "detections": [{"class": "bottle", "confidence": 0.8, "box": box}]})
    rows[0]["detections"].append({"class": "cup", "confidence": 0.6, "box": [300, 300, 320, 350]})
    rows[1]["detections"].append({"class": "cup", "confidence": 0.6, "box": [300, 300, 320, 350]})
    with tempfile.TemporaryDirectory() as run_dir:
        with open(os.path.join(run_dir, drive_map.DETECTIONS_FILE), "w", encoding="utf-8") as f:
            f.write("\n".join(json.dumps(r) for r in rows) + "\n")
        sightings = drive_map.object_sightings(run_dir, poses, INTR, MOUNT, OccupancyMap(), H)
    landmarks = drive_map.cluster_landmarks(sightings)
    if len(landmarks) != 1 or landmarks[0]["class"] != "bottle" or landmarks[0]["sightings"] != 5:
        errors.append(f"expected one bottle seen 5 times, got {landmarks}")
    elif math.hypot(landmarks[0]["x"] - world[0], landmarks[0]["y"] - world[1]) > 0.01:
        errors.append(f"bottle at {landmarks[0]['x']}, {landmarks[0]['y']}, truth {world}")
    return errors


def check_rotation_axis():
    """An in-place turn about the true centre, measured with the lens assumed 0.5 cm further ahead and
    on the centre line (the mount before the user measured it): rotation_axis finds the true pivot."""
    errors = []
    texture = _texture()
    assumed = MOUNT._replace(forward_m=0.18, left_m=0.0)
    top, bottom = floor_rows(INTR, assumed, H)
    mask = np.zeros((H, W), np.uint8)
    mask[top:bottom] = 255
    step = math.radians(1.5)
    motion = np.zeros((drive_map.AXIS_MIN_FRAMES + 2, drive_map.MOTION_COLUMNS))
    prev = _render_floor(texture, (0.0, 0.0, 0.0))
    for k in range(1, len(motion)):
        cur = _render_floor(texture, (0.0, 0.0, k * step))
        m = estimate_motion(*track(prev, cur, mask), INTR, assumed, np.random.default_rng(0))
        prev = cur
        if m is not None:
            motion[k] = (1, m.dx, m.dy, m.dtheta, m.inliers, m.tracks, m.rms_px)
    axis = drive_map.rotation_axis(motion)
    truth = (assumed.forward_m - MOUNT.forward_m, assumed.left_m - MOUNT.left_m)
    if axis is None or math.hypot(axis[0] - truth[0], axis[1] - truth[1]) > 0.01:
        errors.append(f"turn axis {axis}, truth {truth} (the true centre in the assumed body frame)")
    if drive_map.rotation_axis(motion[:drive_map.AXIS_MIN_FRAMES // 2]) is not None:
        errors.append("an axis from too few turning frames")
    return errors


def check_sweep_rule():
    """largest_good_step: the largest stride that, with every smaller one, measured all pairs and the
    turn within MAX_TOTAL_ERR - a good stride past a bad one does not count."""
    errors = []
    row = stream_bench.SweepRow
    rows = [row(1, 0.4, 1.0, 50, 1.0, 1.002), row(2, 0.9, 1.7, 50, 1.0, 0.99), row(5, 2.2, 3.0, 50, 0.96, 0.95),
            row(10, 4.4, 5.4, 50, 1.0, 1.0)]
    for given, want in ((rows, 2), (rows[:2], 2), (rows[2:], None), ([rows[0]._replace(total_ratio=math.nan)], None)):
        got = stream_bench.largest_good_step(given)
        if (got.stride if got else None) != want:
            errors.append(f"strides {[r.stride for r in given]}: largest good {got}, want {want}")
    return errors


def check_video():
    """MapVideo on a synthetic run: real-time length across a gap the camera dropped, both panels, and
    the video's own map ends equal to build_floor's (map.png's) - a no-plane analysis stays out of both."""
    errors = []
    texture = _texture()
    times = np.array([k / 30.0 for k in range(7)] + [0.3 + k / 30.0 for k in range(5)])   # 0.2 -> 0.3 s: 2 dropped
    poses = np.array([(0.0, 0.0, math.radians(0.5 * k)) for k in range(len(times))])
    frames = [{"seq": k, "t_capture": 100.0 + t} for k, t in enumerate(times)]
    labels = np.zeros((H, W), np.uint8)
    labels[400:, :] = PIXEL_FREE
    contacts = [[u, 399] for u in range(0, W, 4)]
    with tempfile.TemporaryDirectory() as run_dir:
        for sub_dir in (drive_map.RAW_DIR, drive_map.FLOOR_DIR):
            os.makedirs(os.path.join(run_dir, sub_dir))
        for fr, pose in zip(frames, poses):
            cv2.imwrite(os.path.join(run_dir, drive_map.RAW_DIR, drive_map.RAW_PATTERN.format(seq=fr["seq"])),
                        cv2.cvtColor(_render_floor(texture, tuple(pose)), cv2.COLOR_GRAY2BGR))
        cv2.imwrite(os.path.join(run_dir, drive_map.FLOOR_DIR, "labels.png"), labels)
        with open(os.path.join(run_dir, drive_map.FLOOR_DIR, drive_map.FLOOR_INDEX), "w", encoding="utf-8") as f:
            for seq, plane in ((2, [0.04, 0.0, -12.0]), (5, None), (8, [0.04, 0.0, -12.0])):
                f.write(json.dumps({"seq": seq, "labels": "labels.png", "plane": plane, "contacts": contacts}) + "\n")
        det = {"class": "bottle", "class_id": 39, "confidence": 0.8, "box": [300, 300, 340, 390], "range_m": 1.2,
               "z_m": 1.2, "xyz_cam": [0.0, 0.2, 1.2], "valid_fraction": None}
        with open(os.path.join(run_dir, drive_map.DETECTIONS_FILE), "w", encoding="utf-8") as f:
            for seq in (0, 4, 9):
                f.write(json.dumps({"seq": seq, "infer_ms": 50.0, "motion": "leg 1/1", "detections": [det]}) + "\n")
        pose_of = {fr["seq"]: tuple(p) for fr, p in zip(frames, poses)}
        rate_of = {fr["seq"]: 0.0 for fr in frames}
        final = OccupancyMap()
        drive_map.build_floor(final, run_dir, pose_of, rate_of, INTR, MOUNT)
        dm = drive_map.DriveMap(run_dir, {"frame_size": [W, H], "floor": {"far_m": 3.0, "half_width_m": None}}, INTR,
                                MOUNT, None, frames, times, np.zeros((len(frames), drive_map.MOTION_COLUMNS)),
                                np.zeros((len(frames), 2)), [], poses, np.zeros(len(frames)), pose_of, rate_of, final,
                                (2, 1), [])
        video = drive_map.MapVideo(dm, view_window(final, [tuple(p) for p in poses]))
        path = os.path.join(run_dir, drive_map.VIDEO_FILE)
        count = video.write(path)
        want = int((times[-1] - times[0]) * drive_map.VIDEO_FPS) + 1
        if count != want:
            errors.append(f"{count} video frames for {times[-1] - times[0]:.3f} s, want {want}")
        cap = cv2.VideoCapture(path)
        ok, image = cap.read()
        cap.release()
        if not ok or image.shape != (H, W + H, 3):
            errors.append(f"video frame {None if not ok else image.shape}, want {(H, W + H, 3)}")
        if not np.array_equal(video.occ_map.log_odds, final.log_odds):
            errors.append("the video's map ends different from build_floor's")
        if not final.free().any():
            errors.append("the synthetic floor made no free cell: the comparison above proves nothing")
    return errors


def main():
    failed = 0
    for check in (check_trapezoid, check_align, check_floor_odometry, check_landmarks, check_rotation_axis,
                  check_sweep_rule, check_video):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
