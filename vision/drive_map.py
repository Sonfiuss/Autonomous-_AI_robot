"""Map of the floor around a demo_drive run, from its recording alone - no camera, no robot
(task 2026-09-25_drive-map, plan steps 5-10).

  python3 vision/drive_map.py --run vision/output/<run_id>

Pose of every raw frame (robot centre, world = its pose at the first frame):
  heading   integrated from frame_motion - floor visual odometry across raw frames ODOMETRY_STRIDE
            apart. The user trusts the images over the wheels for turning (2026-09-25).
  position  the commanded FORWARD legs (drive_timeline, aligned to the measured motion), times
            --distance-scale, laid along that heading. frame_motion's own translation is reported
            beside it as a check.
Map: every floor analysis the recorder logged (floor/<seq>.png + its contacts) at the pose of its seq,
each with FLOOR_WEIGHT of a map_builder capture's evidence, so a cell needs ~2 agreeing frames.
Objects: YOLO boxes whose foot stands on the floor, clustered per class into landmarks.

Needs raw/ (the undrawn frames, recorded since 2026-09-26): without it there is no odometry and
no floor map, and the tool stops saying so.

Writes RUN/map/: map.png, views.png (the camera at the start and end of every leg), map_grid.npy +
map_meta.json, scene.json (scene_export v2.0), trajectory.csv, objects.json, alignment.txt, and
motion.npz (the odometry, reused by the next run with the same camera; --redo-motion recomputes it).
--video adds map.mp4, in real time: every raw frame with the newest YOLO boxes + distances and floor
analysis at or before it, beside the map as it grew up to that frame - to check one against the other.

The camera mount comes from recording.json; --cam-forward / --cam-left override the lens offset (runs
recorded before 2026-09-26 18:30 have forward 0.18, left 0 - the user then measured 0.185 / +0.062).
alignment.txt reports the axis the images say the robot turned about, to check that offset against.
"""
import argparse
import collections
import csv
import json
import logging
import math
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np

import drive_timeline
from drawing import (CONTACT_COLOR, CURRENT_POSE_COLOR, FONT, blend_overlay, class_color, floor_overlay, format_range,
                     map_px, put_text, render_map, trapezoid_outline, view_window)
from floor_geometry import CameraMount
from floor_segment import floor_hit, floor_trapezoid, floor_xy, free_floor_xy
from frame_motion import estimate_motion, floor_rows, track
from object_distance import Intrinsics, ObjectRange
from occupancy_map import OccupancyMap, body_to_world
from scene_export import extract_regions, to_scene

MAP_DIR = "map"
RAW_DIR, RAW_INDEX = "raw", "index.jsonl"
FLOOR_DIR, FLOOR_INDEX = "floor", "index.jsonl"
DETECTIONS_FILE = "detections.jsonl"
RECORDING_FILE = "recording.json"
RAW_PATTERN = "{seq:06d}.jpg"
MOTION_FILE, MAP_IMAGE, VIEWS_IMAGE = "motion.npz", "map.png", "views.png"
GRID_FILE, META_FILE, SCENE_FILE = "map_grid.npy", "map_meta.json", "scene.json"
OBJECTS_FILE, TRAJECTORY_FILE, ALIGNMENT_FILE = "objects.json", "trajectory.csv", "alignment.txt"
DEFAULT_DISTANCE_SCALE = 4.03 / 5.5    # measured roll radius / WHEEL_RADIUS_M (user, 2026-09-26)
FLOOR_WEIGHT = 0.5                     # of a map_builder capture: a cell needs two agreeing frames
FREE_COL_STRIDE = 2
ODOMETRY_STRIDE = 5                    # frames between odometry measurements (~2.3 deg apart at 0.3 rad/s)
LOG_EVERY_FRAMES = 100
MAX_TURN_RATE = 0.8                    # rad/s; floor frames turning faster are blurred and skipped
ALIGN_EARLY_S = 0.5                    # a status line is printed a little after its leg was sent
ALIGN_LATE_S = 1.0                     # ... and a leg may still be decelerating at the next status
ALIGN_TAIL_S = 0.5                     # the last leg's span: its commanded end plus this
OBJECT_MAX_RANGE_M = 3.5               # farther foot points: a pixel of error is > 7 cm
FOOT_EDGE_PX = 2                       # a box this close to the bottom edge has its foot out of view
LANDMARK_RADIUS_M = 0.3
LANDMARK_MIN_SIGHTINGS = 3
MAP_PX = 900
LEGEND_X, LEGEND_Y0, LEGEND_DY = 8, 18, 18
TRAJECTORY_COLOR = (255, 0, 0)
LEG_FOV_COLOR = (200, 160, 0)
LANDMARK_RADIUS_PX, LANDMARK_LINE_PX = 6, 2
LABEL_DX_PX, LABEL_DY_PX = 2, 4        # landmark label: right of the circle, baseline a little below
FOV_WEDGE_M = 1.0
HEADING_ARROW_M, ARROW_TIP = 0.3, 0.3
TEXT_WHITE = (255, 255, 255)
TEXT_SCALE = 0.45
POSE_DOT_EVERY = 15                    # frames between trajectory dots
VIEW_SCALE = 0.5                       # views.png tiles: half the camera frame
M_DIGITS, DEG_DIGITS, T_DIGITS, CONF_DIGITS = 4, 2, 3, 3
# Columns of the per-frame odometry array (measure_motion).
COL_OK, COL_DX, COL_DY, COL_DTHETA, COL_INLIERS, COL_TRACKS, COL_RMS = range(7)
MOTION_COLUMNS = 7
AXIS_MIN_TURN = math.radians(0.2)      # per frame: slower steps pin the turn axis poorly (0.46 at 0.3 rad/s)
AXIS_MIN_FRAMES = 30
VIDEO_FILE = "map.mp4"
VIDEO_FPS = 30.0                       # the camera's rate: raw frames are paced onto it, so it plays in real time
VIDEO_CODECS = ("avc1", "mp4v")        # the first that opens wins
VIDEO_BAR_H = 22                       # top bar of the camera panel (render_view's height)
VIDEO_TEXT_Y0 = 16                     # baseline of the top bar's text
VIDEO_TEXT_SCALE = 0.5                 # put_text's default, which the camera panel uses
VIDEO_TEXT_GAP_PX = 24                 # between the top bar's fields
BOX_LABEL_DX_PX, BOX_LABEL_DY_PX = 2, 6   # a box's label: right of and above its top-left corner (render_view)
VIDEO_FRESH_BOX_PX, VIDEO_HELD_BOX_PX = 2, 1   # a box on its own frame / held on later ones
VIDEO_CROSS_PX, VIDEO_CROSS_LINE_PX, VIDEO_POSE_ARROW_PX = 5, 2, 2
FRESH_COLOR, HELD_COLOR = (0, 220, 0), (0, 200, 255)
BAR_COLOR = (0, 0, 0)
MOTION_TEXT_COLOR = (0, 255, 255)      # the motion step, as record.py draws it
ROBOT_RADIUS_M = 0.225                 # the hull disc, as the MV planner sees it (MV_DEFAULT_ROBOT_RADIUS_M)
ROBOT_COLOR = (160, 160, 160)

# A YOLO box whose foot is on the floor, placed in the world.
Sighting = collections.namedtuple("Sighting", "name x y conf seq")

logger = logging.getLogger(__name__)


@dataclass
class LegFit:
    """A leg placed on the measured motion. span: frames first .. last - 1 it owns; measured: its
    motion along the leg (rad or m); odo_move: metres the odometry saw the centre move over it."""
    leg: drive_timeline.Leg
    start: float
    score: float
    span: tuple = (0, 0)
    measured: float = math.nan
    ratio: float = math.nan
    odo_move: float = math.nan


def load_jsonl(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def measure_motion(run_dir, frames, intrinsics, mount, times, stride=ODOMETRY_STRIDE):
    """frame_motion across frames `stride` apart, spread over the frames in between in proportion to
    their time: (N, MOTION_COLUMNS) array, columns COL_*; row 0 (no previous frame) and unmeasured
    frames have ok 0. On the real floor, steps of one frame (~0.5 deg) summed ~1 % short over a turn
    (72.7 vs 73.5 deg at stride 10); a stride that fails to match is measured frame by frame instead."""
    rows = np.zeros((len(frames), MOTION_COLUMNS))
    rng = np.random.default_rng(0)
    masks = {}

    def gray(k):
        return cv2.imread(os.path.join(run_dir, RAW_DIR, RAW_PATTERN.format(seq=frames[k]["seq"])), cv2.IMREAD_GRAYSCALE)

    def measure(image0, image1):
        if image0 is None or image1 is None:
            return None
        if image0.shape not in masks:
            top, bottom = floor_rows(intrinsics, mount, image0.shape[0])
            masks[image0.shape] = np.zeros(image0.shape, np.uint8)
            masks[image0.shape][top:bottom, :] = 255
        return estimate_motion(*track(image0, image1, masks[image0.shape]), intrinsics, mount, rng)

    def store(first, last, motion):
        """motion between frames first and last onto rows first + 1 .. last."""
        share = np.diff(times[first:last + 1]) / (times[last] - times[first])
        for k, w in zip(range(first + 1, last + 1), share):
            rows[k] = (1, w * motion.dx, w * motion.dy, w * motion.dtheta, motion.inliers, motion.tracks, motion.rms_px)

    keys = list(range(0, len(frames), stride))
    if keys[-1] != len(frames) - 1:
        keys.append(len(frames) - 1)
    image0 = gray(keys[0])
    for k0, k1 in zip(keys, keys[1:]):
        image1 = gray(k1)
        motion = measure(image0, image1) if times[k1] > times[k0] else None
        if motion is not None:
            store(k0, k1, motion)
        else:
            logger.debug("odometry: frames %d-%d did not match, measured one by one", k0, k1)
            prev = image0
            for k in range(k0 + 1, k1 + 1):
                cur = image1 if k == k1 else gray(k)
                single = measure(prev, cur) if times[k] > times[k - 1] else None
                if single is not None:
                    store(k - 1, k, single)
                prev = cur
        image0 = image1
        if k1 // LOG_EVERY_FRAMES != k0 // LOG_EVERY_FRAMES:
            logger.info("odometry: %d / %d frames", k1, len(frames))
    return rows


def cached_motion(run_dir, out_dir, frames, intrinsics, mount, times, redo):
    """measure_motion, reusing out_dir/MOTION_FILE when it was made for the same frames and camera."""
    path = os.path.join(out_dir, MOTION_FILE)
    key = np.array([len(frames), intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
                    mount.height_m, mount.pitch_deg, mount.forward_m, mount.left_m, ODOMETRY_STRIDE])
    if not redo and os.path.exists(path):
        with np.load(path) as saved:
            if saved["key"].shape == key.shape and np.allclose(saved["key"], key):
                logger.info("odometry reused from %s", path)
                return saved["rows"]
    rows = measure_motion(run_dir, frames, intrinsics, mount, times)
    np.savez(path, key=key, rows=rows)
    return rows


def odometry_track(motion):
    """(N,) heading and (N, 2) position of the robot centre from the odometry alone (a failed pair
    counts as no motion) - the check beside the fused poses."""
    dtheta = motion[:, COL_DTHETA] * (motion[:, COL_OK] > 0)
    theta = np.cumsum(dtheta)
    mid = theta - 0.5 * dtheta
    dx, dy = motion[:, COL_DX], motion[:, COL_DY]
    xy = np.column_stack((np.cumsum(dx * np.cos(mid) - dy * np.sin(mid)), np.cumsum(dx * np.sin(mid) + dy * np.cos(mid))))
    return theta, xy


def horizontal_fov(intrinsics):
    """Horizontal field of view, rad, edge pixel to edge pixel."""
    return 2.0 * math.atan((intrinsics.cx + 0.5) / intrinsics.fx)


def rotation_axis(motion):
    """(x, y) in the body frame of the point the robot turned about, or None with too little turning.
    Each turning step b_first = R b_second + t leaves one point in place, p = (I - R)^-1 t; the median
    over the steps. (0, 0): the turns pivot on the centre the mount is measured from. Anything else:
    the lens is not where the mount puts it relative to the turn axis - or the robot does not turn
    about its centre."""
    turning = (motion[:, COL_OK] > 0) & (np.abs(motion[:, COL_DTHETA]) > AXIS_MIN_TURN)
    if turning.sum() < AXIS_MIN_FRAMES:
        return None
    theta, tx, ty = motion[turning, COL_DTHETA], motion[turning, COL_DX], motion[turning, COL_DY]
    a, s = 1.0 - np.cos(theta), np.sin(theta)   # I - R = [[a, s], [-s, a]]
    det = a * a + s * s
    axis = float(np.median((a * tx - s * ty) / det)), float(np.median((s * tx + a * ty) / det))
    logger.debug("turn axis %+.4f %+.4f m from %d turning frames", axis[0], axis[1], int(turning.sum()))
    return axis


def align_legs(run, times, motion, odo_xy):
    """A LegFit per moving leg, starts in wall time. A leg's span runs from its start to the next leg's
    (back-to-back legs blend: the robot is still slowing when the next one starts), the last one's to
    its commanded end plus ALIGN_TAIL_S; spans never overlap, so their motion sums to the run's."""
    dt = np.diff(times, prepend=times[0])
    dt[0] = np.inf
    speed = {drive_timeline.ROTATE: motion[:, COL_DTHETA] / dt, drive_timeline.FORWARD: motion[:, COL_DX] / dt}
    fits, prev_end = [], -np.inf
    for k, leg in enumerate(run.legs):
        if leg.kind == drive_timeline.STOP or leg.status_t is None:
            continue
        later = [l.status_t for l in run.legs[k + 1:] if l.status_t is not None]
        lo = max(leg.status_t - ALIGN_EARLY_S, prev_end)
        hi = (later[0] if later else run.end_t) + ALIGN_LATE_S
        start, score = drive_timeline.align_leg(leg, times, speed[leg.kind], lo, hi)
        logger.debug("leg %d %s: window %.2f-%.2f -> start %.2f, score %.3f", leg.index + 1, leg.kind, lo, hi, start, score)
        fits.append(LegFit(leg, start, score))
        prev_end = start + leg.duration_s
    for k, fit in enumerate(fits):
        end = fits[k + 1].start if k + 1 < len(fits) else fit.start + fit.leg.duration_s + ALIGN_TAIL_S
        first, last = (int(i) for i in np.searchsorted(times, [fit.start, end], side="right"))
        column = COL_DTHETA if fit.leg.kind == drive_timeline.ROTATE else COL_DX
        fit.span = (first, last)
        fit.measured = float(motion[first:last, column].sum())
        fit.ratio = fit.measured / fit.leg.amount if fit.leg.amount else math.nan
        fit.odo_move = float(np.linalg.norm(odo_xy[max(last - 1, 0)] - odo_xy[max(first - 1, 0)]))
    return fits


def commanded(fits, kind, times):
    """Commanded displacement of every leg of `kind`, summed, at `times`."""
    total = np.zeros(len(times))
    for fit in fits:
        if fit.leg.kind == kind:
            total += drive_timeline.displacement(fit.leg, fit.start, times)
    return total


def integrate_poses(times, motion, fits, distance_scale):
    """(N, 3) x, y, theta of the robot centre per frame: odometry heading (the command, scaled by the
    measured turn ratio, where a frame pair failed) and commanded forward distance along it."""
    turn_ratios = [f.ratio for f in fits if f.leg.kind == drive_timeline.ROTATE and math.isfinite(f.ratio)]
    turn_scale = float(np.median(turn_ratios)) if turn_ratios else 1.0
    cmd_turn = np.diff(commanded(fits, drive_timeline.ROTATE, times), prepend=0.0)
    cmd_forward = np.diff(commanded(fits, drive_timeline.FORWARD, times), prepend=0.0) * distance_scale
    dtheta = np.where(motion[:, COL_OK] > 0, motion[:, COL_DTHETA], cmd_turn * turn_scale)
    dtheta[0] = 0.0
    theta = np.cumsum(dtheta)
    mid = theta - 0.5 * dtheta
    return np.column_stack((np.cumsum(cmd_forward * np.cos(mid)), np.cumsum(cmd_forward * np.sin(mid)), theta))


def floor_lines(run_dir):
    """floor/index.jsonl in seq order; [] when the run has no floor analyses."""
    path = os.path.join(run_dir, FLOOR_DIR, FLOOR_INDEX)
    return sorted(load_jsonl(path), key=lambda line: line["seq"]) if os.path.exists(path) else []


def floor_labels(run_dir, line):
    """The per-pixel labels of one floor analysis, None (and a warning) when unreadable."""
    labels = cv2.imread(os.path.join(run_dir, FLOOR_DIR, line["labels"]), cv2.IMREAD_UNCHANGED)
    if labels is None:
        logger.warning("floor labels %s unreadable: skipped", line["labels"])
    return labels


def floor_usable(line, pose_of, rate_of):
    """A floor analysis goes into the map: it has a pose, a floor plane, and was not turning fast."""
    seq = line["seq"]
    return seq in pose_of and abs(rate_of[seq]) <= MAX_TURN_RATE and line["plane"] is not None


def integrate_floor_line(occ_map, labels, line, pose, intrinsics, mount):
    """One floor analysis (labels + its line's contacts) into occ_map at pose."""
    contacts = np.asarray(line["contacts"], np.float64).reshape(-1, 2)
    occ_map.integrate_floor(free_floor_xy(labels, intrinsics, mount, FREE_COL_STRIDE),
                            floor_xy(contacts[:, 0], contacts[:, 1], intrinsics, mount), pose, weight=FLOOR_WEIGHT)


def build_floor(occ_map, run_dir, pose_of, rate_of, intrinsics, mount):
    """Every usable logged floor analysis into occ_map. Returns (used, skipped) counts."""
    used = skipped = 0
    for line in floor_lines(run_dir):
        labels = floor_labels(run_dir, line) if floor_usable(line, pose_of, rate_of) else None
        if labels is None:
            skipped += 1
            continue
        integrate_floor_line(occ_map, labels, line, pose_of[line["seq"]], intrinsics, mount)
        used += 1
    return used, skipped


def row_sightings(row, pose, intrinsics, mount, image_h):
    """(Sighting, (1, 2) body-frame foot) of every box of one detections.jsonl row whose foot is on the
    floor within OBJECT_MAX_RANGE_M, placed in the world from pose."""
    found = []
    for det in row["detections"]:
        x1, _, x2, y2 = det["box"]
        if y2 >= image_h - FOOT_EDGE_PX:
            continue
        fwd, left, z = floor_hit([0.5 * (x1 + x2)], [y2], intrinsics, mount)
        if not math.isfinite(float(z[0])) or float(z[0]) > OBJECT_MAX_RANGE_M:
            continue
        body = np.array([[float(fwd[0]), float(left[0])]])
        world = body_to_world(body, pose)[0]
        found.append((Sighting(det["class"], float(world[0]), float(world[1]), det["confidence"], row["seq"]), body))
    return found


def object_sightings(run_dir, pose_of, intrinsics, mount, occ_map, image_h):
    """Sightings of the YOLO boxes whose foot is on the floor within OBJECT_MAX_RANGE_M. Each also
    votes its class into occ_map for scene_export's labels."""
    sightings = []
    for row in load_jsonl(os.path.join(run_dir, DETECTIONS_FILE)):
        pose = pose_of.get(row["seq"])
        if pose is None:
            continue
        for sighting, body in row_sightings(row, pose, intrinsics, mount, image_h):
            occ_map.add_label_votes(sighting.name, sighting.conf, body, pose)
            sightings.append(sighting)
    return sightings


def cluster_landmarks(sightings):
    """Greedy per-class clusters within LANDMARK_RADIUS_M of their running median; those seen at least
    LANDMARK_MIN_SIGHTINGS times, as JSON-ready dicts. range_m: from the world origin, the robot centre
    at the first frame. A cluster's median is refreshed when it grows, not on every comparison."""
    clusters = {}                          # class -> list of [members, (median x, median y)]
    for sighting in sightings:
        for cluster in clusters.setdefault(sighting.name, []):
            members, (cx, cy) = cluster
            if math.hypot(sighting.x - cx, sighting.y - cy) < LANDMARK_RADIUS_M:
                members.append(sighting)
                cluster[1] = (float(np.median([m.x for m in members])), float(np.median([m.y for m in members])))
                break
        else:
            clusters[sighting.name].append([[sighting], (sighting.x, sighting.y)])
    return [{"class": name, "x": round(cx, M_DIGITS), "y": round(cy, M_DIGITS),
             "range_m": round(math.hypot(cx, cy), M_DIGITS), "sightings": len(members),
             "mean_conf": round(float(np.mean([m.conf for m in members])), CONF_DIGITS),
             "first_seq": min(m.seq for m in members), "last_seq": max(m.seq for m in members)}
            for name, found in clusters.items() for members, (cx, cy) in found
            if len(members) >= LANDMARK_MIN_SIGHTINGS]


def leg_end_frames(fits):
    """Frame index of the start and of the end of every leg's span."""
    return [0] + [max(f.span[1] - 1, 0) for f in fits]


def leg_views(run_dir, frames, poses, fits):
    """The raw camera view at the start and at the end of every leg, side by side and labelled - what
    the map should agree with. None when no frame could be read."""
    tiles = []
    for n, k in enumerate(leg_end_frames(fits)):
        image = cv2.imread(os.path.join(run_dir, RAW_DIR, RAW_PATTERN.format(seq=frames[k]["seq"])))
        if image is None:
            continue
        tile = cv2.resize(image, None, fx=VIEW_SCALE, fy=VIEW_SCALE, interpolation=cv2.INTER_AREA)
        name = "start" if n == 0 else f"end of leg {fits[n - 1].leg.index + 1}"
        put_text(tile, f"{name}: heading {math.degrees(poses[k][2]):+.0f} deg", (LEGEND_X, LEGEND_Y0), TEXT_WHITE, TEXT_SCALE)
        tiles.append(tile)
    return np.hstack(tiles) if tiles else None


def lens_position(mount, pose):
    """World (x, y) of the camera lens when the robot centre is at pose."""
    return tuple(body_to_world(np.array([[mount.forward_m, mount.left_m]]), pose)[0])


def draw_fov(img, px, lens, theta, hfov, color):
    """The camera's horizontal field of view: two FOV_WEDGE_M rays from the lens around heading theta."""
    for side in (-1, 1):
        a = theta + side * hfov / 2
        cv2.line(img, px(lens), px((lens[0] + FOV_WEDGE_M * math.cos(a), lens[1] + FOV_WEDGE_M * math.sin(a))), color, 1)


def draw_overlays(img, occ_map, points, poses, fits, landmarks, hfov, mount):
    """Trajectory, the camera FOV at the start and at the end of every leg (from the lens), and the
    landmarks, on render_map's image (same view: `points` is what render_map got)."""
    px = map_px(occ_map, points, MAP_PX)
    track_px = np.array([px(p) for p in poses[::POSE_DOT_EVERY]] + [px(poses[-1])], np.int32)
    cv2.polylines(img, [track_px], False, TRAJECTORY_COLOR, 1)
    for k in leg_end_frames(fits):
        x, y, theta = poses[min(k, len(poses) - 1)]
        draw_fov(img, px, lens_position(mount, (x, y, theta)), theta, hfov, LEG_FOV_COLOR)
        cv2.arrowedLine(img, px((x, y)), px((x + HEADING_ARROW_M * math.cos(theta), y + HEADING_ARROW_M * math.sin(theta))),
                        LEG_FOV_COLOR, 1, tipLength=ARROW_TIP)
    for lm in landmarks:
        color = class_color(lm["class"])
        cx, cy = px((lm["x"], lm["y"]))
        cv2.circle(img, (cx, cy), LANDMARK_RADIUS_PX, color, LANDMARK_LINE_PX)
        label = f"{lm['class']} {lm['range_m']:.2f}m x{lm['sightings']}"
        (width, _), _ = cv2.getTextSize(label, FONT, TEXT_SCALE, 1)
        x = cx + LANDMARK_RADIUS_PX + LABEL_DX_PX
        if x + width >= img.shape[1]:                  # no room on the right: put it on the left
            x = cx - LANDMARK_RADIUS_PX - LABEL_DX_PX - width
        put_text(img, label, (x, cy + LABEL_DY_PX), color, TEXT_SCALE)


def draw_legend(img, run_id, fits, poses, odo_xy):
    """Top-left text: every leg's command against what the images measured, and the end pose."""
    lines = [f"run {run_id}"]
    for f in fits:
        unit, conv = ("deg", math.degrees) if f.leg.kind == drive_timeline.ROTATE else ("m", float)
        lines.append(f"leg {f.leg.index + 1} {f.leg.kind}: cmd {conv(f.leg.amount):+.1f} {unit}, "
                     f"image {conv(f.measured):+.1f} {unit} ({f.ratio:.2f})")
    x, y, theta = poses[-1]
    lines.append(f"end: x {x:+.2f} y {y:+.2f} m, heading {math.degrees(theta):+.1f} deg "
                 f"(odometry xy {odo_xy[-1][0]:+.2f} {odo_xy[-1][1]:+.2f})")
    for k, text in enumerate(lines):
        put_text(img, text, (LEGEND_X, LEGEND_Y0 + k * LEGEND_DY), TEXT_WHITE, TEXT_SCALE)


def axis_line(axis, mount):
    """alignment.txt's report of rotation_axis against the mount."""
    if axis is None:
        return "turn axis: too little turning to measure"
    return (f"turn axis from the images: ({axis[0]:+.3f}, {axis[1]:+.3f}) m in the body frame -> the lens sits "
            f"{mount.forward_m - axis[0]:.3f} m ahead of it and {mount.left_m - axis[1]:+.3f} m to its left "
            f"(mount: {mount.forward_m:.3f}, {mount.left_m:+.3f}). (0, 0) = the robot turns about the centre "
            "the mount is measured from.")


def write_alignment(path, run_id, run, fits, motion, poses, odo_xy, intrinsics, mount, distance_scale, floor_counts,
                    axis):
    """alignment.txt: the camera used, the odometry quality and turn axis, and per leg its delay, how well
    the command matched, and command vs measurement. Returns its lines."""
    ok = motion[1:, COL_OK] > 0
    odometry = (f"odometry: {int(ok.sum())} / {ok.size} frame pairs measured, median "
                f"{np.median(motion[1:, COL_INLIERS][ok]):.0f} inliers, median rms {np.median(motion[1:, COL_RMS][ok]):.2f} px"
                if ok.any() else "odometry: none")
    lines = [f"run {run_id}   fx {intrinsics.fx:.1f}  pitch {mount.pitch_deg:+.2f} deg  height {mount.height_m:.3f} m  "
             f"forward {mount.forward_m:.3f} m  left {mount.left_m:+.3f} m  distance_scale {distance_scale:.3f}",
             odometry, axis_line(axis, mount),
             f"floor analyses used {floor_counts[0]}, skipped {floor_counts[1]} (turning > {MAX_TURN_RATE} rad/s, "
             "no floor plane, or no pose)", "",
             "leg  kind     commanded   status_s  start_s  delay_s  score   measured   ratio   odo_move_m",
             "     (deg for ROTATE, m for FORWARD; times from the first leg's status line)"]
    t0 = next((leg.status_t for leg in run.legs if leg.status_t is not None), 0.0)
    for f in fits:
        conv = math.degrees if f.leg.kind == drive_timeline.ROTATE else float
        lines.append(f"{f.leg.index + 1:<4d} {f.leg.kind:8s} {conv(f.leg.amount):+9.2f}  {f.leg.status_t - t0:8.2f} "
                     f"{f.start - t0:8.2f} {f.start - f.leg.status_t:8.2f}  {f.score:5.3f} {conv(f.measured):+9.2f}"
                     f"  {f.ratio:6.3f}  {f.odo_move:8.3f}")
    turns = [f.ratio for f in fits if f.leg.kind == drive_timeline.ROTATE]
    if turns:
        lines.append(f"\nturn: measured / commanded = {np.mean(turns):.3f} (legs {', '.join(f'{r:.3f}' for r in turns)}). "
                     f"It scales with 1/fx, and fx {intrinsics.fx:.0f} is not measured on this camera.")
    x, y, theta = poses[-1]
    lines.append(f"end pose: x {x:+.3f} y {y:+.3f} m, heading {math.degrees(theta):+.2f} deg; "
                 f"odometry alone puts the centre at x {odo_xy[-1][0]:+.3f} y {odo_xy[-1][1]:+.3f} m")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return lines


def write_trajectory(path, frames, times, poses, motion):
    """trajectory.csv: per raw frame, the fused pose and the odometry step that led to it."""
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["seq", "t_capture", "t_s", "x_m", "y_m", "theta_deg", "odo_ok", "odo_inliers",
                         "odo_dx_m", "odo_dy_m", "odo_dtheta_deg"])
        for fr, p, m in zip(frames, poses, motion):
            writer.writerow([fr["seq"], fr["t_capture"], round(fr["t_capture"] - times[0], T_DIGITS),
                             round(p[0], M_DIGITS), round(p[1], M_DIGITS), round(math.degrees(p[2]), DEG_DIGITS),
                             int(m[COL_OK]), int(m[COL_INLIERS]), round(m[COL_DX], M_DIGITS), round(m[COL_DY], M_DIGITS),
                             round(math.degrees(m[COL_DTHETA]), DEG_DIGITS + 1)])


def object_range(det):
    """ObjectRange of a detections.jsonl detection, None when it had no distance."""
    if det["range_m"] is None:
        return None
    return ObjectRange(det["range_m"], det["z_m"], tuple(det["xyz_cam"]), det["valid_fraction"])


def leg_at(fits, k):
    """The LegFit whose span holds frame k, or None between and around the legs."""
    return next((f for f in fits if f.span[0] <= k < f.span[1]), None)


class MapVideo:
    """map.mp4: each raw frame beside the map as it stood when that frame was taken. Built on one pass
    over the frames in seq order - the floor analyses go into the video's own map as their seq comes up,
    with drive_map's rules (floor_usable), so its last frame shows the map of map.png."""

    def __init__(self, dm, window):
        """dm: the run's DriveMap; window: the world (x0, y0, span) of map.png (map_view)."""
        self.run_dir, self.frames, self.times, self.poses = dm.run_dir, dm.frames, dm.times, dm.poses
        self.fits, self.landmarks, self.window = dm.fits, dm.landmarks, window
        self.intrinsics, self.mount = dm.intrinsics, dm.mount
        self.pose_of, self.rate_of = dm.pose_of, dm.rate_of
        self.image_w, self.image_h = dm.recording["frame_size"]
        self.map_size = self.image_h            # the map panel: square, as tall as the camera frame
        floor_cfg = dm.recording.get("floor")
        self.outline = () if floor_cfg is None else trapezoid_outline(floor_trapezoid(
            (self.image_h, self.image_w), self.intrinsics, self.mount, floor_cfg["far_m"], floor_cfg["half_width_m"]))
        self.hfov = horizontal_fov(self.intrinsics)
        self.turned = commanded(self.fits, drive_timeline.ROTATE, self.times)
        self.occ_map = OccupancyMap()
        self.cells = (0, 0)                     # free, occupied cells of occ_map, refreshed as it changes
        self.px = map_px(self.occ_map, [], self.map_size, window)
        self.m_px = self.map_size / window[2]
        self.track_px = np.array([self.px(p) for p in self.poses], np.int32)   # the window is fixed

    def write(self, path):
        """Writes the video; returns its frame count. OSError when no codec opens."""
        detections = sorted(load_jsonl(os.path.join(self.run_dir, DETECTIONS_FILE)), key=lambda r: r["seq"])
        floors = floor_lines(self.run_dir)
        writer = self.open_writer(path)
        base = det_row = floor = None
        next_det = next_floor = written = 0
        try:
            for k, fr in enumerate(self.frames):
                seq = fr["seq"]
                while next_det < len(detections) and detections[next_det]["seq"] <= seq:
                    det_row, next_det = detections[next_det], next_det + 1
                while next_floor < len(floors) and floors[next_floor]["seq"] <= seq:
                    floor = self._add_floor(floors[next_floor]) or floor
                    next_floor, base = next_floor + 1, None
                if base is None:                # the map changed: redraw it, overlays go on a copy
                    base = render_map(self.occ_map, [], [], None, self.map_size, window=self.window)
                image = cv2.imread(os.path.join(self.run_dir, RAW_DIR, RAW_PATTERN.format(seq=seq)))
                if image is None:
                    logger.warning("raw frame %d unreadable: not in the video", seq)
                    continue
                panel = np.hstack((self.camera_panel(image, seq, det_row, floor), self.map_panel(base.copy(), k, det_row)))
                due = int((self.times[k] - self.times[0]) * VIDEO_FPS) + 1
                while written < due:           # repeat across the camera's own dropped frames: real time
                    writer.write(panel)
                    written += 1
                if k and k % LOG_EVERY_FRAMES == 0:
                    logger.info("video: %d / %d frames", k, len(self.frames))
        finally:
            writer.release()                   # writes the mp4 index; without it the file does not play
        return written

    def open_writer(self, path):
        """A VideoWriter for this video at path, the first of VIDEO_CODECS that opens. OSError: none did."""
        size = (self.image_w + self.map_size, self.image_h)
        for fourcc in VIDEO_CODECS:
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), VIDEO_FPS, size)
            if writer.isOpened():
                logger.info("video %s (%s, %.0f fps)", path, fourcc, VIDEO_FPS)
                return writer
            writer.release()
        raise OSError(f"no video codec opened for {path}: " + ", ".join(VIDEO_CODECS))

    def _add_floor(self, line):
        """(line, floor_overlay) of a floor analysis for camera_panel, None when unreadable; into the map
        if usable."""
        labels = floor_labels(self.run_dir, line)
        if labels is None:
            return None
        if floor_usable(line, self.pose_of, self.rate_of):
            integrate_floor_line(self.occ_map, labels, line, self.pose_of[line["seq"]], self.intrinsics, self.mount)
            self.cells = int(self.occ_map.free().sum()), int(self.occ_map.occupied().sum())
        return line, floor_overlay(labels)

    def camera_panel(self, image, seq, det_row, floor):
        """The raw frame redrawn: the newest floor analysis ((line, floor_overlay), as _add_floor gives it)
        and YOLO boxes at or before it, green when they are this frame's own, orange with their age in
        frames when held from an earlier one."""
        if floor is not None:
            line, overlay = floor
            image = blend_overlay(image, overlay, self.outline)
            contacts = np.asarray(line["contacts"], np.int64).reshape(-1, 2)
            image[contacts[:, 1], contacts[:, 0]] = CONTACT_COLOR
        width = VIDEO_FRESH_BOX_PX if det_row is not None and det_row["seq"] == seq else VIDEO_HELD_BOX_PX
        for det in det_row["detections"] if det_row is not None else []:
            x1, y1, x2, y2 = (int(round(v)) for v in det["box"])
            color = class_color(det["class_id"])
            cv2.rectangle(image, (x1, y1), (x2, y2), color, width)
            put_text(image, f"{det['class']} {det['confidence']:.2f} | {format_range(object_range(det))}",
                     (x1 + BOX_LABEL_DX_PX, max(y1 - BOX_LABEL_DY_PX, VIDEO_BAR_H + VIDEO_TEXT_Y0)), color)
        cv2.rectangle(image, (0, 0), (image.shape[1], VIDEO_BAR_H), BAR_COLOR, -1)
        texts = [(f"seq {seq}", TEXT_WHITE)]
        for name, got in (("YOLO", det_row), ("DA", None if floor is None else floor[0])):
            age = None if got is None else seq - got["seq"]
            texts.append((f"{name} " + ("--" if age is None else "this frame" if age == 0 else f"-{age} fr"),
                          FRESH_COLOR if age == 0 else HELD_COLOR))
        if det_row is not None:
            texts.append((f"infer {det_row['infer_ms']:.0f} ms", TEXT_WHITE))
        x = LEGEND_X
        for text, color in texts:
            put_text(image, text, (x, VIDEO_TEXT_Y0), color, VIDEO_TEXT_SCALE)
            x += cv2.getTextSize(text, FONT, VIDEO_TEXT_SCALE, 1)[0][0] + VIDEO_TEXT_GAP_PX
        if det_row is not None and det_row.get("motion"):
            put_text(image, det_row["motion"], (LEGEND_X, self.image_h - LEGEND_X), MOTION_TEXT_COLOR, VIDEO_TEXT_SCALE)
        return image

    def map_panel(self, img, k, det_row):
        """Robot, camera FOV and trajectory up to frame k on the map, the landmarks already seen, and where
        the current YOLO boxes put their objects (from the pose of their own frame)."""
        px, pose = self.px, tuple(self.poses[k])
        x, y, theta = pose
        c, s = math.cos(theta), math.sin(theta)
        cv2.polylines(img, [self.track_px[:k + 1]], False, TRAJECTORY_COLOR, 1)
        cv2.circle(img, px((x, y)), max(int(round(ROBOT_RADIUS_M * self.m_px)), 1), ROBOT_COLOR, 1)
        draw_fov(img, px, lens_position(self.mount, pose), theta, self.hfov, CURRENT_POSE_COLOR)
        cv2.arrowedLine(img, px((x, y)), px((x + HEADING_ARROW_M * c, y + HEADING_ARROW_M * s)), CURRENT_POSE_COLOR,
                        VIDEO_POSE_ARROW_PX, tipLength=ARROW_TIP)
        seq = self.frames[k]["seq"]
        for lm in self.landmarks:
            if lm["first_seq"] <= seq:
                cv2.circle(img, px((lm["x"], lm["y"])), LANDMARK_RADIUS_PX, class_color(lm["class"]), LANDMARK_LINE_PX)
        det_pose = None if det_row is None else self.pose_of.get(det_row["seq"])
        if det_pose is not None:
            for sighting, _ in row_sightings(det_row, det_pose, self.intrinsics, self.mount, self.image_h):
                u, v = px((sighting.x, sighting.y))
                color = class_color(sighting.name)
                cv2.drawMarker(img, (u, v), color, cv2.MARKER_CROSS, 2 * VIDEO_CROSS_PX, VIDEO_CROSS_LINE_PX)
                put_text(img, sighting.name, (u + VIDEO_CROSS_PX + LABEL_DX_PX, v + LABEL_DY_PX), color, TEXT_SCALE)
        fit = leg_at(self.fits, k)
        if fit is None:
            leg = "no leg"
        else:
            rotate = fit.leg.kind == drive_timeline.ROTATE
            amount = f"{math.degrees(fit.leg.amount):+.1f} deg" if rotate else f"{fit.leg.amount:+.2f} m"
            leg = f"leg {fit.leg.index + 1} {fit.leg.kind} {amount}"
        lines = [f"t {self.times[k] - self.times[0]:5.2f} s  {leg}",
                 f"heading image {math.degrees(theta):+6.1f} deg | cmd {math.degrees(self.turned[k]):+6.1f} deg",
                 f"x {x:+.2f} y {y:+.2f} m   map {self.cells[0]} free / {self.cells[1]} occ cells"]
        for n, text in enumerate(lines):
            put_text(img, text, (LEGEND_X, LEGEND_Y0 + n * LEGEND_DY), TEXT_WHITE, TEXT_SCALE)
        return img


def _write_json(path, data):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def _write_image(path, image):
    if not cv2.imwrite(path, image):
        raise OSError(f"cannot write {path}")


def _parse_args():
    parser = argparse.ArgumentParser(description="Build a floor map from a demo_drive recording.")
    parser.add_argument("--run", required=True, help="vision/output/<run_id>")
    parser.add_argument("--distance-scale", type=float, default=DEFAULT_DISTANCE_SCALE,
                        help="real / commanded forward distance (default: %(default).3f = 4.03 / 5.5 cm)")
    parser.add_argument("--fx", type=float, help="override the recording's fx (fy follows)")
    parser.add_argument("--cam-pitch", type=float, help="override the recording's pitch, deg, + = down")
    parser.add_argument("--cam-forward", type=float, help="override the recording's lens offset ahead of the robot centre, m")
    parser.add_argument("--cam-left", type=float, help="override the recording's lens offset left of the robot centre, m")
    parser.add_argument("--video", action="store_true", help=f"also write {VIDEO_FILE}: camera + detections beside the growing map")
    parser.add_argument("--redo-motion", action="store_true", help=f"recompute {MOTION_FILE}")
    return parser.parse_args()


@dataclass
class DriveMap:
    """A run's map and what it was built from (build()): main() writes it out, stream_bench times it.
    rate: rad/s of the fused heading per frame; pose_of / rate_of: the same keyed by seq."""
    run_dir: str
    recording: dict
    intrinsics: Intrinsics
    mount: CameraMount
    run: drive_timeline.Run
    frames: list
    times: np.ndarray
    motion: np.ndarray
    odo_xy: np.ndarray
    fits: list
    poses: np.ndarray
    rate: np.ndarray
    pose_of: dict
    rate_of: dict
    occ_map: OccupancyMap
    floor_counts: tuple
    landmarks: list


def load_camera(run_dir, fx=None, pitch_deg=None, forward_m=None, left_m=None):
    """(recording.json, Intrinsics, CameraMount) of a run, with whichever overrides are not None."""
    with open(os.path.join(run_dir, RECORDING_FILE), encoding="utf-8") as f:
        recording = json.load(f)
    intrinsics = Intrinsics(**recording["intrinsics"])
    if fx:
        intrinsics = intrinsics._replace(fx=fx, fy=fx)
    changes = {name: value for name, value in (("pitch_deg", pitch_deg), ("forward_m", forward_m), ("left_m", left_m))
               if value is not None}
    return recording, intrinsics, CameraMount(**recording["mount"])._replace(**changes)


def build(run_dir, recording, intrinsics, mount, distance_scale=DEFAULT_DISTANCE_SCALE, redo_motion=False):
    """The DriveMap of a run. FileNotFoundError when it has no raw/ (recorded before 2026-09-26)."""
    raw_index = os.path.join(run_dir, RAW_DIR, RAW_INDEX)
    if not os.path.exists(raw_index):
        raise FileNotFoundError(f"{run_dir} has no {RAW_DIR}/ (recorded before 2026-09-26): no undrawn frames, "
                                "so no odometry and no floor map")
    out_dir = os.path.join(run_dir, MAP_DIR)
    os.makedirs(out_dir, exist_ok=True)
    run = drive_timeline.load_run(run_dir)
    frames = load_jsonl(raw_index)
    times = np.array([fr["t_capture"] for fr in frames])

    motion = cached_motion(run_dir, out_dir, frames, intrinsics, mount, times, redo_motion)
    _, odo_xy = odometry_track(motion)
    fits = align_legs(run, times, motion, odo_xy)
    poses = integrate_poses(times, motion, fits, distance_scale)
    dt = np.diff(times, prepend=times[0])
    rate = np.divide(np.diff(poses[:, 2], prepend=0.0), dt, out=np.zeros(len(dt)), where=dt > 0)
    pose_of = {fr["seq"]: tuple(p) for fr, p in zip(frames, poses)}
    rate_of = {fr["seq"]: r for fr, r in zip(frames, rate)}

    occ_map = OccupancyMap()
    floor_counts = build_floor(occ_map, run_dir, pose_of, rate_of, intrinsics, mount)
    landmarks = cluster_landmarks(object_sightings(run_dir, pose_of, intrinsics, mount, occ_map,
                                                   recording["frame_size"][1]))
    return DriveMap(run_dir, recording, intrinsics, mount, run, frames, times, motion, odo_xy, fits, poses, rate,
                    pose_of, rate_of, occ_map, floor_counts, landmarks)


def map_view(dm):
    """(dots, extent, window) of map.png: trajectory dots, landmark points kept in view, and the world
    window render_map picks for them - the video uses the same one."""
    dots = [tuple(p) for p in dm.poses[::POSE_DOT_EVERY]]
    extent = [(lm["x"], lm["y"]) for lm in dm.landmarks]
    return dots, extent, view_window(dm.occ_map, dots + [tuple(dm.poses[-1])] + extent)


def main():
    args = _parse_args()
    run_dir = args.run.rstrip("/")
    run_id = os.path.basename(run_dir)
    out_dir = os.path.join(run_dir, MAP_DIR)
    recording, intrinsics, mount = load_camera(run_dir, args.fx, args.cam_pitch, args.cam_forward, args.cam_left)
    try:
        dm = build(run_dir, recording, intrinsics, mount, args.distance_scale, args.redo_motion)
    except FileNotFoundError as exc:
        raise SystemExit(str(exc)) from exc
    occ_map, poses, fits, landmarks = dm.occ_map, dm.poses, dm.fits, dm.landmarks
    regions = extract_regions(occ_map)
    hfov = horizontal_fov(intrinsics)

    dots, extent, window = map_view(dm)
    # No FOV from render_map: it draws it from the centre; draw_overlays draws it from the lens.
    img = render_map(occ_map, regions, dots, tuple(poses[-1]), MAP_PX, None, extent)
    draw_overlays(img, occ_map, dots + [tuple(poses[-1])] + extent, poses, fits, landmarks, hfov, mount)
    draw_legend(img, run_id, fits, poses, dm.odo_xy)
    _write_image(os.path.join(out_dir, MAP_IMAGE), img)
    views = leg_views(run_dir, dm.frames, poses, fits)
    if views is not None:
        _write_image(os.path.join(out_dir, VIEWS_IMAGE), views)

    axis = rotation_axis(dm.motion)
    np.save(os.path.join(out_dir, GRID_FILE), occ_map.log_odds)
    _write_json(os.path.join(out_dir, META_FILE), {
        "run": run_id, "mount": mount._asdict(), "intrinsics": intrinsics._asdict(),
        "distance_scale": args.distance_scale, "floor_weight": FLOOR_WEIGHT, "res_m": occ_map.res,
        "turn_axis_m": None if axis is None else [round(v, M_DIGITS) for v in axis],
        "origin": list(occ_map.origin), "cells": occ_map.n,
        "world": "robot centre at the first raw frame: x forward, y left, theta CCW",
        "grid": "log-odds float32 [iy, ix]; row iy grows with world y"})
    _write_json(os.path.join(out_dir, SCENE_FILE), to_scene(occ_map, regions, tuple(poses[-1])))
    _write_json(os.path.join(out_dir, OBJECTS_FILE), landmarks)
    write_trajectory(os.path.join(out_dir, TRAJECTORY_FILE), dm.frames, dm.times, poses, dm.motion)
    lines = write_alignment(os.path.join(out_dir, ALIGNMENT_FILE), run_id, dm.run, fits, dm.motion, poses, dm.odo_xy,
                            intrinsics, mount, args.distance_scale, dm.floor_counts, axis)
    print("\n".join(lines))
    print(f"\nlandmarks: {[(lm['class'], lm['x'], lm['y'], lm['sightings']) for lm in landmarks]}")
    print(f"map -> {os.path.join(out_dir, MAP_IMAGE)}")
    if args.video:
        count = MapVideo(dm, window).write(os.path.join(out_dir, VIDEO_FILE))
        print(f"video -> {os.path.join(out_dir, VIDEO_FILE)} ({count} frames, {count / VIDEO_FPS:.1f} s)")
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sys.exit(main())
