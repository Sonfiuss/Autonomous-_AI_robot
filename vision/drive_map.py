"""Map of the floor around a demo_drive run, from its recording alone - no camera, no robot
(task 2026-09-25_drive-map, plan steps 5-10).

  python3 vision/drive_map.py --run vision/output/<run_id>

Pose of every raw frame (robot centre, world = its pose at the first frame), both from frame_motion -
floor visual odometry across raw frames ODOMETRY_STRIDE apart. The user trusts the images over the wheels
(turns 2026-09-25, all motion 2026-09-28: the firmware cut backward legs short and the wheels slip, so a
command says little about where the robot went):
  heading   the odometry's turn times the measured turn scale (frame_motion.vo_turn_scale, --vo-turn-scale:
            the floor odometry reads turns ~10 % long, 2026-09-29).
  position  the odometry's translation times the tape measure's scale (frame_motion.vo_scale, --vo-scale),
            laid along that heading.
A frame pair the odometry could not measure takes the command instead (drive_timeline, aligned to the
measured motion): its turn scaled by the run's measured turn ratio, its forward step by what its leg
measured over the command where the pairs did work (--distance-scale converts commanded metres to rolled).
alignment.txt flags a leg the firmware acked well before its planned duration (drive_timeline.cut_short):
it is left out of the slip ratios, since a floor did not cause it.
Map: every floor analysis the recorder logged (floor/<seq>.png + its contacts) at the pose of its seq,
each with FLOOR_WEIGHT of a map_builder capture's evidence, so a cell needs ~2 agreeing frames.
Objects: YOLO boxes whose foot stands on the floor, clustered per class into landmarks; clusters one box held
together are the parts of one large object (merge_by_box). A box's class votes go on what its object lies on
when that is an obstacle in front of its foot (support_points: a bed under a person), for the scene's labels.

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
import itertools
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
from frame_motion import vo_scale, vo_turn_scale
from leg_odometry import (COL_DTHETA, COL_DX, COL_DY, COL_INLIERS, COL_OK, COL_RMS, LOG_EVERY_FRAMES,
                          ODOMETRY_STRIDE, RAW_DIR, RAW_INDEX, RAW_PATTERN, VO_MIN_PAIRS, load_jsonl, measure_motion,
                          measured_share, odometry_track)
from object_color import ColorVotes
from object_distance import Intrinsics, ObjectRange
from occupancy_map import KNOWN_FREE, KNOWN_OCCUPIED, KNOWN_UNSEEN, OccupancyMap, body_to_world
from scene_export import extract_regions, to_scene

MAP_DIR = "map"
FLOOR_DIR, FLOOR_INDEX = "floor", "index.jsonl"
DETECTIONS_FILE = "detections.jsonl"
RECORDING_FILE = "recording.json"
MOTION_FILE, MAP_IMAGE, VIEWS_IMAGE = "motion.npz", "map.png", "views.png"
GRID_FILE, META_FILE, SCENE_FILE = "map_grid.npy", "map_meta.json", "scene.json"
KNOWN_GRID_FILE = "known_grid.npy"
REALROOM_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "realroom")
OBJECTS_FILE, TRAJECTORY_FILE, ALIGNMENT_FILE = "objects.json", "trajectory.csv", "alignment.txt"
FLOOR_WEIGHT = 0.5                     # of a map_builder capture: a cell needs two agreeing frames
FREE_COL_STRIDE = 2
MAX_TURN_RATE = 0.8                    # rad/s; floor frames turning faster are blurred and skipped
ALIGN_EARLY_S = 0.5                    # a status line is printed a little after its leg was sent
ALIGN_LATE_S = 1.0                     # ... and a leg may still be decelerating at the next status
ALIGN_TAIL_S = 0.5                     # the last leg's span: its commanded end plus this
# A forward leg whose measured frames saw less than this share of its command fills its unmeasured frames
# with the run's ratio instead of its own (too little of it measured to say how far it really went).
FILL_MIN_SHARE = 0.25
OBJECT_MAX_RANGE_M = 3.5               # farther foot points: a pixel of error is > 7 cm
# A box this close to the bottom edge has its foot out of view. YOLO's edges wander a few px: at 2, a person's
# box ending at 476-478 px put them at the nearest floor the camera sees (run 20260929_231612).
FOOT_EDGE_PX = 8
LANDMARK_RADIUS_M = 0.3
LANDMARK_MIN_SIGHTINGS = 3
MERGE_MARGIN_PX = 10                   # a landmark whose centre falls this close to a box's edges was in the box
MERGE_SLACK_M = 0.15                   # ... and two it held merge when no farther apart than it is wide plus this
SUPPORT_RAYS = 7                       # rays under a box's bottom edge, searching for what its object lies on
SUPPORT_MID_SHARE = 0.6                # ... over the middle of its width (a body's ends are thinner)
SUPPORT_STEP_M = 0.025                 # half a cell
SUPPORT_AHEAD_M = 0.10                 # an obstacle this far before a box's floor point holds its object up
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
FORWARD_DIGITS = 3                     # alignment.txt: a FORWARD leg in m, to the mm a tape measure reads
PERCENT = 100.0
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
Sighting = collections.namedtuple("Sighting", "name x y conf seq view")
# Where a sighting's box looked: the lens's world xy, the world bearings (rad, CCW) of the box's right and
# left edges from it, widened by MERGE_MARGIN_PX, and the box's width in metres at its foot.
BoxView = collections.namedtuple("BoxView", "lens right left width_m")

logger = logging.getLogger(__name__)


@dataclass
class LegFit:
    """A leg placed on the measured motion. span: frames first .. last - 1 it owns; measured: its
    motion along the leg (rad, or m at the VO scale) over the pairs that were measured; pairs: the share
    of its span's time those pairs cover (frames that were never recorded count as unmeasured - a camera
    lost mid-leg); odo_move: metres the raw odometry saw the centre move over it; firmware_s:
    seconds from its wire line to its K (nan when unknown); cut_short: drive_timeline.cut_short."""
    leg: drive_timeline.Leg
    start: float
    score: float
    span: tuple = (0, 0)
    measured: float = math.nan
    ratio: float = math.nan
    odo_move: float = math.nan
    pairs: float = math.nan
    firmware_s: float = math.nan
    cut_short: bool = False

    def trusted(self):
        """Fit for the slip ratios: the firmware ran it in full and the odometry saw VO_MIN_PAIRS of it."""
        return not self.cut_short and self.pairs >= VO_MIN_PAIRS


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


def align_legs(run, times, motion, odo_xy, scale=1.0, ready_wait=None, turn_scale=1.0):
    """A LegFit per moving leg, starts in wall time. A leg's span runs from its start to the next leg's
    (back-to-back legs blend: the robot is still slowing when the next one starts), the last one's to
    its commanded end plus ALIGN_TAIL_S; spans never overlap, so their motion sums to the run's.
    scale: real metres per odometry metre (vo_scale); turn_scale: real turn per odometry turn
    (vo_turn_scale); ready_wait: drive_timeline.ready_wait_s, None leaves the firmware timing unchecked."""
    dt = np.diff(times, prepend=times[0])
    dt[0] = np.inf
    speed = {drive_timeline.ROTATE: motion[:, COL_DTHETA] * turn_scale / dt, drive_timeline.FORWARD: motion[:, COL_DX] / dt}
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
        turn = fit.leg.kind == drive_timeline.ROTATE
        fit.span = (first, last)
        fit.measured = float(motion[first:last, COL_DTHETA if turn else COL_DX].sum()) * (turn_scale if turn else scale)
        fit.ratio = fit.measured / fit.leg.amount if fit.leg.amount else math.nan
        fit.odo_move = float(np.linalg.norm(odo_xy[max(last - 1, 0)] - odo_xy[max(first - 1, 0)]))
        fit.pairs = measured_share(times, motion, fit.start, end)
        took = drive_timeline.firmware_duration(fit.leg, ready_wait)
        fit.firmware_s = math.nan if took is None else took
        fit.cut_short = drive_timeline.cut_short(fit.leg, ready_wait)
    return fits


def commanded(fits, kind, times):
    """Commanded displacement of every leg of `kind`, summed, at `times`."""
    total = np.zeros(len(times))
    for fit in fits:
        if fit.leg.kind == kind:
            total += drive_timeline.displacement(fit.leg, fit.start, times)
    return total


def forward_fill(fits, motion, cmd_forward, scale):
    """(N,) factor on the commanded forward step of each frame, for the frames whose odometry pair failed:
    real / commanded metres over the measured frames of its own leg; over those of every forward leg the
    firmware did not cut short when its own saw under FILL_MIN_SHARE of its command; else 1."""
    ok = motion[:, COL_OK] > 0
    forwards = [f for f in fits if f.leg.kind == drive_timeline.FORWARD]
    # +1 / -1 along each leg's own direction, so a forward and a backward leg add up instead of cancelling.
    along = np.zeros(len(motion))
    for fit in forwards:
        along[fit.span[0]:fit.span[1]] = math.copysign(1.0, fit.leg.amount)

    def measured_ratio(frames):
        """real / commanded over the measured frames among `frames`, None when they saw too little."""
        cmd_seen = float((cmd_forward * along)[frames & ok].sum())
        if cmd_seen <= 0.0 or cmd_seen < FILL_MIN_SHARE * float((cmd_forward * along)[frames].sum()):
            return None
        return float((motion[:, COL_DX] * along)[frames & ok].sum()) * scale / cmd_seen

    def span_mask(fit):
        mask = np.zeros(len(motion), bool)
        mask[fit.span[0]:fit.span[1]] = True
        return mask

    trusted = np.zeros(len(motion), bool)
    for fit in forwards:
        if not fit.cut_short:
            trusted |= span_mask(fit)
    run_ratio = measured_ratio(trusted) if trusted.any() else None
    fill = np.ones(len(motion))
    for fit in forwards:
        own = measured_ratio(span_mask(fit))
        fill[fit.span[0]:fit.span[1]] = next(r for r in (own, run_ratio, 1.0) if r is not None)
    return fill


def integrate_poses(times, motion, fits, distance_scale, scale=1.0, turn_scale=1.0):
    """(N, 3) x, y, theta of the robot centre per frame, from the odometry: its turn times `turn_scale`
    (vo_turn_scale), and its translation times `scale` (vo_scale) along that heading. A frame whose pair
    failed takes the command: the turn scaled by the median measured turn ratio of the trusted legs
    (LegFit.trusted, fits from align_legs at the same turn_scale), the forward step (commanded metres x
    distance_scale) by forward_fill."""
    turn_ratios = [f.ratio for f in fits if f.leg.kind == drive_timeline.ROTATE and math.isfinite(f.ratio) and f.trusted()]
    turn_fill = float(np.median(turn_ratios)) if turn_ratios else 1.0
    cmd_turn = np.diff(commanded(fits, drive_timeline.ROTATE, times), prepend=0.0)
    cmd_forward = np.diff(commanded(fits, drive_timeline.FORWARD, times), prepend=0.0) * distance_scale
    ok = motion[:, COL_OK] > 0
    dtheta = np.where(ok, motion[:, COL_DTHETA] * turn_scale, cmd_turn * turn_fill)
    dx = np.where(ok, motion[:, COL_DX] * scale, cmd_forward * forward_fill(fits, motion, cmd_forward, scale))
    dy = np.where(ok, motion[:, COL_DY] * scale, 0.0)
    dtheta[0] = dx[0] = dy[0] = 0.0
    theta = np.cumsum(dtheta)
    mid = theta - 0.5 * dtheta
    return np.column_stack((np.cumsum(dx * np.cos(mid) - dy * np.sin(mid)),
                            np.cumsum(dx * np.sin(mid) + dy * np.cos(mid)), theta))


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


def object_colors(dm):
    """object_color.ColorVotes on dm.occ_map's cells: the obstacle pixels above the contacts of every
    usable floor analysis, their colors from the raw frame of the same seq."""
    votes = ColorVotes(dm.occ_map)
    files = {fr["seq"]: fr["file"] for fr in dm.frames}
    for line in floor_lines(dm.run_dir):
        seq = line["seq"]
        if seq not in files or not floor_usable(line, dm.pose_of, dm.rate_of):
            continue
        image = cv2.imread(os.path.join(dm.run_dir, RAW_DIR, files[seq]))
        if image is None:
            logger.warning("raw frame %s unreadable: no colors from it", files[seq])
            continue
        labels = floor_labels(dm.run_dir, line)
        if labels is not None:
            votes.add_view(labels, line["contacts"], image, dm.pose_of[seq], dm.intrinsics, dm.mount,
                           OBJECT_MAX_RANGE_M, line.get("boxes", ()))
    return votes


def box_view(box, pose, intrinsics, mount):
    """BoxView of a box whose foot (the middle of its bottom edge) meets the floor."""
    x1, _, x2, y2 = box
    columns = [x1 - MERGE_MARGIN_PX, x1, x2, x2 + MERGE_MARGIN_PX]
    fwd, side, _ = floor_hit(columns, [y2] * len(columns), intrinsics, mount)
    wide_left, left_edge, right_edge, wide_right = body_to_world(np.column_stack((fwd, side)), pose)
    lens = body_to_world(np.array([[mount.forward_m, mount.left_m]]), pose)[0]
    right, left = (math.atan2(p[1] - lens[1], p[0] - lens[0]) for p in (wide_right, wide_left))
    return BoxView((float(lens[0]), float(lens[1])), right, left, float(math.dist(left_edge, right_edge)))


def row_sightings(row, pose, intrinsics, mount, image_h):
    """(Sighting, (1, 2) body-frame foot, box) of every box of one detections.jsonl row whose foot is on
    the floor within OBJECT_MAX_RANGE_M, placed in the world from pose."""
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
        found.append((Sighting(det["class"], float(world[0]), float(world[1]), det["confidence"], row["seq"],
                               box_view(det["box"], pose, intrinsics, mount)), body, det["box"]))
    return found


def support_points(box, occ_map, occupied, pose, intrinsics, mount):
    """(N, 2) body-frame points a box's object lies on when that is not the floor: along the rays under the
    middle SUPPORT_MID_SHARE of its bottom edge, the first occupied cell more than SUPPORT_AHEAD_M before
    the floor point of the ray - the bed under a person lying on it, whose box ends on the mattress, so its
    floor point lands behind the bed. None unless most rays find one (the object stands on the floor)."""
    x1, _, x2, y2 = box
    mid, half = 0.5 * (x1 + x2), 0.5 * SUPPORT_MID_SHARE * (x2 - x1)
    fwd, left, _ = floor_hit(np.linspace(mid - half, mid + half, SUPPORT_RAYS), np.full(SUPPORT_RAYS, y2),
                             intrinsics, mount)
    lens = np.array([mount.forward_m, mount.left_m])
    held = []
    for foot in np.column_stack((fwd, left)):
        reach = float(np.hypot(*(foot - lens)))
        if not math.isfinite(reach):
            continue
        ray = lens + np.outer(np.arange(SUPPORT_STEP_M, reach - SUPPORT_AHEAD_M, SUPPORT_STEP_M), (foot - lens) / reach)
        iy, ix, inside = occ_map.cell_index(body_to_world(ray, pose))
        hits = np.flatnonzero(inside & occupied[iy.clip(0, occ_map.n - 1), ix.clip(0, occ_map.n - 1)])
        if hits.size:
            held.append(ray[hits[0]])
    return np.array(held) if 2 * len(held) > SUPPORT_RAYS else None


def object_sightings(run_dir, pose_of, intrinsics, mount, occ_map, image_h):
    """Sightings of the YOLO boxes whose foot is on the floor within OBJECT_MAX_RANGE_M. Each also votes
    its class into occ_map for scene_export's labels: on what its object lies on when support_points finds
    that, else under its foot."""
    occupied = occ_map.occupied()
    sightings = []
    supported = 0
    for row in load_jsonl(os.path.join(run_dir, DETECTIONS_FILE)):
        pose = pose_of.get(row["seq"])
        if pose is None:
            continue
        for sighting, body, box in row_sightings(row, pose, intrinsics, mount, image_h):
            held = support_points(box, occ_map, occupied, pose, intrinsics, mount)
            if held is None:
                occ_map.add_label_votes(sighting.name, sighting.conf, body, pose)
            else:
                supported += 1
                for point in held:                     # one vote in all, shared by the points
                    occ_map.add_label_votes(sighting.name, sighting.conf / len(held), point[None], pose)
            sightings.append(sighting)
    logger.debug("%d sightings, %d voted on what their object lies on", len(sightings), supported)
    return sightings


def greedy_clusters(sightings):
    """class -> [[members, (median x, median y)], ...]: each sighting joins the first cluster of its class
    within LANDMARK_RADIUS_M of that cluster's median, else starts one. A cluster's median is refreshed
    when it grows, not on every comparison."""
    clusters = {}
    for sighting in sightings:
        for cluster in clusters.setdefault(sighting.name, []):
            members, (cx, cy) = cluster
            if math.hypot(sighting.x - cx, sighting.y - cy) < LANDMARK_RADIUS_M:
                members.append(sighting)
                cluster[1] = (float(np.median([m.x for m in members])), float(np.median([m.y for m in members])))
                break
        else:
            clusters[sighting.name].append([[sighting], (sighting.x, sighting.y)])
    return clusters


def in_view(view, xy):
    """Whether world point xy lies between the edges of a sighting's box, seen from its lens."""
    bearing = math.atan2(xy[1] - view.lens[1], xy[0] - view.lens[0])
    return math.remainder(bearing - view.right, math.tau) >= 0 and math.remainder(view.left - bearing, math.tau) >= 0


def merge_by_box(found, sightings):
    """Groups (lists of indices) of one class's clusters `found` that are one object: some box of the
    class's `sightings` held both centres and they lie no farther apart than it is wide (+ MERGE_SLACK_M).
    A large object seen in parts - a person lying on a bed, feet only from one spot, whole from another,
    head and chest up close - lands its parts in separate clusters, since each part's box ends at a
    different row; two bottles on one line of sight stay apart, as a bottle's box is a few cm wide."""
    parent = list(range(len(found)))

    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for sighting in sightings:
        inside = [i for i, (_, centre) in enumerate(found) if in_view(sighting.view, centre)]
        for a, b in itertools.combinations(inside, 2):
            if math.dist(found[a][1], found[b][1]) <= sighting.view.width_m + MERGE_SLACK_M:
                parent[root(b)] = root(a)
    groups = collections.defaultdict(list)
    for i in range(len(found)):
        groups[root(i)].append(i)
    return list(groups.values())


def cluster_landmarks(sightings):
    """greedy_clusters joined by merge_by_box; those seen at least LANDMARK_MIN_SIGHTINGS times, as
    JSON-ready dicts: the median of every member, range_m from the world origin (the robot centre at the
    first frame), extent_m [x min, y min, x max, y max] of the members, parts = clusters merged."""
    landmarks = []
    for name, found in greedy_clusters(sightings).items():
        for group in merge_by_box(found, [m for members, _ in found for m in members]):
            members = [m for i in group for m in found[i][0]]
            if len(members) < LANDMARK_MIN_SIGHTINGS:
                continue
            xy = np.array([(m.x, m.y) for m in members])
            cx, cy = (float(v) for v in np.median(xy, axis=0))
            if len(group) > 1:
                logger.debug("%s: %d clusters one box held -> one at (%.2f, %.2f), %d sightings", name, len(group),
                             cx, cy, len(members))
            landmarks.append({
                "class": name, "x": round(cx, M_DIGITS), "y": round(cy, M_DIGITS),
                "range_m": round(math.hypot(cx, cy), M_DIGITS), "sightings": len(members),
                "mean_conf": round(float(np.mean([m.conf for m in members])), CONF_DIGITS),
                "first_seq": min(m.seq for m in members), "last_seq": max(m.seq for m in members),
                "extent_m": [round(float(v), M_DIGITS) for v in (*xy.min(axis=0), *xy.max(axis=0))],
                "parts": len(group)})
    return landmarks


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
                    axis, scale=1.0, turn_scale=1.0):
    """alignment.txt: the camera used, the odometry quality and turn axis, and per leg its delay, how well
    the command matched, command vs measurement, and how long the firmware took on it. Returns its lines."""
    ok = motion[1:, COL_OK] > 0
    odometry = (f"odometry: {int(ok.sum())} / {ok.size} frame pairs measured, median "
                f"{np.median(motion[1:, COL_INLIERS][ok]):.0f} inliers, median rms {np.median(motion[1:, COL_RMS][ok]):.2f} px"
                if ok.any() else "odometry: none")
    lines = [f"run {run_id}   fx {intrinsics.fx:.1f}  pitch {mount.pitch_deg:+.2f} deg  height {mount.height_m:.3f} m  "
             f"forward {mount.forward_m:.3f} m  left {mount.left_m:+.3f} m  distance_scale {distance_scale:.3f}  "
             f"vo_scale {scale:.3f}  vo_turn_scale {turn_scale:.4f}",
             odometry, axis_line(axis, mount),
             f"floor analyses used {floor_counts[0]}, skipped {floor_counts[1]} (turning > {MAX_TURN_RATE} rad/s, "
             "no floor plane, or no pose)", "",
             "leg  kind     commanded   status_s  start_s  delay_s  score   measured   ratio  pairs_%  fw_s   plan_s  "
             "odo_move_m",
             "     (deg for ROTATE at vo_turn_scale, m for FORWARD at vo_scale; times from the first leg's status line; "
             "pairs_%: share "
             "of the leg's time measured; fw_s: wire line to K; CUT = acked before "
             f"{drive_timeline.FIRMWARE_SHORT_SHARE:.0%} of plan_s; CUT legs and legs under {VO_MIN_PAIRS:.0%} "
             "pairs are left out of the ratios below)"]
    t0 = next((leg.status_t for leg in run.legs if leg.status_t is not None), 0.0)
    for f in fits:
        turn = f.leg.kind == drive_timeline.ROTATE
        conv, digits = (math.degrees, DEG_DIGITS) if turn else (float, FORWARD_DIGITS)
        lines.append(f"{f.leg.index + 1:<4d} {f.leg.kind:8s} {conv(f.leg.amount):+9.{digits}f}  {f.leg.status_t - t0:8.2f} "
                     f"{f.start - t0:8.2f} {f.start - f.leg.status_t:8.2f}  {f.score:5.3f} {conv(f.measured):+9.{digits}f}"
                     f"  {f.ratio:6.3f}  {PERCENT * f.pairs:6.0f}  {f.firmware_s:5.2f}  {f.leg.duration_s:6.2f}  "
                     f"{f.odo_move:8.3f}{'  CUT' if f.cut_short else ''}")
    turns = [f.ratio for f in fits if f.leg.kind == drive_timeline.ROTATE and f.trusted()]
    if turns:
        lines.append(f"\nturn: measured / commanded = {np.mean(turns):.3f} (legs {', '.join(f'{r:.3f}' for r in turns)}), "
                     f"measured at vo_turn_scale {turn_scale:.4f} (1 = the odometry as it reads, ~10 % long).")
    cut = [f for f in fits if f.cut_short]
    if cut:
        lines.append("\ncut short by the firmware (acked early - not slip, not learned): "
                     + ", ".join(f"leg {f.leg.index + 1} {f.firmware_s:.2f} / {f.leg.duration_s:.2f} s, "
                                 f"ratio {f.ratio:.3f}" for f in cut))
    weak = [f for f in fits if not f.cut_short and not f.trusted()]
    if weak:
        lines.append("\nweak odometry (too little of the leg measured - not learned): "
                     + ", ".join(f"leg {f.leg.index + 1} {PERCENT * f.pairs:.0f} % pairs" for f in weak))
    forwards = [f for f in fits if f.leg.kind == drive_timeline.FORWARD and f.trusted()]
    commanded_m = sum(abs(f.leg.amount) for f in forwards)
    if commanded_m > 0:
        # Length-weighted, backward legs included: the k of the slip model (task 2026-09-27_explore-map).
        # Signed along each command, so a leg that went the wrong way lowers k instead of raising it.
        ratio = sum(f.measured * math.copysign(1.0, f.leg.amount) for f in forwards) / commanded_m
        drift = []
        for f in forwards:
            span = motion[f.span[0]:f.span[1]]
            drift.append(f"{span[:, COL_DY].sum() * scale:+.{FORWARD_DIGITS}f} m / "
                         f"{math.degrees(span[:, COL_DTHETA].sum()):+.{DEG_DIGITS}f} deg / "
                         f"{PERCENT * f.pairs:.0f} % pairs")
        lines.append(f"\nforward: measured / commanded = {ratio:.3f} over {len(forwards)} legs (length-weighted). "
                     f"Per leg sideways / heading drift / frame pairs measured: {', '.join(drift)}. A leg below "
                     "100 % pairs reads short (an unmeasured pair counts as no motion, not as slip). "
                     "Scales with the camera height and pitch.")
    x, y, theta = poses[-1]
    lines.append(f"end pose: x {x:+.3f} y {y:+.3f} m, heading {math.degrees(theta):+.2f} deg; the raw odometry (scale 1, "
                 f"failed pairs as no motion) puts the centre at x {odo_xy[-1][0]:+.3f} y {odo_xy[-1][1]:+.3f} m")
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
            for sighting, _, _ in row_sightings(det_row, det_pose, self.intrinsics, self.mount, self.image_h):
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
    parser.add_argument("--distance-scale", type=float,
                        help="rolled / commanded forward distance, for frames the odometry missed (default: real "
                             "wheel radius / the one the run was commanded with - 0.727 before the 2026-09-27 fix, 1 after)")
    parser.add_argument("--vo-scale", type=float,
                        help="real / odometry metres (default: frame_motion.vo_scale of the run's mount - the tape "
                             "factor in vision/vo_scale.json when the mount matches, else 1)")
    parser.add_argument("--vo-turn-scale", type=float,
                        help="real / odometry turn (default: frame_motion.vo_turn_scale of the run's mount - the "
                             "measured turns in vision/vo_scale.json when the mount matches, else 1)")
    parser.add_argument("--fx", type=float, help="override the recording's fx (fy follows)")
    parser.add_argument("--cam-pitch", type=float, help="override the recording's pitch, deg, + = down")
    parser.add_argument("--cam-forward", type=float, help="override the recording's lens offset ahead of the robot centre, m")
    parser.add_argument("--cam-left", type=float, help="override the recording's lens offset left of the robot centre, m")
    parser.add_argument("--video", action="store_true", help=f"also write {VIDEO_FILE}: camera + detections beside the growing map")
    parser.add_argument("--redo-motion", action="store_true", help=f"recompute {MOTION_FILE}")
    parser.add_argument("--publish", action="store_true",
                        help="make this map the real room's (realroom/maps/latest.json), for later commands")
    return parser.parse_args()


@dataclass
class DriveMap:
    """A run's map and what it was built from (build()): main() writes it out, stream_bench times it.
    rate: rad/s of the fused heading per frame; pose_of / rate_of: the same keyed by seq;
    distance_scale: rolled / commanded metres, vo_scale: real / odometry metres, vo_turn_scale: real /
    odometry turn the poses were built with."""
    run_dir: str
    recording: dict
    intrinsics: Intrinsics
    mount: CameraMount
    run: drive_timeline.Run
    distance_scale: float
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
    vo_scale: float = 1.0
    vo_turn_scale: float = 1.0


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


def build(run_dir, recording, intrinsics, mount, distance_scale=None, redo_motion=False, scale=None, turn_scale=None):
    """The DriveMap of a run; distance_scale None = drive_timeline.distance_scale of the run, scale None =
    vo_scale of the mount, turn_scale None = its vo_turn_scale. FileNotFoundError when it has no raw/
    (recorded before 2026-09-26)."""
    raw_index = os.path.join(run_dir, RAW_DIR, RAW_INDEX)
    if not os.path.exists(raw_index):
        raise FileNotFoundError(f"{run_dir} has no {RAW_DIR}/ (recorded before 2026-09-26): no undrawn frames, "
                                "so no odometry and no floor map")
    out_dir = os.path.join(run_dir, MAP_DIR)
    os.makedirs(out_dir, exist_ok=True)
    run = drive_timeline.load_run(run_dir)
    if distance_scale is None:
        distance_scale = drive_timeline.distance_scale(run)
    if scale is None:
        scale = vo_scale(mount)
    if turn_scale is None:
        turn_scale = vo_turn_scale(mount)
    frames = load_jsonl(raw_index)
    times = np.array([fr["t_capture"] for fr in frames])

    motion = cached_motion(run_dir, out_dir, frames, intrinsics, mount, times, redo_motion)
    _, odo_xy = odometry_track(motion)
    fits = align_legs(run, times, motion, odo_xy, scale, drive_timeline.ready_wait_s(), turn_scale)
    poses = integrate_poses(times, motion, fits, distance_scale, scale, turn_scale)
    dt = np.diff(times, prepend=times[0])
    rate = np.divide(np.diff(poses[:, 2], prepend=0.0), dt, out=np.zeros(len(dt)), where=dt > 0)
    pose_of = {fr["seq"]: tuple(p) for fr, p in zip(frames, poses)}
    rate_of = {fr["seq"]: r for fr, r in zip(frames, rate)}

    occ_map = OccupancyMap()
    floor_counts = build_floor(occ_map, run_dir, pose_of, rate_of, intrinsics, mount)
    landmarks = cluster_landmarks(object_sightings(run_dir, pose_of, intrinsics, mount, occ_map,
                                                   recording["frame_size"][1]))
    return DriveMap(run_dir, recording, intrinsics, mount, run, distance_scale, frames, times, motion, odo_xy, fits,
                    poses, rate, pose_of, rate_of, occ_map, floor_counts, landmarks, scale, turn_scale)


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
        dm = build(run_dir, recording, intrinsics, mount, args.distance_scale, args.redo_motion, args.vo_scale,
                   args.vo_turn_scale)
    except FileNotFoundError as exc:
        raise SystemExit(str(exc)) from exc
    occ_map, poses, fits, landmarks = dm.occ_map, dm.poses, dm.fits, dm.landmarks
    # The scene and the planner's grid also count as free the floor the robot's body covered: the camera
    # never sees under or behind the robot, so without it the robot would stand in unseen floor (outside the
    # scene's box) with no known way back.
    known = occ_map.copy()
    known.integrate_footprint(poses, ROBOT_RADIUS_M)
    regions = extract_regions(known)
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
    scene = to_scene(known, regions, tuple(poses[-1]), object_colors(dm))
    _write_json(os.path.join(out_dir, SCENE_FILE), scene)
    np.save(os.path.join(out_dir, KNOWN_GRID_FILE), known.known_grid())
    _write_json(os.path.join(out_dir, META_FILE), {
        "run": run_id, "mount": mount._asdict(), "intrinsics": intrinsics._asdict(),
        "distance_scale": dm.distance_scale, "vo_scale": dm.vo_scale, "vo_turn_scale": dm.vo_turn_scale,
        "floor_weight": FLOOR_WEIGHT, "res_m": occ_map.res,
        "turn_axis_m": None if axis is None else [round(v, M_DIGITS) for v in axis],
        "origin": list(occ_map.origin), "cells": occ_map.n,
        "world": "robot centre at the first raw frame: x forward, y left, theta CCW",
        "grid": "log-odds float32 [iy, ix]; row iy grows with world y",
        "known_grid": f"{KNOWN_GRID_FILE}: int8 [iy, ix], {KNOWN_OCCUPIED} occupied, {KNOWN_FREE} free (seen, or under "
                      f"the robot's path), {KNOWN_UNSEEN} unseen; same cells as grid; scene = world - map_offset",
        "map_offset": scene["map_offset"]})
    _write_json(os.path.join(out_dir, OBJECTS_FILE), landmarks)
    write_trajectory(os.path.join(out_dir, TRAJECTORY_FILE), dm.frames, dm.times, poses, dm.motion)
    lines = write_alignment(os.path.join(out_dir, ALIGNMENT_FILE), run_id, dm.run, fits, dm.motion, poses, dm.odo_xy,
                            intrinsics, mount, dm.distance_scale, dm.floor_counts, axis, dm.vo_scale, dm.vo_turn_scale)
    print("\n".join(lines))
    print(f"\nlandmarks: {[(lm['class'], lm['x'], lm['y'], lm['sightings']) for lm in landmarks]}")
    print(f"object colors: {[(o['id'], o['class'], o['color'], o['color_rgb'], o['color_share']) for o in scene['objects']]}")
    print(f"map -> {os.path.join(out_dir, MAP_IMAGE)}")
    if args.publish:
        sys.path.insert(0, REALROOM_DIR)
        import map_store                                      # realroom/: the real room's map and robot pose
        published = map_store.publish_run(run_dir)
        print(f"published -> {published} (the real room's map for later commands)")
    if args.video:
        count = MapVideo(dm, window).write(os.path.join(out_dir, VIDEO_FILE))
        print(f"video -> {os.path.join(out_dir, VIDEO_FILE)} ({count} frames, {count / VIDEO_FPS:.1f} s)")
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sys.exit(main())
