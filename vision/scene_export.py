"""Occupancy map -> walls + objects -> scene JSON v2.0 (schema: simulation/room/README.md).

Walls first: straight runs of occupied cells >= WALL_MIN_LENGTH_M (Hough), segments along the same
line merged into one wall. They must go before anything else because walls meet at the corners -
as connected components the whole room outline is ONE ring-shaped blob, which would come out as a
single room-sized "object".
Objects: each 8-connected group of the occupied cells left once the walls are removed. Polygon =
convex hull when that has <= 6 vertices, else the minimum-area rectangle. Both COVER every
occupied cell: an obstacle polygon may come out too big, never too small.
Known limit: a straight edge >= 1 m on a big object (sofa, bed side) can be taken for a wall - unless the
detector keeps naming what stands there (>= WALL_OBJECT_MIN_VOTES): then it is that object's side, and
stays with the objects (a person lying on a bed, whose front the camera saw as a 1.1 m line, 2026-09-29).
An object's class is the detector class holding most of the votes on its cells and on those within
LABEL_REACH_CELLS nearer to it than to any other object (a box's foot lands on the floor just in front of
its object; a vote counts for one object). Without enough
detector evidence it is "unknown" - an object the detector has no class for is still an obstacle
in the map, the label is annotation only. Its color is object_color's vote over its cells (the camera
pixels standing on them), "unknown" without enough of them.
The robot carries the real robot's outline (robot_shape), in the simulator's fields.
The scene frame shifts the map so the box around everything observed starts at (0, 0); the shift
is kept as "map_offset" (scene = map - offset).
"""
import collections
import logging
import math
import os
import sys
import time

import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(BASE_DIR), "simulation", "room"))
from room_generator import (  # noqa: E402  (path inserted above, same pattern as project/src/CM)
    MAX_POLYGON_VERTICES, NEAR_COUNT, ROBOT_CHASSIS_M, ROBOT_FOOTPRINT_BODY, ROBOT_RADIUS_M, ROBOT_WHEEL_ANGLES_DEG,
    ROUND_DIGITS, SCHEMA_VERSION, SIDE_NAMES, polygon_center, robot_wheels_body,
)
from object_color import UNKNOWN_COLOR  # noqa: E402

MIN_REGION_CELLS = 3           # smaller occupied groups are treated as noise
WALL_MIN_LENGTH_M = 1.0
WALL_HOUGH_VOTES = 15          # occupied cells on a candidate line
WALL_MAX_GAP_CELLS = 3         # gaps up to this (15 cm: occlusion, sparse far returns) stay one segment
WALL_MERGE_ANGLE_DEG = 8.0     # segments this parallel ...
WALL_MERGE_DIST_CELLS = 3.0    # ... and this close to one line are the same wall
WALL_MASK_CELLS = 5            # width of the band removed around a wall before objects are extracted
LABEL_MIN_VOTES = 2.0          # summed detector confidence over a region's cells before it takes a class
LABEL_MIN_SHARE = 0.5          # ... and the winning class must hold this share of the region's votes
LABEL_REACH_CELLS = 2          # votes this close to a region count for it (pose / calibration error: 5-10 cm)
REACH_MASK_SIZE = 5            # distance transform mask: exact enough within LABEL_REACH_CELLS
NO_OWNER = -1                  # a cell no object's votes may claim
WALL_OBJECT_MIN_VOTES = 10.0   # a wall line with this much of one class around it is an object's side
SIDE_CLEARANCE_M = 0.05        # gap between an object and the robot disc tested beside it
SIDE_MIN_FREE_FRACTION = 0.5   # share of that disc that must be SEEN free; the rest may be unknown
SOURCE = "astra_map"
UNKNOWN_CLASS = "unknown"      # a region no detector class holds enough votes for
SHARE_DIGITS = 2               # confidence and color_share: shares, to the percent
# The real robot's outline is the simulator's (user drawing, 2026-09-11) turned half a turn: the simulator
# has its wheels at 60/180/300 deg, the real front (+x, where the camera looks) is a wheel - W1 at 0 deg,
# W2 / W3 at +-120 (project_overview; user, 2026-09-29).
REAL_ROBOT_TURN_DEG = 180
FULL_TURN_DEG = 360

# kind: "wall" | "object". polygon: object -> CCW world vertices, wall -> [p1, p2].
# size: (long, short) meters; yaw: direction of the long side, radians in [0, pi).
# confidence: labelled object -> vote share of its class; otherwise mean occupancy probability.
Region = collections.namedtuple("Region", "kind mask center size yaw polygon label confidence")

logger = logging.getLogger(__name__)


def _ccw(poly):
    area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))
    return poly if area > 0 else poly[::-1]


def _cell_corners(occ_map, mask):
    iy, ix = np.nonzero(mask)
    x0 = occ_map.origin[0] + ix * occ_map.res
    y0 = occ_map.origin[1] + iy * occ_map.res
    r = occ_map.res
    return np.concatenate([np.column_stack((x0 + dx, y0 + dy)) for dx in (0, r) for dy in (0, r)]).astype(np.float32)


def _rect_axes(rect):
    """(long, short, yaw of the long side, corners) - independent of OpenCV's minAreaRect angle
    convention, which changed between releases."""
    box = cv2.boxPoints(rect)
    e0, e1 = box[1] - box[0], box[2] - box[1]
    l0, l1 = float(np.hypot(*e0)), float(np.hypot(*e1))
    edge = e0 if l0 >= l1 else e1
    return max(l0, l1), min(l0, l1), math.atan2(float(edge[1]), float(edge[0])) % math.pi, box


def _near_cells(mask):
    """The cells within LABEL_REACH_CELLS of `mask` - the same distance _vote_areas measures."""
    return cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, REACH_MASK_SIZE) <= LABEL_REACH_CELLS


def _vote_areas(masks):
    """Per object mask, the cells whose votes count for it: its own, and those within LABEL_REACH_CELLS
    that lie nearer to it than to any other object - a vote between two objects counts for one of them."""
    if not masks:
        return []
    owner = np.full(masks[0].shape, NO_OWNER, np.int32)
    for k, mask in enumerate(masks):
        owner[mask] = k
    owned = owner != NO_OWNER
    dist, nearest_px = cv2.distanceTransformWithLabels((~owned).astype(np.uint8), cv2.DIST_L2, REACH_MASK_SIZE,
                                                       labelType=cv2.DIST_LABEL_PIXEL)
    lut = np.full(int(nearest_px.max()) + 1, NO_OWNER, np.int32)     # label of an owned cell -> its object
    lut[nearest_px[owned]] = owner[owned]
    nearest = np.where(dist <= LABEL_REACH_CELLS, lut[nearest_px], NO_OWNER)
    return [nearest == k for k in range(len(masks))]


def _label(occ_map, votes_at, mask, prob, min_votes=LABEL_MIN_VOTES):
    """(class, vote share) of the region `mask` from the votes on the cells `votes_at`, or
    (UNKNOWN_CLASS, its mean occupancy probability) when no class holds min_votes and LABEL_MIN_SHARE."""
    totals = {name: float(grid[votes_at].sum()) for name, grid in occ_map.votes.items()}
    total = sum(totals.values())
    if total > 0:
        name = max(totals, key=totals.get)
        if totals[name] >= min_votes and totals[name] >= LABEL_MIN_SHARE * total:
            return name, totals[name] / total
    return UNKNOWN_CLASS, float(prob[mask].mean()) if mask.any() else 0.0


def _merge_segments(segments):
    """Hough segments (x1, y1, x2, y2 in cells) -> one segment per wall line. Longest first: each
    segment joins the first group whose line it lies on, and a group spans all its endpoints."""
    groups = []   # [angle, unit direction, unit normal, offset along the normal, endpoints]
    max_angle = math.radians(WALL_MERGE_ANGLE_DEG)
    for x1, y1, x2, y2 in sorted(segments, key=lambda s: -math.hypot(s[2] - s[0], s[3] - s[1])):
        angle = math.atan2(y2 - y1, x2 - x1) % math.pi
        for g in groups:
            diff = abs(angle - g[0])
            if (min(diff, math.pi - diff) < max_angle
                    and all(abs(g[2][0] * x + g[2][1] * y - g[3]) < WALL_MERGE_DIST_CELLS for x, y in ((x1, y1), (x2, y2)))):
                g[4] += [(x1, y1), (x2, y2)]
                break
        else:
            u, n = (math.cos(angle), math.sin(angle)), (-math.sin(angle), math.cos(angle))
            groups.append([angle, u, n, n[0] * x1 + n[1] * y1, [(x1, y1), (x2, y2)]])
    merged = []
    for _, u, n, offset, pts in groups:
        ts = [u[0] * x + u[1] * y for x, y in pts]
        base = (n[0] * offset, n[1] * offset)
        merged.append(tuple((base[0] + t * u[0], base[1] + t * u[1]) for t in (min(ts), max(ts))))
    return merged


def _object_region(occ_map, mask, votes_at, prob):
    """The object Region of the occupied cells `mask`, labelled by the votes on `votes_at`."""
    corners = _cell_corners(occ_map, mask)
    long, short, yaw, box = _rect_axes(cv2.minAreaRect(corners))
    hull = cv2.convexHull(corners)[:, 0, :]
    poly = _ccw([(float(x), float(y)) for x, y in (hull if len(hull) <= MAX_POLYGON_VERTICES else box)])
    label, confidence = _label(occ_map, votes_at, mask, prob)
    return Region("object", mask, polygon_center(poly), (long, short), yaw, poly, label, confidence)


def _walls(occ_map, occupied, prob):
    """(walls, sides, band): the wall regions, the masks of the lines that are an object's side, and the band
    of cells both claim. A line the detector names an object (WALL_OBJECT_MIN_VOTES) is no wall but that
    object's side: an object of the line's own cells - not joined to what stands in front of it (a bottle
    before the bed a person lies on)."""
    min_cells = int(round(WALL_MIN_LENGTH_M / occ_map.res))
    lines = cv2.HoughLinesP(occupied.astype(np.uint8) * 255, 1, np.pi / 180, WALL_HOUGH_VOTES,
                            minLineLength=min_cells, maxLineGap=WALL_MAX_GAP_CELLS)
    segments = [] if lines is None else [tuple(float(v) for v in line[0]) for line in lines]
    band = np.zeros(occupied.shape, np.uint8)
    walls, sides = [], []
    for (c1, r1), (c2, r2) in _merge_segments(segments):
        line = np.zeros(occupied.shape, np.uint8)
        cv2.line(line, (int(round(c1)), int(round(r1))), (int(round(c2)), int(round(r2))), 1, WALL_MASK_CELLS)
        mask = occupied & (line > 0)
        p1, p2 = occ_map.cell_to_world(c1, r1), occ_map.cell_to_world(c2, r2)
        band |= line
        name, _ = _label(occ_map, _near_cells(mask), mask, prob, WALL_OBJECT_MIN_VOTES)
        if name != UNKNOWN_CLASS:
            logger.debug("line %s-%s is a %s's side, not a wall", p1, p2, name)
            sides.append(mask)
            continue
        center = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)
        walls.append(Region("wall", mask, center, (math.dist(p1, p2), WALL_MASK_CELLS * occ_map.res),
                            math.atan2(p2[1] - p1[1], p2[0] - p1[0]) % math.pi, [p1, p2], "wall",
                            float(prob[mask].mean()) if mask.any() else 0.0))
    return walls, sides, band > 0


def extract_regions(occ_map):
    """The wall and object Regions of occ_map: walls first, then the objects - the lines that are an object's
    side and the groups of occupied cells outside the walls' band - labelled over their _vote_areas."""
    occupied = occ_map.occupied()
    prob = occ_map.probability()
    walls, object_masks, wall_band = _walls(occ_map, occupied, prob)
    sides = len(object_masks)
    count, labels = cv2.connectedComponents((occupied & ~wall_band).astype(np.uint8), connectivity=8)
    for k in range(1, count):
        mask = labels == k
        if mask.sum() >= MIN_REGION_CELLS:
            object_masks.append(mask)
    logger.debug("%d walls, %d objects (%d of them an object's side)", len(walls), len(object_masks), sides)
    return walls + [_object_region(occ_map, mask, votes_at, prob)
                    for mask, votes_at in zip(object_masks, _vote_areas(object_masks))]


def _disc_is_free(occ_map, x, y, occupied, free):
    r = ROBOT_RADIUS_M
    ix0, iy0 = occ_map.world_to_cell(x - r, y - r)
    ix1, iy1 = occ_map.world_to_cell(x + r, y + r)
    if ix0 < 0 or iy0 < 0 or ix1 >= occ_map.n or iy1 >= occ_map.n:
        return False
    iy, ix = np.mgrid[iy0:iy1 + 1, ix0:ix1 + 1]
    cx, cy = occ_map.cell_to_world(ix, iy)
    inside = (cx - x) ** 2 + (cy - y) ** 2 <= r * r
    if occupied[iy, ix][inside].any():
        return False
    return free[iy, ix][inside].mean() >= SIDE_MIN_FREE_FRACTION


def free_sides(occ_map, region, occupied, free):
    """Sides (object frame +x, +y, -x, -y = SIDE_NAMES) where a robot disc fits next to the object:
    no occupied cell under it and enough of it seen free."""
    long, short = region.size
    sides = []
    for i, side in enumerate(SIDE_NAMES):   # same angle rule as project/src/CM candidates.side_normal
        angle = region.yaw + i * math.pi / 2
        dist = (long if i % 2 == 0 else short) / 2 + ROBOT_RADIUS_M + SIDE_CLEARANCE_M
        if _disc_is_free(occ_map, region.center[0] + dist * math.cos(angle),
                         region.center[1] + dist * math.sin(angle), occupied, free):
            sides.append(side)
    return sides


def robot_shape():
    """The real robot's Scene JSON v2.0 outline, body frame (+x forward): {chassis, wheels [{angle_deg,
    polygon}] from W1 at 0 deg, footprint (convex hull of both)}."""
    c, s = math.cos(math.radians(REAL_ROBOT_TURN_DEG)), math.sin(math.radians(REAL_ROBOT_TURN_DEG))

    def pt(p):
        return {"x": round(c * p[0] - s * p[1], ROUND_DIGITS), "y": round(s * p[0] + c * p[1], ROUND_DIGITS)}

    wheels = sorted(((deg + REAL_ROBOT_TURN_DEG) % FULL_TURN_DEG, poly)
                    for deg, poly in zip(ROBOT_WHEEL_ANGLES_DEG, robot_wheels_body()))
    return {"chassis": [pt(p) for p in ROBOT_CHASSIS_M],
            "wheels": [{"angle_deg": deg, "polygon": [pt(p) for p in poly]} for deg, poly in wheels],
            "footprint": [pt(p) for p in ROBOT_FOOTPRINT_BODY]}


def to_scene(occ_map, regions, robot_pose, colors=None):
    """Scene JSON v2.0 of the regions. colors: object_color.ColorVotes on occ_map's cells (None: every
    object's color unknown)."""
    occupied, free = occ_map.occupied(), occ_map.free()
    iy, ix = np.nonzero(occupied | free)
    if ix.size:
        x_min, y_min = occ_map.origin[0] + ix.min() * occ_map.res, occ_map.origin[1] + iy.min() * occ_map.res
        width, length = (ix.max() + 1 - ix.min()) * occ_map.res, (iy.max() + 1 - iy.min()) * occ_map.res
    else:
        x_min = y_min = width = length = 0.0

    def pt(p):
        return {"x": round(p[0] - x_min, ROUND_DIGITS), "y": round(p[1] - y_min, ROUND_DIGITS)}

    walls = [r for r in regions if r.kind == "wall"]
    objects = sorted((r for r in regions if r.kind == "object"), key=lambda r: (r.center[0], r.center[1]))
    ids = [f"obj_{k + 1}" for k in range(len(objects))]
    out_objects = []
    for k, r in enumerate(objects):
        others = sorted((math.dist(r.center, o.center), ids[j]) for j, o in enumerate(objects) if j != k)
        color = colors.of(r.mask) if colors is not None else None
        out_objects.append({
            "id": ids[k], "class": r.label, "confidence": round(r.confidence, SHARE_DIGITS),
            "color": color.name if color else UNKNOWN_COLOR, "color_rgb": color.rgb if color else None,
            "color_share": round(color.share, SHARE_DIGITS) if color and color.rgb else None,
            "center": pt(r.center), "yaw": round(r.yaw, ROUND_DIGITS),
            "polygon": [pt(p) for p in r.polygon],
            "free_sides": free_sides(occ_map, r, occupied, free),
            "near": [oid for _, oid in others[:NEAR_COUNT]],
        })
    x, y, theta = robot_pose
    return {
        "version": SCHEMA_VERSION, "seed": None, "units": "m", "frame": "world", "source": SOURCE,
        "timestamp": round(time.time(), 1),
        "room": {"width": round(width, ROUND_DIGITS), "length": round(length, ROUND_DIGITS)},
        "walls": [{"id": f"wall_{k + 1}", "name": f"wall_{k + 1}", "p1": pt(w.polygon[0]), "p2": pt(w.polygon[1])}
                  for k, w in enumerate(walls)],
        "doors": [],
        "robot": {"x": round(x - x_min, ROUND_DIGITS), "y": round(y - y_min, ROUND_DIGITS),
                  "theta": round(theta % (2 * math.pi), ROUND_DIGITS), "radius": round(ROBOT_RADIUS_M, ROUND_DIGITS),
                  **robot_shape()},
        "objects": out_objects,
        "map_offset": {"x": round(x_min, ROUND_DIGITS), "y": round(y_min, ROUND_DIGITS)},
    }
