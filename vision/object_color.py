"""Basic color of the objects of a drive map, from the camera pixels standing on each one.

A floor analysis (floor/<seq>.png labels + the contacts in floor/index.jsonl) knows, for every image column
that meets an obstacle, its lowest obstacle row: the object's foot, and so the map cell the object stands
in (floor_hit). The obstacle pixels straight above that foot are the object's face. Their colors - from the
raw frame of the same seq - vote for the foot's cell, each named with one of the 11 basic color terms (a
superset of simulation/room's palette).
The obstacle labels only reach as high as the floor trapezoid, and floor_segment fills a detector box's
part of it whole: above a box's foot that is the floor, cables and shadow between a chair's legs. So a foot
at the bottom of a detector box, in the middle BOX_CORE_SHARE of its width, takes its face up to the box's
top - the seat and the back.
An object's color (scene_export): walls, floor and shadows have no hue, so a hue named on at least
MIN_HUE_SHARE of the pixels over the object's cells is its color; otherwise the name holding most of them,
if it holds MIN_SHARE. rgb is the median of that name's pixels. Too few pixels: "unknown".
Known limit: the camera's white balance and the room light tint every pixel - a name, not a paint match.
"""
import collections
import math

import cv2
import numpy as np

from floor_segment import floor_hit
from occupancy_map import PIXEL_OBSTACLE, body_to_world

UNKNOWN_COLOR = "unknown"
COLOR_NAMES = ("black", "white", "gray", "red", "orange", "yellow", "green", "blue", "purple", "pink", "brown")
BLACK, WHITE, GRAY, RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE, PINK, BROWN = range(len(COLOR_NAMES))
FIRST_HUE = RED           # names from here on have a hue; black, white and gray do not
# Hue bands: each name up to its upper bound in degrees (past 335 wraps back to red).
HUE_BANDS = ((15, RED), (40, ORANGE), (70, YELLOW), (165, GREEN), (255, BLUE), (290, PURPLE), (335, PINK), (360, RED))
HUE_DEG_PER_UNIT = 2      # OpenCV keeps hue as degrees / 2; saturation and value are 0..255
BLACK_MAX_V = 55          # darker than this is black whatever the hue
GRAY_MAX_S = 45           # less saturated than this has no hue: gray, or white when bright
WHITE_MIN_V = 190
BROWN_HUE_DEG = (10, 50)  # orange-yellow hues ...
BROWN_MAX_V = 170         # ... this dark are brown
PINK_MAX_S = 130          # a pale ...
PINK_MIN_V = 170          # ... bright red is pink

FACE_ROWS = 60            # obstacle rows taken above a foot outside a detector box at most
FOOT_SKIP_ROWS = 3        # the foot's own rows blend with the floor and the object's shadow
BOX_FOOT_TOL_PX = 2       # a foot this close to a box's bottom edge is that box's foot
BOX_CORE_SHARE = 0.6      # only the feet in the middle of a box climb to its top: its sides are background
PIXEL_STRIDE = 2          # every 2nd column and row: neighbours carry the same color
MIN_PIXELS = 50           # an object with fewer voting pixels stays "unknown"
MIN_HUE_SHARE = 0.25      # a hue on this share of an object's pixels is its color ...
MIN_SHARE = 0.4           # ... else the most common name, if it holds this share
RGB_FORMAT = "#{:02x}{:02x}{:02x}"
NO_CELL = -1

# name: a COLOR_NAMES entry or UNKNOWN_COLOR; rgb: "#rrggbb", median of the winning name's pixels (None
# when unknown); share: the best name's share of the votes; pixels: the votes on the object's cells.
ObjectColor = collections.namedtuple("ObjectColor", "name rgb share pixels")


def color_names(hsv):
    """COLOR_NAMES index of every OpenCV HSV pixel ((N, 3) uint8)."""
    hue = hsv[:, 0].astype(np.int32) * HUE_DEG_PER_UNIT
    sat, val = hsv[:, 1].astype(np.int32), hsv[:, 2].astype(np.int32)
    bounds = np.array([bound for bound, _ in HUE_BANDS])
    names = np.array([name for _, name in HUE_BANDS])[np.searchsorted(bounds, hue, side="right")]
    names[(hue >= BROWN_HUE_DEG[0]) & (hue < BROWN_HUE_DEG[1]) & (val < BROWN_MAX_V)] = BROWN
    names[((names == RED) | (names == PINK)) & (sat < PINK_MAX_S) & (val >= PINK_MIN_V)] = PINK
    grayish = sat < GRAY_MAX_S
    names[grayish] = np.where(val[grayish] >= WHITE_MIN_V, WHITE, GRAY)
    names[val < BLACK_MAX_V] = BLACK
    return names


class ColorVotes:
    """Color votes of camera pixels on the cells of an OccupancyMap's grid - valid for any copy of that
    map (same origin, resolution and size)."""

    def __init__(self, occ_map):
        self.origin, self.res, self.n = occ_map.origin, occ_map.res, occ_map.n
        self._cells, self._names, self._rgb = [], [], []

    def _cell_ids(self, xy):
        """Flat cell id of every world point ((N, 2)), NO_CELL off the grid - aligned with the points."""
        ix = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return np.where(inside, iy * self.n + ix, NO_CELL)

    def add_view(self, labels, contacts, image, pose, intrinsics, mount, max_range_m, boxes=()):
        """One floor analysis seen from pose: labels (occupancy_map.PIXEL_* per pixel), contacts ((N, 2)
        int u, v: the feet), image (BGR, the labels' size), boxes (the detector boxes x1, y1, x2, y2 the
        analysis used). Feet beyond max_range_m (a pixel of error is centimetres there) or off the grid do
        not vote. Returns the number of pixels added."""
        feet = np.asarray(contacts, np.int64).reshape(-1, 2)[::PIXEL_STRIDE]
        forward, left, depth = floor_hit(feet[:, 0], feet[:, 1], intrinsics, mount)
        with np.errstate(invalid="ignore"):
            near = np.isfinite(depth) & (depth <= max_range_m)
        cells = self._cell_ids(body_to_world(np.column_stack((forward[near], left[near])), pose))
        feet = feet[near][cells != NO_CELL]
        cells = cells[cells != NO_CELL]
        solid, top = self._faces(labels, feet, boxes)
        # (K, N): the rows above each foot; the face runs up while every row so far is solid.
        reach = feet[:, 1] - FOOT_SKIP_ROWS - top
        climb = PIXEL_STRIDE * np.arange(max(int(reach.max(initial=0)), 0) // PIXEL_STRIDE + 1)[:, None]
        rows = feet[None, :, 1] - FOOT_SKIP_ROWS - climb
        cols = np.broadcast_to(feet[None, :, 0], rows.shape)
        allowed = (climb <= reach[None, :]) & (rows >= 0)
        face = np.zeros(rows.shape, bool)
        face[allowed] = solid[rows[allowed], cols[allowed]]
        face = np.cumprod(face, axis=0).astype(bool)
        pixels = image[rows[face], cols[face]]
        if not len(pixels):
            return 0
        hsv = cv2.cvtColor(pixels.reshape(-1, 1, 3), cv2.COLOR_BGR2HSV).reshape(-1, 3)
        self._cells.append(np.broadcast_to(cells[None, :], rows.shape)[face])
        self._names.append(color_names(hsv))
        self._rgb.append(pixels[:, ::-1])
        return len(pixels)

    @staticmethod
    def _faces(labels, feet, boxes):
        """(solid, top): solid = the pixels a face may cross (obstacle, or inside a detector box), top = the
        highest row of each foot's face - FACE_ROWS above it, or its box's top edge for a foot at the
        bottom of a box, in the box's core."""
        solid = labels == PIXEL_OBSTACLE
        top = feet[:, 1] - FACE_ROWS
        height, width = labels.shape
        for x1, y1, x2, y2 in boxes:
            row0, row1 = max(int(y1), 0), min(int(math.ceil(y2)), height)
            solid[row0:row1, max(int(x1), 0):min(int(math.ceil(x2)), width)] = True
            margin = 0.5 * (1.0 - BOX_CORE_SHARE) * (x2 - x1)
            core = (feet[:, 0] >= x1 + margin) & (feet[:, 0] < x2 - margin)
            at_box = core & (feet[:, 1] >= row0) & (feet[:, 1] < row1 + BOX_FOOT_TOL_PX)
            top[at_box] = np.minimum(top[at_box], row0)
        return solid, top

    def _merged(self):
        """(cells, names, rgb) of every vote, each merged into one array once."""
        for parts in (self._cells, self._names, self._rgb):
            if len(parts) > 1:
                parts[:] = [np.concatenate(parts)]
        return self._cells[0], self._names[0], self._rgb[0]

    def of(self, mask):
        """ObjectColor of the cells in mask (bool [iy, ix] over the grid)."""
        if not self._cells:
            return ObjectColor(UNKNOWN_COLOR, None, 0.0, 0)
        cells, names, rgb = self._merged()
        mine = mask.reshape(-1)[cells]
        pixels = int(mine.sum())
        if pixels < MIN_PIXELS:
            return ObjectColor(UNKNOWN_COLOR, None, 0.0, pixels)
        counts = np.bincount(names[mine], minlength=len(COLOR_NAMES))
        hue = FIRST_HUE + int(np.argmax(counts[FIRST_HUE:]))
        best = hue if counts[hue] >= MIN_HUE_SHARE * pixels else int(np.argmax(counts))
        share = float(counts[best]) / pixels
        if share < (MIN_HUE_SHARE if best >= FIRST_HUE else MIN_SHARE):
            return ObjectColor(UNKNOWN_COLOR, None, share, pixels)
        red, green, blue = (int(v) for v in np.median(rgb[mine & (names == best)], axis=0))
        return ObjectColor(COLOR_NAMES[best], RGB_FORMAT.format(red, green, blue), share, pixels)
