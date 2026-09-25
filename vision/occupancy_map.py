"""2D log-odds occupancy grid on the floor, built in the robot body frame from two sources.

Astra depth, every depth frame - OBSTACLES only: a cell holding >= MIN_HITS_PER_CELL points between
OBSTACLE_MIN_HEIGHT_M and the obstacle ceiling (anything the robot could hit) gets LOG_HIT.
The Astra no longer marks anything free. Its "floor" was whatever sat within a few cm of floor height,
and on the real glossy floor that was the foot of every cabinet and a metal tube lying on the tiles,
while the tiles themselves returned no depth at all (task 2026-09-25_vision-da-floor).

Depth Anything floor segmentation (floor_segment.py), once per capture:
- free: floor seen between the robot and the first obstacle of its image column;
- occupied: that first obstacle's contact with the floor (also the bottom of a YOLO box).
Free space still comes only from SEEING floor, never from ray casting: a 2D ray towards a wall passes
over low obstacles and through the camera's < 0.6 m blind zone and would erase both.

World frame: the map origin is the first capture's pose - x forward, y left, theta CCW - meters.
Grid arrays are indexed [iy, ix]; row iy grows with world y.
"""
import math

import numpy as np

GRID_RES_M = 0.05             # same cell as the MV planner (project/config/constants.h GRID_RES_M)
GRID_SIZE_M = 16.0            # square extent centered on the origin
OBSTACLE_MIN_HEIGHT_M = 0.08  # below this, Astra points may be floor noise or residual pitch error
DEFAULT_OBSTACLE_MAX_M = 0.6  # robot height not measured yet; above this the robot passes under
MIN_HITS_PER_CELL = 2         # points needed in a cell within one frame (speckle filter)
LOG_HIT = 0.85                # Astra, per depth frame (3 per capture)
# Depth Anything runs once per capture, on the color frame, so one segmentation carries a capture's
# weight: one capture makes a cell free / occupied, as the Astra's 3 frames do, and one contradicting
# capture pulls it back to unknown.
LOG_HIT_CONTACT = 1.3
LOG_FREE_FLOOR = -0.8
LOG_MIN, LOG_MAX = -2.0, 3.5  # clamped so a cell can still change its mind (people moving, errors)
OCC_THRESHOLD = 1.2           # > one frame's hit: needs repeated evidence
FREE_THRESHOLD = -0.6

# Per-pixel labels shared by floor_segment, the camera overlay (drawing.floor_view) and the map.
PIXEL_NONE, PIXEL_FREE, PIXEL_OBSTACLE, PIXEL_ABOVE, PIXEL_DROP = 0, 1, 2, 3, 4


def classify_heights(heights, max_obstacle_height=DEFAULT_OBSTACLE_MAX_M):
    """Astra pixels by height above the floor, with the thresholds integrate() uses: obstacle / above
    the robot (ignored) / none - no depth, or low enough to be floor (ignored too)."""
    labels = np.full(heights.shape, PIXEL_NONE, np.uint8)
    with np.errstate(invalid="ignore"):   # NaN = no depth, stays PIXEL_NONE
        labels[(heights > OBSTACLE_MIN_HEIGHT_M) & (heights <= max_obstacle_height)] = PIXEL_OBSTACLE
        labels[heights > max_obstacle_height] = PIXEL_ABOVE
    return labels


def body_to_world(xy_body, pose):
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return np.column_stack((x + c * xy_body[:, 0] - s * xy_body[:, 1],
                            y + s * xy_body[:, 0] + c * xy_body[:, 1]))


class OccupancyMap:
    def __init__(self, max_obstacle_height=DEFAULT_OBSTACLE_MAX_M, res=GRID_RES_M, size_m=GRID_SIZE_M):
        self.res = res
        self.n = int(round(size_m / res))
        self.origin = (-size_m / 2.0, -size_m / 2.0)   # world xy of the corner of cell (0, 0)
        self.max_obstacle_height = max_obstacle_height
        self.log_odds = np.zeros((self.n, self.n), np.float32)
        self.votes = {}   # class name -> grid of summed detection confidence

    def _cell_ids(self, xy):
        ix = np.floor((xy[:, 0] - self.origin[0]) / self.res).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.origin[1]) / self.res).astype(np.int64)
        inside = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        return iy[inside] * self.n + ix[inside]

    def cell_to_world(self, ix, iy):
        """World xy of a cell center."""
        return (self.origin[0] + (ix + 0.5) * self.res, self.origin[1] + (iy + 0.5) * self.res)

    def world_to_cell(self, x, y):
        return (int(math.floor((x - self.origin[0]) / self.res)), int(math.floor((y - self.origin[1]) / self.res)))

    def is_obstacle(self, height):
        """Heights (m above the floor) of points the robot could hit."""
        return (height > OBSTACLE_MIN_HEIGHT_M) & (height <= self.max_obstacle_height)

    def integrate(self, points_body, pose):
        """One Astra depth frame: (N, 3) body-frame points seen from `pose` = (x, y, theta).
        Obstacle evidence only. Returns the flat ids of the cells marked occupied."""
        obstacle = self.is_obstacle(points_body[:, 2])
        ids, counts = np.unique(self._cell_ids(body_to_world(points_body[obstacle, :2], pose)), return_counts=True)
        occupied = ids[counts >= MIN_HITS_PER_CELL]
        self.log_odds.reshape(-1)[occupied] += LOG_HIT
        np.clip(self.log_odds, LOG_MIN, LOG_MAX, out=self.log_odds)
        return occupied

    def integrate_floor(self, free_xy, contact_xy, pose, blocked=()):
        """One floor segmentation: (N, 2) body-frame floor points seen free and (M, 2) obstacle contacts.
        A cell holding any contact, or in `blocked` (flat ids the Astra marked in this capture), is never
        marked free. Returns (occupied, free) cell counts."""
        contact_ids, counts = np.unique(self._cell_ids(body_to_world(contact_xy, pose)), return_counts=True)
        occupied = contact_ids[counts >= MIN_HITS_PER_CELL]
        free = np.setdiff1d(np.unique(self._cell_ids(body_to_world(free_xy, pose))),
                            np.union1d(contact_ids, np.asarray(blocked, np.int64)))
        flat = self.log_odds.reshape(-1)
        flat[occupied] += LOG_HIT_CONTACT
        flat[free] += LOG_FREE_FLOOR
        np.clip(self.log_odds, LOG_MIN, LOG_MAX, out=self.log_odds)
        return occupied.size, free.size

    def add_label_votes(self, name, weight, xy_body, pose):
        """Detector evidence: the cells under an object's points (body frame, (N, 2)) vote for its class."""
        ids = np.unique(self._cell_ids(body_to_world(xy_body, pose)))
        if ids.size == 0:
            return
        grid = self.votes.setdefault(name, np.zeros((self.n, self.n), np.float32))
        grid.reshape(-1)[ids] += weight

    def occupied(self):
        return self.log_odds > OCC_THRESHOLD

    def free(self):
        return self.log_odds < FREE_THRESHOLD

    def probability(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))
