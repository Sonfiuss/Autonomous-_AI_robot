"""2D log-odds occupancy grid on the floor, built from depth points in the robot body frame.

Evidence, at most one update per cell per depth frame:
- occupied: the cell holds >= MIN_HITS_PER_CELL points between OBSTACLE_MIN_HEIGHT_M and the
  obstacle ceiling (anything the robot could hit);
- free: the FLOOR is seen in the cell (|height| <= FLOOR_BAND_M) and no obstacle point is;
- otherwise no update - the cell stays unknown.
Free space comes only from seeing the floor, never from ray casting. A 2D ray towards a wall
passes over low obstacles and through the camera's < 0.6 m blind zone and would erase both;
floor seen in a cell is direct evidence that nothing stands on it (as in SemExp, Chaplot 2020).

World frame: the map origin is the first capture's pose - x forward, y left, theta CCW - meters.
Grid arrays are indexed [iy, ix]; row iy grows with world y.
"""
import math

import numpy as np

GRID_RES_M = 0.05             # same cell as the MV planner (project/config/constants.h GRID_RES_M)
GRID_SIZE_M = 16.0            # square extent centered on the origin
FLOOR_BAND_M = 0.05           # |height| up to this counts as floor
OBSTACLE_MIN_HEIGHT_M = 0.08  # the gap above the floor band absorbs residual pitch error / floor noise
DEFAULT_OBSTACLE_MAX_M = 0.6  # robot height not measured yet; above this the robot passes under
MIN_HITS_PER_CELL = 2         # obstacle points needed in a cell within one frame (speckle filter)
LOG_HIT = 0.85
LOG_MISS = -0.4
LOG_MIN, LOG_MAX = -2.0, 3.5  # clamped so a cell can still change its mind (people moving, errors)
OCC_THRESHOLD = 1.2           # > one frame's hit: needs repeated evidence
FREE_THRESHOLD = -0.6


PIXEL_NONE, PIXEL_FLOOR, PIXEL_GAP, PIXEL_OBSTACLE, PIXEL_ABOVE, PIXEL_BELOW = 0, 1, 2, 3, 4, 5


def classify_heights(heights, max_obstacle_height=DEFAULT_OBSTACLE_MAX_M):
    """Per-pixel class with the same thresholds integrate() uses: floor / gap (between the floor band
    and the obstacle band, ignored) / obstacle / above the robot (ignored) / below the floor (ignored:
    a wrong mount pitch or a reflection) / none (no depth)."""
    labels = np.full(heights.shape, PIXEL_NONE, np.uint8)
    with np.errstate(invalid="ignore"):   # NaN = no depth, stays PIXEL_NONE
        labels[heights < -FLOOR_BAND_M] = PIXEL_BELOW
        labels[np.abs(heights) <= FLOOR_BAND_M] = PIXEL_FLOOR
        labels[(heights > FLOOR_BAND_M) & (heights <= OBSTACLE_MIN_HEIGHT_M)] = PIXEL_GAP
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

    def integrate(self, points_body, pose):
        """One depth frame: (N, 3) body-frame points seen from `pose` = (x, y, theta).
        Returns (occupied cells, free cells) updated."""
        height = points_body[:, 2]
        xy = body_to_world(points_body[:, :2], pose)
        obstacle = (height > OBSTACLE_MIN_HEIGHT_M) & (height <= self.max_obstacle_height)
        ids, counts = np.unique(self._cell_ids(xy[obstacle]), return_counts=True)
        occupied = ids[counts >= MIN_HITS_PER_CELL]
        free = np.setdiff1d(np.unique(self._cell_ids(xy[np.abs(height) <= FLOOR_BAND_M])), occupied)
        flat = self.log_odds.reshape(-1)
        flat[occupied] += LOG_HIT
        flat[free] += LOG_MISS
        np.clip(self.log_odds, LOG_MIN, LOG_MAX, out=self.log_odds)
        return occupied.size, free.size

    def add_label_votes(self, name, weight, points_body, pose):
        """Detector evidence: the cells under an object's surface points vote for its class."""
        height = points_body[:, 2]
        keep = (height > OBSTACLE_MIN_HEIGHT_M) & (height <= self.max_obstacle_height)
        ids = np.unique(self._cell_ids(body_to_world(points_body[keep, :2], pose)))
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
