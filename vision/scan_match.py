"""Registration of one occupancy map onto another (task 2026-09-29_multi-stop-map-test step 1, the
explore-map plan's step 8).

A map built at one stop of a run holds the same walls and furniture as the map built at another stop,
displaced by the error of the poses it was built from. match() finds the rigid motion (dx, dy, dtheta)
that lays the moving map's occupied cells onto the reference's:

  score = mean over the moving map's occupied cells p' of  field(p') - FREE_PENALTY * free(p')

field: exp(-d^2 / 2 FIELD_SIGMA_M^2), d the distance from p' to the reference's nearest occupied cell (a
likelihood field, read bilinearly so a centimetre step changes the score); free: 1 where the reference saw
free floor - an obstacle laid on floor the reference saw empty is the ghost this module is here to find.
Search: every (dx, dy, dtheta) of a coarse grid over the window, then a fine grid around the best one.
Rotation is about `pivot`, the robot at the moving map's stop, so dx, dy read as the error of that position
and dtheta as the error of its heading.

Trust: a map of one straight wall pins only the offset across it. For each of dx, dy, dtheta the profile is
the best coarse score at each value of it, over the other two; the spread is how far from the best value
that profile stays within MIN_DROP of the best score. A spread over SPREAD_MAX_M / SPREAD_MAX_DEG leaves that
axis unpinned and the match untrusted, as do too few occupied cells, a best score not above 0, or a best
value on the window's edge (the real error may lie beyond it).

Resolution: the cells. Both maps put a wall at the centre of the 5 cm cell it falls in, so an error of under
half a cell can leave both maps with the same cells: expect 2-3 cm (3 cm seen in test_map_check), and the
angle a cell subtends at the map's extent (0.7 deg at 2 m).

Maps are occupancy_map.OccupancyMap on the same grid (drive_map builds them all the same size).
"""
import logging
import math
from dataclasses import dataclass

import cv2
import numpy as np

WINDOW_M = 0.3                  # search +- this on dx and dy
WINDOW_DEG = 8.0                # and on dtheta
COARSE_STEP_M, COARSE_STEP_DEG = 0.05, 1.0
FINE_STEP_M, FINE_STEP_DEG = 0.01, 0.2
FIELD_SIGMA_M = 0.05            # one cell: a wall 5 cm off still scores 0.6
FREE_PENALTY = 0.5
MIN_POINTS = 20                 # occupied cells of the moving map
MIN_DROP = 0.1                  # share of the best score a profile must lose to count as off the peak
SPREAD_MAX_M = 0.1
SPREAD_MAX_DEG = 3.0
NO_SPREAD = (math.nan,) * 3     # dx, dy, dtheta: not measured

logger = logging.getLogger(__name__)


@dataclass
class Match:
    """The motion that lays the moving map onto the reference: rotate by dtheta about pivot, then shift by
    (dx, dy); m and rad. score / base: the best and the median coarse score; points: occupied cells of
    the moving map; spread: (m, m, rad) of the profiles (module docstring); trusted, and why not."""
    dx: float
    dy: float
    dtheta: float
    pivot: tuple
    score: float
    base: float
    points: int
    spread: tuple
    trusted: bool
    why: str = ""


def cell_points(occ_map, mask):
    """(N, 2) world xy of the centres of the cells of occ_map where mask ([iy, ix] bool) is set."""
    iy, ix = np.nonzero(mask)
    return np.column_stack((occ_map.origin[0] + (ix + 0.5) * occ_map.res, occ_map.origin[1] + (iy + 0.5) * occ_map.res))


def occupied_points(occ_map):
    """(N, 2) world xy of the centres of the occupied cells of occ_map."""
    return cell_points(occ_map, occ_map.occupied())


def distance_field(occ_map):
    """float32 [iy, ix]: metres from every cell centre to the nearest occupied cell centre of occ_map
    (inf everywhere when it has none)."""
    occupied = occ_map.occupied()
    if not occupied.any():
        return np.full(occupied.shape, np.inf, np.float32)
    source = np.where(occupied, 0, 255).astype(np.uint8)
    return cv2.distanceTransform(source, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * np.float32(occ_map.res)


def sample(grid, occ_map, xy, outside=0.0):
    """grid (on occ_map's cells) at world points xy (..., 2), bilinear between cell centres; `outside` off
    the grid."""
    gx = (xy[..., 0] - occ_map.origin[0]) / occ_map.res - 0.5
    gy = (xy[..., 1] - occ_map.origin[1]) / occ_map.res - 0.5
    ix, iy = np.floor(gx).astype(np.int64), np.floor(gy).astype(np.int64)
    fx, fy = gx - ix, gy - iy
    n = grid.shape[0]
    inside = (ix >= 0) & (ix < n - 1) & (iy >= 0) & (iy < n - 1)
    ix, iy = np.where(inside, ix, 0), np.where(inside, iy, 0)
    value = ((1 - fx) * (1 - fy) * grid[iy, ix] + fx * (1 - fy) * grid[iy, ix + 1]
             + (1 - fx) * fy * grid[iy + 1, ix] + fx * fy * grid[iy + 1, ix + 1])
    return np.where(inside, value, outside)


def transform(xy, dx, dy, dtheta, pivot):
    """World points xy (N, 2) rotated by dtheta about pivot, then shifted by (dx, dy)."""
    c, s = math.cos(dtheta), math.sin(dtheta)
    rel = xy - pivot
    return np.column_stack((pivot[0] + c * rel[:, 0] - s * rel[:, 1] + dx, pivot[1] + s * rel[:, 0] + c * rel[:, 1] + dy))


def corrected_pose(pose, found):
    """A robot pose (x, y, theta) of the moving map's frames, moved by the Match `found`."""
    x, y = transform(np.array([pose[:2]], np.float64), found.dx, found.dy, found.dtheta, found.pivot)[0]
    return float(x), float(y), pose[2] + found.dtheta


class Scorer:
    """The reference map's likelihood field and free floor, scoring candidate motions of a point set."""

    def __init__(self, ref):
        self.ref = ref
        # A reference without occupied cells has distance inf everywhere: exp(-inf) = 0, no field.
        self.field = np.exp(-distance_field(ref) ** 2 / (2.0 * FIELD_SIGMA_M ** 2)).astype(np.float32)
        self.free = ref.free().astype(np.float32)

    def scores(self, points, pivot, thetas, shifts_x, shifts_y):
        """float [len(thetas), len(shifts_x), len(shifts_y)] of the score of every motion of the grid."""
        out = np.empty((len(thetas), len(shifts_x), len(shifts_y)))
        offsets = np.stack(np.meshgrid(shifts_x, shifts_y, indexing="ij"), axis=-1)       # (X, Y, 2)
        for i, theta in enumerate(thetas):
            turned = transform(points, 0.0, 0.0, theta, pivot)                              # (N, 2)
            moved = turned[None, None, :, :] + offsets[:, :, None, :]                       # (X, Y, N, 2)
            value = sample(self.field, self.ref, moved) - FREE_PENALTY * sample(self.free, self.ref, moved)
            out[i] = value.mean(axis=-1)
        return out


def _spread(profile, values, best_value, best):
    """How far from best_value the profile stays within MIN_DROP of the best score (best > 0)."""
    return float(np.max(np.abs(values[profile > (1.0 - MIN_DROP) * best] - best_value)))


def _grid(half, step):
    """-half .. +half in steps of step, 0 included."""
    count = int(round(half / step))
    return np.arange(-count, count + 1) * step


def match(ref, moving, pivot, window_m=WINDOW_M, window_deg=WINDOW_DEG):
    """Match of the occupancy map `moving` onto `ref` (module docstring); pivot: world (x, y)."""
    pivot = (float(pivot[0]), float(pivot[1]))
    points = occupied_points(moving)
    if len(points) < MIN_POINTS or not ref.occupied().any():
        return Match(0.0, 0.0, 0.0, pivot, math.nan, math.nan, len(points), NO_SPREAD, False,
                     f"too few occupied cells (moving {len(points)}, reference {int(ref.occupied().sum())}, "
                     f"need {MIN_POINTS})")
    scorer = Scorer(ref)
    thetas, shifts = np.radians(_grid(window_deg, COARSE_STEP_DEG)), _grid(window_m, COARSE_STEP_M)
    coarse = scorer.scores(points, pivot, thetas, shifts, shifts)
    it, ix, iy = np.unravel_index(int(np.argmax(coarse)), coarse.shape)
    best, base = float(coarse[it, ix, iy]), float(np.median(coarse))
    fine_t = thetas[it] + np.radians(_grid(COARSE_STEP_DEG, FINE_STEP_DEG))
    fine_x, fine_y = shifts[ix] + _grid(COARSE_STEP_M, FINE_STEP_M), shifts[iy] + _grid(COARSE_STEP_M, FINE_STEP_M)
    fine = scorer.scores(points, pivot, fine_t, fine_x, fine_y)
    jt, jx, jy = np.unravel_index(int(np.argmax(fine)), fine.shape)
    found = Match(float(fine_x[jx]), float(fine_y[jy]), float(fine_t[jt]), pivot, float(fine[jt, jx, jy]), base,
                  len(points), NO_SPREAD, False)
    logger.debug("coarse best %.3f at %+.2f %+.2f m %+.1f deg (median %.3f); fine %+.3f %+.3f m %+.2f deg, %d points",
                 best, shifts[ix], shifts[iy], math.degrees(thetas[it]), base, found.dx, found.dy,
                 math.degrees(found.dtheta), len(points))
    if best <= 0.0:
        found.why = "no motion scores above 0: the maps share nothing"
        return found
    found.spread = (_spread(coarse.max(axis=(0, 2)), shifts, shifts[ix], best),
                    _spread(coarse.max(axis=(0, 1)), shifts, shifts[iy], best),
                    _spread(coarse.max(axis=(1, 2)), thetas, thetas[it], best))
    loose = [name for name, value, limit in zip(("dx", "dy", "dtheta"), found.spread,
                                                (SPREAD_MAX_M, SPREAD_MAX_M, math.radians(SPREAD_MAX_DEG)))
             if value > limit]
    edge = [name for name, k, last in (("dtheta", it, len(thetas) - 1), ("dx", ix, len(shifts) - 1),
                                       ("dy", iy, len(shifts) - 1)) if k in (0, last)]
    if loose:
        found.why = "not pinned along " + ", ".join(loose)
    elif edge:
        found.why = "best on the window's edge along " + ", ".join(edge)
    found.trusted = not loose and not edge
    return found
