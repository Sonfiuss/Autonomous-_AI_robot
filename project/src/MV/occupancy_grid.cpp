#include "MV/occupancy_grid.h"

#include <cmath>

#include "MV/mv_debug.h"

namespace mv {

namespace {

constexpr float INF_T = 1e30f;  // "never crosses this axis" for the ray cast

int floorDiv(float v, float res) { return static_cast<int>(std::floor(v / res)); }

int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

float distanceToSegment(const Point& p, const Point& a, const Point& b) {
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float len2 = abx * abx + aby * aby;
    float t = 0.0f;
    if (len2 > 0.0f) {
        t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / len2;
        t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    }
    const float dx = p.x - (a.x + t * abx);
    const float dy = p.y - (a.y + t * aby);
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

OccupancyGrid::OccupancyGrid() : cols_(0), rows_(0), inflate_(0.0f) {
    for (int i = 0; i < cfg::MAX_CELLS; ++i) cells_[i] = 0;
}

bool OccupancyGrid::reset(float roomW, float roomL, float inflateM) {
    const int cols = static_cast<int>(std::ceil(roomW / cfg::GRID_RES_M));
    const int rows = static_cast<int>(std::ceil(roomL / cfg::GRID_RES_M));
    cols_ = 0;
    rows_ = 0;
    if (cols <= 0 || rows <= 0 || cols > cfg::MAX_GRID_COLS || rows > cfg::MAX_GRID_ROWS) {
        MV_DLOG("[mv] grid %dx%d exceeds %dx%d\n", cols, rows, cfg::MAX_GRID_COLS, cfg::MAX_GRID_ROWS);
        return false;
    }
    cols_ = cols;
    rows_ = rows;
    inflate_ = inflateM < 0.0f ? 0.0f : inflateM;
    for (int i = 0; i < cols_ * rows_; ++i) cells_[i] = 0;

    // Wall band: cell centres closer than inflate_ to any of the four walls.
    for (int row = 0; row < rows_; ++row) {
        const float cy = (row + 0.5f) * cfg::GRID_RES_M;
        const bool rowBlocked = cy < inflate_ || cy > roomL - inflate_;
        for (int col = 0; col < cols_; ++col) {
            const float cx = (col + 0.5f) * cfg::GRID_RES_M;
            if (rowBlocked || cx < inflate_ || cx > roomW - inflate_) block(col, row);
        }
    }
    return true;
}

void OccupancyGrid::addPolygon(const Point* pts, int n) {
    if (pts == nullptr || n < 3 || cols_ == 0) return;
    float minX = pts[0].x, maxX = pts[0].x, minY = pts[0].y, maxY = pts[0].y;
    for (int i = 1; i < n; ++i) {
        if (pts[i].x < minX) minX = pts[i].x;
        if (pts[i].x > maxX) maxX = pts[i].x;
        if (pts[i].y < minY) minY = pts[i].y;
        if (pts[i].y > maxY) maxY = pts[i].y;
    }
    const int c0 = clampInt(floorDiv(minX - inflate_, cfg::GRID_RES_M), 0, cols_ - 1);
    const int c1 = clampInt(floorDiv(maxX + inflate_, cfg::GRID_RES_M), 0, cols_ - 1);
    const int r0 = clampInt(floorDiv(minY - inflate_, cfg::GRID_RES_M), 0, rows_ - 1);
    const int r1 = clampInt(floorDiv(maxY + inflate_, cfg::GRID_RES_M), 0, rows_ - 1);
    for (int row = r0; row <= r1; ++row) {
        for (int col = c0; col <= c1; ++col) {
            if (cells_[index(col, row)] != 0) continue;
            const Point c = toWorld(Cell{col, row});
            if (pointInPolygon(c, pts, n) || distanceToEdges(c, pts, n) <= inflate_) block(col, row);
        }
    }
}

Cell OccupancyGrid::toCell(const Point& p) const {
    return Cell{floorDiv(p.x, cfg::GRID_RES_M), floorDiv(p.y, cfg::GRID_RES_M)};
}

Point OccupancyGrid::toWorld(const Cell& c) const {
    return Point{(c.col + 0.5f) * cfg::GRID_RES_M, (c.row + 0.5f) * cfg::GRID_RES_M};
}

bool OccupancyGrid::nearestFree(const Point& p, Cell& out) const {
    const Cell own = toCell(p);
    if (isFree(own)) {
        out = own;
        return true;
    }
    const int   reach = static_cast<int>(std::ceil(cfg::SNAP_RADIUS_M / cfg::GRID_RES_M));
    const float maxD2 = cfg::SNAP_RADIUS_M * cfg::SNAP_RADIUS_M;
    float bestD2 = maxD2;
    bool  found = false;
    for (int row = own.row - reach; row <= own.row + reach; ++row) {
        for (int col = own.col - reach; col <= own.col + reach; ++col) {
            if (!isFree(col, row)) continue;
            const Point c = toWorld(Cell{col, row});
            const float d2 = (c.x - p.x) * (c.x - p.x) + (c.y - p.y) * (c.y - p.y);
            if (d2 <= bestD2) {
                bestD2 = d2;
                out = Cell{col, row};
                found = true;
            }
        }
    }
    return found;
}

// Amanatides & Woo grid traversal. When the ray passes exactly through a cell
// vertex both orthogonal neighbours are tested, so two obstacles touching at a
// corner are never slipped between.
bool OccupancyGrid::lineOfSight(const Point& a, const Point& b) const {
    Cell cur = toCell(a);
    const Cell end = toCell(b);
    if (!isFree(cur) || !isFree(end)) return false;
    const float dx = b.x - a.x;
    const float dy = b.y - a.y;
    const int stepX = dx > 0.0f ? 1 : (dx < 0.0f ? -1 : 0);
    const int stepY = dy > 0.0f ? 1 : (dy < 0.0f ? -1 : 0);
    const float res = cfg::GRID_RES_M;
    float tMaxX = INF_T, tMaxY = INF_T, tDeltaX = INF_T, tDeltaY = INF_T;
    if (stepX != 0) {
        const float edge = (stepX > 0 ? (cur.col + 1) : cur.col) * res;
        tMaxX = (edge - a.x) / dx;
        tDeltaX = res / std::fabs(dx);
    }
    if (stepY != 0) {
        const float edge = (stepY > 0 ? (cur.row + 1) : cur.row) * res;
        tMaxY = (edge - a.y) / dy;
        tDeltaY = res / std::fabs(dy);
    }
    const int maxSteps = 2 * (cols_ + rows_) + 4;
    for (int i = 0; i < maxSteps; ++i) {
        if (cur.col == end.col && cur.row == end.row) return true;
        if (tMaxX > 1.0f && tMaxY > 1.0f) return true;  // segment ends inside the current cell
        if (std::fabs(tMaxX - tMaxY) < cfg::LOS_EPS) {
            if (!isFree(cur.col + stepX, cur.row) || !isFree(cur.col, cur.row + stepY)) return false;
            cur.col += stepX;
            cur.row += stepY;
            tMaxX += tDeltaX;
            tMaxY += tDeltaY;
        } else if (tMaxX < tMaxY) {
            cur.col += stepX;
            tMaxX += tDeltaX;
        } else {
            cur.row += stepY;
            tMaxY += tDeltaY;
        }
        if (!isFree(cur)) return false;
    }
    return false;  // traversal did not converge: treat as blocked
}

bool OccupancyGrid::pointInPolygon(const Point& p, const Point* pts, int n) {
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const Point& a = pts[i];
        const Point& b = pts[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const float xCross = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < xCross) inside = !inside;
        }
    }
    return inside;
}

float OccupancyGrid::distanceToEdges(const Point& p, const Point* pts, int n) {
    float best = distanceToSegment(p, pts[n - 1], pts[0]);
    for (int i = 0; i + 1 < n; ++i) {
        const float d = distanceToSegment(p, pts[i], pts[i + 1]);
        if (d < best) best = d;
    }
    return best;
}

}  // namespace mv
