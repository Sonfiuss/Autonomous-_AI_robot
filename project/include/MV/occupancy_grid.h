// OccupancyGrid: rectangular room rasterised into cfg::GRID_RES_M cells.
//
// The robot is treated as a point: every obstacle polygon and the room walls
// are inflated by (hull radius + cfg::SAFETY_MARGIN_M). A cell is blocked when
// its centre lies inside a polygon or within the inflation distance of one of
// its edges (exact distance test, no kernel approximation).
// Fixed-size storage, no heap (about 58 KB for the maximum grid).
#ifndef MV_OCCUPANCY_GRID_H
#define MV_OCCUPANCY_GRID_H

#include <cstdint>

#include "constants.h"
#include "types.h"

namespace mv {

class OccupancyGrid {
public:
    OccupancyGrid();

    // Clears the grid for a roomW x roomL room and blocks the wall band.
    // Returns false (grid left empty) when the room exceeds MAX_GRID_COLS/ROWS.
    bool reset(float roomW, float roomL, float inflateM);

    // Blocks every cell within inflateM of the closed polygon pts[0..n-1] (n >= 3).
    void addPolygon(const Point* pts, int n);

    int   cols() const { return cols_; }
    int   rows() const { return rows_; }
    float resolution() const { return cfg::GRID_RES_M; }
    float inflation() const { return inflate_; }
    const uint8_t* data() const { return cells_; }  // row-major, 1 = blocked

    bool inBounds(int col, int row) const { return col >= 0 && row >= 0 && col < cols_ && row < rows_; }
    bool isFree(int col, int row) const { return inBounds(col, row) && cells_[index(col, row)] == 0; }
    bool isFree(const Cell& c) const { return isFree(c.col, c.row); }
    int  index(int col, int row) const { return row * cols_ + col; }

    Cell  toCell(const Point& p) const;   // world -> cell (may be out of bounds)
    Point toWorld(const Cell& c) const;   // cell -> centre in world

    // Nearest free cell to p within cfg::SNAP_RADIUS_M (Euclidean on cell
    // centres). Returns false when none exists.
    bool nearestFree(const Point& p, Cell& out) const;

    // True when the segment a->b crosses only free cells (exact grid ray cast;
    // a crossing through a cell vertex checks both adjacent cells).
    bool lineOfSight(const Point& a, const Point& b) const;

private:
    static bool  pointInPolygon(const Point& p, const Point* pts, int n);
    static float distanceToEdges(const Point& p, const Point* pts, int n);
    void         block(int col, int row) { cells_[index(col, row)] = 1; }

    uint8_t cells_[cfg::MAX_CELLS];
    int     cols_;
    int     rows_;
    float   inflate_;
};

}  // namespace mv

#endif  // MV_OCCUPANCY_GRID_H
