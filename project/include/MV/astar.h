// AStar: 8-connected grid search with octile heuristic.
//
// Diagonal moves are refused when either orthogonal neighbour is blocked, so
// the cell path never cuts an obstacle corner. The open list is a binary heap
// with decrease-key on static arrays (about 1 MB for the maximum grid), so one
// instance is meant to live for the whole program (static inside the API).
#ifndef MV_ASTAR_H
#define MV_ASTAR_H

#include <cstdint>

#include "constants.h"
#include "occupancy_grid.h"
#include "types.h"

namespace mv {

class AStar {
public:
    static constexpr int NO_PATH  = -1;
    static constexpr int TOO_LONG = -2;

    AStar();

    // Searches start -> goal (both must be free cells). Writes the cell path,
    // start first, into out[0..maxLen-1]. Returns the path length in cells,
    // NO_PATH when unreachable, or TOO_LONG when the path exceeds maxLen.
    int search(const OccupancyGrid& grid, const Cell& start, const Cell& goal, Cell* out, int maxLen);

private:
    static float heuristic(const Cell& a, const Cell& b);

    void    heapPush(int32_t node, float key);
    int32_t heapPop();
    void    heapDecrease(int32_t node, float key);
    void    heapSwap(int a, int b);
    void    siftUp(int slot);
    void    siftDown(int slot);

    float   gScore_[cfg::MAX_CELLS];
    int32_t parent_[cfg::MAX_CELLS];
    uint8_t closed_[cfg::MAX_CELLS];
    int32_t heap_[cfg::MAX_CELLS];     // node ids ordered by key
    float   heapKey_[cfg::MAX_CELLS];  // f score per heap slot
    int32_t heapPos_[cfg::MAX_CELLS];  // node id -> heap slot, -1 when not queued
    int     heapSize_;
};

}  // namespace mv

#endif  // MV_ASTAR_H
