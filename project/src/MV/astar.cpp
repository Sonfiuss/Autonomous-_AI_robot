#include "MV/astar.h"

#include <cmath>

#include "MV/mv_debug.h"

namespace mv {

namespace {

constexpr float INF_G = 1e30f;

// Neighbour offsets: 4 orthogonal first, then 4 diagonal.
constexpr int DC[cfg::NUM_NEIGHBOURS] = {1, -1, 0, 0, 1, 1, -1, -1};
constexpr int DR[cfg::NUM_NEIGHBOURS] = {0, 0, 1, -1, 1, -1, 1, -1};

}  // namespace

constexpr int AStar::NO_PATH;
constexpr int AStar::TOO_LONG;

AStar::AStar() : heapSize_(0) {}

float AStar::heuristic(const Cell& a, const Cell& b) {
    const float dx = static_cast<float>(std::abs(a.col - b.col));
    const float dy = static_cast<float>(std::abs(a.row - b.row));
    const float lo = dx < dy ? dx : dy;
    return (dx + dy) + (cfg::DIAGONAL_COST - 2.0f) * lo;  // octile distance
}

int AStar::search(const OccupancyGrid& grid, const Cell& start, const Cell& goal, Cell* out, int maxLen) {
    if (out == nullptr || maxLen <= 0 || !grid.isFree(start) || !grid.isFree(goal)) return NO_PATH;
    const int cols = grid.cols();
    const int n = cols * grid.rows();
    for (int i = 0; i < n; ++i) {
        gScore_[i] = INF_G;
        parent_[i] = -1;
        closed_[i] = 0;
        heapPos_[i] = -1;
    }
    heapSize_ = 0;

    const int32_t startId = grid.index(start.col, start.row);
    const int32_t goalId = grid.index(goal.col, goal.row);
    gScore_[startId] = 0.0f;
    heapPush(startId, heuristic(start, goal));

    bool reached = false;
    while (heapSize_ > 0) {
        const int32_t cur = heapPop();
        if (cur == goalId) {
            reached = true;
            break;
        }
        closed_[cur] = 1;
        const Cell c{cur % cols, cur / cols};
        for (int k = 0; k < cfg::NUM_NEIGHBOURS; ++k) {
            const int nc = c.col + DC[k];
            const int nr = c.row + DR[k];
            if (!grid.isFree(nc, nr)) continue;
            const bool diagonal = k >= 4;
            if (diagonal && (!grid.isFree(c.col + DC[k], c.row) || !grid.isFree(c.col, c.row + DR[k]))) continue;
            const int32_t id = grid.index(nc, nr);
            if (closed_[id]) continue;
            const float g = gScore_[cur] + (diagonal ? cfg::DIAGONAL_COST : 1.0f);
            if (g >= gScore_[id]) continue;
            gScore_[id] = g;
            parent_[id] = cur;
            const float f = g + heuristic(Cell{nc, nr}, goal);
            if (heapPos_[id] < 0) heapPush(id, f);
            else heapDecrease(id, f);
        }
    }
    if (!reached) {
        MV_DLOG("[mv] astar: no path (%d,%d)->(%d,%d)\n", start.col, start.row, goal.col, goal.row);
        return NO_PATH;
    }

    int len = 0;
    for (int32_t id = goalId; id >= 0; id = parent_[id]) ++len;
    if (len > maxLen) return TOO_LONG;
    int i = len - 1;
    for (int32_t id = goalId; id >= 0; id = parent_[id], --i) out[i] = Cell{id % cols, id / cols};
    return len;
}

// ---------------------------------------------------------------- binary heap
void AStar::heapSwap(int a, int b) {
    const int32_t na = heap_[a];
    const int32_t nb = heap_[b];
    heap_[a] = nb;
    heap_[b] = na;
    const float ka = heapKey_[a];
    heapKey_[a] = heapKey_[b];
    heapKey_[b] = ka;
    heapPos_[nb] = a;
    heapPos_[na] = b;
}

void AStar::siftUp(int slot) {
    while (slot > 0) {
        const int parent = (slot - 1) / 2;
        if (heapKey_[parent] <= heapKey_[slot]) break;
        heapSwap(parent, slot);
        slot = parent;
    }
}

void AStar::siftDown(int slot) {
    for (;;) {
        const int left = 2 * slot + 1;
        const int right = left + 1;
        int smallest = slot;
        if (left < heapSize_ && heapKey_[left] < heapKey_[smallest]) smallest = left;
        if (right < heapSize_ && heapKey_[right] < heapKey_[smallest]) smallest = right;
        if (smallest == slot) break;
        heapSwap(slot, smallest);
        slot = smallest;
    }
}

void AStar::heapPush(int32_t node, float key) {
    const int slot = heapSize_++;
    heap_[slot] = node;
    heapKey_[slot] = key;
    heapPos_[node] = slot;
    siftUp(slot);
}

int32_t AStar::heapPop() {
    const int32_t top = heap_[0];
    heapPos_[top] = -1;
    --heapSize_;
    if (heapSize_ > 0) {
        heap_[0] = heap_[heapSize_];
        heapKey_[0] = heapKey_[heapSize_];
        heapPos_[heap_[0]] = 0;
        siftDown(0);
    }
    return top;
}

void AStar::heapDecrease(int32_t node, float key) {
    const int slot = heapPos_[node];
    if (slot < 0 || key >= heapKey_[slot]) return;
    heapKey_[slot] = key;
    siftUp(slot);
}

}  // namespace mv
