#include "MV/mv_api.h"

#include "MV/astar.h"
#include "MV/mv_debug.h"
#include "MV/occupancy_grid.h"
#include "MV/path.h"

namespace {

constexpr int MV_VERSION = 1;
// Cell path plus the exact start and goal points.
constexpr int MAX_PATH_POINTS = mv::cfg::MAX_PATH_CELLS + 2;

// One planner instance for the process: the grid and A* arrays are large and
// the API is not re-entrant (documented in mv_api.h / README).
mv::OccupancyGrid g_grid;
mv::AStar         g_astar;
mv::Cell          g_cells[mv::cfg::MAX_PATH_CELLS];
mv::Point         g_path[MAX_PATH_POINTS];
mv::Point         g_waypoints[MAX_PATH_POINTS];

bool validRequest(const MvRequest* req) {
    if (req == nullptr || req->room_w <= 0.0f || req->room_l <= 0.0f || req->robot_radius < 0.0f) return false;
    if (req->n_polys < 0) return false;
    if (req->n_polys > 0 && (req->vertices == nullptr || req->poly_sizes == nullptr)) return false;
    for (int i = 0; i < req->n_polys; ++i) {
        if (req->poly_sizes[i] < 3) return false;
    }
    return true;
}

// Builds the inflated grid from the request. Returns an MvStatus.
int buildGrid(const MvRequest* req) {
    if (!validRequest(req)) return MV_ERR_ARGS;
    const float inflate = req->robot_radius + mv::cfg::SAFETY_MARGIN_M;
    if (!g_grid.reset(req->room_w, req->room_l, inflate)) return MV_ERR_ROOM_TOO_BIG;
    const mv::Point* verts = reinterpret_cast<const mv::Point*>(req->vertices);
    for (int i = 0; i < req->n_polys; ++i) {
        g_grid.addPolygon(verts, req->poly_sizes[i]);
        verts += req->poly_sizes[i];
    }
    return MV_OK;
}

bool validResult(const MvResult* res) {
    return res != nullptr && res->path != nullptr && res->path_cap > 0 && res->waypoints != nullptr &&
           res->wp_cap > 0 && res->prims != nullptr && res->prim_cap > 0;
}

}  // namespace

extern "C" {

int mv_plan(const MvRequest* req, MvResult* res) {
    if (!validResult(res)) return MV_ERR_ARGS;
    res->path_len = 0;
    res->wp_len = 0;
    res->prim_len = 0;
    res->length_m = 0.0f;
    const int gridStatus = buildGrid(req);
    if (gridStatus != MV_OK) return gridStatus;

    const mv::Point start{req->start.x, req->start.y};
    const mv::Point goal{req->goal.x, req->goal.y};
    mv::Cell startCell, goalCell;
    if (!g_grid.nearestFree(start, startCell)) return MV_ERR_START_BLOCKED;
    if (!g_grid.nearestFree(goal, goalCell)) return MV_ERR_GOAL_BLOCKED;
    const mv::Point snappedStart = g_grid.toWorld(startCell);
    const mv::Point snappedGoal = g_grid.toWorld(goalCell);
    res->snapped_start = MvPoint{snappedStart.x, snappedStart.y};
    res->snapped_goal = MvPoint{snappedGoal.x, snappedGoal.y};

    const int cellCount = g_astar.search(g_grid, startCell, goalCell, g_cells, mv::cfg::MAX_PATH_CELLS);
    if (cellCount == mv::AStar::NO_PATH) return MV_ERR_NO_PATH;
    if (cellCount == mv::AStar::TOO_LONG) return MV_ERR_BUFFER;

    // Exact start, the cell centres, exact goal.
    int n = 0;
    g_path[n++] = start;
    for (int i = 0; i < cellCount; ++i) g_path[n++] = g_grid.toWorld(g_cells[i]);
    g_path[n++] = goal;
    MV_DLOG("[mv] cells=%d\n", cellCount);

    res->path_len = n;
    if (n > res->path_cap) return MV_ERR_BUFFER;
    for (int i = 0; i < n; ++i) res->path[i] = MvPoint{g_path[i].x, g_path[i].y};

    const int wpCount = mv::smoothPath(g_grid, g_path, n, g_waypoints, MAX_PATH_POINTS);
    if (wpCount < 0) return MV_ERR_BUFFER;
    res->wp_len = wpCount;
    if (wpCount > res->wp_cap) return MV_ERR_BUFFER;
    for (int i = 0; i < wpCount; ++i) res->waypoints[i] = MvPoint{g_waypoints[i].x, g_waypoints[i].y};

    mv::Primitive* prims = reinterpret_cast<mv::Primitive*>(res->prims);
    const int primCount = mv::toPrimitives(g_waypoints, wpCount, req->start.theta, req->goal.theta,
                                           req->holonomic != 0, prims, res->prim_cap);
    if (primCount < 0) return MV_ERR_BUFFER;
    res->prim_len = primCount;
    res->length_m = mv::pathLength(g_waypoints, wpCount);
    return MV_OK;
}

int mv_grid(const MvRequest* req, unsigned char* out, int cap, int* cols, int* rows, float* res_m) {
    if (out == nullptr || cols == nullptr || rows == nullptr || res_m == nullptr || cap <= 0) return MV_ERR_ARGS;
    const int gridStatus = buildGrid(req);
    if (gridStatus != MV_OK) return gridStatus;
    *cols = g_grid.cols();
    *rows = g_grid.rows();
    *res_m = g_grid.resolution();
    const int n = g_grid.cols() * g_grid.rows();
    if (n > cap) return MV_ERR_BUFFER;
    const uint8_t* data = g_grid.data();
    for (int i = 0; i < n; ++i) out[i] = data[i];
    return MV_OK;
}

const char* mv_status_text(int status) {
    switch (status) {
        case MV_OK: return "ok";
        case MV_ERR_ARGS: return "invalid arguments";
        case MV_ERR_ROOM_TOO_BIG: return "room exceeds grid size";
        case MV_ERR_START_BLOCKED: return "start pose has no free cell nearby";
        case MV_ERR_GOAL_BLOCKED: return "goal pose has no free cell nearby";
        case MV_ERR_NO_PATH: return "no path between start and goal";
        case MV_ERR_BUFFER: return "output buffer too small";
        default: return "unknown status";
    }
}

int mv_version(void) { return MV_VERSION; }

}  // extern "C"
