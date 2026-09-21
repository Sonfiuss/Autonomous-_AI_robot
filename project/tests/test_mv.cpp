// MV unit tests: plain asserts, no framework, so they build anywhere:
//   g++ -std=c++14 -Iinclude -Iconfig src/MV/*.cpp tests/test_mv.cpp -o test_mv && ./test_mv
#include <cmath>
#include <cstdio>

#include "MV/mv_api.h"
#include "MV/path.h"

namespace {

constexpr float PI = 3.14159265358979f;
constexpr float EPS_POSE_M = 0.02f;              // replayed primitives vs goal position
constexpr float EPS_POSE_RAD = 1.0f * PI / 180.0f;
constexpr float EPS_TIGHT = 1e-3f;
constexpr float ROBOT_RADIUS = 0.225f;           // simulator hull radius
constexpr int   CAP = 4098;
constexpr int   GRID_CAP = 240 * 240;

MvPoint       g_path[CAP];
MvPoint       g_waypoints[CAP];
MvPrimitive   g_prims[2 * CAP + 2];
unsigned char g_grid[GRID_CAP];

int g_failures = 0;

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float eps, const char* what) {
    if (std::fabs(actual - expected) > eps) {
        std::printf("  FAIL %s: got %.4f expected %.4f (eps %.1e)\n", what, actual, expected, eps);
        ++g_failures;
    }
}

MvResult makeResult() {
    MvResult res{};
    res.path = g_path;
    res.path_cap = CAP;
    res.waypoints = g_waypoints;
    res.wp_cap = CAP;
    res.prims = g_prims;
    res.prim_cap = 2 * CAP + 2;
    return res;
}

MvRequest makeRequest(float w, float l, const MvPoint* verts, const int* sizes, int nPolys,
                      MvPose start, MvPose goal, int holonomic = 0) {
    MvRequest req{};
    req.room_w = w;
    req.room_l = l;
    req.robot_radius = ROBOT_RADIUS;
    req.vertices = verts;
    req.poly_sizes = sizes;
    req.n_polys = nPolys;
    req.start = start;
    req.goal = goal;
    req.holonomic = holonomic;
    return req;
}

// Integrates the primitives from start and returns the final pose.
MvPose replay(const MvPose& start, const MvPrimitive* prims, int n) {
    MvPose p = start;
    for (int i = 0; i < n; ++i) {
        switch (prims[i].type) {
            case 0: p.theta += prims[i].a; break;
            case 1: p.x += prims[i].a * std::cos(p.theta); p.y += prims[i].a * std::sin(p.theta); break;
            case 2: p.x += prims[i].a; p.y += prims[i].b; break;
            default: break;
        }
    }
    return p;
}

void expectReplayReachesGoal(const MvRequest& req, const MvResult& res, const char* what) {
    const MvPose end = replay(req.start, res.prims, res.prim_len);
    expectNear(end.x, req.goal.x, EPS_POSE_M, what);
    expectNear(end.y, req.goal.y, EPS_POSE_M, what);
    expectNear(mv::wrapAngle(end.theta - req.goal.theta), 0.0f, EPS_POSE_RAD, what);
    expectTrue(res.prim_len > 0 && res.prims[res.prim_len - 1].type == 3, "last primitive is STOP");
}

// Every path point except the exact start/goal must sit on a free grid cell.
void expectPathCellsFree(const MvRequest& req, const MvResult& res, const char* what) {
    int cols = 0, rows = 0;
    float resM = 0.0f;
    expectTrue(mv_grid(&req, g_grid, GRID_CAP, &cols, &rows, &resM) == MV_OK, "mv_grid ok");
    for (int i = 1; i + 1 < res.path_len; ++i) {
        const int col = static_cast<int>(std::floor(res.path[i].x / resM));
        const int row = static_cast<int>(std::floor(res.path[i].y / resM));
        if (g_grid[row * cols + col] != 0) {
            std::printf("  FAIL %s: path point %d (%.3f, %.3f) is blocked\n", what, i, res.path[i].x, res.path[i].y);
            ++g_failures;
            return;
        }
    }
}

// ------------------------------------------------------------ tests
void testEmptyRoomDiagonal() {
    std::printf("empty room diagonal\n");
    MvRequest req = makeRequest(4.0f, 4.0f, nullptr, nullptr, 0, MvPose{0.5f, 0.5f, 0.0f}, MvPose{3.5f, 3.5f, PI / 2});
    MvResult res = makeResult();
    expectTrue(mv_plan(&req, &res) == MV_OK, "status ok");
    expectTrue(res.wp_len == 2, "straight line -> 2 waypoints");
    expectTrue(res.prim_len == 4, "ROTATE FORWARD ROTATE STOP");
    expectNear(res.prims[0].a, PI / 4, EPS_TIGHT, "first turn 45 deg");
    expectNear(res.prims[1].a, std::sqrt(18.0f), EPS_TIGHT, "forward 3*sqrt2");
    expectNear(res.length_m, std::sqrt(18.0f), EPS_TIGHT, "length");
    expectReplayReachesGoal(req, res, "replay");
}

void testDetourAroundBox() {
    std::printf("detour around box\n");
    const MvPoint box[4] = {{1.5f, 0.0f}, {2.5f, 0.0f}, {2.5f, 3.0f}, {1.5f, 3.0f}};
    const int sizes[1] = {4};
    MvRequest req = makeRequest(4.0f, 4.0f, box, sizes, 1, MvPose{0.5f, 1.0f, 0.0f}, MvPose{3.5f, 1.0f, PI});
    MvResult res = makeResult();
    expectTrue(mv_plan(&req, &res) == MV_OK, "status ok");
    expectTrue(res.wp_len >= 3, "detour needs corners");
    bool crossesTop = false;
    for (int i = 0; i < res.path_len; ++i) crossesTop = crossesTop || res.path[i].y > 3.0f;
    expectTrue(crossesTop, "path goes over the box");
    expectPathCellsFree(req, res, "cells");
    expectReplayReachesGoal(req, res, "replay");
    for (int i = 0; i + 1 < res.wp_len; ++i) {
        for (float t = 0.0f; t <= 1.0f; t += 0.01f) {
            const float x = res.waypoints[i].x + t * (res.waypoints[i + 1].x - res.waypoints[i].x);
            const float y = res.waypoints[i].y + t * (res.waypoints[i + 1].y - res.waypoints[i].y);
            const bool insideBox = x > 1.5f && x < 2.5f && y < 3.0f;
            if (insideBox) {
                std::printf("  FAIL waypoint segment %d crosses the box at (%.2f, %.2f)\n", i, x, y);
                ++g_failures;
                break;
            }
        }
    }
}

void testGoalNextToObject() {
    std::printf("goal 5 cm from object (CM spot)\n");
    const MvPoint box[4] = {{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}};
    const int sizes[1] = {4};
    const float gx = 2.0f + ROBOT_RADIUS + 0.05f;
    MvRequest req = makeRequest(4.0f, 4.0f, box, sizes, 1, MvPose{0.5f, 3.5f, 0.0f}, MvPose{gx, 1.5f, PI});
    MvResult res = makeResult();
    const int status = mv_plan(&req, &res);
    expectTrue(status == MV_OK, "status ok");
    if (status != MV_OK) return;
    expectNear(res.path[res.path_len - 1].x, gx, 1e-6f, "path ends at exact goal");
    expectNear(res.waypoints[res.wp_len - 1].y, 1.5f, 1e-6f, "waypoints end at exact goal");
    expectPathCellsFree(req, res, "cells");
    expectReplayReachesGoal(req, res, "replay");
}

void testHolonomic() {
    std::printf("holonomic primitives\n");
    const MvPoint box[4] = {{1.5f, 0.0f}, {2.5f, 0.0f}, {2.5f, 3.0f}, {1.5f, 3.0f}};
    const int sizes[1] = {4};
    MvRequest req = makeRequest(4.0f, 4.0f, box, sizes, 1, MvPose{0.5f, 1.0f, 0.3f}, MvPose{3.5f, 1.0f, -1.0f}, 1);
    MvResult res = makeResult();
    expectTrue(mv_plan(&req, &res) == MV_OK, "status ok");
    bool onlyMoveRotateStop = true;
    for (int i = 0; i < res.prim_len; ++i) onlyMoveRotateStop = onlyMoveRotateStop && res.prims[i].type != 1;
    expectTrue(onlyMoveRotateStop, "no FORWARD in holonomic mode");
    expectTrue(res.prim_len == res.wp_len - 1 + 2, "MOVE per segment + ROTATE + STOP");
    expectReplayReachesGoal(req, res, "replay");
}

void testGoalInsideObject() {
    std::printf("goal inside object\n");
    const MvPoint box[4] = {{1.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 3.0f}, {1.0f, 3.0f}};
    const int sizes[1] = {4};
    MvRequest req = makeRequest(4.0f, 4.0f, box, sizes, 1, MvPose{0.5f, 0.5f, 0.0f}, MvPose{2.0f, 2.0f, 0.0f});
    MvResult res = makeResult();
    expectTrue(mv_plan(&req, &res) == MV_ERR_GOAL_BLOCKED, "goal blocked");
    req.start = MvPose{2.0f, 2.0f, 0.0f};
    req.goal = MvPose{0.5f, 0.5f, 0.0f};
    expectTrue(mv_plan(&req, &res) == MV_ERR_START_BLOCKED, "start blocked");
}

void testNoPath() {
    std::printf("room split by a wall\n");
    const MvPoint wall[4] = {{1.9f, 0.0f}, {2.1f, 0.0f}, {2.1f, 4.0f}, {1.9f, 4.0f}};
    const int sizes[1] = {4};
    MvRequest req = makeRequest(4.0f, 4.0f, wall, sizes, 1, MvPose{0.5f, 0.5f, 0.0f}, MvPose{3.5f, 3.5f, 0.0f});
    MvResult res = makeResult();
    expectTrue(mv_plan(&req, &res) == MV_ERR_NO_PATH, "no path");
}

void testArgumentsAndBuffers() {
    std::printf("arguments and buffers\n");
    MvRequest req = makeRequest(4.0f, 4.0f, nullptr, nullptr, 0, MvPose{0.5f, 0.5f, 0.0f}, MvPose{3.5f, 3.5f, 0.0f});
    MvResult res = makeResult();
    expectTrue(mv_plan(nullptr, &res) == MV_ERR_ARGS, "null request");
    expectTrue(mv_plan(&req, nullptr) == MV_ERR_ARGS, "null result");
    const int badSizes[1] = {2};
    const MvPoint pts[2] = {{1.0f, 1.0f}, {2.0f, 2.0f}};
    MvRequest bad = makeRequest(4.0f, 4.0f, pts, badSizes, 1, req.start, req.goal);
    expectTrue(mv_plan(&bad, &res) == MV_ERR_ARGS, "polygon with 2 vertices");
    MvRequest big = makeRequest(20.0f, 20.0f, nullptr, nullptr, 0, req.start, req.goal);
    expectTrue(mv_plan(&big, &res) == MV_ERR_ROOM_TOO_BIG, "room too big");
    MvResult small = makeResult();
    small.path_cap = 2;
    expectTrue(mv_plan(&req, &small) == MV_ERR_BUFFER, "path buffer too small");
    expectTrue(small.path_len > 2, "needed path length reported");
    MvResult fewPrims = makeResult();
    fewPrims.prim_cap = 1;
    expectTrue(mv_plan(&req, &fewPrims) == MV_ERR_BUFFER, "primitive buffer too small");
}

void testGridExport() {
    std::printf("grid export\n");
    const MvPoint box[4] = {{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}};
    const int sizes[1] = {4};
    MvRequest req = makeRequest(4.0f, 4.0f, box, sizes, 1, MvPose{}, MvPose{});
    int cols = 0, rows = 0;
    float resM = 0.0f;
    expectTrue(mv_grid(&req, g_grid, GRID_CAP, &cols, &rows, &resM) == MV_OK, "status ok");
    expectTrue(cols == 80 && rows == 80, "4 m -> 80 cells");
    expectNear(resM, 0.05f, 1e-6f, "resolution");
    expectTrue(g_grid[0] == 1, "corner blocked by wall band");
    expectTrue(g_grid[(rows / 2) * cols + 4] == 1, "wall band 0.255 m wide (x = 0.225 blocked)");
    expectTrue(g_grid[(rows / 2) * cols + 5] == 0, "free just outside the wall band (x = 0.275)");
    expectTrue(g_grid[30 * cols + 30] == 1, "inside the box (1.525, 1.525)");
    expectTrue(g_grid[30 * cols + 44] == 1, "inflated band east of the box (2.225)");
    expectTrue(g_grid[30 * cols + 46] == 0, "free beyond the inflation (2.325)");
    expectTrue(g_grid[70 * cols + 70] == 0, "free far corner");
    expectTrue(mv_grid(&req, g_grid, 10, &cols, &rows, &resM) == MV_ERR_BUFFER, "small grid buffer");
}

}  // namespace

int main() {
    testEmptyRoomDiagonal();
    testDetourAroundBox();
    testGoalNextToObject();
    testHolonomic();
    testGoalInsideObject();
    testNoPath();
    testArgumentsAndBuffers();
    testGridExport();
    if (g_failures == 0) {
        std::printf("ALL MV TESTS PASSED (mv_version %d)\n", mv_version());
        return 0;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
}
