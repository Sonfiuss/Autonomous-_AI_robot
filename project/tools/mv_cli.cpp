// mv_cli: drives the MV C API from a text script on stdin, for manual tests
// without Python.
//
//   room  <w> <l>                 room size, metres
//   robot <radius>                hull radius, metres
//   poly  <n> <x1> <y1> ... <xn> <yn>
//   start <x> <y> <theta>
//   goal  <x> <y> <theta>
//   holonomic <0|1>
//   grid                          print the inflated grid ('#' blocked, '.' free, row 0 last)
//   plan                          run mv_plan and print status, path, waypoints, primitives
#include <cstdio>
#include <cstring>

#include "MV/mv_api.h"

namespace {

constexpr int MAX_VERTICES = 4096;
constexpr int MAX_POLYS = 256;
constexpr int MAX_PATH = 4098;
constexpr int MAX_PRIMS = 2 * MAX_PATH + 2;
constexpr int MAX_GRID_BYTES = 240 * 240;
constexpr int LINE_MAX = 256;

MvPoint     g_vertices[MAX_VERTICES];
int         g_polySizes[MAX_POLYS];
MvPoint     g_path[MAX_PATH];
MvPoint     g_waypoints[MAX_PATH];
MvPrimitive g_prims[MAX_PRIMS];
unsigned char g_gridBytes[MAX_GRID_BYTES];

const char* PRIM_NAMES[] = {"ROTATE", "FORWARD", "MOVE", "STOP"};

void printPlan(const MvRequest& req) {
    MvResult res{};
    res.path = g_path;
    res.path_cap = MAX_PATH;
    res.waypoints = g_waypoints;
    res.wp_cap = MAX_PATH;
    res.prims = g_prims;
    res.prim_cap = MAX_PRIMS;
    const int status = mv_plan(&req, &res);
    std::printf("status %d %s\n", status, mv_status_text(status));
    if (status != MV_OK) return;
    std::printf("snapped %.3f %.3f -> %.3f %.3f\n", res.snapped_start.x, res.snapped_start.y,
                res.snapped_goal.x, res.snapped_goal.y);
    std::printf("path %d\n", res.path_len);
    for (int i = 0; i < res.path_len; ++i) std::printf("  %.3f %.3f\n", res.path[i].x, res.path[i].y);
    std::printf("waypoints %d\n", res.wp_len);
    for (int i = 0; i < res.wp_len; ++i) std::printf("  %.3f %.3f\n", res.waypoints[i].x, res.waypoints[i].y);
    std::printf("primitives %d\n", res.prim_len);
    for (int i = 0; i < res.prim_len; ++i) {
        const MvPrimitive& p = res.prims[i];
        const char* name = (p.type >= 0 && p.type < 4) ? PRIM_NAMES[p.type] : "?";
        std::printf("  %s %.4f %.4f\n", name, p.a, p.b);
    }
    std::printf("length %.3f\n", res.length_m);
}

void printGrid(const MvRequest& req) {
    int cols = 0, rows = 0;
    float res = 0.0f;
    const int status = mv_grid(&req, g_gridBytes, MAX_GRID_BYTES, &cols, &rows, &res);
    std::printf("status %d %s\n", status, mv_status_text(status));
    if (status != MV_OK) return;
    std::printf("grid %d %d %.3f\n", cols, rows, res);
    for (int row = rows - 1; row >= 0; --row) {
        for (int col = 0; col < cols; ++col) std::putchar(g_gridBytes[row * cols + col] ? '#' : '.');
        std::putchar('\n');
    }
}

}  // namespace

int main() {
    MvRequest req{};
    req.vertices = g_vertices;
    req.poly_sizes = g_polySizes;
    int vertexCount = 0;
    char word[LINE_MAX];
    while (std::scanf("%255s", word) == 1) {
        if (std::strcmp(word, "room") == 0) {
            if (std::scanf("%f %f", &req.room_w, &req.room_l) != 2) break;
        } else if (std::strcmp(word, "robot") == 0) {
            if (std::scanf("%f", &req.robot_radius) != 1) break;
        } else if (std::strcmp(word, "poly") == 0) {
            int n = 0;
            if (std::scanf("%d", &n) != 1 || n < 3 || vertexCount + n > MAX_VERTICES || req.n_polys >= MAX_POLYS) break;
            for (int i = 0; i < n; ++i) {
                if (std::scanf("%f %f", &g_vertices[vertexCount + i].x, &g_vertices[vertexCount + i].y) != 2) return 1;
            }
            g_polySizes[req.n_polys++] = n;
            vertexCount += n;
        } else if (std::strcmp(word, "start") == 0) {
            if (std::scanf("%f %f %f", &req.start.x, &req.start.y, &req.start.theta) != 3) break;
        } else if (std::strcmp(word, "goal") == 0) {
            if (std::scanf("%f %f %f", &req.goal.x, &req.goal.y, &req.goal.theta) != 3) break;
        } else if (std::strcmp(word, "holonomic") == 0) {
            if (std::scanf("%d", &req.holonomic) != 1) break;
        } else if (std::strcmp(word, "grid") == 0) {
            printGrid(req);
        } else if (std::strcmp(word, "plan") == 0) {
            printPlan(req);
        } else {
            std::fprintf(stderr, "unknown command: %s\n", word);
            return 1;
        }
    }
    return 0;
}
