#include "MV/path.h"

#include <cmath>

#include "constants.h"

namespace mv {

float wrapAngle(float angle) {
    while (angle > rm::cfg::PI) angle -= rm::cfg::TWO_PI;
    while (angle <= -rm::cfg::PI) angle += rm::cfg::TWO_PI;
    return angle;
}

float pathLength(const Point* pts, int n) {
    float total = 0.0f;
    for (int i = 0; i + 1 < n; ++i) {
        const float dx = pts[i + 1].x - pts[i].x;
        const float dy = pts[i + 1].y - pts[i].y;
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

int smoothPath(const OccupancyGrid& grid, const Point* path, int n, Point* out, int maxOut) {
    if (path == nullptr || out == nullptr || n <= 0 || maxOut <= 0) return n <= 0 ? 0 : -1;
    out[0] = path[0];
    int count = 1;
    int i = 0;
    while (i < n - 1) {
        // Farthest point still visible from path[i]; the direct successor is
        // always accepted (adjacent cells, or an exact start/goal point).
        int j = n - 1;
        while (j > i + 1 && !grid.lineOfSight(path[i], path[j])) --j;
        if (count >= maxOut) return -1;
        out[count++] = path[j];
        i = j;
    }
    return count;
}

int toPrimitives(const Point* waypoints, int n, float startTheta, float goalTheta, bool holonomic,
                 Primitive* out, int maxOut) {
    if (out == nullptr || maxOut <= 0 || (n > 0 && waypoints == nullptr)) return -1;
    int count = 0;
    float theta = startTheta;
    for (int i = 0; i + 1 < n; ++i) {
        const float dx = waypoints[i + 1].x - waypoints[i].x;
        const float dy = waypoints[i + 1].y - waypoints[i].y;
        const float len = std::sqrt(dx * dx + dy * dy);
        if (len < cfg::MIN_SEGMENT_M) continue;
        if (holonomic) {
            if (count >= maxOut) return -1;
            out[count++] = Primitive{PrimitiveType::MOVE, dx, dy};
            continue;
        }
        const float heading = std::atan2(dy, dx);
        const float turn = wrapAngle(heading - theta);
        if (std::fabs(turn) >= cfg::MIN_ROTATE_RAD) {
            if (count >= maxOut) return -1;
            out[count++] = Primitive{PrimitiveType::ROTATE, turn, 0.0f};
        }
        if (count >= maxOut) return -1;
        out[count++] = Primitive{PrimitiveType::FORWARD, len, 0.0f};
        theta = heading;
    }
    const float finalTurn = wrapAngle(goalTheta - theta);
    if (std::fabs(finalTurn) >= cfg::MIN_ROTATE_RAD) {
        if (count >= maxOut) return -1;
        out[count++] = Primitive{PrimitiveType::ROTATE, finalTurn, 0.0f};
    }
    if (count >= maxOut) return -1;
    out[count++] = Primitive{PrimitiveType::STOP, 0.0f, 0.0f};
    return count;
}

}  // namespace mv
