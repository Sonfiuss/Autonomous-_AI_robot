// Path post-processing: cell path -> few straight waypoints -> motion primitives
// (roadmap L4 frozen set ROTATE / FORWARD / STOP; MOVE added for the omni base).
#ifndef MV_PATH_H
#define MV_PATH_H

#include "occupancy_grid.h"
#include "types.h"

namespace mv {

enum class PrimitiveType : int {
    ROTATE  = 0,  // a = angle rad (CCW positive)
    FORWARD = 1,  // a = distance m along the current heading
    MOVE    = 2,  // a = dx m, b = dy m in the world frame (holonomic translation)
    STOP    = 3
};

struct Primitive {
    PrimitiveType type = PrimitiveType::STOP;
    float         a    = 0.0f;
    float         b    = 0.0f;
};

// Greedy line-of-sight smoothing: keeps path[0] and path[n-1] and drops every
// intermediate point still visible from the last kept one. Returns the
// waypoint count, or -1 when out (maxOut slots) is too small.
int smoothPath(const OccupancyGrid& grid, const Point* path, int n, Point* out, int maxOut);

// Turn-go-turn (holonomic = false): ROTATE to each segment heading, FORWARD its
// length, final ROTATE to goalTheta, STOP. Holonomic: MOVE dx dy per segment,
// final ROTATE, STOP. Returns the primitive count or -1 when out is too small.
int toPrimitives(const Point* waypoints, int n, float startTheta, float goalTheta, bool holonomic,
                 Primitive* out, int maxOut);

// Sum of segment lengths of a polyline.
float pathLength(const Point* pts, int n);

// Wraps an angle to (-pi, pi].
float wrapAngle(float angle);

}  // namespace mv

#endif  // MV_PATH_H
