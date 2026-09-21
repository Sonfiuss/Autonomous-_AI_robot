// Plain data types shared by all MV components. World frame, metres, radians.
#ifndef MV_TYPES_H
#define MV_TYPES_H

namespace mv {

struct Point {
    float x = 0.0f;  // m
    float y = 0.0f;  // m
};

struct Pose {
    float x     = 0.0f;  // m
    float y     = 0.0f;  // m
    float theta = 0.0f;  // rad, CCW from world +x
};

// Grid cell address: col along +x, row along +y (row 0 at y = 0).
struct Cell {
    int col = 0;
    int row = 0;
};

}  // namespace mv

#endif  // MV_TYPES_H
