// Plain data types shared by all RM components.
#ifndef RM_TYPES_H
#define RM_TYPES_H

#include "constants.h"

namespace rm {

// Velocity (or displacement) in the robot body frame: u forward, v left, r CCW.
struct BodyVel {
    float u = 0.0f;  // m/s  (or m when used as a displacement)
    float v = 0.0f;  // m/s
    float r = 0.0f;  // rad/s (or rad)
};

// Velocity in the world frame: vx east/forward-at-θ=0, vy left, wz CCW.
struct GlobalVel {
    float vx = 0.0f;  // m/s
    float vy = 0.0f;  // m/s
    float wz = 0.0f;  // rad/s
};

// Angular speed (or angle) of each wheel, index 0..NUM_WHEELS-1.
struct WheelSpeeds {
    float w[cfg::NUM_WHEELS] = {0.0f, 0.0f, 0.0f};  // rad/s (or rad)
};

// Robot pose in the world frame.
struct Pose {
    float x     = 0.0f;  // m
    float y     = 0.0f;  // m
    float theta = 0.0f;  // rad, wrapped to (-π, π]
};

}  // namespace rm

#endif  // RM_TYPES_H
