// Plain data types for the MC motion executor. Body frame, metres, radians.
// One MotionStep is what the control loop needs for a single tick.
#ifndef MC_TYPES_H
#define MC_TYPES_H

#include "RM/driver_stepdir.h"
#include "RM/types.h"
#include "constants.h"

namespace mc {

// Motion primitive as produced by MV. The numeric values MUST stay identical to
// MvPrimitive.type in include/MV/mv_api.h: the two modules exchange raw ints.
enum class PrimitiveType {
    ROTATE  = 0,  // a = signed angle (rad), b unused
    FORWARD = 1,  // a = signed distance along body +x (m), b unused
    MOVE    = 2,  // a = dx, b = dy: a straight leg in the WORLD frame (m)
    STOP    = 3   // a, b unused
};

// The two modules exchange raw ints, so pin the codes at compile time.
static_assert(static_cast<int>(PrimitiveType::ROTATE) == 0, "ROTATE must stay 0 (MvPrimitive.type)");
static_assert(static_cast<int>(PrimitiveType::FORWARD) == 1, "FORWARD must stay 1 (MvPrimitive.type)");
static_assert(static_cast<int>(PrimitiveType::MOVE) == 2, "MOVE must stay 2 (MvPrimitive.type)");
static_assert(static_cast<int>(PrimitiveType::STOP) == 3, "STOP must stay 3 (MvPrimitive.type)");

struct Primitive {
    PrimitiveType type = PrimitiveType::STOP;
    float         a    = 0.0f;
    float         b    = 0.0f;
};

// Speed the caller asks for. The executor lowers these per leg when the
// direction cannot sustain them (see RM/speed_limit.h); it never raises them.
struct MotionLimits {
    float cruiseSpeed = cfg::CRUISE_SPEED_M_S;  // m/s, for FORWARD and MOVE
    float yawRate     = cfg::YAW_RATE_RAD_S;    // rad/s, for ROTATE
};

// One control tick. `body` is what the wheels actually produce this tick
// (forward kinematics of `wheels`), not what was requested, so integrating
// these rows reproduces the motion the robot really performs.
struct MotionStep {
    float             t = 0.0f;  // s since the start of the trajectory
    rm::BodyVel       body;      // achieved body velocity
    rm::WheelSpeeds   wheels;    // rad/s per wheel
    rm::StepCommand   steps[rm::cfg::NUM_WHEELS];  // pulse rate + DIR per wheel
};

}  // namespace mc

#endif  // MC_TYPES_H
