// Speed_Limit — how fast the chassis may travel in a given direction.
//
// A 3-omni base is not equally fast in every direction: driving along body +x
// loads the wheels differently from driving sideways or spinning. For a body
// velocity direction d, inverse kinematics gives wheel speeds proportional to
// |d|, so the fastest wheel fixes the ceiling:
//
//     peak(d)  = max_i |ω_i(d)|   for the unit direction d
//     v_max    = MAX_WHEEL_OMEGA_RAD_S / peak(d)
//     a_max    = WHEEL_ACCEL_RAD_S2   / peak(d)
//
// Scaling the whole vector (rather than clipping one wheel at its limit) is
// what keeps the robot on the planned line: clipping a single wheel changes the
// direction of motion, scaling does not.
//
// Pure chassis maths, so it lives with the rest of it in RM. Both callers need
// it: MC when previewing a trajectory, and the ESP32 firmware when planning a
// leg live.
#ifndef RM_SPEED_LIMIT_H
#define RM_SPEED_LIMIT_H

#include "RM/omni_kinematics.h"
#include "RM/types.h"

namespace rm {

// Feasible speed and ramp for one direction of travel, in that direction's own
// units (m and m/s for a linear leg, rad and rad/s for a rotation).
struct AxisLimits {
    float vMax  = 0.0f;
    float accel = 0.0f;
    float decel = 0.0f;
};

// Largest |ω| any wheel reaches for this body velocity, rad/s.
float peakWheelOmega(const OmniKinematics& kinematics, const BodyVel& direction);

// Feasible speed and ramp for travelling along `direction`. Only the direction
// matters, not its magnitude. `requested` is the caller's speed in the same
// units as the direction vector; the result is never faster than `requested`.
// A direction that moves no wheel returns all-zero limits.
AxisLimits limitsFor(const OmniKinematics& kinematics, const BodyVel& direction, float requested);

}  // namespace rm

#endif  // RM_SPEED_LIMIT_H
