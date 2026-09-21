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
#ifndef MC_SPEED_LIMIT_H
#define MC_SPEED_LIMIT_H

#include "MC/types.h"
#include "RM/omni_kinematics.h"

namespace mc {

// Largest |ω| any wheel reaches for this body velocity, rad/s.
float peakWheelOmega(const rm::OmniKinematics& kinematics, const rm::BodyVel& direction);

// Feasible speed and ramp for travelling along `direction`. Only the direction
// matters, not its magnitude. `requested` is the caller's speed in the same
// units as the direction vector; the result is never faster than `requested`.
// A direction that moves no wheel returns all-zero limits.
AxisLimits limitsFor(const rm::OmniKinematics& kinematics, const rm::BodyVel& direction,
                     float requested);

}  // namespace mc

#endif  // MC_SPEED_LIMIT_H
