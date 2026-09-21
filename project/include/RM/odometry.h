// Odometry — dead-reckoning pose from per-wheel count deltas.
//
// Count source is cfg::ODOM_COUNTS_PER_REV: today the steps actually commanded
// to the driver (no encoder fitted), later real encoder counts. Each update
// converts counts -> wheel angle delta -> body displacement (forward
// kinematics) -> world pose, integrating with the mid-point heading so arcs
// are tracked to second order.
#ifndef RM_ODOMETRY_H
#define RM_ODOMETRY_H

#include <cstdint>

#include "omni_kinematics.h"
#include "types.h"

namespace rm {

class Odometry {
public:
    Odometry();

    // Sets the pose (default: origin) and zeroes the velocity estimate.
    void reset(const Pose& pose = Pose());

    // Integrates raw count deltas since the previous call over dt seconds.
    // The pose always advances; the velocity estimate is kept from the previous
    // call when dt < cfg::MIN_DT_S.
    void update(const int32_t deltaCounts[cfg::NUM_WHEELS], float dt);

    // Same as update() but with wheel angle deltas already in radians.
    void updateWheelAngles(const float deltaRad[cfg::NUM_WHEELS], float dt);

    // False if the wheel layout in constants.h is singular (pose would never move).
    bool valid() const { return kinematics_.valid(); }

    const Pose&    pose() const { return pose_; }
    const BodyVel& bodyVelocity() const { return bodyVel_; }
    GlobalVel      globalVelocity() const;

    // Wraps an angle to (-π, π].
    static float wrapAngle(float angle);

private:
    OmniKinematics kinematics_;
    Pose           pose_;
    BodyVel        bodyVel_;
};

}  // namespace rm

#endif  // RM_ODOMETRY_H
