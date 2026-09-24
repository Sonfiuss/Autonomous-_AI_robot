#include "RM/speed_limit.h"

#include <cmath>

#include "RM/rm_debug.h"

namespace rm {

namespace {

// Normalises a body velocity to unit length in its own mixed metric. A leg is
// either purely linear or purely rotational, so exactly one term is non-zero.
BodyVel unitOf(const BodyVel& direction) {
    const float linear = std::sqrt(direction.u * direction.u + direction.v * direction.v);
    const float length = linear > 0.0f ? linear : std::fabs(direction.r);
    BodyVel out;
    if (length <= 0.0f) {
        return out;
    }
    out.u = direction.u / length;
    out.v = direction.v / length;
    out.r = direction.r / length;
    return out;
}

}  // namespace

float peakWheelOmega(const OmniKinematics& kinematics, const BodyVel& direction) {
    const WheelSpeeds wheels = kinematics.inverse(direction);
    float peak = 0.0f;
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        peak = std::fmax(peak, std::fabs(wheels.w[i]));
    }
    return peak;
}

AxisLimits limitsFor(const OmniKinematics& kinematics, const BodyVel& direction, float requested) {
    AxisLimits   limits;
    const BodyVel unit = unitOf(direction);
    const float   peak = peakWheelOmega(kinematics, unit);
    if (peak < cfg::MIN_WHEEL_COEFF) {
        RM_DLOG("limitsFor: direction moves no wheel (peak %.6f)\n", peak);
        return limits;
    }
    const float feasible = cfg::MAX_WHEEL_OMEGA_RAD_S / peak;
    limits.vMax  = std::fmin(std::fabs(requested), feasible);
    limits.accel = cfg::WHEEL_ACCEL_RAD_S2 / peak;
    limits.decel = cfg::WHEEL_DECEL_RAD_S2 / peak;
    if (std::fabs(requested) > feasible) {
        RM_DLOG("limitsFor: requested %.3f over feasible %.3f, using feasible\n", requested, feasible);
    }
    return limits;
}

}  // namespace rm
