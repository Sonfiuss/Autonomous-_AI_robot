#include "MC/speed_limit.h"

#include <cmath>

#include "MC/mc_debug.h"

namespace mc {

namespace {

// Normalises a body velocity to unit length in its own mixed metric. A leg is
// either purely linear or purely rotational, so exactly one term is non-zero.
rm::BodyVel unitOf(const rm::BodyVel& direction) {
    const float linear = std::sqrt(direction.u * direction.u + direction.v * direction.v);
    const float length = linear > 0.0f ? linear : std::fabs(direction.r);
    rm::BodyVel out;
    if (length <= 0.0f) {
        return out;
    }
    out.u = direction.u / length;
    out.v = direction.v / length;
    out.r = direction.r / length;
    return out;
}

}  // namespace

float peakWheelOmega(const rm::OmniKinematics& kinematics, const rm::BodyVel& direction) {
    const rm::WheelSpeeds wheels = kinematics.inverse(direction);
    float peak = 0.0f;
    for (int i = 0; i < rm::cfg::NUM_WHEELS; ++i) {
        peak = std::fmax(peak, std::fabs(wheels.w[i]));
    }
    return peak;
}

AxisLimits limitsFor(const rm::OmniKinematics& kinematics, const rm::BodyVel& direction,
                     float requested) {
    AxisLimits limits;
    const rm::BodyVel unit = unitOf(direction);
    const float       peak = peakWheelOmega(kinematics, unit);
    if (peak < cfg::MIN_WHEEL_COEFF) {
        MC_DLOG("limitsFor: direction moves no wheel (peak %.6f)\n", peak);
        return limits;
    }
    const float feasible = rm::cfg::MAX_WHEEL_OMEGA_RAD_S / peak;
    limits.vMax  = std::fmin(std::fabs(requested), feasible);
    limits.accel = rm::cfg::WHEEL_ACCEL_RAD_S2 / peak;
    limits.decel = rm::cfg::WHEEL_DECEL_RAD_S2 / peak;
    if (std::fabs(requested) > feasible) {
        MC_DLOG("limitsFor: requested %.3f over feasible %.3f, using feasible\n", requested, feasible);
    }
    return limits;
}

}  // namespace mc
