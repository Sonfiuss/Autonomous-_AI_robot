#include "RM/odometry.h"

#include <cmath>

#include "RM/rm_debug.h"

namespace rm {

Odometry::Odometry() : kinematics_(), pose_(), bodyVel_() {}

void Odometry::reset(const Pose& pose) {
    pose_       = pose;
    pose_.theta = wrapAngle(pose.theta);
    bodyVel_    = BodyVel();
}

void Odometry::update(const int32_t deltaCounts[cfg::NUM_WHEELS], float dt) {
    float deltaRad[cfg::NUM_WHEELS];
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        deltaRad[i] = static_cast<float>(deltaCounts[i]) * cfg::RAD_PER_ODOM_COUNT;
    }
    updateWheelAngles(deltaRad, dt);
}

void Odometry::updateWheelAngles(const float deltaRad[cfg::NUM_WHEELS], float dt) {
    WheelSpeeds wheelDelta;
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        wheelDelta.w[i] = deltaRad[i];
    }
    // Forward kinematics is linear, so feeding angle deltas yields body displacement.
    const BodyVel disp = kinematics_.forward(wheelDelta);

    // Rotate the body displacement using the heading at the middle of the step.
    const float midTheta = pose_.theta + 0.5f * disp.r;
    const float c        = std::cos(midTheta);
    const float s        = std::sin(midTheta);
    pose_.x += c * disp.u - s * disp.v;
    pose_.y += s * disp.u + c * disp.v;
    pose_.theta = wrapAngle(pose_.theta + disp.r);

    // Displacement is always integrated so no counts are lost; the velocity
    // estimate is only refreshed when dt is long enough to be meaningful.
    if (dt >= cfg::MIN_DT_S) {
        const float invDt = 1.0f / dt;
        bodyVel_.u        = disp.u * invDt;
        bodyVel_.v        = disp.v * invDt;
        bodyVel_.r        = disp.r * invDt;
    }
    RM_DLOG("Odometry: pose (%.4f, %.4f, %.4f) vel (%.3f, %.3f, %.3f)\n",
            pose_.x, pose_.y, pose_.theta, bodyVel_.u, bodyVel_.v, bodyVel_.r);
}

GlobalVel Odometry::globalVelocity() const {
    return OmniKinematics::bodyToGlobal(bodyVel_, pose_.theta);
}

float Odometry::wrapAngle(float angle) {
    // O(1) wrap to (-π, π]: shift by π, reduce into (0, 2π], shift back.
    float shifted = std::fmod(angle + cfg::PI, cfg::TWO_PI);
    if (shifted <= 0.0f) {
        shifted += cfg::TWO_PI;
    }
    return shifted - cfg::PI;
}

}  // namespace rm
