#include "fw/motion_state.h"

#include <cmath>

#include "RM/speed_limit.h"
#include "fw/fw_debug.h"

namespace fw {

namespace {

// The trapezoid is sampled at the middle of the tick, not its start: that
// integrates to second order and keeps the travelled distance accurate. Same
// choice MC makes, so a previewed trajectory and the real motion agree.
constexpr float TICK_MIDPOINT = 0.5f;

float signOf(float value) {
    return value < 0.0f ? -1.0f : 1.0f;
}

}  // namespace

MotionState::MotionState()
    : kinematics_(),
      slew_(),
      profile_(),
      odometry_(),
      accumulators_(),
      direction_(),
      mode_(MotionMode::IDLE),
      legTime_(0.0f),
      legDuration_(0.0f) {}

void MotionState::resetOdometry() {
    odometry_.reset();
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        accumulators_[wheel].reset();
    }
    FW_DLOG("motion: odometry reset\n");
}

bool MotionState::beginLeg(float length, const rm::BodyVel& direction, float requested) {
    const rm::AxisLimits axis = rm::limitsFor(kinematics_, direction, requested);
    if (!profile_.plan(length, axis.vMax, axis.accel, axis.decel)) {
        FW_DLOG("motion: leg rejected (len %.4f vMax %.3f)\n", length, axis.vMax);
        return false;
    }
    direction_   = direction;
    legTime_     = 0.0f;
    legDuration_ = profile_.duration();
    mode_        = MotionMode::RUNNING_LEG;
    FW_DLOG("motion: leg len %.4f vMax %.3f -> %.3f s\n", length, axis.vMax, legDuration_);
    return true;
}

bool MotionState::beginForward(float distanceM, float speedMs) {
    if (mode_ == MotionMode::RUNNING_LEG) {
        return false;
    }
    const float length = std::fabs(distanceM);
    if (length < mc::cfg::MIN_LEG_LENGTH_M) {
        return false;
    }
    rm::BodyVel direction;
    direction.u = signOf(distanceM);
    return beginLeg(length, direction, speedMs);
}

bool MotionState::beginTurn(float angleRad, float rateRadS) {
    if (mode_ == MotionMode::RUNNING_LEG) {
        return false;
    }
    const float length = std::fabs(angleRad);
    if (length < mc::cfg::MIN_LEG_ANGLE_RAD) {
        return false;
    }
    rm::BodyVel direction;
    direction.r = signOf(angleRad);
    return beginLeg(length, direction, rateRadS);
}

void MotionState::requestStop() {
    mode_        = MotionMode::ESTOP;
    legTime_     = 0.0f;
    legDuration_ = 0.0f;
    direction_   = rm::BodyVel();
    FW_DLOG("motion: stop requested\n");
}

void MotionState::clearStop() {
    if (mode_ == MotionMode::ESTOP) {
        mode_ = MotionMode::IDLE;
        FW_DLOG("motion: stop released\n");
    }
}

rm::BodyVel MotionState::legVelocity(float legTime) const {
    const float speed = profile_.velocityAt(legTime);
    rm::BodyVel body;
    body.u = direction_.u * speed;
    body.v = direction_.v * speed;
    body.r = direction_.r * speed;
    return body;
}

void MotionState::tick(const rm::BodyVel& velocity, float dt, MotionTick* out) {
    if (out == nullptr || dt < rm::cfg::MIN_DT_S) {
        return;
    }

    out->legFinished = false;
    rm::BodyVel target;   // zero unless a branch below says otherwise

    switch (mode_) {
        case MotionMode::ESTOP:
            // Target stays zero, and stays zero: only clearStop() or a new
            // explicit move leaves this state. Reaching rest is not consent to
            // start moving again.
            break;

        case MotionMode::RUNNING_LEG:
            target = legVelocity(legTime_ + TICK_MIDPOINT * dt);
            legTime_ += dt;
            if (legTime_ >= legDuration_) {
                mode_            = MotionMode::IDLE;
                out->legFinished = true;
            }
            break;

        case MotionMode::IDLE:
            target = velocity;
            break;
    }

    const rm::WheelSpeeds wheels = slew_.step(kinematics_.inverse(target), dt);

    int32_t counts[rm::cfg::NUM_WHEELS];
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        out->steps[wheel] = rm::DriverStepDir::toStepCommand(wheels.w[wheel]);
        counts[wheel]     = accumulators_[wheel].accumulate(out->steps[wheel], dt);
    }

    // No encoder: the pose is dead-reckoned from the steps actually commanded.
    odometry_.update(counts, dt);
    out->pose = odometry_.pose();
}

}  // namespace fw
