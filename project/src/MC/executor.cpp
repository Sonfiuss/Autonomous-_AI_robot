#include "MC/executor.h"

#include <cmath>

#include "MC/mc_debug.h"
#include "RM/odometry.h"   // wrapAngle
#include "RM/speed_limit.h"

namespace mc {

namespace {

// The trapezoid is sampled at the middle of the tick, not its start: that
// integrates to second order and keeps the travelled distance accurate.
constexpr float TICK_MIDPOINT = 0.5f;

}  // namespace

Executor::Executor()
    : primitives_(nullptr),
      count_(0),
      index_(0),
      limits_(),
      kinematics_(),
      slew_(),
      profile_(),
      direction_(),
      legTime_(0.0f),
      legDuration_(0.0f),
      theta_(0.0f),
      elapsed_(0.0f),
      legIsStop_(false) {}

bool Executor::load(const Primitive* primitives, int count, const MotionLimits& limits,
                    float startTheta) {
    if (primitives == nullptr || count < 0 || !kinematics_.valid()) {
        MC_DLOG("load: bad arguments (prims %p count %d valid %d)\n",
                static_cast<const void*>(primitives), count, kinematics_.valid() ? 1 : 0);
        return false;
    }
    for (int i = 0; i < count; ++i) {
        const PrimitiveType type = primitives[i].type;
        if (type != PrimitiveType::ROTATE && type != PrimitiveType::FORWARD &&
            type != PrimitiveType::MOVE && type != PrimitiveType::STOP) {
            MC_DLOG("load: primitive %d has unknown type %d\n", i, static_cast<int>(type));
            return false;
        }
    }
    primitives_ = primitives;
    count_      = count;
    index_      = 0;
    limits_     = limits;
    theta_      = startTheta;
    elapsed_    = 0.0f;
    slew_.reset();
    // An empty (or all-skipped) list is a valid load with nothing to execute:
    // beginLeg() simply leaves the executor finished.
    beginLeg();
    return true;
}

bool Executor::beginLeg() {
    while (index_ < count_) {
        const Primitive& prim = primitives_[index_];
        legTime_   = 0.0f;
        legIsStop_ = false;
        direction_ = rm::BodyVel();

        if (prim.type == PrimitiveType::STOP) {
            legIsStop_   = true;
            legDuration_ = cfg::STOP_HOLD_S;
            MC_DLOG("leg %d: STOP for %.3f s\n", index_, legDuration_);
            return true;
        }

        // Leg length and its unit direction in the body frame.
        float length = 0.0f;
        float requested = 0.0f;
        if (prim.type == PrimitiveType::ROTATE) {
            length      = std::fabs(prim.a);
            requested   = limits_.yawRate;
            direction_.r = prim.a < 0.0f ? -1.0f : 1.0f;
            if (length < cfg::MIN_LEG_ANGLE_RAD) {
                ++index_;
                continue;
            }
        } else if (prim.type == PrimitiveType::FORWARD) {
            length      = std::fabs(prim.a);
            requested   = limits_.cruiseSpeed;
            direction_.u = prim.a < 0.0f ? -1.0f : 1.0f;
            if (length < cfg::MIN_LEG_LENGTH_M) {
                ++index_;
                continue;
            }
        } else {  // MOVE: a world-frame delta, constant heading during the leg
            length    = std::sqrt(prim.a * prim.a + prim.b * prim.b);
            requested = limits_.cruiseSpeed;
            if (length < cfg::MIN_LEG_LENGTH_M) {
                ++index_;
                continue;
            }
            rm::GlobalVel world;
            world.vx   = prim.a / length;
            world.vy   = prim.b / length;
            direction_ = rm::OmniKinematics::globalToBody(world, theta_);
        }

        const rm::AxisLimits axis = rm::limitsFor(kinematics_, direction_, requested);
        if (!profile_.plan(length, axis.vMax, axis.accel, axis.decel)) {
            MC_DLOG("leg %d: trapezoid rejected (len %.4f vMax %.3f), skipped\n",
                    index_, length, axis.vMax);
            ++index_;
            continue;
        }
        legDuration_ = profile_.duration();
        MC_DLOG("leg %d: type %d len %.4f vMax %.3f accel %.3f -> %.3f s\n",
                index_, static_cast<int>(prim.type), length, axis.vMax, axis.accel, legDuration_);
        return true;
    }
    return false;
}

rm::BodyVel Executor::commandAt(float legTime) const {
    rm::BodyVel body;
    if (legIsStop_) {
        return body;  // zero: the slew guard ramps the wheels down
    }
    const float speed = profile_.velocityAt(legTime);
    body.u = direction_.u * speed;
    body.v = direction_.v * speed;
    body.r = direction_.r * speed;
    return body;
}

bool Executor::step(float dt, MotionStep* out) {
    if (out == nullptr || dt < rm::cfg::MIN_DT_S || finished()) {
        return false;
    }

    // Sample the middle of the tick: second-order accurate over the ramp.
    const rm::BodyVel target = commandAt(legTime_ + TICK_MIDPOINT * dt);
    const rm::WheelSpeeds wheels = slew_.step(kinematics_.inverse(target), dt);

    out->t      = elapsed_;
    out->wheels = wheels;
    out->body   = kinematics_.forward(wheels);   // what the wheels actually produce
    for (int i = 0; i < rm::cfg::NUM_WHEELS; ++i) {
        out->steps[i] = rm::DriverStepDir::toStepCommand(wheels.w[i]);
    }

    theta_   = rm::Odometry::wrapAngle(theta_ + out->body.r * dt);
    elapsed_ += dt;
    legTime_ += dt;
    // A STOP leg holds until the wheels are genuinely at rest, not just until
    // its nominal time is up: every trapezoid ends at zero speed, but a caller
    // that starts mid-motion must still come to a halt before the list ends.
    bool legDone = legTime_ >= legDuration_;
    if (legDone && legIsStop_ && !slew_.atTarget(rm::WheelSpeeds())) {
        legDone = false;
    }
    if (legDone) {
        ++index_;
        beginLeg();   // return ignored on purpose: finished() reports the end of the list
    }
    return true;
}

}  // namespace mc
