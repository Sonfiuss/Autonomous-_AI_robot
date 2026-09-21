// Executor — walks an MV primitive list one control tick at a time and reports
// the angular speed of each wheel.
//
// Per leg it plans an rm::TrapezoidalProfile over the leg length, using the
// feasible speed and ramp for that direction (speed_limit.h), then each tick:
//
//   trapezoid velocity -> body velocity -> rm::OmniKinematics::inverse
//     -> rm::VelocityProfile (per-wheel slew guard) -> rm::DriverStepDir
//
// The trapezoid already respects the wheel acceleration limit, so the slew
// guard only acts at leg boundaries; it is a safety net, not the main ramp.
// The velocity is sampled at the middle of each tick, which integrates to
// second order and keeps the travelled distance accurate at a 50 Hz tick.
//
// MOVE legs carry a WORLD-frame delta, so the executor tracks the heading
// (integrated from the rotation it actually commands) and rotates the leg into
// the body frame before inverse kinematics.
#ifndef MC_EXECUTOR_H
#define MC_EXECUTOR_H

#include "MC/types.h"
#include "RM/omni_kinematics.h"
#include "RM/velocity_profile.h"

namespace mc {

class Executor {
public:
    Executor();

    // Loads a primitive list. `startTheta` is the robot heading (rad) at the
    // first tick, needed to place MOVE legs. Returns false on a null list, a
    // negative count, an unknown primitive type or a singular wheel layout.
    bool load(const Primitive* primitives, int count, const MotionLimits& limits, float startTheta);

    // Advances one tick of dt seconds and fills `out`. Returns false when the
    // whole list has been executed (nothing written) or dt is too small.
    bool step(float dt, MotionStep* out);

    bool  finished() const { return index_ >= count_; }
    float heading() const { return theta_; }          // rad, integrated from commanded rotation
    float elapsed() const { return elapsed_; }        // s since the first tick

private:
    // Prepares legState_ for primitives_[index_]; skips legs shorter than the
    // minimum length. Returns false when the list is exhausted.
    bool beginLeg();

    // Commanded body velocity for the active leg at `legTime` seconds into it.
    rm::BodyVel commandAt(float legTime) const;

    const Primitive*    primitives_;
    int                 count_;
    int                 index_;
    MotionLimits        limits_;

    rm::OmniKinematics    kinematics_;
    rm::VelocityProfile   slew_;
    rm::TrapezoidalProfile profile_;

    rm::BodyVel direction_;   // unit body-frame direction of the active leg
    float       legTime_;     // s since the active leg started
    float       legDuration_; // s, trapezoid duration (STOP: the hold time)
    float       theta_;       // rad, current heading
    float       elapsed_;     // s since the first tick
    bool        legIsStop_;
};

}  // namespace mc

#endif  // MC_EXECUTOR_H
