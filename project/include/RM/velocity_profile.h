// Velocity_Profile — smooth wheel-speed ramps so stepper motors never skip.
//
// Two profiles for the two kinds of command the Jetson sends:
//   VelocityProfile     streaming velocity commands (`M vx vy ω`): a per-tick
//                       slew-rate limiter that moves the current wheel speeds
//                       toward the target without exceeding accel/decel limits.
//   TrapezoidalProfile  distance moves (`F`, `T`): plans accel/hold/decel over
//                       a known distance, falling back to a triangle profile
//                       when the distance is too short to reach cruise speed.
#ifndef RM_VELOCITY_PROFILE_H
#define RM_VELOCITY_PROFILE_H

#include "types.h"

namespace rm {

class VelocityProfile {
public:
    explicit VelocityProfile(float accel    = cfg::WHEEL_ACCEL_RAD_S2,
                             float decel    = cfg::WHEEL_DECEL_RAD_S2,
                             float maxOmega = cfg::MAX_WHEEL_OMEGA_RAD_S);

    // Restarts the profile from the given wheel speeds (default: all zero).
    void reset(const WheelSpeeds& current = WheelSpeeds());

    // Advances one control tick of dt seconds toward target and returns the new
    // current speeds. All three wheels are scaled by a common factor so the
    // direction of motion is preserved while the fastest-changing wheel obeys
    // its limit. Targets above maxOmega are scaled down together the same way.
    WheelSpeeds step(const WheelSpeeds& target, float dt);

    const WheelSpeeds& current() const { return current_; }

    // True when every wheel is within cfg::OMEGA_TOLERANCE_RAD_S of target.
    bool atTarget(const WheelSpeeds& target) const;

private:
    // Scales target so no |ω_i| exceeds maxOmega_, preserving ratios.
    WheelSpeeds clampTarget(const WheelSpeeds& target) const;

    float       accel_;
    float       decel_;
    float       maxOmega_;
    WheelSpeeds current_;
};

class TrapezoidalProfile {
public:
    TrapezoidalProfile();

    // Plans a move of `distance` (signed; rad for a wheel, m for the chassis)
    // with cruise speed vMax and the given accel/decel (same units per s / s²).
    // Returns false and leaves an empty profile if the inputs are invalid.
    bool plan(float distance, float vMax, float accel, float decel);

    // Signed velocity at time t seconds after the start of the move.
    float velocityAt(float t) const;

    // Signed position travelled at time t.
    float positionAt(float t) const;

    float duration() const { return tAccel_ + tHold_ + tDecel_; }
    float peakVelocity() const { return sign_ * vPeak_; }
    bool  finished(float t) const { return t >= duration(); }

private:
    float sign_;    // +1 / -1 direction of travel
    float vPeak_;   // reached cruise (or triangle peak) speed, >= 0
    float accel_;
    float decel_;
    float tAccel_;
    float tHold_;
    float tDecel_;
};

}  // namespace rm

#endif  // RM_VELOCITY_PROFILE_H
