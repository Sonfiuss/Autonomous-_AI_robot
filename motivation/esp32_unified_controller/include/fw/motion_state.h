// Motion_State — the Motion task's whole brain: the IDLE / RUNNING_LEG / ESTOP
// machine plus every RM object it drives.
//
// Deliberately free of FreeRTOS, GPIO and timers: the task around it only moves
// data in and out, which is what makes this the one part of the firmware that
// can be unit tested on a PC.
//
// Velocity commands arrive in the BODY frame (the link protocol's `M`), so no
// world-to-body rotation happens anywhere in the firmware.
#ifndef FW_MOTION_STATE_H
#define FW_MOTION_STATE_H

#include <cstdint>

#include "RM/driver_stepdir.h"
#include "RM/odometry.h"
#include "RM/omni_kinematics.h"
#include "RM/types.h"
#include "RM/velocity_profile.h"

namespace fw {

enum class MotionMode : uint8_t {
    IDLE,         // following the streamed velocity command
    RUNNING_LEG,  // executing one F/T move through to its end
    ESTOP,        // ramping the wheels down after S
};

// What one control tick produced.
struct MotionTick {
    rm::StepCommand steps[rm::cfg::NUM_WHEELS];
    rm::Pose        pose;
    bool            legFinished = false;  // a one-shot move ended on this tick -> send K
};

class MotionState {
public:
    MotionState();

    // False if constants.h describes a singular wheel layout, in which case the
    // firmware must not drive anything.
    bool valid() const { return kinematics_.valid() && odometry_.valid(); }

    // Command R. The pose jumps to the origin; a running leg is unaffected,
    // because legs are timed, not pose-driven.
    void resetOdometry();

    // Command F: a distance move along body +x. Refused only while another leg
    // is running, or when the move is too short to plan — an explicit move is
    // new operator intent, so it releases a latched stop.
    bool beginForward(float distanceM, float speedMs);

    // Command T: a turn in place. Same rules as beginForward.
    bool beginTurn(float angleRad, float rateRadS);

    // Command S: abandon any leg and ramp down. The stop LATCHES: the machine
    // stays in ESTOP driving zero even if velocity commands keep streaming in,
    // because a stop that a stale queued command can undo is not a stop.
    void requestStop();

    // Releases a latched stop so streamed velocity is followed again. The
    // Motion task calls this only for a velocity command stamped AFTER the
    // stop, never for one that was already in flight when the stop arrived.
    void clearStop();

    // One control tick. `velocity` is the newest streamed command and is used
    // ONLY in IDLE: a running leg ignores it and a stop overrides it.
    void tick(const rm::BodyVel& velocity, float dt, MotionTick* out);

    MotionMode      mode() const { return mode_; }
    bool            busy() const { return mode_ == MotionMode::RUNNING_LEG; }
    const rm::Pose& pose() const { return odometry_.pose(); }

private:
    // Plans one leg of `length` travelling along the unit `direction`, at the
    // fastest speed that direction can sustain, capped by `requested`.
    bool beginLeg(float length, const rm::BodyVel& direction, float requested);

    rm::BodyVel legVelocity(float legTime) const;

    rm::OmniKinematics     kinematics_;
    rm::VelocityProfile    slew_;
    rm::TrapezoidalProfile profile_;
    rm::Odometry           odometry_;
    rm::StepAccumulator    accumulators_[rm::cfg::NUM_WHEELS];
    rm::BodyVel            direction_;    // unit body-frame direction of the active leg
    MotionMode             mode_;
    float                  legTime_;
    float                  legDuration_;
};

}  // namespace fw

#endif  // FW_MOTION_STATE_H
