// Driver_Stepdir — unit conversion between RM's rad/s world and the
// step/direction world of a stepper driver. NO GPIO, timers or pulse
// generation here: the firmware takes StepCommand and drives the pins.
#ifndef RM_DRIVER_STEPDIR_H
#define RM_DRIVER_STEPDIR_H

#include <cstdint>

#include "types.h"

namespace rm {

// What the pulse generator needs for one wheel.
struct StepCommand {
    float freqHz  = 0.0f;  // pulse rate, always >= 0; 0 means hold
    bool  forward = true;  // DIR pin level: true = positive ω
};

class DriverStepDir {
public:
    // ω (rad/s) -> pulse frequency + direction. |ω| below
    // cfg::MIN_WHEEL_OMEGA_RAD_S gives 0 Hz; above the limit is clamped to
    // cfg::MAX_PULSE_HZ.
    static StepCommand toStepCommand(float omega);

    // Inverse of toStepCommand (signed rad/s).
    static float toOmega(const StepCommand& cmd);

    // Wheel angle (rad) <-> microstep count, rounded to nearest step.
    static int32_t angleToSteps(float rad);
    static float   stepsToAngle(int32_t steps);

    // Rim distance (m) <-> microstep count.
    static int32_t distanceToSteps(float meters);
    static float   stepsToDistance(int32_t steps);
};

// Tracks how many whole steps a pulse train at a given frequency has emitted,
// carrying the fractional remainder between ticks. Lets the firmware feed
// Odometry with the steps it commanded when no encoder is present.
class StepAccumulator {
public:
    StepAccumulator();

    void reset();

    // Adds dt seconds of cmd and returns the signed whole steps produced this tick.
    int32_t accumulate(const StepCommand& cmd, float dt);

    // Signed running total since reset().
    int32_t total() const { return total_; }

private:
    float   remainder_;  // fractional step carried to the next tick, signed
    int32_t total_;
};

}  // namespace rm

#endif  // RM_DRIVER_STEPDIR_H
