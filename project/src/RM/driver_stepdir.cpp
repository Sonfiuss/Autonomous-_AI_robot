#include "RM/driver_stepdir.h"

#include <cmath>

#include "RM/rm_debug.h"

namespace rm {

// ============================================================= DriverStepDir

StepCommand DriverStepDir::toStepCommand(float omega) {
    StepCommand cmd;
    cmd.forward = omega >= 0.0f;
    const float magnitude = std::fabs(omega);
    if (magnitude < cfg::MIN_WHEEL_OMEGA_RAD_S) {
        cmd.freqHz = 0.0f;
        return cmd;
    }
    // f = ω / (2π) · steps_per_rev
    cmd.freqHz = magnitude * cfg::STEPS_PER_RAD;
    if (cmd.freqHz > cfg::MAX_PULSE_HZ) {
        RM_DLOG("DriverStepDir: %.1f Hz clamped to %.1f Hz\n", cmd.freqHz, cfg::MAX_PULSE_HZ);
        cmd.freqHz = cfg::MAX_PULSE_HZ;
    }
    return cmd;
}

float DriverStepDir::toOmega(const StepCommand& cmd) {
    const float omega = cmd.freqHz * cfg::RAD_PER_STEP;
    return cmd.forward ? omega : -omega;
}

int32_t DriverStepDir::angleToSteps(float rad) {
    return static_cast<int32_t>(std::lround(rad * cfg::STEPS_PER_RAD));
}

float DriverStepDir::stepsToAngle(int32_t steps) {
    return static_cast<float>(steps) * cfg::RAD_PER_STEP;
}

int32_t DriverStepDir::distanceToSteps(float meters) {
    return static_cast<int32_t>(std::lround(meters / cfg::METERS_PER_STEP));
}

float DriverStepDir::stepsToDistance(int32_t steps) {
    return static_cast<float>(steps) * cfg::METERS_PER_STEP;
}

// =========================================================== StepAccumulator

StepAccumulator::StepAccumulator() : remainder_(0.0f), total_(0) {}

void StepAccumulator::reset() {
    remainder_ = 0.0f;
    total_     = 0;
}

int32_t StepAccumulator::accumulate(const StepCommand& cmd, float dt) {
    if (dt <= 0.0f || cmd.freqHz <= 0.0f) {
        return 0;
    }
    const float signedRate = cmd.forward ? cmd.freqHz : -cmd.freqHz;
    remainder_ += signedRate * dt;
    const int32_t whole = static_cast<int32_t>(remainder_);  // truncates toward zero
    remainder_ -= static_cast<float>(whole);
    total_ += whole;
    return whole;
}

}  // namespace rm
