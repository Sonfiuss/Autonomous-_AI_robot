#pragma once

#include <array>

/**
 * Omni-wheel inverse kinematics for a 3-wheel robot.
 *
 * Robot constants:
 *   Wheel positions: W1=150°, W2=270°, W3=30° (from forward axis)
 *   Wheel radius r  = 4.1 cm
 *   Center-to-wheel L = 14.4 cm
 *
 * Frame convention:
 *   +dx  = forward
 *   +dy  = left
 *   +rot = counter-clockwise (CCW)
 */
namespace kinematics {

// EFFECTIVE rolling radius, calibrated from measurement — NOT the nominal
// geometric radius (4.03 cm = ⌀11 cm wheel). Omni wheels lose travel to passive
// rollers/slip, so the effective radius is smaller. Calibrate as:
//   r_eff = r_old * (measured_distance / commanded_distance)
// 2026-06-29: commanded 30 cm -> measured 22 cm at r=4.03 -> r_eff = 4.03*22/30 ≈ 4.03.
constexpr float WHEEL_RADIUS_CM = 4.03f;
constexpr float WHEEL_DIST_CM   = 21.0f;   // robot center -> wheel contact (rotation scale)

// Degrees each wheel must rotate to execute the command.
// Positive = forward spin direction for that wheel.
struct WheelAngles {
    float w1_deg;  // Wheel 1 at 150°
    float w2_deg;  // Wheel 2 at 270°
    float w3_deg;  // Wheel 3 at 30°
};

// Unified command: all motion types reduce to these three values.
struct Command {
    float dx_cm     = 0.f;  // forward(+) / backward(-)
    float dy_cm     = 0.f;  // left(+) / right(-)
    float rotate_deg = 0.f; // CCW(+) / CW(-)
};

// Compute wheel rotation angles from a command.
WheelAngles compute(const Command& cmd);

// Print a human-readable breakdown to stdout.
void printResult(const Command& cmd, const WheelAngles& wa);

} // namespace kinematics
