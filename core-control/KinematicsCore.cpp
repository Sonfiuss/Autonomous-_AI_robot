#include "KinematicsCore.hpp"

#include <cmath>
#include <cstdio>

namespace kinematics {

// Wheel mounting angles in radians
static constexpr float DEG2RAD = M_PI / 180.f;
static constexpr float RAD2DEG = 180.f / M_PI;

static constexpr float THETA[3] = {
    150.f * DEG2RAD,   // W1
    270.f * DEG2RAD,   // W2
     30.f * DEG2RAD,   // W3
};

WheelAngles compute(const Command& cmd) {
    // Translation contribution per wheel (cm of wheel arc)
    // d_i = dx·cos(θi) + dy·sin(θi)
    // → wheel rotation = d_i / r  (rad)  × RAD2DEG  (deg)

    // Rotation contribution: each wheel travels rotate_rad × L tangentially
    // → wheel rotation = rotate_rad × L / r  (rad) × RAD2DEG  (deg)

    const float rotate_rad = cmd.rotate_deg * DEG2RAD;
    const float rot_contribution = rotate_rad * WHEEL_DIST_CM / WHEEL_RADIUS_CM * RAD2DEG;

    WheelAngles wa;
    float* out[3] = { &wa.w1_deg, &wa.w2_deg, &wa.w3_deg };

    for (int i = 0; i < 3; ++i) {
        float linear_cm = cmd.dx_cm * std::cos(THETA[i])
                        + cmd.dy_cm * std::sin(THETA[i]);
        float linear_deg = (linear_cm / WHEEL_RADIUS_CM) * RAD2DEG;
        *out[i] = linear_deg + rot_contribution;
    }

    return wa;
}

void printResult(const Command& cmd, const WheelAngles& wa) {
    printf("─────────────────────────────────────\n");
    printf("  Command  : dx=%.1f cm  dy=%.1f cm  rot=%.1f°\n",
           cmd.dx_cm, cmd.dy_cm, cmd.rotate_deg);
    printf("─────────────────────────────────────\n");
    printf("  W1 (150°): %+8.2f °\n", wa.w1_deg);
    printf("  W2 (270°): %+8.2f °\n", wa.w2_deg);
    printf("  W3 ( 30°): %+8.2f °\n", wa.w3_deg);
    printf("─────────────────────────────────────\n");
}

} // namespace kinematics
