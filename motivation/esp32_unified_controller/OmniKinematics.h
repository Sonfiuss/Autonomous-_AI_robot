#pragma once
#include <math.h>

/**
 * Omni 3-wheel inverse / forward kinematics
 *
 * Vị trí bánh (đối xứng 120°):
 *
 *      W1 (60°)   W3 (300°)
 *           \     /
 *            \   /
 *       W2 (180°)
 *
 * α1=60°, α2=180°, α3=300°
 *
 * Inverse:  ωi = (-sin(αi)·vx + cos(αi)·vy + L·ωz) / r
 *
 *   ω1 = (-S3H·vx + 0.5·vy  + L·ωz) / r
 *   ω2 = (         -vy       + L·ωz) / r
 *   ω3 = ( S3H·vx + 0.5·vy  + L·ωz) / r
 *
 * Forward (pseudoinverse):
 *   vx    = r · INV_S3 · (-w1        + w3)
 *   vy    = r · (1/3·w1  - 2/3·w2  + 1/3·w3)
 *   omega = r / (3·L) · (w1 + w2 + w3)
 */

struct WheelVel   { float w1, w2, w3; };   // rad/s mỗi bánh
struct RobotVel   { float vx, vy, omega; }; // m/s, m/s, rad/s

class OmniKinematics {
public:
    OmniKinematics(float wheel_r, float robot_r)
        : r(wheel_r), L(robot_r) {}

    WheelVel inverse(const RobotVel& v) const {
        return {
            (-S3H * v.vx + 0.5f * v.vy + L * v.omega) / r,
            (              -v.vy         + L * v.omega) / r,
            ( S3H * v.vx + 0.5f * v.vy + L * v.omega) / r
        };
    }

    RobotVel forward(const WheelVel& w) const {
        return {
            r * INV_S3 * (-w.w1          + w.w3),
            r * (1.f/3.f*w.w1 - 2.f/3.f*w.w2 + 1.f/3.f*w.w3),
            r / (3.f * L) * (w.w1 + w.w2 + w.w3)
        };
    }

    /** Chuyển vận tốc góc bánh → tần số bước + chiều quay */
    static void toStep(float omega_rads, int steps_rev,
                       float& out_hz, bool& out_fwd) {
        out_fwd = (omega_rads >= 0.f);
        out_hz  = fabsf(omega_rads) / (2.f * (float)M_PI) * (float)steps_rev;
    }

private:
    float r, L;
    static constexpr float S3H    = 0.8660254f;   // √3/2
    static constexpr float INV_S3 = 0.5773503f;   // 1/√3
};
