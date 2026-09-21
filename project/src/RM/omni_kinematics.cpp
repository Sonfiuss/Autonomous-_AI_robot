#include "RM/omni_kinematics.h"

#include <cmath>

#include "RM/rm_debug.h"

namespace rm {

namespace {

constexpr int DIM = 3;

}  // namespace

OmniKinematics::OmniKinematics() : ik_{}, fk_{}, valid_(false) {
    // Paper Eq. 16 generalised: ω_i = (-sin(α_i)·u + cos(α_i)·v + L·r) / a
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        const float alpha = cfg::WHEEL_ANGLE_RAD[i];
        ik_[i][0] = -std::sin(alpha) / cfg::WHEEL_RADIUS_M;
        ik_[i][1] = std::cos(alpha) / cfg::WHEEL_RADIUS_M;
        ik_[i][2] = cfg::ROBOT_RADIUS_M / cfg::WHEEL_RADIUS_M;
    }
    valid_ = invert3x3(ik_, fk_);
    RM_DLOG("OmniKinematics: matrix %s\n", valid_ ? "ok" : "SINGULAR");
}

WheelSpeeds OmniKinematics::inverse(const BodyVel& body) const {
    WheelSpeeds out;
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        out.w[i] = ik_[i][0] * body.u + ik_[i][1] * body.v + ik_[i][2] * body.r;
    }
    return out;
}

BodyVel OmniKinematics::forward(const WheelSpeeds& wheels) const {
    BodyVel out;
    if (!valid_) {
        return out;
    }
    out.u = fk_[0][0] * wheels.w[0] + fk_[0][1] * wheels.w[1] + fk_[0][2] * wheels.w[2];
    out.v = fk_[1][0] * wheels.w[0] + fk_[1][1] * wheels.w[1] + fk_[1][2] * wheels.w[2];
    out.r = fk_[2][0] * wheels.w[0] + fk_[2][1] * wheels.w[1] + fk_[2][2] * wheels.w[2];
    return out;
}

BodyVel OmniKinematics::globalToBody(const GlobalVel& global, float theta) {
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    BodyVel out;
    out.u = c * global.vx + s * global.vy;
    out.v = -s * global.vx + c * global.vy;
    out.r = global.wz;
    return out;
}

GlobalVel OmniKinematics::bodyToGlobal(const BodyVel& body, float theta) {
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    GlobalVel out;
    out.vx = c * body.u - s * body.v;
    out.vy = s * body.u + c * body.v;
    out.wz = body.r;
    return out;
}

// Cofactor (adjugate) inverse of a 3x3 matrix. Returns false when singular.
bool OmniKinematics::invert3x3(const float m[3][3], float out[3][3]) {
    const float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const float det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (std::fabs(det) < cfg::MIN_DETERMINANT) {
        return false;
    }
    const float invDet = 1.0f / det;
    out[0][0] = c00 * invDet;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invDet;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
    out[1][0] = c01 * invDet;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * invDet;
    out[2][0] = c02 * invDet;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * invDet;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
    for (int i = 0; i < DIM; ++i) {
        for (int j = 0; j < DIM; ++j) {
            if (std::isnan(out[i][j]) || std::isinf(out[i][j])) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace rm
