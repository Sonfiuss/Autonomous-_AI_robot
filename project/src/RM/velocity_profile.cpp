#include "RM/velocity_profile.h"

#include <cmath>

#include "RM/rm_debug.h"

namespace rm {

// ============================================================ VelocityProfile

VelocityProfile::VelocityProfile(float accel, float decel, float maxOmega)
    : accel_(std::fabs(accel)), decel_(std::fabs(decel)), maxOmega_(std::fabs(maxOmega)), current_() {}

void VelocityProfile::reset(const WheelSpeeds& current) {
    current_ = current;
}

WheelSpeeds VelocityProfile::clampTarget(const WheelSpeeds& target) const {
    float maxAbs = 0.0f;
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        maxAbs = std::fmax(maxAbs, std::fabs(target.w[i]));
    }
    if (maxAbs <= maxOmega_ || maxAbs <= 0.0f) {
        return target;
    }
    const float scale = maxOmega_ / maxAbs;
    RM_DLOG("VelocityProfile: target %.3f rad/s over limit, scale %.3f\n", maxAbs, scale);
    WheelSpeeds out;
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        out.w[i] = target.w[i] * scale;
    }
    return out;
}

WheelSpeeds VelocityProfile::step(const WheelSpeeds& target, float dt) {
    if (dt < cfg::MIN_DT_S) {
        return current_;
    }
    const WheelSpeeds goal = clampTarget(target);

    // ratio = how many times the fastest wheel would exceed its own limit this tick.
    float ratio = 1.0f;
    float delta[cfg::NUM_WHEELS];
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        delta[i] = goal.w[i] - current_.w[i];
        // Slowing down (toward zero or through zero) uses the decel limit.
        const bool braking = std::fabs(goal.w[i]) < std::fabs(current_.w[i]) ||
                             (goal.w[i] * current_.w[i] < 0.0f);
        const float limit = (braking ? decel_ : accel_) * dt;
        if (limit > 0.0f) {
            ratio = std::fmax(ratio, std::fabs(delta[i]) / limit);
        }
    }
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        current_.w[i] += delta[i] / ratio;
    }
    return current_;
}

bool VelocityProfile::atTarget(const WheelSpeeds& target) const {
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        if (std::fabs(target.w[i] - current_.w[i]) > cfg::OMEGA_TOLERANCE_RAD_S) {
            return false;
        }
    }
    return true;
}

// ========================================================= TrapezoidalProfile

TrapezoidalProfile::TrapezoidalProfile()
    : sign_(1.0f), vPeak_(0.0f), accel_(0.0f), decel_(0.0f), tAccel_(0.0f), tHold_(0.0f), tDecel_(0.0f) {}

bool TrapezoidalProfile::plan(float distance, float vMax, float accel, float decel) {
    sign_   = distance < 0.0f ? -1.0f : 1.0f;
    vPeak_  = 0.0f;
    accel_  = std::fabs(accel);
    decel_  = std::fabs(decel);
    tAccel_ = tHold_ = tDecel_ = 0.0f;

    const float length = std::fabs(distance);
    vMax               = std::fabs(vMax);
    if (length < cfg::MIN_PROFILE_LENGTH || vMax <= 0.0f || accel_ <= 0.0f || decel_ <= 0.0f) {
        RM_DLOG("TrapezoidalProfile: invalid plan (len %.4f vMax %.3f a %.3f d %.3f)\n",
                length, vMax, accel_, decel_);
        return false;
    }

    // Distance needed to reach vMax and to brake from it.
    const float sAccel = vMax * vMax / (2.0f * accel_);
    const float sDecel = vMax * vMax / (2.0f * decel_);

    if (sAccel + sDecel >= length) {
        // Triangle: ramp up then straight into braking, never reaching vMax.
        // s = v²/(2a) + v²/(2d)  →  v = sqrt(2·s·a·d / (a + d))
        vPeak_ = std::sqrt(2.0f * length * accel_ * decel_ / (accel_ + decel_));
        tHold_ = 0.0f;
    } else {
        vPeak_ = vMax;
        tHold_ = (length - sAccel - sDecel) / vMax;
    }
    tAccel_ = vPeak_ / accel_;
    tDecel_ = vPeak_ / decel_;
    RM_DLOG("TrapezoidalProfile: len %.4f vPeak %.3f t=[%.3f %.3f %.3f]\n",
            length, vPeak_, tAccel_, tHold_, tDecel_);
    return true;
}

float TrapezoidalProfile::velocityAt(float t) const {
    if (t <= 0.0f || vPeak_ <= 0.0f) {
        return 0.0f;
    }
    float v;
    if (t < tAccel_) {
        v = accel_ * t;
    } else if (t < tAccel_ + tHold_) {
        v = vPeak_;
    } else if (t < duration()) {
        v = vPeak_ - decel_ * (t - tAccel_ - tHold_);
    } else {
        v = 0.0f;
    }
    return sign_ * std::fmax(v, 0.0f);
}

float TrapezoidalProfile::positionAt(float t) const {
    if (t <= 0.0f || vPeak_ <= 0.0f) {
        return 0.0f;
    }
    const float sAccel = 0.5f * accel_ * tAccel_ * tAccel_;
    const float sHold  = vPeak_ * tHold_;
    const float sDecel = 0.5f * decel_ * tDecel_ * tDecel_;
    float s;
    if (t < tAccel_) {
        s = 0.5f * accel_ * t * t;
    } else if (t < tAccel_ + tHold_) {
        s = sAccel + vPeak_ * (t - tAccel_);
    } else if (t < duration()) {
        const float td = t - tAccel_ - tHold_;
        s              = sAccel + sHold + vPeak_ * td - 0.5f * decel_ * td * td;
    } else {
        s = sAccel + sHold + sDecel;
    }
    return sign_ * s;
}

}  // namespace rm
