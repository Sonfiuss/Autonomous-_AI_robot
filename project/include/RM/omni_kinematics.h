// Omni_Kinematics — body velocity <-> wheel angular velocity for a 3-wheel
// omni base (paper Eq. 16 in general matrix form). Pure math, no hardware.
#ifndef RM_OMNI_KINEMATICS_H
#define RM_OMNI_KINEMATICS_H

#include "types.h"

namespace rm {

class OmniKinematics {
public:
    // Builds the inverse-kinematics matrix from cfg::WHEEL_ANGLE_RAD and its
    // 3x3 inverse for forward kinematics. Check valid() once after construction.
    OmniKinematics();

    // Inverse kinematics: body [u, v, r] -> wheel [ω1, ω2, ω3] (rad/s).
    // Linear, so it also maps body displacement (m, rad) -> wheel angle (rad).
    WheelSpeeds inverse(const BodyVel& body) const;

    // Forward kinematics: wheel [ω1, ω2, ω3] -> body [u, v, r].
    // Same linear map applies to wheel angle deltas -> body displacement.
    BodyVel forward(const WheelSpeeds& wheels) const;

    // Rotates a world-frame velocity into the body frame using heading theta (rad).
    static BodyVel globalToBody(const GlobalVel& global, float theta);

    // Rotates a body-frame velocity into the world frame using heading theta (rad).
    static GlobalVel bodyToGlobal(const BodyVel& body, float theta);

    // False if the wheel layout in constants.h produced a singular matrix.
    bool valid() const { return valid_; }

private:
    static bool invert3x3(const float m[3][3], float out[3][3]);

    float ik_[3][3];  // rows: wheel i; cols: u, v, r
    float fk_[3][3];  // inverse of ik_
    bool  valid_;
};

}  // namespace rm

#endif  // RM_OMNI_KINEMATICS_H
