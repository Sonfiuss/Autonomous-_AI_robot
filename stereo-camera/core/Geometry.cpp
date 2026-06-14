#include "Geometry.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geom {

static constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);

// R_pan: rotation about Z (up). +pan turns forward (+X) toward +Y (right).
//   X' = X cosθ - Y sinθ
//   Y' = X sinθ + Y cosθ
//   Z' = Z
static Mat3 R_pan(float th) {
    const float ct = std::cos(th), st = std::sin(th);
    return { ct, -st, 0.f,
             st,  ct, 0.f,
             0.f, 0.f, 1.f };
}

// R_tilt: rotation about Y (right). +tilt turns forward (+X) toward +Z (up).
// Built by hand so that (1,0,0) at φ=90° -> (0,0,1) under the left-handed body
// contract (standard right-handed-about-Y would give -Z).
//   X' = X cosφ - Z sinφ
//   Y' = Y
//   Z' = X sinφ + Z cosφ
static Mat3 R_tilt(float ph) {
    const float cf = std::cos(ph), sf = std::sin(ph);
    return { cf,  0.f, -sf,
             0.f, 1.f,  0.f,
             sf,  0.f,  cf };
}

static Mat3 matmul(const Mat3& A, const Mat3& B) {
    Mat3 C{};
    for (int r = 0; r < 3; ++r)
        for (int col = 0; col < 3; ++col)
            for (int k = 0; k < 3; ++k)
                C[r*3 + col] += A[r*3 + k] * B[k*3 + col];
    return C;
}

Mat3 rotation(float pan_deg, float tilt_deg, const Config& c) {
    const Mat3 Rp = R_pan(pan_deg  * DEG2RAD);
    const Mat3 Rt = R_tilt(tilt_deg * DEG2RAD);
    // Default contract: R = R_pan · R_tilt  ("pan_then_tilt").
    if (c.rotation_order == "tilt_then_pan")
        return matmul(Rt, Rp);
    return matmul(Rp, Rt);
}

} // namespace geom
