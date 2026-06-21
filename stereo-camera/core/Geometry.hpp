#pragma once

#include <array>
#include <vector>
#include "Config.hpp"

/**
 * Pure geometry functions implementing the coordinate contract in Config.hpp.
 * No OpenCV dependency, no pixel loops in the hot path (callers vectorize over
 * arrays). Angles passed in DEGREES; converted internally.
 *
 * This module REPLACES the spherical projection that used to live in
 * MapBuilder::addFrame (forbidden by implement_plan_stereo.md §3).
 */
namespace geom {

using Vec3 = std::array<float, 3>;   // {x, y, z}
using Mat3 = std::array<float, 9>;   // row-major 3x3

// ── Per-pixel unprojection (OpenCV camera frame) ──────────────────────────
// P_cam = K^-1 · [u,v,1]^T · Z  ->  ((u-cx)/fx·Z, (v-cy)/fy·Z, Z)
inline Vec3 unproject(float u, float v, float Z, const Config& c) {
    return { (u - c.cx) / c.fx * Z,
             (v - c.cy) / c.fy * Z,
             Z };
}

// ── Axis remap: OpenCV camera (x right, y down, z fwd) -> body (X fwd, Y right, Z up) ──
//   X_body = +z_cam,  Y_body = +x_cam,  Z_body = -y_cam
inline Vec3 remapAxes(const Vec3& cam) {
    return { cam[2], cam[0], -cam[1] };
}

// ── Rotation R(pan,tilt) honouring config.rotation_order ──────────────────
// Built by hand to match the left-handed body contract (see Config.hpp).
Mat3 rotation(float pan_deg, float tilt_deg, const Config& c);

// ── Mat3 · Vec3 ───────────────────────────────────────────────────────────
inline Vec3 apply(const Mat3& R, const Vec3& p) {
    return { R[0]*p[0] + R[1]*p[1] + R[2]*p[2],
             R[3]*p[0] + R[4]*p[1] + R[5]*p[2],
             R[6]*p[0] + R[7]*p[1] + R[8]*p[2] };
}

// ── Full transform for one body-frame point: P_world = R · (r + P_body) ───
inline Vec3 toWorld(const Vec3& p_body, const Mat3& R, const Config& c) {
    Vec3 shifted = { p_body[0] + c.lever_arm[0],
                     p_body[1] + c.lever_arm[1],
                     p_body[2] + c.lever_arm[2] };
    return geom::apply(R, shifted);
}

// Convenience: pixel + depth -> world point at given pan/tilt.
inline Vec3 pixelToWorld(float u, float v, float Z,
                         const Mat3& R, const Config& c) {
    return toWorld(remapAxes(unproject(u, v, Z, c)), R, c);
}

} // namespace geom
