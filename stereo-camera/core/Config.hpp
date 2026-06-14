#pragma once

#include <array>
#include <string>

/**
 * ============================================================================
 *  COORDINATE & MATH CONTRACT  (implement_plan_stereo.md §3 — never deviate)
 * ============================================================================
 *
 *  Units: ALL lengths are in MILLIMETRES (mm). Angles in DEGREES at the API
 *  boundary, radians internally.
 *
 *  Body / world axes:   X = forward,  Y = right,  Z = up.
 *    (This triple is LEFT-handed. We do NOT hand it to a stock right-handed
 *     rotation library; Geometry.cpp builds R by hand to match this contract.
 *     handedness = "left".)
 *
 *  Depth Z: distance along the optical axis (what stereo gives), NOT euclidean
 *  range.
 *
 *  Per-pixel unprojection (OpenCV camera frame, x right / y down / z forward):
 *      P_cam = K^-1 · [u, v, 1]^T · Z
 *            = ( (u-cx)/fx · Z,  (v-cy)/fy · Z,  Z )
 *
 *  Axis remap  camera -> body:
 *      X_body = +z_cam      (forward)
 *      Y_body = +x_cam      (right)
 *      Z_body = -y_cam      (up;  camera y points down)
 *
 *  Lever arm r: camera optical-centre position relative to the pan/tilt axis
 *  intersection, expressed in the body frame (mm).
 *
 *  Rotation order:  R(θ,φ) = R_pan(θ) · R_tilt(φ)   ("pan_then_tilt").
 *      R_pan  rotates about Z (up);    +pan  turns forward toward +Y (right).
 *      R_tilt rotates about Y (right); +tilt turns forward toward +Z (up).
 *
 *  Full transform:  P_world = R(θ,φ) · (r + P_body)
 *
 *  Validation (Geometry test G1–G3), centre pixel, Z=1000, r=0:
 *      θ=0,  φ=0   -> (1000,    0,    0)   forward
 *      θ=90, φ=0   -> (   0, 1000,    0)   right
 *      θ=0,  φ=90  -> (   0,    0, 1000)   up
 *
 *  Do NOT use the spherical form X = d·cosφ·cosθ … — that is only valid for a
 *  single-spot rangefinder and double-counts the angle on a depth map.
 * ============================================================================
 */
struct Config {
    // ── Intrinsics (pixels) ───────────────────────────────────────────────
    float fx = 600.f;
    float fy = 600.f;
    float cx = 320.f;
    float cy = 240.f;

    // ── Stereo ────────────────────────────────────────────────────────────
    float baseline = 60.f;          // stereo baseline (mm)

    // ── Geometry ──────────────────────────────────────────────────────────
    std::array<float, 3> lever_arm = {0.f, 0.f, 0.f};  // r = (rx,ry,rz) body mm
    std::string rotation_order = "pan_then_tilt";       // R = R_pan · R_tilt
    std::string handedness     = "left";                // body frame is left-handed

    // ── Sync ──────────────────────────────────────────────────────────────
    double t_offset_ms            = 0.0;   // camera-clock minus angle-clock (ms)
    double angle_gap_threshold_ms = 20.0;  // reject frames whose nearest angle is farther

    // ── Depth validity ────────────────────────────────────────────────────
    float z_min = 200.f;            // mm — drop closer than this
    float z_max = 6000.f;           // mm — drop farther (stereo noise ~Z² explodes)

    // ── Density / accumulation ────────────────────────────────────────────
    int   column_decimation = 9;    // keep every Nth column (≈ width/72 ~ 1 pt/deg)
    float edge_trim_frac    = 0.07f;// drop this fraction of outer columns per frame
    float voxel_size        = 5.f;  // global voxel grid spacing (mm) = target spacing
    size_t max_points       = 20000000; // cloud-size guard

    // ── Continuous-sweep gate ─────────────────────────────────────────────
    bool shutter_synced = false;    // require global-shutter + genlock for continuous mode
};
