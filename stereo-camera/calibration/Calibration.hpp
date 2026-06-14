#pragma once

#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include "../core/Config.hpp"

/**
 * Calibration routines (plan §8/§9). Hardware-free given recorded fixtures.
 *
 *   estimateLeverArm : flat-wall bow minimization -> lever arm r (mm).
 *   estimateTOffset  : temporal camera/angle offset (delegates to core/Sync).
 *   handEyeOffset    : optical-axis vs mechanical-zero offset (scaffold; needs
 *                      checkerboard detections — documented below).
 */
namespace calib {

struct WallFrame {
    float   pan_deg;
    float   tilt_deg;
    cv::Mat depth_mm;   // CV_32F
};

// Planarity RMS (mm) of the merged cloud reconstructed from `frames` using the
// given config (the objective minimized by lever-arm calibration).
float mergedPlanarityRMS(const std::vector<WallFrame>& frames, const Config& cfg);

// Coordinate-descent search for the lever arm r that minimizes the wall's bow
// across pan angles. `base` supplies intrinsics etc.; r is searched in a box of
// ±search_mm around base.lever_arm. Returns the optimized config (lever_arm set).
Config estimateLeverArm(const std::vector<WallFrame>& frames, const Config& base,
                        float search_mm = 100.f, float tol_mm = 0.5f);

/**
 * hand_eye scaffold. Real implementation: image a fixed checkerboard at several
 * known firmware angles, solve for the constant rotation between the camera
 * frame and the firmware zero (AX = XB). Requires per-pose board poses, so it is
 * left as a documented entry point returning identity (no offset) for now.
 */
std::array<float, 3> handEyeOffset();

} // namespace calib
