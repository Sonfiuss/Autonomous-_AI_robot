#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "Config.hpp"
#include "Geometry.hpp"

/**
 * Reconstruct world points from one depth frame captured at (pan, tilt).
 * Combines Depth validity + Geometry (unproject -> remap -> rotate -> world).
 *
 * Pipeline (plan §5 / activity diagram §5):
 *   validity mask -> edge-column trim -> column decimation -> unproject ->
 *   remap axes -> + lever arm -> rotate into world.
 *
 * Pure / hardware-free; depth is CV_32F in mm.
 */
namespace recon {

// If `mask` is provided (CV_8U, 255=keep) it overrides the built-in z-range
// validity test; otherwise a pixel is valid when finite, >0, in [z_min,z_max].
std::vector<geom::Vec3> reconstruct(const cv::Mat& depth_mm,
                                    float pan_deg, float tilt_deg,
                                    const Config& c,
                                    const cv::Mat* mask = nullptr);

} // namespace recon
