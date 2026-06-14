#pragma once

#include <opencv2/opencv.hpp>
#include "Config.hpp"

/**
 * Depth utilities — pure, hardware-free, unit-testable.
 *
 *   disparityToDepth : SGBM disparity (px, CV_32F) -> depth Z (mm, CV_32F)
 *                      using  Z = fx · baseline / disparity.
 *   validityMask     : 8-bit mask (255=keep) dropping NaN/inf/non-positive
 *                      disparity and depths outside [z_min, z_max].
 *
 * Both are used by StereoCamera and exercised directly by test_depth.
 */
namespace depthutil {

// Z = fx * baseline / disparity. Pixels with disparity <= 0 (or non-finite)
// become NaN so validityMask drops them.
cv::Mat disparityToDepth(const cv::Mat& disparity32f, float baseline_mm, float fx);

// 255 where depth is finite, > 0, and within [z_min, z_max]; else 0.
cv::Mat validityMask(const cv::Mat& depth32f, float z_min, float z_max);

// Convenience: apply mask to depth, setting rejected pixels to NaN (in place).
void applyMask(cv::Mat& depth32f, const cv::Mat& mask);

} // namespace depthutil
