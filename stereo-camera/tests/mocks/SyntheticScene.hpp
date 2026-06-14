#pragma once
/**
 * Offline fixtures replacing hardware (plan §4 MockStereoCapture / MockFirmware).
 *
 * syntheticWallDepth: returns the CV_32F depth map (mm) a camera at pan `θ`
 * (tilt 0) would measure of a flat wall fixed at world X = D. Derived from the
 * reconstruction contract: a pixel with normalised x a=(u-cx)/fx maps to a world
 * point whose X equals  Z·(cosθ - a·sinθ);  setting that to D gives
 *     Z(u) = D / (cosθ - a·sinθ).
 * Feeding these depths back through reconstruct() must return points on X = D
 * for every pose — the basis of the end-to-end planarity check (E1).
 *
 * MockFirmware: trivial angle source returning the commanded pose, so the
 * pipeline runs without an ESP32.
 */
#include <opencv2/opencv.hpp>
#include <cmath>
#include <limits>
#include "../../core/Config.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mockscene {

inline cv::Mat syntheticWallDepth(float D, float pan_deg, const Config& c,
                                  int W = 320, int H = 240) {
    const float th = pan_deg * (float)(M_PI / 180.0);
    const float ct = std::cos(th), st = std::sin(th);
    cv::Mat depth(H, W, CV_32F);
    for (int v = 0; v < H; ++v) {
        float* row = depth.ptr<float>(v);
        for (int u = 0; u < W; ++u) {
            const float a = ((float)u - c.cx) / c.fx;
            const float denom = ct - a * st;
            row[u] = (denom > 1e-3f) ? (D / denom)
                                     : std::numeric_limits<float>::quiet_NaN();
        }
    }
    return depth;
}

struct MockFirmware {
    float pan = 0.f, tilt = 0.f;
    void moveTo(float p, float t) { pan = p; tilt = t; }
    void readAngle(float& p, float& t) const { p = pan; t = tilt; }
};

} // namespace mockscene
