#include "Depth.hpp"
#include <limits>
#include <cmath>

namespace depthutil {

static const float NaNf = std::numeric_limits<float>::quiet_NaN();

cv::Mat disparityToDepth(const cv::Mat& disp, float baseline_mm, float fx) {
    CV_Assert(disp.type() == CV_32F);
    cv::Mat depth(disp.size(), CV_32F);
    const float fxb = fx * baseline_mm;
    for (int v = 0; v < disp.rows; ++v) {
        const float* d = disp.ptr<float>(v);
        float* z = depth.ptr<float>(v);
        for (int u = 0; u < disp.cols; ++u) {
            const float dv = d[u];
            z[u] = (std::isfinite(dv) && dv > 0.f) ? (fxb / dv) : NaNf;
        }
    }
    return depth;
}

cv::Mat validityMask(const cv::Mat& depth, float z_min, float z_max) {
    CV_Assert(depth.type() == CV_32F);
    cv::Mat mask(depth.size(), CV_8U);
    for (int v = 0; v < depth.rows; ++v) {
        const float* z = depth.ptr<float>(v);
        uchar* m = mask.ptr<uchar>(v);
        for (int u = 0; u < depth.cols; ++u) {
            const float zv = z[u];
            m[u] = (std::isfinite(zv) && zv > 0.f && zv >= z_min && zv <= z_max)
                   ? 255 : 0;
        }
    }
    return mask;
}

void applyMask(cv::Mat& depth, const cv::Mat& mask) {
    CV_Assert(depth.size() == mask.size());
    depth.setTo(NaNf, mask == 0);
}

} // namespace depthutil
