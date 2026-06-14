#include "Reconstruct.hpp"
#include <cmath>
#include <algorithm>

namespace recon {

std::vector<geom::Vec3> reconstruct(const cv::Mat& depth, float pan, float tilt,
                                    const Config& c, const cv::Mat* mask) {
    CV_Assert(depth.type() == CV_32F);
    if (mask) CV_Assert(mask->type() == CV_8U && mask->size() == depth.size());

    const int H = depth.rows;
    const int W = depth.cols;

    // Edge-column trim (plan §7): drop outer edge_trim_frac on each side.
    int trim = (int)std::lround(c.edge_trim_frac * W);
    trim = std::max(0, std::min(trim, W / 2 - 1 < 0 ? 0 : W / 2 - 1));
    const int u0 = trim;
    const int u1 = W - trim;

    const int step = std::max(1, c.column_decimation);

    const geom::Mat3 R = geom::rotation(pan, tilt, c);

    std::vector<geom::Vec3> out;
    out.reserve(((u1 - u0) / step + 1) * H);

    for (int v = 0; v < H; ++v) {
        const float* zrow = depth.ptr<float>(v);
        const uchar* mrow = mask ? mask->ptr<uchar>(v) : nullptr;
        for (int u = u0; u < u1; u += step) {
            const float Z = zrow[u];
            bool ok;
            if (mrow) ok = mrow[u] != 0;
            else      ok = std::isfinite(Z) && Z > 0.f && Z >= c.z_min && Z <= c.z_max;
            if (!ok) continue;
            out.push_back(geom::pixelToWorld((float)u, (float)v, Z, R, c));
        }
    }
    return out;
}

} // namespace recon
