#include "Calibration.hpp"
#include "../core/Reconstruct.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace calib {

// Plane-fit RMS via PCA: smallest-eigenvalue direction of the covariance.
static float planeRMS(const std::vector<geom::Vec3>& pts) {
    if (pts.size() < 3) return 0.f;
    double c[3] = {0,0,0};
    for (auto& p : pts) { c[0]+=p[0]; c[1]+=p[1]; c[2]+=p[2]; }
    double n = (double)pts.size();
    c[0]/=n; c[1]/=n; c[2]/=n;
    double cov[3][3] = {{0}};
    for (auto& p : pts) {
        double d[3] = {p[0]-c[0], p[1]-c[1], p[2]-c[2]};
        for (int i=0;i<3;++i) for(int j=0;j<3;++j) cov[i][j]+=d[i]*d[j];
    }
    for (int i=0;i<3;++i) for(int j=0;j<3;++j) cov[i][j]/=n;
    // smallest eigenvalue of symmetric 3x3 via OpenCV
    cv::Matx33d C(cov[0][0],cov[0][1],cov[0][2],
                  cov[1][0],cov[1][1],cov[1][2],
                  cov[2][0],cov[2][1],cov[2][2]);
    cv::Vec3d eval; cv::Matx33d evec;
    cv::eigen(C, eval, evec);
    double lmin = std::min(eval[0], std::min(eval[1], eval[2]));
    if (lmin < 0) lmin = 0;
    return (float)std::sqrt(lmin);
}

float mergedPlanarityRMS(const std::vector<WallFrame>& frames, const Config& cfg) {
    std::vector<geom::Vec3> merged;
    for (const auto& f : frames) {
        auto pts = recon::reconstruct(f.depth_mm, f.pan_deg, f.tilt_deg, cfg);
        merged.insert(merged.end(), pts.begin(), pts.end());
    }
    return planeRMS(merged);
}

Config estimateLeverArm(const std::vector<WallFrame>& frames, const Config& base,
                        float search_mm, float tol_mm) {
    Config best = base;
    float best_rms = mergedPlanarityRMS(frames, best);

    // Coordinate descent with shrinking step on each of rx, ry, rz.
    float step = search_mm;
    while (step >= tol_mm) {
        bool improved = false;
        for (int axis = 0; axis < 3; ++axis) {
            for (float delta : {step, -step}) {
                Config cand = best;
                cand.lever_arm[axis] += delta;
                float rms = mergedPlanarityRMS(frames, cand);
                if (rms < best_rms - 1e-6f) {
                    best_rms = rms;
                    best = cand;
                    improved = true;
                }
            }
        }
        if (!improved) step *= 0.5f;
    }
    printf("[calib] lever arm r = (%.2f, %.2f, %.2f) mm  planarity RMS=%.3f mm\n",
           best.lever_arm[0], best.lever_arm[1], best.lever_arm[2], best_rms);
    return best;
}

std::array<float, 3> handEyeOffset() {
    // Scaffold — see header. Returns no offset until checkerboard solve is wired.
    return {0.f, 0.f, 0.f};
}

} // namespace calib
