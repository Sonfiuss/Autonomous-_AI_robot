/**
 * test_reconstruct — implement_plan_stereo.md §8 table (R1, R2).
 */
#include "test_util.hpp"
#include "../core/Reconstruct.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>

static void R1_decimation() {
    section("R1 decimation=9 -> ≈ (trimmed width)/9 columns kept");
    Config c;
    c.column_decimation = 9;
    c.edge_trim_frac = 0.f;        // isolate decimation
    c.z_min = 0.f; c.z_max = 100000.f;
    const int W = 640, H = 1;      // single row to count columns
    cv::Mat depth(H, W, CV_32F, cv::Scalar(2000.f));
    auto pts = recon::reconstruct(depth, 0.f, 0.f, c);
    int expected = (W + 8) / 9;    // ceil(640/9) = 72
    printf("  kept %zu cols, expected ≈ %d\n", pts.size(), expected);
    CHECK(std::abs((int)pts.size() - expected) <= 1);
}

static void R2_known_plane() {
    section("R2 fronto-parallel plane Z=2000 -> reconstructed X≈2000 (normal=+X)");
    Config c;
    c.column_decimation = 4;
    c.edge_trim_frac = 0.05f;
    c.z_min = 0.f; c.z_max = 100000.f;
    cv::Mat depth(120, 160, CV_32F, cv::Scalar(2000.f));
    auto pts = recon::reconstruct(depth, 0.f, 0.f, c);
    CHECK(!pts.empty());
    float maxerr = 0.f;
    for (auto& p : pts) maxerr = std::max(maxerr, std::fabs(p[0] - 2000.f));
    printf("  max |X-2000| = %.4f mm\n", maxerr);
    CHECK(maxerr < 1.0f);          // plane offset within tolerance
}

static void R_edge_trim() {
    section("edge_trim_frac drops outer columns");
    Config c;
    c.column_decimation = 1;
    c.edge_trim_frac = 0.10f;      // drop 10% each side -> keep middle 80%
    c.z_min = 0.f; c.z_max = 100000.f;
    cv::Mat depth(1, 100, CV_32F, cv::Scalar(2000.f));
    auto pts = recon::reconstruct(depth, 0.f, 0.f, c);
    // trim=10 each side -> columns [10,90) = 80 kept
    CHECK(pts.size() == 80u);
}

int main() {
    printf("=== test_reconstruct ===\n");
    R1_decimation();
    R2_known_plane();
    R_edge_trim();
    return test_report("test_reconstruct");
}
