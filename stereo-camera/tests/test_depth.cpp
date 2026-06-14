/**
 * test_depth — implement_plan_stereo.md §8 table (D1–D3).
 */
#include "test_util.hpp"
#include "../core/Depth.hpp"
#include <opencv2/opencv.hpp>
#include <limits>

static const float NaNf = std::numeric_limits<float>::quiet_NaN();
static const float Inf  = std::numeric_limits<float>::infinity();

static void D1_constant_disparity() {
    section("D1 constant disparity d -> Z = fx·B/d everywhere");
    const float fx = 600.f, B = 60.f, d = 12.f;
    cv::Mat disp(8, 8, CV_32F, cv::Scalar(d));
    cv::Mat depth = depthutil::disparityToDepth(disp, B, fx);
    const float expected = fx * B / d;   // 3000 mm
    bool all_ok = true;
    for (int v = 0; v < depth.rows; ++v)
        for (int u = 0; u < depth.cols; ++u)
            if (std::fabs(depth.at<float>(v,u) - expected) > 1e-2f) all_ok = false;
    printf("  expected Z=%.1f mm\n", expected);
    CHECK(all_ok);
}

static void D2_invalid_masked() {
    section("D2 NaN/0/inf disparity -> masked out, mask count exact");
    const float fx = 600.f, B = 60.f;
    cv::Mat disp(1, 6, CV_32F);
    disp.at<float>(0,0) = 10.f;   // valid -> Z=3600
    disp.at<float>(0,1) = 0.f;    // invalid
    disp.at<float>(0,2) = NaNf;   // invalid
    disp.at<float>(0,3) = Inf;    // invalid (isfinite false)
    disp.at<float>(0,4) = -5.f;   // invalid (<=0)
    disp.at<float>(0,5) = 20.f;   // valid -> Z=1800
    cv::Mat depth = depthutil::disparityToDepth(disp, B, fx);
    cv::Mat mask = depthutil::validityMask(depth, 0.f, 100000.f);
    int valid = cv::countNonZero(mask);
    CHECK(valid == 2);
    CHECK(mask.at<uchar>(0,0) == 255);
    CHECK(mask.at<uchar>(0,5) == 255);
}

static void D3_range_clamp() {
    section("D3 depths outside [z_min,z_max] dropped");
    cv::Mat depth(1, 4, CV_32F);
    depth.at<float>(0,0) = 100.f;    // < z_min
    depth.at<float>(0,1) = 1000.f;   // in range
    depth.at<float>(0,2) = 5000.f;   // in range
    depth.at<float>(0,3) = 9000.f;   // > z_max
    cv::Mat mask = depthutil::validityMask(depth, 200.f, 6000.f);
    CHECK(cv::countNonZero(mask) == 2);
    CHECK(mask.at<uchar>(0,0) == 0);
    CHECK(mask.at<uchar>(0,1) == 255);
    CHECK(mask.at<uchar>(0,2) == 255);
    CHECK(mask.at<uchar>(0,3) == 0);
}

int main() {
    printf("=== test_depth ===\n");
    D1_constant_disparity();
    D2_invalid_masked();
    D3_range_clamp();
    return test_report("test_depth");
}
