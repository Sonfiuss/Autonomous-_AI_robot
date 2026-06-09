/**
 * Unit tests for MapBuilder::addFrame and coordinate projection.
 *
 * Run: ./test_map_builder
 * Expected: all tests PASS, exit code 0.
 */

#include <cstdio>
#include <cmath>
#include <thread>
#include <vector>
#include <limits>

#include <opencv2/opencv.hpp>

#include "../vision/MapBuilder.hpp"

// ── Minimal test harness ──────────────────────────────────────────────────────

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(expr) do { \
    if (expr) { \
        printf("  PASS  %s\n", #expr); \
        ++s_pass; \
    } else { \
        printf("  FAIL  %s  (line %d)\n", #expr, __LINE__); \
        ++s_fail; \
    } \
} while(0)

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs(static_cast<double>(a) - static_cast<double>(b)) < (tol))

static void section(const char* name) { printf("\n── %s\n", name); }

// ── Helpers ───────────────────────────────────────────────────────────────────

// Create a 1×1 depth map with a single value d at the image centre.
static cv::Mat single_pixel_depth(float d) {
    cv::Mat m(1, 1, CV_32F);
    m.at<float>(0, 0) = d;
    return m;
}

// Create a depth map where all pixels are NaN.
static cv::Mat all_nan_depth(int rows = 4, int cols = 4) {
    cv::Mat m(rows, cols, CV_32F,
              cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    return m;
}

// Create a depth map where all pixels equal d.
static cv::Mat uniform_depth(int rows, int cols, float d) {
    return cv::Mat(rows, cols, CV_32F, cv::Scalar(d));
}

// ── Tests ─────────────────────────────────────────────────────────────────────

static void test_empty_builder_zero_points() {
    section("empty MapBuilder has 0 points");
    MapBuilder b;
    CHECK(b.pointCount() == 0u);
}

static void test_all_nan_adds_no_points() {
    section("all-NaN depth map contributes 0 points");
    MapBuilder b;
    b.addFrame(0.f, 0.f, all_nan_depth());
    CHECK(b.pointCount() == 0u);
}

static void test_zero_depth_adds_no_points() {
    section("zero depth (≤ 0) is rejected");
    MapBuilder b;
    cv::Mat m(2, 2, CV_32F, cv::Scalar(0.f));
    b.addFrame(0.f, 0.f, m);
    CHECK(b.pointCount() == 0u);
}

static void test_single_pixel_at_origin_angles() {
    section("1×1 depth at pan=0 tilt=0 → Z ≈ depth");
    // At pan=0, tilt=0, alpha=0, beta=0:
    //   theta = 0, phi = 0
    //   X = d * cos(0) * sin(0) = 0
    //   Y = d * sin(0)          = 0
    //   Z = d * cos(0) * cos(0) = d
    MapBuilder b;
    const float d = 1.5f;
    b.addFrame(0.f, 0.f, single_pixel_depth(d));

    CHECK(b.pointCount() == 1u);
    cv::Mat cloud = b.getCloud();   // shape (1,3), CV_32F
    CHECK(cloud.rows == 1);

    float X = cloud.at<float>(0, 0);
    float Y = cloud.at<float>(0, 1);
    float Z = cloud.at<float>(0, 2);

    CHECK_NEAR(X, 0.f, 0.01f);
    CHECK_NEAR(Y, 0.f, 0.01f);
    CHECK_NEAR(Z, d,   0.01f);
}

static void test_pan_90_maps_to_positive_x() {
    section("1×1 depth at pan=90° tilt=0 → X ≈ depth, Z ≈ 0");
    // theta = 90°, phi = 0:
    //   X = d * cos(0) * sin(90°) = d
    //   Y = d * sin(0)            = 0
    //   Z = d * cos(0) * cos(90°) ≈ 0
    MapBuilder b;
    const float d = 2.0f;
    b.addFrame(90.f, 0.f, single_pixel_depth(d));

    cv::Mat cloud = b.getCloud();
    float X = cloud.at<float>(0, 0);
    float Z = cloud.at<float>(0, 2);

    CHECK_NEAR(X,  d,   0.02f);
    CHECK_NEAR(Z,  0.f, 0.02f);
}

static void test_tilt_90_maps_to_positive_y() {
    section("1×1 depth at pan=0 tilt=90° → Y ≈ depth");
    // phi = 90°:
    //   X = d * cos(90°) * sin(0) ≈ 0
    //   Y = d * sin(90°)           = d
    //   Z = d * cos(90°) * cos(0) ≈ 0
    MapBuilder b;
    const float d = 3.0f;
    b.addFrame(0.f, 90.f, single_pixel_depth(d));

    cv::Mat cloud = b.getCloud();
    float Y = cloud.at<float>(0, 1);
    CHECK_NEAR(Y, d, 0.02f);
}

static void test_multiple_frames_accumulate() {
    section("multiple addFrame calls accumulate points");
    MapBuilder b;
    const float d = 1.0f;
    b.addFrame(  0.f, 0.f, single_pixel_depth(d));
    b.addFrame( 45.f, 0.f, single_pixel_depth(d));
    b.addFrame(-45.f, 0.f, single_pixel_depth(d));
    CHECK(b.pointCount() == 3u);
}

static void test_uniform_depth_map_fills_all_pixels() {
    section("uniform 4×4 depth adds 16 valid points");
    MapBuilder b;
    cv::Mat m = uniform_depth(4, 4, 1.0f);
    b.addFrame(0.f, 0.f, m);
    CHECK(b.pointCount() == 16u);
}

static void test_clear_resets_point_count() {
    section("clear() resets point count to 0");
    MapBuilder b;
    b.addFrame(0.f, 0.f, uniform_depth(3, 3, 1.0f));
    CHECK(b.pointCount() == 9u);
    b.clear();
    CHECK(b.pointCount() == 0u);
}

static void test_concurrent_addframe_is_thread_safe() {
    section("concurrent addFrame from 4 threads is thread-safe");
    MapBuilder b;
    const int N_THREADS = 4;
    const int FRAMES_PER_THREAD = 10;

    std::vector<std::thread> threads;
    for (int t = 0; t < N_THREADS; ++t)
        threads.emplace_back([&, t]{
            for (int i = 0; i < FRAMES_PER_THREAD; ++i)
                b.addFrame(t * 10.f, 0.f, single_pixel_depth(1.0f));
        });
    for (auto& th : threads) th.join();

    // Each addFrame adds exactly 1 point (1×1 depth)
    CHECK(b.pointCount() == static_cast<size_t>(N_THREADS * FRAMES_PER_THREAD));
}

static void test_savePLY_produces_valid_header() {
    section("savePLY creates file with valid PLY header");
    MapBuilder b;
    b.addFrame(0.f, 0.f, uniform_depth(2, 2, 1.0f));   // 4 points

    const std::string path = "/tmp/test_mapbuilder_out.ply";
    CHECK(b.savePLY(path) == true);

    // Verify the file contains PLY magic and element count
    std::FILE* f = std::fopen(path.c_str(), "r");
    CHECK(f != nullptr);
    if (f) {
        char line[256];
        bool found_ply  = false;
        bool found_elem = false;
        while (std::fgets(line, sizeof(line), f)) {
            std::string s(line);
            if (s.find("ply") != std::string::npos)   found_ply  = true;
            if (s.find("element vertex 4") != std::string::npos) found_elem = true;
        }
        std::fclose(f);
        CHECK(found_ply);
        CHECK(found_elem);
        std::remove(path.c_str());
    }
}

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
    printf("=== test_map_builder ===\n");

    test_empty_builder_zero_points();
    test_all_nan_adds_no_points();
    test_zero_depth_adds_no_points();
    test_single_pixel_at_origin_angles();
    test_pan_90_maps_to_positive_x();
    test_tilt_90_maps_to_positive_y();
    test_multiple_frames_accumulate();
    test_uniform_depth_map_fills_all_pixels();
    test_clear_resets_point_count();
    test_concurrent_addframe_is_thread_safe();
    test_savePLY_produces_valid_header();

    printf("\n  %d passed  /  %d failed\n", s_pass, s_fail);
    return s_fail > 0 ? 1 : 0;
}
