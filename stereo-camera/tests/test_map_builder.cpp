/**
 * test_map_builder — MapBuilder with the new plan convention
 * (X fwd, Y right, Z up; mm; proper unprojection, voxel dedup).
 * Geometry validation proper lives in test_geometry / test_reconstruct.
 */
#include "test_util.hpp"
#include "../vision/MapBuilder.hpp"
#include <opencv2/opencv.hpp>
#include <limits>

static Config wide() {
    Config c;
    c.column_decimation = 1;
    c.edge_trim_frac = 0.f;
    c.z_min = 0.f; c.z_max = 100000.f;
    c.voxel_size = 1.f;
    return c;
}

static cv::Mat uniform(int r, int col, float d) {
    return cv::Mat(r, col, CV_32F, cv::Scalar(d));
}

static void empty_zero() {
    section("empty MapBuilder has 0 points");
    MapBuilder b(wide());
    CHECK(b.pointCount() == 0u);
}

static void all_nan_zero() {
    section("all-NaN depth contributes 0 points");
    MapBuilder b(wide());
    cv::Mat m(4, 4, CV_32F, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
    b.addFrame(0.f, 0.f, m);
    CHECK(b.pointCount() == 0u);
}

static void centre_pixel_forward() {
    section("centre pixel at pan=0 tilt=0 -> X≈depth, Y≈0, Z≈0 (forward)");
    Config c = wide();
    c.cx = 0.5f; c.cy = 0.5f;          // 2x2 image: centre between pixels
    MapBuilder b(c);
    cv::Mat m = uniform(2, 2, 1000.f);
    b.addFrame(0.f, 0.f, m);
    cv::Mat cloud = b.getCloud();
    CHECK(cloud.rows == 4);
    // all points share Z (forward) ≈ 1000
    bool fwd = true;
    for (int i = 0; i < cloud.rows; ++i)
        if (std::fabs(cloud.at<float>(i,0) - 1000.f) > 1e-2f) fwd = false;
    CHECK(fwd);
}

static void clear_resets() {
    section("clear() resets point count");
    MapBuilder b(wide());
    b.addFrame(0.f, 0.f, uniform(3, 3, 1000.f));
    CHECK(b.pointCount() > 0u);
    b.clear();
    CHECK(b.pointCount() == 0u);
}

static void savePLY_header() {
    section("savePLY writes valid header with vertex count");
    MapBuilder b(wide());
    b.addFrame(0.f, 0.f, uniform(2, 2, 1000.f));
    size_t n = b.pointCount();
    const std::string path = "test_mb_out.ply";
    CHECK(b.savePLY(path));
    std::FILE* f = std::fopen(path.c_str(), "r");
    CHECK(f != nullptr);
    if (f) {
        char line[256]; size_t pn = 0; bool ok=false;
        while (std::fgets(line, sizeof(line), f))
            if (std::sscanf(line, "element vertex %zu", &pn) == 1) { ok=true; break; }
        std::fclose(f);
        CHECK(ok);
        CHECK(pn == n);
        std::remove(path.c_str());
    }
}

int main() {
    printf("=== test_map_builder ===\n");
    empty_zero();
    all_nan_zero();
    centre_pixel_forward();
    clear_resets();
    savePLY_header();
    return test_report("test_map_builder");
}
