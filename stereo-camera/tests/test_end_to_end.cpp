/**
 * test_end_to_end — implement_plan_stereo.md §8 (E1).
 * Mock stop-and-shoot scan of a synthetic wall across the 3 pan tiles
 * {-44,0,+44}; assert the merged cloud is planar (single world plane X=D) and
 * covers the wide pan range.
 */
#include "test_util.hpp"
#include "../vision/MapBuilder.hpp"
#include "../pipeline/Scanner.hpp"
#include "mocks/SyntheticScene.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

static void E1_synthetic_room() {
    section("E1 mock scan, 3 pan poses, stop-and-shoot -> walls planar, no doubling");
    Config c;
    c.column_decimation = 2;
    c.edge_trim_frac = 0.05f;
    c.z_min = 0.f; c.z_max = 100000.f;
    c.voxel_size = 5.f;

    const float D = 2500.f;          // wall at world X = 2500 mm
    MapBuilder builder(c);

    for (float pan : Scanner::kPanTiles) {
        cv::Mat depth = mockscene::syntheticWallDepth(D, pan, c);
        builder.addFrame(pan, 0.f, depth);
    }

    cv::Mat cloud = builder.getCloud();
    CHECK(cloud.rows > 0);

    // Planarity: every merged point lies on X = D.
    float maxerr = 0.f, miny = 1e9f, maxy = -1e9f;
    for (int i = 0; i < cloud.rows; ++i) {
        float X = cloud.at<float>(i, 0);
        float Y = cloud.at<float>(i, 1);
        maxerr = std::max(maxerr, std::fabs(X - D));
        miny = std::min(miny, Y);
        maxy = std::max(maxy, Y);
    }
    printf("  merged %d pts, max|X-D|=%.3f mm, Y range [%.0f, %.0f]\n",
           cloud.rows, maxerr, miny, maxy);
    CHECK(maxerr < 2.0f);            // planar across all poses (rotation/lever consistent)
    CHECK((maxy - miny) > 1000.f);   // wide horizontal coverage from pan tiling
}

static void E1_raster_pan_tiling() {
    section("Scanner buildRaster with pan_tiling uses {-44,0,44} per tilt row");
    Scanner::Config sc;
    sc.pan_tiling = true;
    sc.tilt_min = 0.f; sc.tilt_max = 0.f; sc.step_tilt = 20.f;
    auto raster = Scanner::buildRaster(sc);
    CHECK(raster.size() == 3u);
    CHECK_NEAR(raster[0].first, -44.f, 1e-3f);
    CHECK_NEAR(raster[1].first,   0.f, 1e-3f);
    CHECK_NEAR(raster[2].first,  44.f, 1e-3f);
}

int main() {
    printf("=== test_end_to_end ===\n");
    E1_synthetic_room();
    E1_raster_pan_tiling();
    return test_report("test_end_to_end");
}
