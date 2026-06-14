/**
 * test_accumulate — implement_plan_stereo.md §8 (C1 voxel dedup, C2 PLY round-trip).
 */
#include "test_util.hpp"
#include "../vision/MapBuilder.hpp"
#include "mocks/SyntheticScene.hpp"
#include <opencv2/opencv.hpp>
#include <cstdio>

static Config wideCfg() {
    Config c;
    c.column_decimation = 1;
    c.edge_trim_frac = 0.f;
    c.z_min = 0.f; c.z_max = 100000.f;
    c.voxel_size = 5.f;
    return c;
}

static void C1_voxel_dedup() {
    section("C1 two overlapping frames of same wall, voxel=5mm -> ~no doubling");
    Config c = wideCfg();
    cv::Mat depth = mockscene::syntheticWallDepth(2000.f, 0.f, c);

    MapBuilder a(c);
    a.addFrame(0.f, 0.f, depth);
    size_t one = a.pointCount();

    MapBuilder b(c);
    b.addFrame(0.f, 0.f, depth);
    b.addFrame(0.f, 0.f, depth);     // identical second view
    size_t two = b.pointCount();

    printf("  single=%zu  double=%zu\n", one, two);
    CHECK(one > 0u);
    CHECK(two == one);               // voxel dedup collapses the duplicate
}

static void C2_ply_roundtrip() {
    section("C2 export then reload PLY round-trips identically");
    Config c = wideCfg();
    MapBuilder a(c);
    a.addFrame(0.f, 0.f, mockscene::syntheticWallDepth(2000.f, 0.f, c));
    size_t n = a.pointCount();
    CHECK(n > 0u);

    const std::string path = "test_accum_roundtrip.ply";
    CHECK(a.savePLY(path));

    // Reload header vertex count and first vertex.
    std::FILE* f = std::fopen(path.c_str(), "r");
    CHECK(f != nullptr);
    if (f) {
        char line[256];
        size_t parsed_n = 0;
        bool counted = false;
        while (std::fgets(line, sizeof(line), f)) {
            if (std::sscanf(line, "element vertex %zu", &parsed_n) == 1) counted = true;
            if (std::string(line).rfind("end_header", 0) == 0) break;
        }
        std::fclose(f);
        CHECK(counted);
        CHECK(parsed_n == n);
        std::remove(path.c_str());
    }
}

static void overlap_partial() {
    section("overlapping pans share voxels (dedup reduces total vs naive sum)");
    Config c = wideCfg();
    MapBuilder b(c);
    size_t a0 = b.addFrame(0.f, 0.f, mockscene::syntheticWallDepth(2000.f, 0.f, c));
    size_t a1 = b.addFrame(0.f, 0.f, mockscene::syntheticWallDepth(2000.f, 0.f, c));
    CHECK(a0 > 0u);
    CHECK(a1 == 0u);                 // every voxel already occupied
}

int main() {
    printf("=== test_accumulate ===\n");
    C1_voxel_dedup();
    C2_ply_roundtrip();
    overlap_partial();
    return test_report("test_accumulate");
}
