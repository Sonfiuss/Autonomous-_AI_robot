/**
 * Unit tests for Scanner::buildRaster, loadConfig, saveConfig.
 *
 * Does NOT require hardware or OpenCV.
 * Run: ./test_scanner_raster
 * Expected: all tests PASS, exit code 0.
 */

#include <cstdio>
#include <cmath>
#include <string>
#include <vector>
#include <cstdlib>
#include <unistd.h>

#include "../pipeline/Scanner.hpp"

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

#define CHECK_NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) < (tol))

static void section(const char* name) { printf("\n── %s\n", name); }

// ── Helpers ───────────────────────────────────────────────────────────────────

static Scanner::Config make_cfg(float pan_step = 80.f, float tilt_step = 50.f) {
    Scanner::Config c;
    c.pan_min   = -80.f; c.pan_max   = 80.f;
    c.tilt_min  = -70.f; c.tilt_max  = 30.f;
    c.step_pan  = pan_step;
    c.step_tilt = tilt_step;
    return c;
}

// ── Raster tests ──────────────────────────────────────────────────────────────

static void test_first_point_is_pan_min_tilt_min() {
    section("first point = (pan_min, tilt_min)");
    auto r = Scanner::buildRaster(make_cfg());
    CHECK(!r.empty());
    CHECK_NEAR(r[0].first,  -80.f, 0.01f);
    CHECK_NEAR(r[0].second, -70.f, 0.01f);
}

static void test_snake_direction_alternates() {
    section("snake: rows alternate left-to-right / right-to-left");
    // 3 pan positions per row: -80, 0, +80
    // 3 tilt rows: -70, -20, +30
    auto r = Scanner::buildRaster(make_cfg(80.f, 50.f));

    // Row 0 (tilt=-70): left → right  →  pan should be ascending
    CHECK(r[0].first < r[1].first);   // -80 < 0
    CHECK(r[1].first < r[2].first);   //   0 < 80

    // Row 1 (tilt=-20): right → left  →  pan should be descending
    CHECK(r[3].first > r[4].first);   // 80 > 0
    CHECK(r[4].first > r[5].first);   //  0 > -80

    // Row 2 (tilt=+30): left → right again
    CHECK(r[6].first < r[7].first);
    CHECK(r[7].first < r[8].first);
}

static void test_total_count() {
    section("total step count = pan_steps × tilt_steps");
    // pan: -80, 0, +80    → 3 positions
    // tilt: -70, -20, +30 → 3 positions
    auto r = Scanner::buildRaster(make_cfg(80.f, 50.f));
    CHECK(r.size() == 9u);
}

static void test_all_tilt_values_covered() {
    section("all tilt rows are present");
    auto r = Scanner::buildRaster(make_cfg(80.f, 50.f));
    // collect unique tilts
    bool saw_neg70 = false, saw_neg20 = false, saw_pos30 = false;
    for (auto& [p, t] : r) {
        if (std::fabs(t - (-70.f)) < 0.1f) saw_neg70 = true;
        if (std::fabs(t - (-20.f)) < 0.1f) saw_neg20 = true;
        if (std::fabs(t -   30.f)  < 0.1f) saw_pos30 = true;
    }
    CHECK(saw_neg70);
    CHECK(saw_neg20);
    CHECK(saw_pos30);
}

static void test_tilt_does_not_exceed_max() {
    section("tilt values never exceed tilt_max");
    auto r = Scanner::buildRaster(make_cfg(80.f, 40.f));
    for (auto& [p, t] : r)
        CHECK(t <= 30.f + 0.01f);
}

static void test_pan_does_not_exceed_limits() {
    section("pan values stay within [pan_min, pan_max]");
    auto r = Scanner::buildRaster(make_cfg(80.f, 50.f));
    for (auto& [p, t] : r) {
        CHECK(p >= -80.f - 0.01f);
        CHECK(p <=  80.f + 0.01f);
    }
}

static void test_single_step_raster() {
    section("single-step raster produces exactly 1 point");
    Scanner::Config c;
    c.pan_min = c.pan_max = 0.f;
    c.tilt_min = c.tilt_max = 0.f;
    c.step_pan = c.step_tilt = 10.f;
    auto r = Scanner::buildRaster(c);
    CHECK(r.size() == 1u);
    CHECK_NEAR(r[0].first,  0.f, 0.01f);
    CHECK_NEAR(r[0].second, 0.f, 0.01f);
}

// ── Config I/O tests ──────────────────────────────────────────────────────────

static std::string tmp_path() {
    return "/tmp/test_scan_config_" + std::to_string(getpid()) + ".txt";
}

static void test_save_and_load_roundtrip() {
    section("saveConfig / loadConfig round-trip");
    std::string path = tmp_path();

    Scanner::saveConfig(path, 7, -40.f, 10.f);

    size_t idx = 999;
    CHECK(Scanner::loadConfig(path, idx) == true);
    CHECK(idx == 7u);

    std::remove(path.c_str());
}

static void test_load_missing_file_returns_false() {
    section("loadConfig on missing file returns false");
    size_t idx = 0;
    CHECK(Scanner::loadConfig("/tmp/no_such_file_xyz.cfg", idx) == false);
}

static void test_resume_start_index_is_correct() {
    section("saved idx = next step, so resume skips completed steps");
    std::string path = tmp_path();

    // Simulate: step 4 just finished, next to run is 5
    Scanner::saveConfig(path, 5, 0.f, 0.f);

    size_t idx;
    Scanner::loadConfig(path, idx);
    CHECK(idx == 5u);   // raster iteration starts at 5

    std::remove(path.c_str());
}

static void test_overwrite_updates_index() {
    section("successive saves overwrite previous checkpoint");
    std::string path = tmp_path();

    Scanner::saveConfig(path, 2, 0.f, 0.f);
    Scanner::saveConfig(path, 6, 0.f, 0.f);

    size_t idx;
    Scanner::loadConfig(path, idx);
    CHECK(idx == 6u);

    std::remove(path.c_str());
}

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
    printf("=== test_scanner_raster ===\n");

    test_first_point_is_pan_min_tilt_min();
    test_snake_direction_alternates();
    test_total_count();
    test_all_tilt_values_covered();
    test_tilt_does_not_exceed_max();
    test_pan_does_not_exceed_limits();
    test_single_step_raster();

    test_save_and_load_roundtrip();
    test_load_missing_file_returns_false();
    test_resume_start_index_is_correct();
    test_overwrite_updates_index();

    printf("\n  %d passed  /  %d failed\n", s_pass, s_fail);
    return s_fail > 0 ? 1 : 0;
}
