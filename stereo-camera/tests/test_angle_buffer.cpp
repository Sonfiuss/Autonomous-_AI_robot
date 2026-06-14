/**
 * test_angle_buffer — implement_plan_stereo.md §8 table (A1–A4).
 */
#include "test_util.hpp"
#include "../core/AngleBuffer.hpp"

static void A1_interpolation() {
    section("A1 samples (0ms,0°),(10ms,10°); angle_at(5ms)=5°, gap≈5ms");
    AngleBuffer b;
    b.push(0.0, 0.f, 0.f);
    b.push(10.0, 10.f, 10.f);
    auto q = b.angleAt(5.0);
    CHECK(q.valid);
    CHECK(!q.extrapolated);
    CHECK_NEAR(q.pan, 5.f, 1e-4f);
    CHECK_NEAR(q.tilt, 5.f, 1e-4f);
    CHECK_NEAR(q.gap_ms, 5.0, 1e-6);
}

static void A2_exact_sample() {
    section("A2 angle_at(0ms) = exactly 0°");
    AngleBuffer b;
    b.push(0.0, 0.f, 0.f);
    b.push(10.0, 10.f, 10.f);
    auto q = b.angleAt(0.0);
    CHECK(q.valid);
    CHECK_NEAR(q.pan, 0.f, 1e-6f);
}

static void A3_gap_flagged() {
    section("A3 50ms gap around query -> gap > threshold flagged");
    AngleBuffer b;
    b.push(0.0, 0.f, 0.f);
    b.push(50.0, 50.f, 0.f);
    auto q = b.angleAt(25.0);
    CHECK(q.valid);
    const double threshold = 20.0;
    CHECK(q.gap_ms > threshold);   // 25ms gap to nearest sample
    CHECK_NEAR(q.pan, 25.f, 1e-4f);
}

static void A4_wraparound() {
    section("A4 wrap past capacity: oldest evicted, newest retrievable, no crash");
    AngleBuffer b(4);
    for (int i = 0; i < 10; ++i)
        b.push((double)i, (float)i, 0.f);   // 0..9, capacity 4 -> keeps 6,7,8,9
    CHECK(b.size() == 4u);
    auto newest = b.angleAt(9.0);
    CHECK(newest.valid);
    CHECK_NEAR(newest.pan, 9.f, 1e-4f);
    // querying an evicted-old time clamps to oldest retained (t=6)
    auto old = b.angleAt(0.0);
    CHECK(old.valid);
    CHECK(old.extrapolated);
    CHECK_NEAR(old.pan, 6.f, 1e-4f);
    // interpolation still works within retained range
    auto mid = b.angleAt(7.5);
    CHECK_NEAR(mid.pan, 7.5f, 1e-4f);
}

int main() {
    printf("=== test_angle_buffer ===\n");
    A1_interpolation();
    A2_exact_sample();
    A3_gap_flagged();
    A4_wraparound();
    return test_report("test_angle_buffer");
}
