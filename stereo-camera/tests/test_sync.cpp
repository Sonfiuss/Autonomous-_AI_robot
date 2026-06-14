/**
 * test_sync — implement_plan_stereo.md §8 (S1, S2, S3).
 */
#include "test_util.hpp"
#include "../core/Sync.hpp"
#include <vector>
#include <cmath>

using sync::TimedValue;

// Angle series: linear v(t) = t (deg per ms), sampled every 10 ms.
static std::vector<TimedValue> angleSeries(int n = 50, double dt = 10.0) {
    std::vector<TimedValue> a;
    for (int i = 0; i < n; ++i) a.push_back({i * dt, (float)(i * dt)});
    return a;
}

static void S1_interp_beats_nearest() {
    section("S1 interp error < nearest error on a moving angle");
    auto angles = angleSeries();
    // frames mid-way between angle samples
    std::vector<double> qs = {5.0, 15.0, 23.0, 47.0};
    double interp_err = 0.0, nearest_err = 0.0;
    for (double t : qs) {
        float truth = (float)t;                       // ground-truth angle
        interp_err  += std::fabs(sync::interpAt(angles, t)  - truth);
        nearest_err += std::fabs(sync::nearestAt(angles, t) - truth);
    }
    printf("  interp_err=%.3f  nearest_err=%.3f\n", interp_err, nearest_err);
    CHECK(interp_err < nearest_err);
    CHECK(interp_err < 1e-3);
}

static void S2_recover_offset() {
    section("S2 angles shifted by known 8 ms -> recovered |t_offset| ≈ 8 ms");
    auto angles = angleSeries();
    const double true_off = 8.0;
    // Each frame observes the angle as it was 8 ms earlier (clock skew).
    std::vector<TimedValue> frames;
    for (double t = 12.0; t < 400.0; t += 7.0)
        frames.push_back({t, (float)(t - true_off)});
    double rec = sync::estimateTOffset(frames, angles, 50.0, 0.5);
    printf("  recovered offset = %.2f ms\n", rec);
    CHECK_NEAR(std::fabs(rec), 8.0, 1.0);
}

static void S3_discard_no_angle() {
    section("S3 frame with no angle within threshold is counted/discarded");
    // angles only up to t=100; gap of 60ms before next would-be sample
    std::vector<TimedValue> angles = {{0,0},{20,20},{40,40}};
    std::vector<TimedValue> frames = {{30, 30}, {200, 200}};  // 200 is far out
    size_t discarded = sync::countDiscarded(frames, angles, 20.0);
    printf("  discarded = %zu\n", discarded);
    CHECK(discarded == 1u);   // t=200 has gap 160ms > 20ms
}

int main() {
    printf("=== test_sync ===\n");
    S1_interp_beats_nearest();
    S2_recover_offset();
    S3_discard_no_angle();
    return test_report("test_sync");
}
