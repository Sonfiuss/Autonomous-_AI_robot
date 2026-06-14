/**
 * test_calibration — lever-arm bow minimization converges (plan §8/§9).
 *
 * Synthetic wall frames are generated consistent with r=0. Starting the search
 * from a wrong lever arm must drive the merged planarity RMS back down and
 * recover r ≈ 0.
 */
#include "test_util.hpp"
#include "../calibration/Calibration.hpp"
#include "mocks/SyntheticScene.hpp"

static std::vector<calib::WallFrame> wallFrames(const Config& c) {
    std::vector<calib::WallFrame> fr;
    for (float pan : {-44.f, 0.f, 44.f})
        fr.push_back({pan, 0.f, mockscene::syntheticWallDepth(2500.f, pan, c)});
    return fr;
}

static void lever_arm_converges() {
    section("estimateLeverArm reduces bow and recovers r ≈ 0");
    Config truth;
    truth.column_decimation = 4;
    truth.edge_trim_frac = 0.f;
    truth.z_min = 0.f; truth.z_max = 100000.f;

    auto frames = wallFrames(truth);

    Config wrong = truth;
    wrong.lever_arm = {30.f, 0.f, 0.f};      // deliberately wrong
    float rms0 = calib::mergedPlanarityRMS(frames, wrong);

    Config est = calib::estimateLeverArm(frames, wrong, 60.f, 0.5f);
    float rms1 = calib::mergedPlanarityRMS(frames, est);

    printf("  RMS before=%.3f  after=%.3f  r=(%.2f,%.2f,%.2f)\n",
           rms0, rms1, est.lever_arm[0], est.lever_arm[1], est.lever_arm[2]);
    CHECK(rms0 > 1.0f);                       // wrong r really does bow the wall
    CHECK(rms1 < rms0);                       // optimization improves it
    CHECK(rms1 < 1.0f);                       // back to (near) planar
    CHECK(std::fabs(est.lever_arm[0]) < 5.f); // recovered r near the true 0
}

int main() {
    printf("=== test_calibration ===\n");
    lever_arm_converges();
    return test_report("test_calibration");
}
