// RM unit tests — plain asserts, no framework, so they build anywhere:
//   g++ -std=c++14 -Iinclude -Iconfig src/RM/*.cpp tests/test_rm.cpp -o test_rm && ./test_rm
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "RM/driver_stepdir.h"
#include "RM/odometry.h"
#include "RM/omni_kinematics.h"
#include "RM/velocity_profile.h"

using namespace rm;

namespace {

constexpr float EPS_TIGHT = 1e-4f;
constexpr float EPS_LOOSE = 5e-3f;
constexpr float CONTROL_DT_S = 0.01f;
constexpr float EPS_ODOM_M    = 1e-2f;   // dead-reckoning closure tolerance after ~30 s
constexpr float EPS_INTEGRATE = 2e-2f;   // numeric integration vs closed form
constexpr float EPS_TICKS     = 1.5f;    // ramp length tolerance in control ticks

int g_failures = 0;

void expectNear(float actual, float expected, float eps, const char* what) {
    if (std::fabs(actual - expected) > eps) {
        std::printf("  FAIL %s: got %.6f expected %.6f (eps %.1e)\n", what, actual, expected, eps);
        ++g_failures;
    }
}

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

// ------------------------------------------------------------ kinematics

void testKinematicsRoundTrip() {
    std::printf("kinematics round trip\n");
    OmniKinematics kin;
    expectTrue(kin.valid(), "matrix invertible");
    expectTrue(Odometry().valid(), "odometry kinematics valid");

    const BodyVel cases[] = {{0.3f, 0.0f, 0.0f}, {0.0f, 0.2f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.1f, -0.2f, 0.5f}};
    for (const BodyVel& in : cases) {
        const BodyVel back = kin.forward(kin.inverse(in));
        expectNear(back.u, in.u, EPS_TIGHT, "u");
        expectNear(back.v, in.v, EPS_TIGHT, "v");
        expectNear(back.r, in.r, EPS_TIGHT, "r");
    }
}

void testKinematicsMatchesOverviewFormula() {
    std::printf("kinematics vs overview formula\n");
    OmniKinematics kin;
    const BodyVel body{0.25f, -0.1f, 0.7f};
    const WheelSpeeds w = kin.inverse(body);
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        const float a        = cfg::WHEEL_ANGLE_RAD[i];
        const float expected = (-std::sin(a) * body.u + std::cos(a) * body.v + cfg::ROBOT_RADIUS_M * body.r) /
                               cfg::WHEEL_RADIUS_M;
        expectNear(w.w[i], expected, EPS_TIGHT, "wheel omega");
    }
    // Pure rotation: all wheels equal, magnitude L/a · r.
    const WheelSpeeds spin = kin.inverse(BodyVel{0.0f, 0.0f, 1.0f});
    for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
        expectNear(spin.w[i], cfg::ROBOT_RADIUS_M / cfg::WHEEL_RADIUS_M, EPS_TIGHT, "spin wheel");
    }
}

void testFrameTransforms() {
    std::printf("frame transforms\n");
    const GlobalVel g{1.0f, 0.0f, 0.3f};
    // Heading 90°: moving +x in the world is moving -v (right) in the body frame.
    const BodyVel b = OmniKinematics::globalToBody(g, 0.5f * cfg::PI);
    expectNear(b.u, 0.0f, EPS_TIGHT, "u at 90deg");
    expectNear(b.v, -1.0f, EPS_TIGHT, "v at 90deg");
    expectNear(b.r, 0.3f, EPS_TIGHT, "r passthrough");
    const GlobalVel back = OmniKinematics::bodyToGlobal(b, 0.5f * cfg::PI);
    expectNear(back.vx, g.vx, EPS_TIGHT, "vx round trip");
    expectNear(back.vy, g.vy, EPS_TIGHT, "vy round trip");
}

// ------------------------------------------------------------ velocity profile

void testSlewLimiter() {
    std::printf("slew-rate limiter\n");
    const float accel = 2.0f;
    VelocityProfile profile(accel, accel, cfg::MAX_WHEEL_OMEGA_RAD_S);
    WheelSpeeds target;
    target.w[0] = 4.0f;
    target.w[1] = 2.0f;
    target.w[2] = -1.0f;

    float maxObservedAccel = 0.0f;
    WheelSpeeds prev       = profile.current();
    int ticks              = 0;
    while (!profile.atTarget(target) && ticks < 10000) {
        const WheelSpeeds cur = profile.step(target, CONTROL_DT_S);
        for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
            maxObservedAccel = std::fmax(maxObservedAccel, std::fabs(cur.w[i] - prev.w[i]) / CONTROL_DT_S);
        }
        // Direction preserved: wheel ratios equal the target ratios while ramping.
        if (std::fabs(cur.w[0]) > EPS_TIGHT) {
            expectNear(cur.w[1] / cur.w[0], target.w[1] / target.w[0], EPS_LOOSE, "ratio w1/w0");
            expectNear(cur.w[2] / cur.w[0], target.w[2] / target.w[0], EPS_LOOSE, "ratio w2/w0");
        }
        prev = cur;
        ++ticks;
    }
    expectTrue(profile.atTarget(target), "reaches target");
    expectTrue(maxObservedAccel <= accel * (1.0f + EPS_LOOSE), "accel limit respected");
    // Fastest wheel needs 4 rad/s / 2 rad/s² = 2 s = 200 ticks.
    expectNear(static_cast<float>(ticks), 200.0f, EPS_TICKS, "ramp ticks");

    // Over-limit target is scaled, not clipped per wheel.
    WheelSpeeds tooFast;
    tooFast.w[0] = 2.0f * cfg::MAX_WHEEL_OMEGA_RAD_S;
    tooFast.w[1] = cfg::MAX_WHEEL_OMEGA_RAD_S;
    profile.reset();
    for (int i = 0; i < 100000 && !profile.atTarget(profile.current()); ++i) {
        profile.step(tooFast, CONTROL_DT_S);
    }
    for (int i = 0; i < 5000; ++i) {
        profile.step(tooFast, CONTROL_DT_S);
    }
    expectNear(profile.current().w[0], cfg::MAX_WHEEL_OMEGA_RAD_S, EPS_LOOSE, "w0 scaled to max");
    expectNear(profile.current().w[1], 0.5f * cfg::MAX_WHEEL_OMEGA_RAD_S, EPS_LOOSE, "w1 scaled half");
}

void testTrapezoid() {
    std::printf("trapezoidal profile\n");
    TrapezoidalProfile prof;
    // Long move: full trapezoid. s_accel = 2²/(2·2) = 1, s_decel = 1, hold = 8/2 = 4 s.
    expectTrue(prof.plan(10.0f, 2.0f, 2.0f, 2.0f), "plan long");
    expectNear(prof.peakVelocity(), 2.0f, EPS_TIGHT, "cruise reached");
    expectNear(prof.duration(), 1.0f + 4.0f + 1.0f, EPS_TIGHT, "duration");
    expectNear(prof.velocityAt(0.5f), 1.0f, EPS_TIGHT, "v mid accel");
    expectNear(prof.velocityAt(3.0f), 2.0f, EPS_TIGHT, "v hold");
    expectNear(prof.velocityAt(5.5f), 1.0f, EPS_TIGHT, "v mid decel");
    expectNear(prof.velocityAt(7.0f), 0.0f, EPS_TIGHT, "v after end");
    expectNear(prof.positionAt(prof.duration()), 10.0f, EPS_LOOSE, "total distance");

    // Short move: triangle. v = sqrt(2·0.5·2·2/4) = 1.
    expectTrue(prof.plan(-0.5f, 2.0f, 2.0f, 2.0f), "plan short");
    expectNear(prof.peakVelocity(), -1.0f, EPS_TIGHT, "triangle peak (negative)");
    expectNear(prof.duration(), 1.0f, EPS_TIGHT, "triangle duration");
    expectNear(prof.positionAt(prof.duration()), -0.5f, EPS_LOOSE, "triangle distance");

    // Numeric integration of velocity must match positionAt.
    expectTrue(prof.plan(3.0f, 1.5f, 1.0f, 3.0f), "plan asymmetric");
    float s = 0.0f;
    for (float t = 0.0f; t < prof.duration(); t += CONTROL_DT_S) {
        s += prof.velocityAt(t + 0.5f * CONTROL_DT_S) * CONTROL_DT_S;
    }
    expectNear(s, 3.0f, EPS_INTEGRATE, "integrated distance");
    expectTrue(!prof.plan(0.0f, 1.0f, 1.0f, 1.0f), "zero distance rejected");
}

// ------------------------------------------------------------ driver + odometry

void testDriverConversions() {
    std::printf("driver conversions\n");
    const StepCommand cmd = DriverStepDir::toStepCommand(-cfg::TWO_PI);  // one rev/s backwards
    expectTrue(!cmd.forward, "direction backward");
    expectNear(cmd.freqHz, static_cast<float>(cfg::STEPS_PER_REV), EPS_TIGHT, "one rev/s = steps/rev Hz");
    expectNear(DriverStepDir::toOmega(cmd), -cfg::TWO_PI, EPS_TIGHT, "omega round trip");
    expectNear(DriverStepDir::toStepCommand(0.0f).freqHz, 0.0f, EPS_TIGHT, "deadband zero");
    expectNear(DriverStepDir::toStepCommand(1000.0f).freqHz, cfg::MAX_PULSE_HZ, EPS_TIGHT, "clamp to max Hz");
    expectTrue(DriverStepDir::angleToSteps(cfg::TWO_PI) == cfg::STEPS_PER_REV, "rev -> steps");
    expectTrue(DriverStepDir::distanceToSteps(cfg::WHEEL_CIRCUMFERENCE_M) == cfg::STEPS_PER_REV,
               "circumference -> steps");

    StepAccumulator acc;
    StepCommand slow;
    slow.freqHz  = 150.0f;  // 1.5 steps per 10 ms tick: remainder must carry
    slow.forward = false;
    int32_t sum  = 0;
    for (int i = 0; i < 100; ++i) {
        sum += acc.accumulate(slow, CONTROL_DT_S);
    }
    expectTrue(sum == -150, "accumulator carries fraction");
    expectTrue(acc.total() == sum, "accumulator total");
}

// Drives a closed square (forward, strafe left, backward, strafe right) plus a
// full spin through IK -> driver -> step accumulator -> odometry; the pose must
// return to the origin.
void testOdometrySquareWalk() {
    std::printf("odometry square walk\n");
    OmniKinematics kin;
    Odometry odom;
    StepAccumulator acc[cfg::NUM_WHEELS];

    const BodyVel legs[] = {{0.2f, 0.0f, 0.0f}, {0.0f, 0.2f, 0.0f}, {-0.2f, 0.0f, 0.0f},
                            {0.0f, -0.2f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    const float legDurations[] = {5.0f, 5.0f, 5.0f, 5.0f, cfg::TWO_PI};
    for (int leg = 0; leg < 5; ++leg) {
        const WheelSpeeds w = kin.inverse(legs[leg]);
        const int ticks     = static_cast<int>(std::lround(legDurations[leg] / CONTROL_DT_S));
        for (int t = 0; t < ticks; ++t) {
            int32_t counts[cfg::NUM_WHEELS];
            for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
                counts[i] = acc[i].accumulate(DriverStepDir::toStepCommand(w.w[i]), CONTROL_DT_S);
            }
            odom.update(counts, CONTROL_DT_S);
        }
        if (leg == 0) {
            expectNear(odom.pose().x, 1.0f, EPS_LOOSE, "x after forward leg");
            expectNear(odom.bodyVelocity().u, 0.2f, EPS_INTEGRATE, "u estimate");
        }
    }
    expectNear(odom.pose().x, 0.0f, EPS_ODOM_M, "square x closes");
    expectNear(odom.pose().y, 0.0f, EPS_ODOM_M, "square y closes");
    expectNear(odom.pose().theta, 0.0f, EPS_INTEGRATE, "theta after full spin wraps to 0");

    // Arc: constant u + r for a half turn traces a semicircle of radius u/r.
    odom.reset();
    const BodyVel arc{0.3f, 0.0f, 0.6f};
    const float radius = arc.u / arc.r;
    const int ticks = static_cast<int>(std::lround(cfg::PI / arc.r / CONTROL_DT_S));
    const WheelSpeeds w = kin.inverse(arc);
    for (int t = 0; t < ticks; ++t) {
        float deltaRad[cfg::NUM_WHEELS];
        for (int i = 0; i < cfg::NUM_WHEELS; ++i) {
            deltaRad[i] = w.w[i] * CONTROL_DT_S;
        }
        odom.updateWheelAngles(deltaRad, CONTROL_DT_S);
    }
    expectNear(odom.pose().x, 0.0f, EPS_LOOSE, "arc x");
    expectNear(odom.pose().y, 2.0f * radius, EPS_LOOSE, "arc y = diameter");
    expectNear(std::fabs(odom.pose().theta), cfg::PI, EPS_LOOSE, "arc theta = pi");

    expectNear(Odometry::wrapAngle(3.0f * cfg::PI), cfg::PI, EPS_TIGHT, "wrap 3pi");
    expectNear(Odometry::wrapAngle(-cfg::PI), cfg::PI, EPS_TIGHT, "wrap -pi -> pi");
}

}  // namespace

int main() {
    testKinematicsRoundTrip();
    testKinematicsMatchesOverviewFormula();
    testFrameTransforms();
    testSlewLimiter();
    testTrapezoid();
    testDriverConversions();
    testOdometrySquareWalk();
    if (g_failures == 0) {
        std::printf("ALL RM TESTS PASSED\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d FAILURE(S)\n", g_failures);
    return EXIT_FAILURE;
}
