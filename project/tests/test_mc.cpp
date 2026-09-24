// MC unit tests: plain asserts, no framework, so they build anywhere:
//   g++ -std=c++14 -Iinclude -Iconfig src/MC/*.cpp src/RM/*.cpp tests/test_mc.cpp -o test_mc && ./test_mc
#include <cmath>
#include <cstdio>

#include "MC/mc_api.h"
#include "MV/mv_api.h"
#include "RM/driver_stepdir.h"

namespace {

constexpr float PI           = 3.14159265358979f;
constexpr float EPS_POSE_M   = 0.02f;                 // trajectory end vs commanded move
constexpr float EPS_POSE_RAD = 1.0f * PI / 180.0f;
constexpr float EPS_TIGHT    = 1e-3f;
constexpr float EPS_REST     = 0.05f;                 // "at rest": one tick of ramp either side
constexpr float EPS_OMEGA    = 1e-2f;                 // wheel speed / body component comparisons
constexpr float DT           = 0.02f;                 // 50 Hz, the compiled default
constexpr int   STEP_CAP     = 4096;
constexpr int   MV_CAP       = 4098;
constexpr float ROBOT_RADIUS = 0.225f;

McStep g_steps[STEP_CAP];

int g_failures = 0;

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float eps, const char* what) {
    if (std::fabs(actual - expected) > eps) {
        std::printf("  FAIL %s: got %.4f expected %.4f (eps %.1e)\n", what, actual, expected, eps);
        ++g_failures;
    }
}

McResult makeResult() {
    McResult res{};
    res.steps    = g_steps;
    res.step_cap = STEP_CAP;
    return res;
}

McRequest makeRequest(const McPrimitive* prims, int n, float theta = 0.0f) {
    McRequest req{};
    req.prims       = prims;
    req.n_prims     = n;
    req.start_theta = theta;
    req.dt          = DT;
    return req;
}

// Every wheel stays inside the speed limit, and no tick changes a wheel speed
// by more than the acceleration limit allows.
void checkLimits(const McResult& res, const char* what) {
    char label[128];
    for (int i = 0; i < res.step_len; ++i) {
        for (int wheel = 0; wheel < 3; ++wheel) {
            const float omega = g_steps[i].w[wheel];
            if (std::fabs(omega) > rm::cfg::MAX_WHEEL_OMEGA_RAD_S + EPS_TIGHT) {
                std::snprintf(label, sizeof(label), "%s: wheel %d over speed limit at step %d",
                              what, wheel, i);
                expectTrue(false, label);
                return;
            }
            if (i > 0) {
                const float dOmega = std::fabs(omega - g_steps[i - 1].w[wheel]);
                const float allowed = rm::cfg::WHEEL_ACCEL_RAD_S2 * DT + EPS_TIGHT;
                if (dOmega > allowed) {
                    std::snprintf(label, sizeof(label),
                                  "%s: wheel %d jumped %.4f rad/s at step %d (max %.4f)",
                                  what, wheel, dOmega, i, allowed);
                    expectTrue(false, label);
                    return;
                }
            }
        }
    }
}

// Integrates the reported body velocities in the world frame, which is what a
// caller replaying the trajectory would do.
void integrate(const McResult& res, float startTheta, float* x, float* y, float* theta) {
    *x = *y = 0.0f;
    *theta = startTheta;
    for (int i = 0; i < res.step_len; ++i) {
        const McStep& s = g_steps[i];
        const float mid = *theta + 0.5f * s.r * DT;      // mid-point heading, like rm::Odometry
        *x += (std::cos(mid) * s.u - std::sin(mid) * s.v) * DT;
        *y += (std::sin(mid) * s.u + std::cos(mid) * s.v) * DT;
        *theta += s.r * DT;
    }
}

// ---------------------------------------------------------------- tests

void testForwardMetre() {
    std::printf("forward 1 m\n");
    const McPrimitive prims[] = {{1, 1.0f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    expectTrue(res.step_len > 0, "trajectory not empty");
    expectNear(res.end_x, 1.0f, EPS_POSE_M, "end x");
    expectNear(res.end_y, 0.0f, EPS_POSE_M, "end y");
    expectNear(res.end_theta, 0.0f, EPS_POSE_RAD, "heading unchanged");
    // The per-tick pose must end where the result says the trajectory ends.
    expectNear(g_steps[res.step_len - 1].x, res.end_x, EPS_TIGHT, "last step x is the end pose");
    expectNear(g_steps[res.step_len - 1].y, res.end_y, EPS_TIGHT, "last step y is the end pose");
    expectTrue(g_steps[0].x < g_steps[res.step_len / 2].x, "pose advances monotonically");
    expectNear(g_steps[0].w[0], 0.0f, EPS_REST, "starts from rest");
    expectNear(g_steps[res.step_len - 1].w[0], 0.0f, EPS_REST, "ends at rest");
    checkLimits(res, "forward");
    // Wheel 2 sits at 180 deg, so pure forward motion must leave it still.
    float peak = 0.0f;
    for (int i = 0; i < res.step_len; ++i) peak = std::fmax(peak, std::fabs(g_steps[i].w[1]));
    expectNear(peak, 0.0f, EPS_TIGHT, "wheel at 180 deg idle when driving forward");
}

void testRotateQuarterTurn() {
    std::printf("rotate 90 deg\n");
    const McPrimitive prims[] = {{0, PI / 2.0f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    expectNear(res.end_theta, PI / 2.0f, EPS_POSE_RAD, "end heading");
    expectNear(res.end_x, 0.0f, EPS_POSE_M, "no translation x");
    expectNear(res.end_y, 0.0f, EPS_POSE_M, "no translation y");
    checkLimits(res, "rotate");
    // A spin drives all three wheels the same way at the same speed.
    const int mid = res.step_len / 2;
    expectNear(g_steps[mid].w[0], g_steps[mid].w[1], EPS_TIGHT, "wheels 1 and 2 equal in a spin");
    expectNear(g_steps[mid].w[1], g_steps[mid].w[2], EPS_TIGHT, "wheels 2 and 3 equal in a spin");
    expectTrue(g_steps[mid].w[0] > 0.0f, "CCW spin drives wheels positive");
}

void testMoveFromRotatedHeading() {
    std::printf("holonomic MOVE from a non-zero heading\n");
    // Facing +y (90 deg) and sliding along world +x: the robot must move right
    // without turning, which only works if the world delta is rotated into the body frame.
    const McPrimitive prims[] = {{2, 0.5f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2, PI / 2.0f);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    expectNear(res.end_x, 0.5f, EPS_POSE_M, "end x");
    expectNear(res.end_y, 0.0f, EPS_POSE_M, "end y");
    expectNear(res.end_theta, PI / 2.0f, EPS_POSE_RAD, "heading unchanged by MOVE");
    checkLimits(res, "move");
    // In the body frame this is sideways travel: u stays ~0, v is negative.
    const int mid = res.step_len / 2;
    expectNear(g_steps[mid].u, 0.0f, EPS_OMEGA, "no forward component");
    expectTrue(g_steps[mid].v < 0.0f, "slides to body right");
}

void testSpeedCapPerDirection() {
    std::printf("speed cap per direction\n");
    float forwardMax = 0.0f, forwardAccel = 0.0f, sideMax = 0.0f, sideAccel = 0.0f;
    expectTrue(mc_max_speed(1.0f, 0.0f, 0.0f, 0.0f, &forwardMax, &forwardAccel) == MC_OK, "forward limit ok");
    expectTrue(mc_max_speed(0.0f, 1.0f, 0.0f, 0.0f, &sideMax, &sideAccel) == MC_OK, "sideways limit ok");
    // Wheel coefficients differ per direction: forward peaks at sin(60), sideways at cos(180).
    expectNear(forwardMax, rm::cfg::MAX_WHEEL_OMEGA_RAD_S * rm::cfg::WHEEL_RADIUS_M / std::sin(PI / 3.0f),
               EPS_TIGHT, "forward ceiling");
    expectNear(sideMax, rm::cfg::MAX_WHEEL_OMEGA_RAD_S * rm::cfg::WHEEL_RADIUS_M,
               EPS_TIGHT, "sideways ceiling");
    expectTrue(forwardMax > sideMax, "forward is faster than sideways");

    // Asking for far more than the chassis can do must be scaled down, not clipped per wheel.
    const McPrimitive prims[] = {{1, 0.5f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2);
    req.cruise_speed = 5.0f;
    McResult res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    expectNear(res.end_x, 0.5f, EPS_POSE_M, "still reaches the target distance");
    checkLimits(res, "over-speed request");
}

void testStopAndSkips() {
    std::printf("STOP, zero-length legs and empty list\n");
    const McPrimitive prims[] = {{1, 0.0f, 0.0f},        // zero distance: skipped
                                 {0, 0.0f, 0.0f},        // zero angle: skipped
                                 {3, 0.0f, 0.0f}};       // STOP: still emits held ticks
    McRequest req = makeRequest(prims, 3);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    expectTrue(res.step_len > 0, "STOP emits ticks");
    expectNear(res.end_x, 0.0f, EPS_TIGHT, "no movement");
    for (int i = 0; i < res.step_len; ++i) {
        expectNear(g_steps[i].w[0], 0.0f, EPS_TIGHT, "wheel held at zero during STOP");
        expectNear(g_steps[i].hz[0], 0.0f, EPS_TIGHT, "no pulses during STOP");
    }
    McRequest empty = makeRequest(prims, 0);
    McResult  emptyRes = makeResult();
    expectTrue(mc_run(&empty, &emptyRes) == MC_OK, "empty list is not an error");
    expectTrue(emptyRes.step_len == 0, "empty list produces no steps");
}

void testStepCommands() {
    std::printf("step/dir conversion\n");
    const McPrimitive prims[] = {{1, 0.6f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "status ok");
    for (int i = 0; i < res.step_len; ++i) {
        for (int wheel = 0; wheel < 3; ++wheel) {
            expectTrue(g_steps[i].hz[wheel] >= 0.0f, "pulse rate is never negative");
            expectTrue(g_steps[i].dir[wheel] == 1 || g_steps[i].dir[wheel] == -1, "dir is +1 or -1");
            expectTrue(g_steps[i].hz[wheel] <= rm::cfg::MAX_PULSE_HZ + EPS_TIGHT, "pulse rate clamped");
            // Round-trip: the reported Hz and direction rebuild the wheel speed.
            rm::StepCommand cmd;
            cmd.freqHz  = g_steps[i].hz[wheel];
            cmd.forward = g_steps[i].dir[wheel] > 0;
            const float omega = rm::DriverStepDir::toOmega(cmd);
            const float want  = g_steps[i].w[wheel];
            // Speeds under the driver deadband are reported as 0 Hz by design.
            if (std::fabs(want) >= rm::cfg::MIN_WHEEL_OMEGA_RAD_S) {
                expectNear(omega, want, EPS_OMEGA, "Hz round-trips to the wheel speed");
            }
        }
    }
}

void testBufferTooSmall() {
    std::printf("buffer too small\n");
    const McPrimitive prims[] = {{1, 1.0f, 0.0f}, {3, 0.0f, 0.0f}};
    McRequest req = makeRequest(prims, 2);
    McResult  res = makeResult();
    res.step_cap = 3;                                    // a 1 m move needs far more ticks
    expectTrue(mc_run(&req, &res) == MC_ERR_BUFFER, "reports MC_ERR_BUFFER");
    McResult bad = makeResult();
    bad.steps = nullptr;
    expectTrue(mc_run(&req, &bad) == MC_ERR_ARGS, "null buffer rejected");
    const McPrimitive unknown[] = {{7, 0.0f, 0.0f}};
    McRequest badReq = makeRequest(unknown, 1);
    McResult  badRes = makeResult();
    expectTrue(mc_run(&badReq, &badRes) == MC_ERR_PRIMITIVE, "unknown primitive rejected");
}

// End to end: MV plans a route around a box, MC turns it into wheel speeds, and
// integrating those speeds must land on the pose MV asked for.
void testMvPlanExecuted() {
    std::printf("MV plan -> MC trajectory -> pose\n");
    const MvPoint box[] = {{1.5f, 0.0f}, {2.0f, 0.0f}, {2.0f, 1.6f}, {1.5f, 1.6f}};
    const int     sizes[] = {4};
    MvRequest mvReq{};
    mvReq.room_w       = 4.0f;
    mvReq.room_l       = 3.0f;
    mvReq.robot_radius = ROBOT_RADIUS;
    mvReq.vertices     = box;
    mvReq.poly_sizes   = sizes;
    mvReq.n_polys      = 1;
    mvReq.start        = {0.5f, 0.5f, 0.0f};
    mvReq.goal         = {3.5f, 0.5f, PI / 2.0f};

    static MvPoint     path[MV_CAP];
    static MvPoint     waypoints[MV_CAP];
    static MvPrimitive prims[2 * MV_CAP + 2];
    MvResult mvRes{};
    mvRes.path      = path;      mvRes.path_cap = MV_CAP;
    mvRes.waypoints = waypoints; mvRes.wp_cap   = MV_CAP;
    mvRes.prims     = prims;     mvRes.prim_cap = 2 * MV_CAP + 2;
    expectTrue(mv_plan(&mvReq, &mvRes) == MV_OK, "MV found a path");
    expectTrue(mvRes.prim_len > 2, "MV produced a real primitive list");

    static McPrimitive mcPrims[2 * MV_CAP + 2];
    for (int i = 0; i < mvRes.prim_len; ++i) {
        mcPrims[i].type = prims[i].type;
        mcPrims[i].a    = prims[i].a;
        mcPrims[i].b    = prims[i].b;
    }
    McRequest req = makeRequest(mcPrims, mvRes.prim_len, mvReq.start.theta);
    McResult  res = makeResult();
    expectTrue(mc_run(&req, &res) == MC_OK, "MC expanded the plan");
    checkLimits(res, "mv plan");

    // MC starts from the snapped start cell, so compare against it, not the raw start.
    const float wantX = mvRes.snapped_goal.x - mvRes.snapped_start.x;
    const float wantY = mvRes.snapped_goal.y - mvRes.snapped_start.y;
    expectNear(res.end_x, wantX, EPS_POSE_M, "end x matches the planned displacement");
    expectNear(res.end_y, wantY, EPS_POSE_M, "end y matches the planned displacement");
    expectNear(res.end_theta, mvReq.goal.theta, EPS_POSE_RAD, "end heading matches the goal");

    // The independent world-frame integration must agree with the API's own pose.
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    integrate(res, mvReq.start.theta, &x, &y, &theta);
    expectNear(x, res.end_x, EPS_POSE_M, "integration agrees with reported end x");
    expectNear(y, res.end_y, EPS_POSE_M, "integration agrees with reported end y");
    std::printf("  %d ticks, %.2f s over %.2f m\n", res.step_len, res.duration_s, mvRes.length_m);
}

}  // namespace

int main() {
    testForwardMetre();
    testRotateQuarterTurn();
    testMoveFromRotatedHeading();
    testSpeedCapPerDirection();
    testStopAndSkips();
    testStepCommands();
    testBufferTooSmall();
    testMvPlanExecuted();
    if (g_failures != 0) {
        std::printf("%d MC TEST FAILURE(S)\n", g_failures);
        return 1;
    }
    std::printf("ALL MC TESTS PASSED (mc_version %d)\n", mc_version());
    return 0;
}
