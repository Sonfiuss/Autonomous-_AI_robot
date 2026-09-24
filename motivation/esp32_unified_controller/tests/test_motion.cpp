// Firmware unit tests that do NOT need an ESP32: the motion state machine and
// the RTOS port's queue semantics, both built with -DFW_HOST_BUILD.
//   tools/build_test_motion.sh
#include <cmath>
#include <cstdio>

#include "fw/config.h"
#include "fw/motion_state.h"
#include "fw/rt_port.h"

namespace {

constexpr float DT           = fw::cfg::TICK_S;
constexpr float EPS_POSE_M   = 0.03f;
constexpr float EPS_POSE_RAD = 2.0f * rm::cfg::DEG_TO_RAD;
constexpr float EPS_REST     = 0.05f;   // "at rest": one tick of ramp either side
constexpr float EPS_ACCEL    = 0.05f;   // step quantisation on top of the accel limit
constexpr int   MAX_TICKS    = 20000;   // ~400 s at 50 Hz: any real leg ends long before
constexpr int   SETTLE_TICKS = 100;

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

rm::BodyVel bodyVel(float u, float v, float r) {
    rm::BodyVel body;
    body.u = u;
    body.v = v;
    body.r = r;
    return body;
}

float wheelOmega(const fw::MotionTick& tick, int wheel) {
    return rm::DriverStepDir::toOmega(tick.steps[wheel]);
}

// Runs ticks until the leg reports it finished, checking the acceleration limit
// on every wheel along the way. Returns the number of ticks consumed.
int runLeg(fw::MotionState* motion, const rm::BodyVel& streamed, bool* finished) {
    fw::MotionTick tick;
    float previous[rm::cfg::NUM_WHEELS] = {0.0f, 0.0f, 0.0f};
    const float allowed = rm::cfg::WHEEL_ACCEL_RAD_S2 * DT + EPS_ACCEL;

    *finished = false;
    int ticks = 0;
    while (ticks < MAX_TICKS) {
        motion->tick(streamed, DT, &tick);
        ++ticks;
        for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
            const float omega = wheelOmega(tick, wheel);
            if (std::fabs(omega - previous[wheel]) > allowed) {
                std::printf("  FAIL wheel %d jumped %.4f rad/s in one tick\n",
                            wheel, omega - previous[wheel]);
                ++g_failures;
            }
            previous[wheel] = omega;
        }
        if (tick.legFinished) {
            *finished = true;
            break;
        }
    }
    return ticks;
}

void settle(fw::MotionState* motion) {
    fw::MotionTick tick;
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion->tick(rm::BodyVel(), DT, &tick);
    }
}

void testForwardLeg() {
    std::printf("forward leg\n");

    fw::MotionState motion;
    expectTrue(motion.valid(), "the wheel layout is not singular");
    expectTrue(motion.beginForward(1.0f, mc::cfg::CRUISE_SPEED_M_S), "F starts");
    expectTrue(motion.busy(), "F puts the machine in RUNNING_LEG");

    // A streamed velocity must be ignored for the whole leg: if it leaked in,
    // the robot would drift sideways while driving forward.
    bool      finished = false;
    const int ticks    = runLeg(&motion, bodyVel(0.0f, 0.5f, 1.0f), &finished);
    expectTrue(finished, "F reports legFinished");
    expectTrue(ticks < MAX_TICKS, "F finishes in a sane number of ticks");
    expectTrue(!motion.busy(), "F returns to IDLE");

    settle(&motion);
    expectNear(motion.pose().x, 1.0f, EPS_POSE_M, "F travelled one metre");
    expectNear(motion.pose().y, 0.0f, EPS_POSE_M, "F did not drift sideways");
    expectNear(motion.pose().theta, 0.0f, EPS_POSE_RAD, "F did not rotate");
}

void testTurnLeg() {
    std::printf("turn leg\n");

    fw::MotionState motion;
    const float quarter = 90.0f * rm::cfg::DEG_TO_RAD;
    expectTrue(motion.beginTurn(quarter, mc::cfg::YAW_RATE_RAD_S), "T starts");

    bool finished = false;
    runLeg(&motion, rm::BodyVel(), &finished);
    expectTrue(finished, "T reports legFinished");

    settle(&motion);
    expectNear(motion.pose().theta, quarter, EPS_POSE_RAD, "T turned 90 degrees");
    expectNear(motion.pose().x, 0.0f, EPS_POSE_M, "T stayed in place (x)");
    expectNear(motion.pose().y, 0.0f, EPS_POSE_M, "T stayed in place (y)");
}

void testStreamedVelocity() {
    std::printf("streamed velocity\n");

    fw::MotionState motion;
    fw::MotionTick  tick;
    const rm::BodyVel command = bodyVel(0.1f, 0.0f, 0.0f);

    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(command, DT, &tick);
    }
    expectTrue(motion.pose().x > 0.0f, "a streamed velocity moves the robot");
    expectTrue(!tick.legFinished, "streaming never reports a finished leg");

    // Dropping to zero must bring it to rest, which is what the Motion task's
    // watchdog relies on when the Jetson goes quiet.
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(rm::BodyVel(), DT, &tick);
    }
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        expectNear(wheelOmega(tick, wheel), 0.0f, EPS_REST, "a zero command stops the wheel");
    }
}

void testStopInterruptsLeg() {
    std::printf("stop during a leg\n");

    fw::MotionState motion;
    fw::MotionTick  tick;
    expectTrue(motion.beginForward(5.0f, mc::cfg::CRUISE_SPEED_M_S), "a long F starts");

    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(rm::BodyVel(), DT, &tick);
    }
    expectTrue(motion.busy(), "the leg is still running before the stop");

    motion.requestStop();
    expectTrue(motion.mode() == fw::MotionMode::ESTOP, "S enters ESTOP immediately");
    expectTrue(!motion.busy(), "S abandons the leg");

    // Even with a non-zero command streaming in, ESTOP must ramp to rest.
    float previous[rm::cfg::NUM_WHEELS] = {0.0f, 0.0f, 0.0f};
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        previous[wheel] = wheelOmega(tick, wheel);
    }
    const float allowed = rm::cfg::WHEEL_ACCEL_RAD_S2 * DT + EPS_ACCEL;
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(bodyVel(0.2f, 0.0f, 0.0f), DT, &tick);
        for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
            const float omega = wheelOmega(tick, wheel);
            expectTrue(std::fabs(omega - previous[wheel]) <= allowed,
                       "the stop respects the deceleration limit");
            previous[wheel] = omega;
        }
    }
    for (int wheel = 0; wheel < rm::cfg::NUM_WHEELS; ++wheel) {
        expectNear(wheelOmega(tick, wheel), 0.0f, EPS_REST, "the stop reached rest");
    }
    // The latch is the point: a velocity command that was already streaming
    // must not restart the robot the moment the wheels stop turning.
    expectTrue(motion.mode() == fw::MotionMode::ESTOP, "the stop stays latched at rest");

    motion.clearStop();
    expectTrue(motion.mode() == fw::MotionMode::IDLE, "clearStop releases the latch");
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(bodyVel(0.2f, 0.0f, 0.0f), DT, &tick);
    }
    expectTrue(wheelOmega(tick, 0) != 0.0f, "a released stop follows velocity again");

    // A fresh explicit move is new operator intent, so it releases the latch too.
    fw::MotionState second;
    second.requestStop();
    expectTrue(second.beginForward(1.0f, mc::cfg::CRUISE_SPEED_M_S), "F is accepted after a stop");
    expectTrue(second.busy(), "F after a stop actually runs");
}

void testRefusalsAndReset() {
    std::printf("refusals and reset\n");

    fw::MotionState motion;
    expectTrue(motion.beginForward(1.0f, mc::cfg::CRUISE_SPEED_M_S), "the first F is accepted");
    expectTrue(!motion.beginForward(1.0f, mc::cfg::CRUISE_SPEED_M_S),
               "a second F is refused while one is running");
    expectTrue(!motion.beginTurn(1.0f, mc::cfg::YAW_RATE_RAD_S),
               "a T is refused while an F is running");

    // The first ticks produce no whole steps at all: the ramp starts below the
    // driver's minimum pulse rate, so the pose only moves once it is past it.
    fw::MotionTick tick;
    for (int i = 0; i < SETTLE_TICKS; ++i) {
        motion.tick(rm::BodyVel(), DT, &tick);
    }
    expectTrue(motion.pose().x > 0.0f, "the leg moved the pose");
    motion.resetOdometry();
    expectNear(motion.pose().x, 0.0f, 1e-6f, "R zeroes x");
    expectNear(motion.pose().theta, 0.0f, 1e-6f, "R zeroes theta");

    fw::MotionState fresh;
    expectTrue(!fresh.beginForward(0.0f, mc::cfg::CRUISE_SPEED_M_S), "a zero-length F is refused");
    expectTrue(!fresh.beginTurn(0.0f, mc::cfg::YAW_RATE_RAD_S), "a zero-angle T is refused");
}

// The mailbox/queue split is the core of the task design, so its semantics are
// tested rather than assumed.
void testRtPort() {
    std::printf("rtos port semantics\n");

    fw::Mailbox<int> mailbox;
    expectTrue(mailbox.create(), "the mailbox is created");
    int value = -1;
    expectTrue(!mailbox.peek(&value), "an empty mailbox yields nothing");
    mailbox.post(1);
    mailbox.post(2);
    mailbox.post(3);
    expectTrue(mailbox.peek(&value) && value == 3, "the mailbox keeps only the newest value");
    expectTrue(mailbox.peek(&value) && value == 3, "peeking does not consume the value");

    fw::Queue<int, 2> queue;
    expectTrue(queue.create(), "the queue is created");
    expectTrue(queue.push(10), "the first push fits");
    expectTrue(queue.push(20), "the second push fits");
    expectTrue(!queue.push(30), "a full queue refuses instead of blocking");
    expectTrue(queue.pop(&value) && value == 10, "the queue is first in, first out");
    expectTrue(queue.pop(&value) && value == 20, "the queue drains in order");
    expectTrue(!queue.pop(&value), "an empty queue yields nothing");
    expectTrue(queue.push(40), "space freed by a pop is reused");

    fw::Flag flag;
    expectTrue(!flag.isSet(), "a flag starts clear");
    flag.set();
    expectTrue(flag.isSet(), "a raised flag reads as set");
    expectTrue(flag.testAndClear(), "the first consumer sees the flag");
    expectTrue(!flag.testAndClear(), "the second consumer does not");

    fw::Snapshot<rm::Pose> snapshot;
    expectTrue(snapshot.create(), "the snapshot is created");
    rm::Pose pose;
    pose.x = 1.5f;
    snapshot.set(pose);
    expectNear(snapshot.get().x, 1.5f, 1e-6f, "the snapshot round trips");
}

}  // namespace

int main() {
    testForwardLeg();
    testTurnLeg();
    testStreamedVelocity();
    testStopInterruptsLeg();
    testRefusalsAndReset();
    testRtPort();

    if (g_failures == 0) {
        std::printf("test_motion: all checks passed\n");
        return 0;
    }
    std::printf("test_motion: %d check(s) FAILED\n", g_failures);
    return 1;
}
