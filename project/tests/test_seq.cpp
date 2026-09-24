// SEQ unit tests: plain asserts, no framework, so they build anywhere:
//   tools/build_seq.sh
//
// The sequencer has no port and no clock of its own, which is what lets this file
// stand a fake firmware in for the ESP32: it accepts one leg at a time, refuses to
// accept a second while one is running, and acks on a schedule the test controls.
// Every safety property worth having is therefore testable on a PC.
#include <cmath>
#include <cstdio>

// Only testTimeoutCoversTheRealLeg() needs these: it re-plans a leg exactly the
// way the firmware would, to check the ack timeout clears the real motion.
#include "RM/speed_limit.h"
#include "RM/velocity_profile.h"
#include "SEQ/sequencer.h"

namespace {

constexpr int      WIRE_CAP      = 64;
constexpr uint32_t POLL_MS       = 20;    // the rate a caller would poll the port
constexpr uint32_t ACK_DELAY_MS  = 100;   // how long the fake takes over a leg
constexpr int      MAX_POLLS     = 20000;
constexpr float    EPS_ANGLE     = 1e-4f;
constexpr float    EPS_DIST      = 1e-4f;
constexpr float    DEG90         = rm::cfg::PI / 2.0f;

int g_failures = 0;

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float eps, const char* what) {
    if (std::fabs(actual - expected) > eps) {
        std::printf("  FAIL %s: got %.5f expected %.5f\n", what, actual, expected);
        ++g_failures;
    }
}

void expectInt(int actual, int expected, const char* what) {
    if (actual != expected) {
        std::printf("  FAIL %s: got %d expected %d\n", what, actual, expected);
        ++g_failures;
    }
}

// Stands in for the ESP32. Deliberately strict about the one thing the real
// firmware is strict about: it takes a one-shot only while idle, so a sequencer
// that sends ahead of the ack is caught here rather than on the robot.
class FakeFirmware {
public:
    seq::Feedback feedback;

    uint32_t      ackDelayMs = ACK_DELAY_MS;
    bool          ackLegs    = true;    // false: legs are accepted and never finish
    link::Command wire[WIRE_CAP];
    int           sent       = 0;
    int           violations = 0;       // legs sent while another was still running
    bool          stopped    = false;   // a stop has latched

    void accept(const link::Command& command, uint32_t nowMs) {
        if (sent < WIRE_CAP) {
            wire[sent] = command;
        }
        ++sent;

        switch (command.kind) {
            case link::CmdKind::FORWARD:
            case link::CmdKind::TURN:
                if (busy_) {
                    ++violations;
                }
                // An explicit move releases a latched stop, as beginLeg does.
                stopped = false;
                if (ackLegs) {
                    busy_    = true;
                    ackAtMs_ = nowMs + ackDelayMs;
                } else {
                    busy_ = true;
                }
                break;

            case link::CmdKind::STOP:
                stopped = true;
                busy_   = false;   // a stop abandons the running leg, unacked
                break;

            default:
                break;
        }
    }

    // Delivers whatever the firmware would have reported by now.
    void tick(uint32_t nowMs) {
        if (busy_ && ackLegs && nowMs >= ackAtMs_) {
            busy_ = false;
            ++feedback.acks;
        }
        feedback.nowMs = nowMs;
    }

    void bumpFault(link::ErrCode code) { ++feedback.faults[static_cast<int>(code)]; }

    // A reboot: READY again, the odometry and every tally back at zero.
    void reboot() {
        ++feedback.readies;
        feedback.acks = 0;
        for (int slot = 0; slot < static_cast<int>(link::ErrCode::COUNT); ++slot) {
            feedback.faults[slot] = 0;
        }
        busy_   = false;
        stopped = false;
    }

    int  kindAt(int index) const { return static_cast<int>(wire[index].kind); }
    bool busy() const { return busy_; }

private:
    bool     busy_    = false;
    uint32_t ackAtMs_ = 0;
};

// One poll of the caller's loop: let the firmware report, ask the sequencer, send
// whatever it asked for. Returns the sequencer's action so a test can inspect it.
seq::Action poll(seq::Sequencer* sequencer, FakeFirmware* firmware, uint32_t nowMs) {
    firmware->tick(nowMs);
    const seq::Action action = sequencer->update(firmware->feedback);
    if (action.send) {
        firmware->accept(action.command, nowMs);
    }
    return action;
}

// Polls until the sequencer stops running or the budget is spent. Returns the
// millisecond clock it got to, so a test can keep going from there.
uint32_t runToEnd(seq::Sequencer* sequencer, FakeFirmware* firmware, uint32_t fromMs = 0) {
    uint32_t now = fromMs;
    for (int step = 0; step < MAX_POLLS && sequencer->running(); ++step) {
        now += POLL_MS;
        poll(sequencer, firmware, now);
    }
    return now;
}

mc::Primitive rotate(float rad) { return mc::Primitive{mc::PrimitiveType::ROTATE, rad, 0.0f}; }
mc::Primitive forward(float m) { return mc::Primitive{mc::PrimitiveType::FORWARD, m, 0.0f}; }
mc::Primitive move(float dx, float dy) { return mc::Primitive{mc::PrimitiveType::MOVE, dx, dy}; }
mc::Primitive stop() { return mc::Primitive{mc::PrimitiveType::STOP, 0.0f, 0.0f}; }

// ---------------------------------------------------------------------------

void testLoadRejects() {
    std::printf("load rejects bad input\n");
    seq::Sequencer  sequencer;
    seq::Config     config;
    mc::Primitive   plan[2] = {forward(1.0f), stop()};

    expectTrue(!sequencer.load(nullptr, 2, 0.0f, config), "a null list is refused");
    expectTrue(!sequencer.load(plan, 0, 0.0f, config), "an empty list is refused");
    expectTrue(!sequencer.load(plan, seq::cfg::MAX_PRIMITIVES + 1, 0.0f, config),
               "an oversized list is refused");
    expectTrue(sequencer.status() == seq::Status::ARGS, "a refusal reports ARGS");

    mc::Primitive bad[1];
    bad[0].type = static_cast<mc::PrimitiveType>(7);
    expectTrue(!sequencer.load(bad, 1, 0.0f, config), "an unknown primitive type is refused");

    // A speed the wire cannot carry: the firmware would call every F malformed,
    // which would only surface as a fault seconds into the run.
    seq::Config tooFast;
    tooFast.cruiseSpeed = link::cfg::MAX_LINEAR_SPEED_M_S * 2.0f;
    expectTrue(!sequencer.load(plan, 2, 0.0f, tooFast), "an out-of-range speed is refused");
    seq::Config tooSpinny;
    tooSpinny.yawRate = link::cfg::MAX_YAW_RATE_RAD_S * 2.0f;
    expectTrue(!sequencer.load(plan, 2, 0.0f, tooSpinny), "an out-of-range yaw rate is refused");

    expectTrue(sequencer.load(plan, 2, 0.0f, config), "a valid list loads");
    expectTrue(sequencer.status() == seq::Status::OK, "a loaded plan reports OK");
    expectInt(sequencer.primitiveCount(), 2, "primitiveCount is the list length");

    // Zero means "the compiled default", not "refuse me".
    seq::Config zeroed;
    zeroed.cruiseSpeed = 0.0f;
    zeroed.yawRate     = 0.0f;
    expectTrue(sequencer.load(plan, 2, 0.0f, zeroed), "zero speeds fall back to the defaults");
}

// The property the whole design rests on: never more than one leg in flight,
// because the firmware pops a one-shot only while it is idle.
void testHandshakeIsSerial() {
    std::printf("one leg in flight at a time\n");
    mc::Primitive plan[] = {rotate(DEG90), forward(1.0f), rotate(-DEG90), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 4, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    firmware.reboot();          // the port was opened while the ESP32 was booting
    firmware.feedback.acks = 0;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes");
    expectInt(firmware.violations, 0, "no leg is sent while another is running");
    expectInt(firmware.sent, 4, "one wire line per primitive");
    expectInt(firmware.kindAt(0), static_cast<int>(link::CmdKind::TURN), "leg 1 is a turn");
    expectInt(firmware.kindAt(1), static_cast<int>(link::CmdKind::FORWARD), "leg 2 is a drive");
    expectInt(firmware.kindAt(2), static_cast<int>(link::CmdKind::TURN), "leg 3 is a turn");
    expectInt(firmware.kindAt(3), static_cast<int>(link::CmdKind::STOP), "the plan ends with a stop");
    expectTrue(firmware.stopped, "the stop latches at the end");
}

// A non-holonomic plan needs no rewriting at all: every ROTATE and FORWARD goes
// out with the value MV computed.
void testNonHolonomicMapsOneToOne() {
    std::printf("a non-holonomic plan maps 1:1\n");
    mc::Primitive plan[] = {rotate(0.7f), forward(1.25f), rotate(-1.1f), forward(0.4f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    config.cruiseSpeed = 0.12f;
    config.yawRate     = 0.6f;
    expectTrue(sequencer.load(plan, 5, 0.3f, config), "plan loads");

    FakeFirmware firmware;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes");
    expectInt(firmware.sent, 5, "five lines for five primitives");
    expectNear(firmware.wire[0].a, 0.7f, EPS_ANGLE, "the first turn is unchanged");
    expectNear(firmware.wire[0].b, 0.6f, EPS_ANGLE, "the turn carries the configured yaw rate");
    expectNear(firmware.wire[1].a, 1.25f, EPS_DIST, "the first drive is unchanged");
    expectNear(firmware.wire[1].b, 0.12f, EPS_DIST, "the drive carries the configured speed");
    expectNear(firmware.wire[2].a, -1.1f, EPS_ANGLE, "the second turn is unchanged");
    expectNear(firmware.wire[3].a, 0.4f, EPS_DIST, "the second drive is unchanged");
    // 0.3 + 0.7 - 1.1 = -0.1
    expectNear(sequencer.heading(), -0.1f, EPS_ANGLE, "the commanded heading follows the plan");
}

// Decision A1. MV's holonomic output leaves the heading alone through every MOVE
// and corrects it once at the end, so decomposing a MOVE has to keep that final
// ROTATE landing on the same ABSOLUTE heading it would have without decomposition.
void testMoveDecomposition() {
    std::printf("MOVE decomposes and the final heading still lands\n");
    // What mv::toPrimitives emits for a holonomic plan from theta 0 to theta 0:
    // two MOVE legs, then the turn from startTheta to goalTheta, then a stop.
    mc::Primitive plan[] = {move(1.0f, 0.0f), move(0.0f, 1.0f), rotate(0.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 4, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes");
    // MOVE(1,0) is already on the bearing, so it needs no turn: one drive.
    // MOVE(0,1) turns +90 deg then drives. The trailing ROTATE(0) targets the
    // plan's own heading, 0, which is now 90 deg away: one more turn.
    expectInt(firmware.sent, 5, "one drive, a turn and a drive, a turn, a stop");
    expectInt(firmware.kindAt(0), static_cast<int>(link::CmdKind::FORWARD), "leg 1 drives");
    expectNear(firmware.wire[0].a, 1.0f, EPS_DIST, "leg 1 is one metre");
    expectInt(firmware.kindAt(1), static_cast<int>(link::CmdKind::TURN), "leg 2 turns onto y");
    expectNear(firmware.wire[1].a, DEG90, EPS_ANGLE, "leg 2 turns +90 deg");
    expectInt(firmware.kindAt(2), static_cast<int>(link::CmdKind::FORWARD), "leg 3 drives");
    expectNear(firmware.wire[2].a, 1.0f, EPS_DIST, "leg 3 is one metre");
    expectInt(firmware.kindAt(3), static_cast<int>(link::CmdKind::TURN), "leg 4 is the final turn");
    expectNear(firmware.wire[3].a, -DEG90, EPS_ANGLE, "the final turn undoes the decomposition");
    expectInt(firmware.kindAt(4), static_cast<int>(link::CmdKind::STOP), "the plan ends with a stop");
    expectNear(sequencer.heading(), 0.0f, EPS_ANGLE, "the robot ends on the goal heading");
}

// A non-zero goal heading, to make sure the bias is subtracted and not just
// cancelled by a symmetric case.
void testMoveDecompositionWithGoalHeading() {
    std::printf("MOVE decomposition with a non-zero goal heading\n");
    // Holonomic plan from theta 0 to goalTheta = -0.5 rad: MV emits ROTATE(-0.5).
    mc::Primitive plan[] = {move(0.0f, 2.0f), rotate(-0.5f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 3, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes");
    expectInt(firmware.sent, 4, "a turn, a drive, the final turn, a stop");
    expectNear(firmware.wire[0].a, DEG90, EPS_ANGLE, "the bearing turn is +90 deg");
    expectNear(firmware.wire[2].a, -0.5f - DEG90, EPS_ANGLE,
               "the final turn accounts for the bearing turn");
    expectNear(sequencer.heading(), -0.5f, EPS_ANGLE, "the robot ends on the goal heading");
}

// The nastiest failure mode in the whole chain: applyOneShot throws away the bool
// from beginForward/beginTurn, so a leg the firmware refuses produces no ack and
// bumps no counter. Anything the firmware would refuse must never be sent.
void testShortLegsAreSkipped() {
    std::printf("legs the firmware would silently refuse are skipped\n");
    mc::Primitive plan[] = {rotate(mc::cfg::MIN_LEG_ANGLE_RAD * 0.5f),
                            forward(mc::cfg::MIN_LEG_LENGTH_M * 0.5f),
                            move(1e-6f, 1e-6f),
                            forward(1.0f),
                            stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 5, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes rather than hanging");
    expectInt(firmware.sent, 2, "only the real drive and the stop reach the wire");
    expectInt(firmware.kindAt(0), static_cast<int>(link::CmdKind::FORWARD), "the drive is sent");
    expectNear(firmware.wire[0].a, 1.0f, EPS_DIST, "and it is the one-metre leg");
    expectInt(firmware.kindAt(1), static_cast<int>(link::CmdKind::STOP), "then the stop");
}

// The timeout has to be generous enough for the motion the firmware will actually
// perform, including its ramps — otherwise every long leg would "time out" while
// the robot is still driving it correctly.
void testTimeoutCoversTheRealLeg() {
    std::printf("the ack timeout clears a real leg duration\n");
    // A two-metre leg at the default cruise speed, planned the way the firmware
    // plans it, so the duration here is the duration there.
    rm::OmniKinematics kinematics;
    rm::BodyVel        direction;
    direction.u = 1.0f;
    const rm::AxisLimits   axis = rm::limitsFor(kinematics, direction, mc::cfg::CRUISE_SPEED_M_S);
    rm::TrapezoidalProfile profile;
    expectTrue(profile.plan(2.0f, axis.vMax, axis.accel, axis.decel), "the leg plans");
    const uint32_t realMs = static_cast<uint32_t>(profile.duration() * 1000.0f);
    expectTrue(realMs > 2000u, "a two-metre leg really does take seconds");

    mc::Primitive  plan[] = {forward(2.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 2, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    firmware.ackDelayMs = realMs;   // the firmware takes exactly as long as planned
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "a leg that takes its planned time is not timed out");
    expectTrue(sequencer.status() == seq::Status::OK, "and the plan reports OK");
}

void testAckTimeout() {
    std::printf("a leg that never acks times out\n");
    mc::Primitive  plan[] = {forward(0.5f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 2, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    firmware.ackLegs = false;   // accepted, never finished
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.failed(), "the sequencer gives up");
    expectTrue(sequencer.status() == seq::Status::ACK_TIMEOUT, "and reports ACK_TIMEOUT");
    expectTrue(firmware.stopped, "a stop is sent so the robot does not keep driving");
    expectInt(firmware.kindAt(firmware.sent - 1), static_cast<int>(link::CmdKind::STOP),
              "the last line on the wire is that stop");
}

void testAbortOnQueueFull() {
    std::printf("a dropped one-shot fails immediately\n");
    mc::Primitive  plan[] = {forward(1.0f), forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 3, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    uint32_t     now = 0;
    // Get the first leg onto the wire, then tell the sequencer the firmware threw
    // one away. Its ack is never coming, so waiting out the timeout is pointless.
    for (int step = 0; step < 200 && firmware.sent == 0; ++step) {
        now += POLL_MS;
        poll(&sequencer, &firmware, now);
    }
    expectInt(firmware.sent, 1, "the first leg went out");
    firmware.bumpFault(link::ErrCode::QUEUE_FULL);
    now += POLL_MS;
    const seq::Action action = poll(&sequencer, &firmware, now);

    expectTrue(action.send && action.command.kind == link::CmdKind::STOP,
               "the very next update stops the robot");
    expectTrue(sequencer.failed(), "the sequencer fails");
    expectTrue(sequencer.status() == seq::Status::QUEUE_FULL, "and reports QUEUE_FULL");
}

void testAbortOnFirmwareReject() {
    std::printf("a rejected line fails immediately\n");
    mc::Primitive  plan[] = {forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 2, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    uint32_t     now = 0;
    for (int step = 0; step < 200 && firmware.sent == 0; ++step) {
        now += POLL_MS;
        poll(&sequencer, &firmware, now);
    }
    expectInt(firmware.sent, 1, "the first leg went out");

    firmware.bumpFault(link::ErrCode::MALFORMED_LINE);
    now += POLL_MS;
    poll(&sequencer, &firmware, now);

    expectTrue(sequencer.failed(), "the sequencer fails");
    expectTrue(sequencer.status() == seq::Status::FIRMWARE_REJECT, "and reports FIRMWARE_REJECT");
    expectTrue(firmware.stopped, "a stop is sent");
}

void testAbortOnReboot() {
    std::printf("a reboot mid-plan voids the plan\n");
    mc::Primitive  plan[] = {forward(1.0f), forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 3, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    uint32_t     now = 0;
    for (int step = 0; step < 200 && firmware.sent == 0; ++step) {
        now += POLL_MS;
        poll(&sequencer, &firmware, now);
    }
    expectInt(firmware.sent, 1, "the first leg went out");

    firmware.reboot();   // the odometry is back at zero, so the plan means nothing
    now += POLL_MS;
    poll(&sequencer, &firmware, now);

    expectTrue(sequencer.failed(), "the sequencer fails");
    expectTrue(sequencer.status() == seq::Status::REBOOTED, "and reports REBOOTED");
    expectTrue(firmware.stopped, "a stop is sent");
}

// READY may already be long gone when the port opens, so it cannot be a gate.
void testReadyGracePeriod() {
    std::printf("a missing READY delays the first leg but does not block it\n");
    mc::Primitive  plan[] = {forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 2, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    uint32_t     now = 0;
    while (now + POLL_MS < seq::cfg::READY_WAIT_MS) {
        now += POLL_MS;
        poll(&sequencer, &firmware, now);
        // Something else on the link upset the firmware while we were waiting. We
        // had sent nothing, so it cannot have been us, and it must not fail us.
        if (now == POLL_MS * 2) {
            firmware.bumpFault(link::ErrCode::MALFORMED_LINE);
        }
    }
    expectInt(firmware.sent, 0, "nothing is sent inside the grace period");
    expectTrue(sequencer.state() == seq::State::WAIT_READY, "the sequencer is still waiting");

    runToEnd(&sequencer, &firmware, now);
    expectTrue(sequencer.finished(), "the plan runs once the grace period expires");
    expectTrue(sequencer.status() == seq::Status::OK,
               "a fault raised before the first leg is not blamed on us");
}

// A READY during the grace period is the firmware booting, not rebooting — and it
// zeroes the firmware's tallies, which must not read as a fresh fault.
void testReadyDuringGraceIsNotAFault() {
    std::printf("a READY while waiting is a boot, not a fault\n");
    mc::Primitive  plan[] = {forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 2, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    // Faults left over from before we opened the port.
    firmware.bumpFault(link::ErrCode::MALFORMED_LINE);
    firmware.bumpFault(link::ErrCode::QUEUE_FULL);

    uint32_t now = POLL_MS;
    poll(&sequencer, &firmware, now);          // takes the baseline
    expectInt(firmware.sent, 0, "still inside the grace period");

    firmware.reboot();                          // tallies drop back to zero
    now += POLL_MS;
    poll(&sequencer, &firmware, now);
    expectTrue(!sequencer.failed(), "the tallies dropping to zero is not a failure");

    runToEnd(&sequencer, &firmware, now);
    expectTrue(sequencer.finished(), "and the plan runs straight away");
    expectTrue(sequencer.status() == seq::Status::OK, "reporting OK");
}

void testAbortByCaller() {
    std::printf("abort stops the robot once and stays quiet\n");
    mc::Primitive  plan[] = {forward(1.0f), forward(1.0f), stop()};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 3, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    uint32_t     now = 0;
    for (int step = 0; step < 200 && firmware.sent == 0; ++step) {
        now += POLL_MS;
        poll(&sequencer, &firmware, now);
    }
    sequencer.abort();
    expectTrue(sequencer.status() == seq::Status::ABORTED, "the status is ABORTED at once");

    now += POLL_MS;
    const seq::Action first = poll(&sequencer, &firmware, now);
    expectTrue(first.send && first.command.kind == link::CmdKind::STOP, "the next update stops");

    now += POLL_MS;
    const seq::Action second = poll(&sequencer, &firmware, now);
    expectTrue(!second.send, "and nothing is sent after that");
    expectTrue(sequencer.status() == seq::Status::ABORTED, "the status still reads ABORTED");
}

// A plan without a trailing STOP must not invent one: a stop LATCHES, and
// latching the e-stop is not something to do behind the caller's back.
void testNoStopWithoutStopPrimitive() {
    std::printf("no stop is invented for a plan that does not ask for one\n");
    mc::Primitive  plan[] = {forward(1.0f)};
    seq::Sequencer sequencer;
    seq::Config    config;
    expectTrue(sequencer.load(plan, 1, 0.0f, config), "plan loads");

    FakeFirmware firmware;
    runToEnd(&sequencer, &firmware);

    expectTrue(sequencer.finished(), "the plan finishes");
    expectInt(firmware.sent, 1, "only the drive is sent");
    expectTrue(!firmware.stopped, "no stop latches");
}

}  // namespace

int main() {
    testLoadRejects();
    testHandshakeIsSerial();
    testNonHolonomicMapsOneToOne();
    testMoveDecomposition();
    testMoveDecompositionWithGoalHeading();
    testShortLegsAreSkipped();
    testTimeoutCoversTheRealLeg();
    testAckTimeout();
    testAbortOnQueueFull();
    testAbortOnFirmwareReject();
    testAbortOnReboot();
    testReadyGracePeriod();
    testReadyDuringGraceIsNotAFault();
    testAbortByCaller();
    testNoStopWithoutStopPrimitive();

    if (g_failures == 0) {
        std::printf("test_seq: all checks passed\n");
        return 0;
    }
    std::printf("test_seq: %d check(s) FAILED\n", g_failures);
    return 1;
}
