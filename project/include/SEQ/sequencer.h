// Sequencer — walks an MV primitive list onto the wire, one leg at a time:
// send a leg, wait for the firmware's ack, send the next. This is the piece
// between a plan and a moving robot; without it the planner runs and the robot
// stands still.
//
// Why the handshake is not optional: the firmware's Motion task takes a one-shot
// off its queue only while it is idle (see motivation/.../motion_task.cpp), so
// legs sent back to back pile up in an 8-deep FIFO and the ninth is dropped with
// fault 4. One leg in flight at a time is the protocol, not a precaution.
//
// Pure, like LINK: no OS, no port, no heap, no exceptions, no clock of its own.
// The caller polls update() with what it knows (the millisecond clock, the ack
// and READY counts, the firmware's error tallies) and sends back whatever
// command comes out. That is what makes the whole state machine testable on a PC
// with no ESP32 and no serial port — see tests/test_seq.cpp.
//
// Units: metres and radians throughout, as everywhere in project/. The degrees
// on the wire are LINK's business and appear nowhere here.
#ifndef SEQ_SEQUENCER_H
#define SEQ_SEQUENCER_H

#include <cstdint>

#include "LINK/protocol.h"
// For mc::Primitive only — the shared C++ mirror of MvPrimitive, whose codes are
// pinned by static_assert. MC/types.h is header-only, so SEQ links no MC code:
// MC stays the offline preview tool it is documented to be.
#include "MC/types.h"
#include "RM/omni_kinematics.h"
#include "RM/types.h"
#include "constants.h"

namespace seq {

namespace cfg {

// One slot per link::ErrCode: enough to hold the firmware's whole error tally.
// The rest of seq::cfg is in config/constants.h, but this one cannot be — it
// derives from LINK, and LINK/protocol.h includes constants.h.
constexpr int FAULT_SLOTS = static_cast<int>(link::ErrCode::COUNT);

}  // namespace cfg

enum class State : uint8_t {
    IDLE,        // nothing loaded
    WAIT_READY,  // port freshly opened: giving READY a moment before leg one
    SEND_LEG,    // the next leg goes out on the next update()
    WAIT_ACK,    // a leg is on the wire; waiting for its ack
    FINISHED,    // the whole list ran
    FAILED,      // stopped early; status() says why, and a stop has been sent
};

enum class Status : uint8_t {
    OK = 0,
    ARGS,             // null list, bad count, unknown primitive type, singular chassis
    ACK_TIMEOUT,      // a leg was never acknowledged
    FIRMWARE_REJECT,  // fault 1 or 2 moved: the firmware refused a line we sent
    QUEUE_FULL,       // fault 4 moved: a one-shot was dropped, so its ack will never come
    REBOOTED,         // a fresh READY: the odometry is back at zero and the plan is void
    LINK_ERROR,       // the caller could not read or write the port
    ABORTED,          // abort() by the caller
};

// Speeds the legs are requested at. The firmware lowers either one when the
// direction cannot sustain it (rm::limitsFor); it never raises them. Zero means
// "use the compiled default", the same convention mc_run uses.
struct Config {
    float cruiseSpeed = mc::cfg::CRUISE_SPEED_M_S;  // m/s, for FORWARD and MOVE legs
    float yawRate     = mc::cfg::YAW_RATE_RAD_S;    // rad/s, for ROTATE legs
};

// Everything the caller knows about the link, sampled once per update(). The
// three counters are compared for INEQUALITY, never for growth, so they stay
// correct across the uint32 wrap.
struct Feedback {
    uint32_t nowMs   = 0;      // monotonic milliseconds
    uint32_t acks    = 0;      // ack lines seen so far
    uint32_t readies = 0;      // READY lines seen so far
    uint32_t faults[cfg::FAULT_SLOTS] = {};  // by ErrCode, cumulative
    bool     linkOk  = true;   // false once a read or write has failed
};

// What the caller should put on the wire. At most one command per update(), so a
// caller never has to buffer.
struct Action {
    bool          send = false;
    link::Command command;
};

class Sequencer {
public:
    Sequencer();

    // Loads a plan. startTheta is the heading the plan was made from, needed to
    // place MOVE legs. Returns false (and sets status()) on a null list, a count
    // outside 1..cfg::MAX_PRIMITIVES, an unknown primitive type, a speed the wire
    // would refuse to carry, or a singular wheel layout in constants.h.
    bool load(const mc::Primitive* primitives, int count, float startTheta, const Config& config);

    // Gives up on the current plan. The next update() emits a stop.
    void abort();

    // Advances the state machine as far as this feedback allows and returns the
    // one command to send, if any. Cheap, and safe to call as fast as the caller
    // polls the port.
    Action update(const Feedback& feedback);

    State  state() const { return state_; }
    Status status() const { return status_; }

    bool running() const {
        return state_ == State::WAIT_READY || state_ == State::SEND_LEG || state_ == State::WAIT_ACK;
    }
    bool finished() const { return state_ == State::FINISHED; }
    bool failed() const { return state_ == State::FAILED; }

    // Progress, in primitives rather than wire legs: one MOVE primitive becomes
    // two legs, so counting legs would make a plan look longer than MV's list.
    int primitiveIndex() const { return index_; }
    int primitiveCount() const { return count_; }

    // Heading (rad) the robot has been commanded to by the legs sent so far.
    // Equals the plan's own heading unless a MOVE has been decomposed.
    float heading() const { return thetaCmd_; }

private:
    // One line bound for the wire, plus how long the firmware will take over it.
    struct Leg {
        link::Command command;
        bool          awaitAck  = false;  // false for a stop, which is never acked
        uint32_t      timeoutMs = 0;
    };

    // Expands primitives_[index_] into legs_ — 0, 1 or 2 of them. A MOVE becomes
    // a ROTATE onto its bearing plus a FORWARD along it (decision A1), because
    // the protocol has no world-frame straight-line command and streaming a
    // velocity would leave the leg with no acknowledgement to wait for.
    //
    // 0 legs is a normal outcome, not an error: a primitive the firmware would
    // silently refuse MUST be dropped here. applyOneShot throws away the bool
    // from beginForward/beginTurn, so a leg under mc::cfg::MIN_LEG_LENGTH_M or
    // MIN_LEG_ANGLE_RAD produces no ack and bumps no counter — sending one would
    // hang this state machine until the ack timeout.
    void expandLeg();

    // Appends a ROTATE onto the absolute heading targetTheta, if the turn is
    // worth making, and moves thetaCmd_ with it.
    void pushTurnTo(float targetTheta);

    // Appends a straight leg of signed distance metres along body +x.
    void pushForward(float distance);

    // How long the firmware's own leg planner will take over this move, seconds,
    // or a negative value when it would refuse the leg outright. Uses the same
    // rm::limitsFor + rm::TrapezoidalProfile the firmware uses, so the estimate
    // cannot drift away from the real motion.
    float legDuration(const rm::BodyVel& direction, float length, float requested) const;

    // Ack timeout for a leg of durationS seconds: the motion itself plus a margin
    // for serial latency, the 50 Hz tick and the caller's own scheduling.
    static uint32_t timeoutFor(float durationS);

    // Latches a failure and asks for a stop, so the robot halts rather than
    // finishing a leg nobody is watching any more.
    void fail(Status reason, Action* out);

    // Re-takes every feedback counter as the new baseline. Used at the first poll
    // and again when the firmware announces a boot, because a boot zeroes its own
    // tallies and the drop to zero would otherwise read as a fresh fault.
    void rebaseline(const Feedback& feedback);

    const mc::Primitive* primitives_;
    int                  count_;
    int                  index_;        // primitive being executed
    Config               config_;
    rm::OmniKinematics   kinematics_;

    // The current primitive expanded into wire legs. expandedCount_ is negative
    // until the first expansion, which is how the advance loop tells "not started"
    // apart from "this primitive produced nothing".
    Leg legs_[cfg::MAX_LEGS_PER_PRIMITIVE];
    int expandedCount_;
    int expandedIndex_;

    State    state_;
    Status   status_;
    float    thetaPlan_;  // heading the plan assumes at index_
    float    thetaCmd_;   // heading actually commanded; differs after a MOVE
    uint32_t legSentMs_;
    uint32_t legTimeoutMs_;
    uint32_t acks_;       // baselines the feedback counters are compared against
    uint32_t readies_;
    uint32_t faults_[cfg::FAULT_SLOTS];
    bool     haveBaseline_;
    bool     stopPending_;  // abort() asked for a stop the next update() must emit
    uint32_t startedMs_;
};

// Human-readable status, for a CLI or a log line.
const char* statusText(Status status);

}  // namespace seq

#endif  // SEQ_SEQUENCER_H
