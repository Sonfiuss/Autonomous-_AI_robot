#include "SEQ/sequencer.h"

#include <cmath>

#include "RM/speed_limit.h"
#include "RM/velocity_profile.h"
#include "SEQ/seq_debug.h"

namespace seq {

namespace {

constexpr uint32_t MS_PER_S = 1000;
// expandedCount_ before any primitive has been expanded, so the advance loop can
// tell "nothing started yet" from "the primitive produced no legs at all".
constexpr int NOT_EXPANDED = -1;

float wrapAngle(float angle) {
    while (angle > rm::cfg::PI) angle -= rm::cfg::TWO_PI;
    while (angle <= -rm::cfg::PI) angle += rm::cfg::TWO_PI;
    return angle;
}

float signOf(float value) {
    return value < 0.0f ? -1.0f : 1.0f;
}

// True when the firmware's tally for this code has moved since the baseline.
// Inequality rather than growth, so the 49-day uint32 wrap changes nothing.
bool faultMoved(const uint32_t* baseline, const uint32_t* current, link::ErrCode code) {
    const int slot = static_cast<int>(code);
    return current[slot] != baseline[slot];
}

// A stop command, which is what every failure ends with.
link::Command stopCommand() {
    link::Command command;
    command.kind = link::CmdKind::STOP;
    return command;
}

}  // namespace

const char* statusText(Status status) {
    switch (status) {
        case Status::OK:              return "ok";
        case Status::ARGS:            return "bad arguments";
        case Status::ACK_TIMEOUT:     return "leg not acknowledged in time";
        case Status::FIRMWARE_REJECT: return "firmware rejected a command";
        case Status::QUEUE_FULL:      return "firmware dropped a command (queue full)";
        case Status::REBOOTED:        return "firmware rebooted mid-plan";
        case Status::LINK_ERROR:      return "serial link error";
        case Status::ABORTED:         return "aborted by the caller";
    }
    return "unknown";
}

Sequencer::Sequencer()
    : primitives_(nullptr),
      count_(0),
      index_(0),
      config_(),
      kinematics_(),
      legs_(),
      expandedCount_(NOT_EXPANDED),
      expandedIndex_(0),
      state_(State::IDLE),
      status_(Status::OK),
      thetaPlan_(0.0f),
      thetaCmd_(0.0f),
      legSentMs_(0),
      legTimeoutMs_(0),
      acks_(0),
      readies_(0),
      faults_(),
      haveBaseline_(false),
      stopPending_(false),
      startedMs_(0) {}

bool Sequencer::load(const mc::Primitive* primitives, int count, float startTheta,
                     const Config& config) {
    state_  = State::IDLE;
    status_ = Status::ARGS;

    if (primitives == nullptr || count <= 0 || count > cfg::MAX_PRIMITIVES) {
        SEQ_DLOG("seq: refused a list of %d primitive(s)\n", count);
        return false;
    }
    if (!kinematics_.valid()) {
        // Every leg duration would be meaningless, so no timeout could be trusted.
        SEQ_DLOG("seq: singular wheel layout, refusing to sequence\n");
        return false;
    }
    for (int i = 0; i < count; ++i) {
        const int type = static_cast<int>(primitives[i].type);
        if (type < static_cast<int>(mc::PrimitiveType::ROTATE) ||
            type > static_cast<int>(mc::PrimitiveType::STOP)) {
            SEQ_DLOG("seq: primitive %d has unknown type %d\n", i, type);
            return false;
        }
    }

    // Zero means "the compiled default", the convention mc_run already uses.
    Config resolved = config;
    if (resolved.cruiseSpeed <= 0.0f) resolved.cruiseSpeed = mc::cfg::CRUISE_SPEED_M_S;
    if (resolved.yawRate <= 0.0f) resolved.yawRate = mc::cfg::YAW_RATE_RAD_S;

    // A speed the wire will not carry has to be caught here. The firmware would
    // reject every F or T built from it as malformed, which surfaces as a fault
    // several seconds into the run -- long after the caller could tell why.
    if (resolved.cruiseSpeed < link::cfg::MIN_MOVE_RATE ||
        resolved.cruiseSpeed > link::cfg::MAX_LINEAR_SPEED_M_S ||
        resolved.yawRate < link::cfg::MIN_MOVE_RATE ||
        resolved.yawRate > link::cfg::MAX_YAW_RATE_RAD_S) {
        SEQ_DLOG("seq: speeds %.3f m/s, %.3f rad/s are outside the wire's range\n",
                 resolved.cruiseSpeed, resolved.yawRate);
        return false;
    }

    primitives_ = primitives;
    count_      = count;
    index_      = 0;
    config_     = resolved;

    expandedCount_ = NOT_EXPANDED;
    expandedIndex_ = 0;
    thetaPlan_     = startTheta;
    thetaCmd_      = startTheta;
    legSentMs_     = 0;
    legTimeoutMs_  = 0;
    haveBaseline_  = false;
    stopPending_   = false;
    startedMs_     = 0;
    status_        = Status::OK;
    state_         = State::WAIT_READY;
    SEQ_DLOG("seq: loaded %d primitive(s) from theta %.3f\n", count, startTheta);
    return true;
}

void Sequencer::abort() {
    if (!running()) {
        return;
    }
    status_      = Status::ABORTED;
    state_       = State::FAILED;
    stopPending_ = true;
    // The stop line itself is emitted by the next update(), so abort() never
    // touches the port and stays callable from anywhere.
    SEQ_DLOG("seq: aborted at primitive %d\n", index_);
}

float Sequencer::legDuration(const rm::BodyVel& direction, float length, float requested) const {
    const rm::AxisLimits   axis = rm::limitsFor(kinematics_, direction, requested);
    rm::TrapezoidalProfile profile;
    if (!profile.plan(length, axis.vMax, axis.accel, axis.decel)) {
        return -1.0f;
    }
    return profile.duration();
}

uint32_t Sequencer::timeoutFor(float durationS) {
    const float scaled = durationS * cfg::ACK_TIMEOUT_MARGIN * static_cast<float>(MS_PER_S);
    return static_cast<uint32_t>(scaled) + cfg::ACK_TIMEOUT_FLOOR_MS;
}

void Sequencer::pushTurnTo(float targetTheta) {
    if (expandedCount_ >= cfg::MAX_LEGS_PER_PRIMITIVE) {
        return;
    }
    const float turn = wrapAngle(targetTheta - thetaCmd_);
    if (std::fabs(turn) < mc::cfg::MIN_LEG_ANGLE_RAD) {
        // The firmware would refuse this and say nothing about it. Skipping leaves
        // a heading error below MIN_LEG_ANGLE_RAD, which no leg could correct.
        return;
    }
    rm::BodyVel direction;
    direction.r          = signOf(turn);
    const float duration = legDuration(direction, std::fabs(turn), config_.yawRate);
    if (duration < 0.0f) {
        SEQ_DLOG("seq: dropping unplannable turn of %.4f rad\n", turn);
        return;
    }

    Leg& leg         = legs_[expandedCount_++];
    leg.command.kind = link::CmdKind::TURN;
    leg.command.a    = turn;   // radians here; LINK writes degrees on the wire
    leg.command.b    = config_.yawRate;
    leg.awaitAck     = true;
    leg.timeoutMs    = timeoutFor(duration);
    thetaCmd_        = wrapAngle(thetaCmd_ + turn);
}

void Sequencer::pushForward(float distance) {
    if (expandedCount_ >= cfg::MAX_LEGS_PER_PRIMITIVE) {
        return;
    }
    const float length = std::fabs(distance);
    if (length < mc::cfg::MIN_LEG_LENGTH_M) {
        return;
    }
    rm::BodyVel direction;
    direction.u          = signOf(distance);
    const float duration = legDuration(direction, length, config_.cruiseSpeed);
    if (duration < 0.0f) {
        SEQ_DLOG("seq: dropping unplannable move of %.4f m\n", distance);
        return;
    }

    Leg& leg         = legs_[expandedCount_++];
    leg.command.kind = link::CmdKind::FORWARD;
    leg.command.a    = distance;
    leg.command.b    = config_.cruiseSpeed;
    leg.awaitAck     = true;
    leg.timeoutMs    = timeoutFor(duration);
}

void Sequencer::expandLeg() {
    expandedCount_ = 0;
    expandedIndex_ = 0;

    const mc::Primitive& primitive = primitives_[index_];
    switch (primitive.type) {
        case mc::PrimitiveType::ROTATE: {
            // The plan states a turn RELATIVE to the heading it assumes, so the
            // absolute target is what survives a MOVE decomposed earlier.
            const float target = wrapAngle(thetaPlan_ + primitive.a);
            thetaPlan_         = target;
            pushTurnTo(target);
            break;
        }

        case mc::PrimitiveType::FORWARD:
            pushForward(primitive.a);
            break;

        case mc::PrimitiveType::MOVE: {
            // Decision A1: turn onto the bearing, then drive it. MV emits MOVE with
            // the planned heading left untouched, so thetaPlan_ must not move here —
            // only thetaCmd_ does, and the bias that opens is exactly what
            // pushTurnTo() absorbs at the next ROTATE.
            const float length = std::sqrt(primitive.a * primitive.a + primitive.b * primitive.b);
            if (length < mc::cfg::MIN_LEG_LENGTH_M) {
                break;
            }
            pushTurnTo(std::atan2(primitive.b, primitive.a));
            pushForward(length);
            break;
        }

        case mc::PrimitiveType::STOP: {
            // A stop LATCHES in the firmware, and that is how a plan ends: the
            // wheels hold at zero until an explicit new move releases the latch.
            Leg& leg      = legs_[expandedCount_++];
            leg.command   = stopCommand();
            leg.awaitAck  = false;   // the firmware never acks a stop
            leg.timeoutMs = 0;
            break;
        }
    }
    SEQ_DLOG("seq: primitive %d expanded into %d leg(s)\n", index_, expandedCount_);
}

void Sequencer::fail(Status reason, Action* out) {
    status_      = reason;
    state_       = State::FAILED;
    stopPending_ = false;
    out->send    = true;
    out->command = stopCommand();
    SEQ_DLOG("seq: failed at primitive %d: %s\n", index_, statusText(reason));
}

void Sequencer::rebaseline(const Feedback& feedback) {
    acks_    = feedback.acks;
    readies_ = feedback.readies;
    for (int slot = 0; slot < cfg::FAULT_SLOTS; ++slot) {
        faults_[slot] = feedback.faults[slot];
    }
}

Action Sequencer::update(const Feedback& feedback) {
    Action action;

    if (stopPending_) {
        // abort() deferred its stop to us. Emit it once and then stay quiet.
        stopPending_   = false;
        action.send    = true;
        action.command = stopCommand();
        return action;
    }
    if (!running()) {
        return action;
    }

    if (!haveBaseline_) {
        // First poll of this plan: whatever the firmware has counted so far is
        // history, so only movement from here on can mean anything.
        rebaseline(feedback);
        startedMs_    = feedback.nowMs;
        haveBaseline_ = true;
    }

    if (!feedback.linkOk) {
        fail(Status::LINK_ERROR, &action);
        return action;
    }

    if (state_ == State::WAIT_READY) {
        // A READY here is welcome, not a fault: the firmware booted while we were
        // waiting. Past this state the same event means it rebooted under us, which
        // voids the odometry and the plan built on it.
        if (feedback.readies != readies_) {
            state_ = State::SEND_LEG;
            SEQ_DLOG("seq: firmware announced READY\n");
        } else if (feedback.nowMs - startedMs_ >= cfg::READY_WAIT_MS) {
            // It most likely booted long before the port was opened, in which case
            // its READY is gone for good and waiting on it would deadlock.
            state_ = State::SEND_LEG;
            SEQ_DLOG("seq: no READY within the grace period, starting anyway\n");
        } else {
            return action;
        }
        // The plan goes live here, so re-take every counter now. Two reasons: a
        // boot zeroes the firmware's own tallies and the drop would read as a
        // fresh fault, and a fault raised before we sent anything cannot have
        // been caused by a line of ours.
        rebaseline(feedback);
    }

    if (feedback.readies != readies_) {
        fail(Status::REBOOTED, &action);
        return action;
    }
    if (faultMoved(faults_, feedback.faults, link::ErrCode::QUEUE_FULL)) {
        // The one-shot queue overflowed, so a leg of ours was thrown away and its
        // ack will never come. Failing now beats waiting out the timeout.
        fail(Status::QUEUE_FULL, &action);
        return action;
    }
    if (faultMoved(faults_, feedback.faults, link::ErrCode::MALFORMED_LINE) ||
        faultMoved(faults_, feedback.faults, link::ErrCode::UNKNOWN_COMMAND)) {
        fail(Status::FIRMWARE_REJECT, &action);
        return action;
    }

    if (state_ == State::WAIT_ACK) {
        if (feedback.acks != acks_) {
            acks_ = feedback.acks;
            ++expandedIndex_;
            state_ = State::SEND_LEG;
        } else if (feedback.nowMs - legSentMs_ >= legTimeoutMs_) {
            fail(Status::ACK_TIMEOUT, &action);
            return action;
        } else {
            return action;
        }
    }

    // SEND_LEG. A primitive can expand into no legs at all, so walk forward until
    // one is actually due rather than sending an empty leg.
    while (expandedIndex_ >= expandedCount_) {
        if (expandedCount_ != NOT_EXPANDED) {
            ++index_;   // every leg of the previous primitive has been acknowledged
        }
        if (index_ >= count_) {
            state_  = State::FINISHED;
            status_ = Status::OK;
            SEQ_DLOG("seq: plan complete\n");
            return action;
        }
        expandLeg();
    }

    const Leg& leg = legs_[expandedIndex_];
    action.send    = true;
    action.command = leg.command;
    if (leg.awaitAck) {
        legSentMs_    = feedback.nowMs;
        legTimeoutMs_ = leg.timeoutMs;
        state_        = State::WAIT_ACK;
    } else {
        ++expandedIndex_;   // nothing to wait for; the next update() moves on
        state_ = State::SEND_LEG;
    }
    return action;
}

}  // namespace seq
