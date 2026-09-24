#include "jetson/robot_link.h"

#include <cstdio>
#include <ctime>

namespace jetson {

namespace {

constexpr int      LINE_CAP  = link::cfg::MAX_LINE_LEN + 2;
constexpr uint32_t MS_PER_S  = 1000;
constexpr uint32_t NS_PER_MS = 1000000;

}  // namespace

uint32_t monotonicMs() {
    timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<uint32_t>(now.tv_sec) * MS_PER_S +
           static_cast<uint32_t>(now.tv_nsec / NS_PER_MS);
}

RobotLink::RobotLink()
    : port_(), assembler_(), state_(), lastVelocity_(), streaming_(false), lastVelocityMs_(0),
      decodeErrors_(0) {}

bool RobotLink::connect(const char* device) {
    if (!port_.open(device != nullptr ? device : cfg::DEFAULT_DEVICE, cfg::BAUD)) {
        return false;
    }
    assembler_.reset();
    state_        = RobotState();
    streaming_    = false;
    decodeErrors_ = 0;
    return true;
}

void RobotLink::disconnect() {
    // Best effort: if the robot is moving it should be told to stop before the
    // port closes. Its own watchdog is the backstop if this write fails.
    if (port_.isOpen() && streaming_) {
        sendStop();
    }
    port_.close();
    streaming_ = false;
}

bool RobotLink::send(const link::Command& command) {
    char      line[LINE_CAP];
    const int length = link::formatCommand(command, line, LINE_CAP);
    if (length <= 0) {
        return false;
    }
    return port_.write(line, length);
}

bool RobotLink::sendVelocity(float vx, float vy, float omega) {
    link::Command command;
    command.kind = link::CmdKind::VELOCITY;
    command.a    = vx;
    command.b    = vy;
    command.c    = omega;

    lastVelocity_   = command;
    streaming_      = true;
    lastVelocityMs_ = monotonicMs();
    return send(command);
}

bool RobotLink::sendForward(float distanceM, float speedMs) {
    streaming_ = false;
    link::Command command;
    command.kind = link::CmdKind::FORWARD;
    command.a    = distanceM;
    command.b    = speedMs;
    return send(command);
}

bool RobotLink::sendTurn(float angleRad, float rateRadS) {
    streaming_ = false;
    link::Command command;
    command.kind = link::CmdKind::TURN;
    command.a    = angleRad;   // radians here, degrees on the wire
    command.b    = rateRadS;
    return send(command);
}

bool RobotLink::sendStop() {
    streaming_ = false;
    link::Command command;
    command.kind = link::CmdKind::STOP;
    return send(command);
}

bool RobotLink::sendReset() {
    streaming_ = false;
    link::Command command;
    command.kind = link::CmdKind::RESET;
    return send(command);
}

bool RobotLink::sendServo(float panRateDegS, float tiltRateDegS) {
    link::Command command;
    command.kind = link::CmdKind::SERVO;
    command.a    = panRateDegS;
    command.b    = tiltRateDegS;
    return send(command);
}

void RobotLink::consume(const link::Telemetry& telemetry) {
    switch (telemetry.kind) {
        case link::TeleKind::ODOM:
            state_.pose.x     = telemetry.a;
            state_.pose.y     = telemetry.b;
            state_.pose.theta = telemetry.c;
            break;

        case link::TeleKind::SERVO:
            state_.panDeg  = telemetry.a;
            state_.tiltDeg = telemetry.b;
            break;

        case link::TeleKind::ACK:
            ++state_.acks;
            break;

        case link::TeleKind::READY:
            // The firmware booted: anything we thought it was doing is gone. The
            // COUNT is what a sequencer watches -- `ready` latches true on the
            // first one and so cannot tell a boot from a reboot mid-plan.
            state_.ready = true;
            streaming_   = false;
            ++state_.readies;
            break;

        case link::TeleKind::FAULT: {
            const int slot = static_cast<int>(telemetry.code);
            if (slot > 0 && slot < cfg::ERR_SLOTS) {
                state_.errorCounts[slot] = telemetry.count;
            }
            break;
        }

        default:
            break;
    }
}

bool RobotLink::keepAlive() {
    if (!streaming_) {
        return true;
    }
    if (monotonicMs() - lastVelocityMs_ < link::cfg::KEEPALIVE_MS) {
        return true;
    }
    lastVelocityMs_ = monotonicMs();
    return send(lastVelocity_);
}

bool RobotLink::poll(int timeoutMs) {
    if (!port_.isOpen()) {
        return false;
    }

    char      chunk[cfg::READ_CHUNK];
    const int count = port_.read(chunk, cfg::READ_CHUNK, timeoutMs);
    if (count < 0) {
        return false;
    }

    for (int index = 0; index < count; ++index) {
        if (!assembler_.push(chunk[index])) {
            continue;
        }
        if (assembler_.overflowed()) {
            ++decodeErrors_;
            continue;
        }
        if (assembler_.length() == 0) {
            continue;
        }
        link::Telemetry telemetry;
        if (!link::parseTelemetry(assembler_.line(), &telemetry)) {
            ++decodeErrors_;
            continue;
        }
        consume(telemetry);
    }

    return keepAlive();
}

}  // namespace jetson
