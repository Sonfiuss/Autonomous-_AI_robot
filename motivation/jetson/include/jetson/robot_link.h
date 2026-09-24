// Robot_Link — the Jetson side of the serial link: sends commands, decodes
// telemetry, and keeps the last state the firmware reported.
//
// It uses the SAME protocol implementation the firmware does
// (project/src/LINK/protocol.cpp), so a line can never mean one thing here and
// another thing there.
#ifndef JETSON_ROBOT_LINK_H
#define JETSON_ROBOT_LINK_H

#include <cstdint>

#include "LINK/protocol.h"
#include "RM/types.h"
#include "jetson/serial_port.h"

namespace jetson {

namespace cfg {

constexpr char DEFAULT_DEVICE[] = "/dev/ttyUSB0";
constexpr int  BAUD             = 115200;
constexpr int  READ_CHUNK       = 128;
constexpr int  POLL_TIMEOUT_MS  = 20;
constexpr int  ERR_SLOTS        = static_cast<int>(link::ErrCode::COUNT);

}  // namespace cfg

// Monotonic milliseconds since an arbitrary start, for callers pacing their own
// loops. Shared so a caller does not grow its own second clock.
uint32_t monotonicMs();

// Everything the firmware has told us. Cheap to copy.
struct RobotState {
    rm::Pose pose;
    float    panDeg  = 0.0f;
    float    tiltDeg = 0.0f;
    bool     ready   = false;   // READY seen since the port was opened
    uint32_t acks    = 0;       // K lines: one per completed F/T leg
    // READY lines. `ready` alone cannot see a reboot, because it latches true the
    // first time and never clears; a count can, and a reboot mid-plan voids the
    // odometry every plan is built on.
    uint32_t readies = 0;
    // Cumulative counts as last reported by `E`, indexed by ErrCode.
    uint32_t errorCounts[cfg::ERR_SLOTS] = {};
};

class RobotLink {
public:
    RobotLink();

    bool connect(const char* device);
    void disconnect();
    bool connected() const { return port_.isOpen(); }

    // Streams a BODY-frame velocity: forward, left, CCW. poll() keeps repeating
    // it, because the firmware zeroes a command older than its watchdog — going
    // quiet means "stop", never "hold course".
    bool sendVelocity(float vx, float vy, float omega);

    // One-shot commands. Each cancels the streamed velocity, so a keep-alive
    // cannot restart the robot the moment a leg finishes.
    bool sendForward(float distanceM, float speedMs);
    bool sendTurn(float angleRad, float rateRadS);
    bool sendStop();
    bool sendReset();

    // Servo rate, degrees per second. Has its own watchdog in the firmware.
    bool sendServo(float panRateDegS, float tiltRateDegS);

    // Reads whatever arrived, updates state(), and refreshes the streamed
    // velocity when it is due. False on a link error.
    bool poll(int timeoutMs);

    const RobotState& state() const { return state_; }

    // Lines that arrived but could not be decoded — a cable or baud problem,
    // as opposed to the firmware's own counters in state().errorCounts.
    uint32_t decodeErrors() const { return decodeErrors_; }

private:
    bool send(const link::Command& command);
    bool keepAlive();
    void consume(const link::Telemetry& telemetry);

    SerialPort          port_;
    link::LineAssembler assembler_;
    RobotState          state_;
    link::Command       lastVelocity_;
    bool                streaming_;
    uint32_t            lastVelocityMs_;
    uint32_t            decodeErrors_;
};

}  // namespace jetson

#endif  // JETSON_ROBOT_LINK_H
