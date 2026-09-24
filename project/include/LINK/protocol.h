// LINK — the Jetson <-> ESP32 wire protocol, in ONE implementation that is
// compiled into both the firmware and the Jetson application. Neither side can
// drift away from the other's idea of a line, because there is only one idea.
//
// Pure: no OS, no GPIO, no heap, no iostream, no exceptions — the same sources
// build on a PC (tests), on the Jetson, and inside the ESP32 firmware.
//
// UNIT RULE, and it is the important one: every struct in this header is SI and
// RADIANS. The wire carries degrees where `interfaces.md` says degrees (`T`'s
// angle, `O`'s theta, the servo lines). Conversion happens inside the parsers
// and formatters and nowhere else, so no caller ever has to think about it.
#ifndef LINK_PROTOCOL_H
#define LINK_PROTOCOL_H

#include <atomic>
#include <cstdint>

#include "constants.h"

namespace link {

// Jetson -> ESP32.
enum class CmdKind : uint8_t {
    NONE = 0,
    VELOCITY,  // M <vx> <vy> <omega>   body-frame velocity, m/s m/s rad/s
    FORWARD,   // F <dist> <speed>      m, m/s
    TURN,      // T <angle> <rate>      wire degrees -> struct radians, rad/s
    SERVO,     // V <pan> <tilt>        deg/s
    STOP,      // S
    RESET,     // R
};

// ESP32 -> Jetson.
enum class TeleKind : uint8_t {
    NONE = 0,
    ODOM,   // O <x> <y> <theta>   m, m, wire degrees -> struct radians
    SERVO,  // P <pan> <tilt>      deg
    ACK,    // K
    READY,  // READY
    FAULT,  // E <code> <count>
};

// Codes carried by the `E` line. Also the index into ErrorCounters, so the
// numbering must stay stable: the Jetson side decodes it.
enum class ErrCode : uint8_t {
    NONE            = 0,
    MALFORMED_LINE  = 1,  // bad argument count, or a value out of range
    UNKNOWN_COMMAND = 2,  // command character nobody recognises
    LINE_OVERFLOW   = 3,  // line longer than MAX_LINE_LEN, dropped
    QUEUE_FULL      = 4,  // one-shot queue full, F/T/R dropped
    TX_DROPPED      = 5,  // TX buffer full, periodic message dropped
    SERVO_CLAMPED   = 6,  // servo target hit a mechanical limit
    COUNT                 // not a code: the number of slots to allocate
};

// A command travelling Jetson -> ESP32. Which of a/b/c are meaningful depends
// on `kind`; see the enum above.
struct Command {
    CmdKind kind = CmdKind::NONE;
    float   a    = 0.0f;
    float   b    = 0.0f;
    float   c    = 0.0f;
};

// A telemetry line travelling ESP32 -> Jetson.
struct Telemetry {
    TeleKind kind  = TeleKind::NONE;
    float    a     = 0.0f;              // ODOM: x   | SERVO: pan
    float    b     = 0.0f;              // ODOM: y   | SERVO: tilt
    float    c     = 0.0f;              // ODOM: theta (radians)
    ErrCode  code  = ErrCode::NONE;     // FAULT only
    uint32_t count = 0;                 // FAULT only, cumulative
};

// Cumulative error tallies, one per ErrCode. Bumped from several tasks at once,
// so the slots are atomic; never reset, so a lost `E` line costs no information.
class ErrorCounters {
public:
    ErrorCounters();

    void     bump(ErrCode code);
    uint32_t get(ErrCode code) const;

private:
    static constexpr int SLOTS = static_cast<int>(ErrCode::COUNT);

    std::atomic<uint32_t> counts_[SLOTS];
};

// Assembles incoming bytes into whole lines. Swallows '\r', cuts at '\n', and
// DROPS — rather than truncates — a line longer than cfg::MAX_LINE_LEN, so a
// runaway sender can never bleed into the line that follows it.
class LineAssembler {
public:
    LineAssembler();

    void reset();

    // Feeds one byte. Returns true when a line has completed; the line itself
    // is then in line(), unless overflowed() says it was thrown away.
    bool push(char byte);

    const char* line() const { return line_; }
    int         length() const { return length_; }

    // True when the line that just completed was dropped for being too long.
    bool overflowed() const { return overflowed_; }

private:
    char line_[cfg::MAX_LINE_LEN + 1];
    int  length_;
    bool dropping_;     // discarding the rest of an over-long line
    bool overflowed_;   // the completed line was dropped
    bool restart_;      // the next push starts a fresh line
};

// True when `value` is a command character this protocol defines. Lets a
// receiver tell ErrCode::UNKNOWN_COMMAND apart from ErrCode::MALFORMED_LINE
// without keeping its own copy of the command set.
bool isCommandChar(char value);

// Parses one received line. Both return false when the line is malformed; the
// caller is the one that decides which ErrCode to count.
bool parseCommand(const char* line, Command* out);
bool parseTelemetry(const char* line, Telemetry* out);

// Formatters. Each writes a '\n'-terminated, NUL-terminated line into `out` and
// returns its length excluding the NUL, or 0 when `cap` is too small to hold it.
int formatCommand(const Command& cmd, char* out, int cap);
int formatOdom(float x, float y, float thetaRad, char* out, int cap);
int formatServo(float panDeg, float tiltDeg, char* out, int cap);
int formatAck(char* out, int cap);
int formatReady(char* out, int cap);
int formatFault(ErrCode code, uint32_t count, char* out, int cap);

}  // namespace link

#endif  // LINK_PROTOCOL_H
