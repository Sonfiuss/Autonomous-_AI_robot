#include "LINK/protocol.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

#include "LINK/link_debug.h"

namespace link {

namespace {

constexpr char READY_TEXT[] = "READY";
constexpr int  READY_LEN    = 5;
constexpr float ROUND_HALF  = 0.5f;
constexpr int   MAX_DIGITS  = 10;   // digits in the largest uint32

// Builds one wire line in a caller-owned buffer. Every put* call is a no-op
// once the buffer has overflowed, so a formatter checks for failure once, at
// finish(), instead of after every field.
class LineWriter {
public:
    LineWriter(char* out, int cap)
        : out_(out), cap_(cap), length_(0), ok_(out != nullptr && cap > 0) {}

    void putChar(char value) {
        if (!ok_) {
            return;
        }
        if (length_ + 1 >= cap_) {   // +1 keeps room for the terminating NUL
            ok_ = false;
            return;
        }
        out_[length_++] = value;
    }

    void putText(const char* text) {
        for (const char* cursor = text; *cursor != '\0'; ++cursor) {
            putChar(*cursor);
        }
    }

    void putInt(uint32_t value) {
        char digits[MAX_DIGITS];
        int  count = 0;
        do {
            digits[count++] = static_cast<char>('0' + (value % 10));
            value /= 10;
        } while (value != 0 && count < MAX_DIGITS);
        while (count > 0) {
            putChar(digits[--count]);
        }
    }

    // Fixed-decimal float. Hand-rolled rather than snprintf("%f") because
    // ESP-IDF's newlib-nano option drops %f, which would silently turn every
    // telemetry line into garbage on a build that enabled it.
    void putFloat(float value) {
        if (!ok_) {
            return;
        }
        if (!std::isfinite(value)) {
            value = 0.0f;
        }
        if (value > cfg::MAX_WIRE_VALUE) {
            value = cfg::MAX_WIRE_VALUE;
        }
        if (value < -cfg::MAX_WIRE_VALUE) {
            value = -cfg::MAX_WIRE_VALUE;
        }
        if (value < 0.0f) {
            value = -value;
            putChar('-');
        }
        const uint32_t scale  = static_cast<uint32_t>(cfg::FLOAT_SCALE);
        const uint32_t scaled = static_cast<uint32_t>(value * cfg::FLOAT_SCALE + ROUND_HALF);
        putInt(scaled / scale);
        putChar('.');
        putFraction(scaled % scale);
    }

    // Terminates the line and returns its length, or 0 if anything overflowed.
    int finish() {
        putChar('\n');
        if (!ok_) {
            return 0;
        }
        out_[length_] = '\0';
        return length_;
    }

private:
    // Leading zeros matter here: 7 thousandths is ".007", never ".7".
    void putFraction(uint32_t fraction) {
        uint32_t divisor = static_cast<uint32_t>(cfg::FLOAT_SCALE) / 10;
        for (int i = 0; i < cfg::FLOAT_DECIMALS; ++i) {
            putChar(static_cast<char>('0' + (fraction / divisor) % 10));
            divisor /= 10;
        }
    }

    char* out_;
    int   cap_;
    int   length_;
    bool  ok_;
};

bool isBlank(const char* text) {
    for (const char* cursor = text; *cursor != '\0'; ++cursor) {
        if (*cursor != ' ' && *cursor != '\t') {
            return false;
        }
    }
    return true;
}

// Reads exactly `count` finite floats and rejects any leftover character, so a
// truncated or padded line never parses as a valid command.
bool readArgs(const char* text, int count, float* out) {
    const char* cursor = text;
    for (int i = 0; i < count; ++i) {
        char*       end   = nullptr;
        const float value = std::strtof(cursor, &end);
        if (end == cursor || !std::isfinite(value)) {
            return false;
        }
        out[i] = value;
        cursor = end;
    }
    return isBlank(cursor);
}

bool withinAbs(float value, float limit) {
    return value >= -limit && value <= limit;
}

// `F` and `T` carry the speed to move at; zero or negative would never finish.
bool isMoveRate(float value, float limit) {
    return value >= cfg::MIN_MOVE_RATE && value <= limit;
}

}  // namespace

ErrorCounters::ErrorCounters() {
    for (int slot = 0; slot < SLOTS; ++slot) {
        counts_[slot].store(0);
    }
}

void ErrorCounters::bump(ErrCode code) {
    const int slot = static_cast<int>(code);
    if (slot <= 0 || slot >= SLOTS) {
        return;
    }
    counts_[slot].fetch_add(1);
}

uint32_t ErrorCounters::get(ErrCode code) const {
    const int slot = static_cast<int>(code);
    if (slot <= 0 || slot >= SLOTS) {
        return 0;
    }
    return counts_[slot].load();
}

LineAssembler::LineAssembler() {
    reset();
}

void LineAssembler::reset() {
    line_[0]    = '\0';
    length_     = 0;
    dropping_   = false;
    overflowed_ = false;
    restart_    = false;
}

bool LineAssembler::push(char byte) {
    if (restart_) {
        line_[0]    = '\0';
        length_     = 0;
        dropping_   = false;
        overflowed_ = false;
        restart_    = false;
    }
    if (byte == '\r') {
        return false;   // tolerate a CRLF sender
    }
    if (byte == '\n') {
        overflowed_ = dropping_;
        if (dropping_) {
            length_ = 0;
        }
        line_[length_] = '\0';
        restart_       = true;
        return true;
    }
    if (dropping_) {
        return false;
    }
    if (length_ >= cfg::MAX_LINE_LEN) {
        dropping_ = true;
        LINK_DLOG("LineAssembler: line longer than %d chars, dropping\n", cfg::MAX_LINE_LEN);
        return false;
    }
    line_[length_++] = byte;
    return false;
}

bool isCommandChar(char value) {
    return value == 'M' || value == 'F' || value == 'T' || value == 'V' || value == 'S' ||
           value == 'R';
}

bool parseCommand(const char* line, Command* out) {
    if (line == nullptr || out == nullptr || line[0] == '\0') {
        return false;
    }

    const char* args = line + 1;
    float       values[3] = {0.0f, 0.0f, 0.0f};

    switch (line[0]) {
        case 'M':
            if (!readArgs(args, 3, values)) {
                return false;
            }
            if (!withinAbs(values[0], cfg::MAX_LINEAR_SPEED_M_S) ||
                !withinAbs(values[1], cfg::MAX_LINEAR_SPEED_M_S) ||
                !withinAbs(values[2], cfg::MAX_YAW_RATE_RAD_S)) {
                LINK_DLOG("parseCommand: M out of range\n");
                return false;
            }
            out->kind = CmdKind::VELOCITY;
            break;

        case 'F':
            if (!readArgs(args, 2, values)) {
                return false;
            }
            if (!withinAbs(values[0], cfg::MAX_DISTANCE_M) ||
                !isMoveRate(values[1], cfg::MAX_LINEAR_SPEED_M_S)) {
                LINK_DLOG("parseCommand: F out of range\n");
                return false;
            }
            out->kind = CmdKind::FORWARD;
            break;

        case 'T':
            if (!readArgs(args, 2, values)) {
                return false;
            }
            if (!withinAbs(values[0], cfg::MAX_TURN_DEG) ||
                !isMoveRate(values[1], cfg::MAX_YAW_RATE_RAD_S)) {
                LINK_DLOG("parseCommand: T out of range\n");
                return false;
            }
            values[0] *= rm::cfg::DEG_TO_RAD;   // wire degrees -> struct radians
            out->kind = CmdKind::TURN;
            break;

        case 'V':
            if (!readArgs(args, 2, values)) {
                return false;
            }
            if (!withinAbs(values[0], cfg::MAX_SERVO_RATE_DEG_S) ||
                !withinAbs(values[1], cfg::MAX_SERVO_RATE_DEG_S)) {
                LINK_DLOG("parseCommand: V out of range\n");
                return false;
            }
            out->kind = CmdKind::SERVO;
            break;

        case 'S':
            if (!isBlank(args)) {
                return false;
            }
            out->kind = CmdKind::STOP;
            break;

        case 'R':
            if (!isBlank(args)) {
                return false;
            }
            out->kind = CmdKind::RESET;
            break;

        default:
            LINK_DLOG("parseCommand: unknown command '%c'\n", line[0]);
            return false;
    }

    out->a = values[0];
    out->b = values[1];
    out->c = values[2];
    return true;
}

bool parseTelemetry(const char* line, Telemetry* out) {
    if (line == nullptr || out == nullptr || line[0] == '\0') {
        return false;
    }

    // Checked before the switch: READY also begins with 'R'.
    if (std::strncmp(line, READY_TEXT, READY_LEN) == 0 && isBlank(line + READY_LEN)) {
        out->kind = TeleKind::READY;
        return true;
    }

    const char* args = line + 1;
    float       values[3] = {0.0f, 0.0f, 0.0f};

    switch (line[0]) {
        case 'O':
            if (!readArgs(args, 3, values)) {
                return false;
            }
            out->kind = TeleKind::ODOM;
            out->a    = values[0];
            out->b    = values[1];
            out->c    = values[2] * rm::cfg::DEG_TO_RAD;   // wire degrees -> struct radians
            return true;

        case 'P':
            if (!readArgs(args, 2, values)) {
                return false;
            }
            out->kind = TeleKind::SERVO;
            out->a    = values[0];
            out->b    = values[1];
            return true;

        case 'K':
            if (!isBlank(args)) {
                return false;
            }
            out->kind = TeleKind::ACK;
            return true;

        case 'E': {
            // Parsed as integers, not floats: a counter above 2^24 would lose
            // its low digits if it went through a float.
            char*      codeEnd  = nullptr;
            const long code     = std::strtol(args, &codeEnd, 10);
            if (codeEnd == args) {
                return false;
            }
            char*               countEnd = nullptr;
            const unsigned long count    = std::strtoul(codeEnd, &countEnd, 10);
            if (countEnd == codeEnd || !isBlank(countEnd)) {
                return false;
            }
            if (code <= 0 || code >= static_cast<long>(ErrCode::COUNT)) {
                return false;
            }
            out->kind  = TeleKind::FAULT;
            out->code  = static_cast<ErrCode>(code);
            out->count = static_cast<uint32_t>(count);
            return true;
        }

        default:
            return false;
    }
}

int formatCommand(const Command& cmd, char* out, int cap) {
    LineWriter writer(out, cap);
    switch (cmd.kind) {
        case CmdKind::VELOCITY:
            writer.putText("M ");
            writer.putFloat(cmd.a);
            writer.putChar(' ');
            writer.putFloat(cmd.b);
            writer.putChar(' ');
            writer.putFloat(cmd.c);
            break;

        case CmdKind::FORWARD:
            writer.putText("F ");
            writer.putFloat(cmd.a);
            writer.putChar(' ');
            writer.putFloat(cmd.b);
            break;

        case CmdKind::TURN:
            writer.putText("T ");
            writer.putFloat(cmd.a * rm::cfg::RAD_TO_DEG);   // struct radians -> wire degrees
            writer.putChar(' ');
            writer.putFloat(cmd.b);
            break;

        case CmdKind::SERVO:
            writer.putText("V ");
            writer.putFloat(cmd.a);
            writer.putChar(' ');
            writer.putFloat(cmd.b);
            break;

        case CmdKind::STOP:
            writer.putChar('S');
            break;

        case CmdKind::RESET:
            writer.putChar('R');
            break;

        default:
            return 0;
    }
    return writer.finish();
}

int formatOdom(float x, float y, float thetaRad, char* out, int cap) {
    LineWriter writer(out, cap);
    writer.putText("O ");
    writer.putFloat(x);
    writer.putChar(' ');
    writer.putFloat(y);
    writer.putChar(' ');
    writer.putFloat(thetaRad * rm::cfg::RAD_TO_DEG);
    return writer.finish();
}

int formatServo(float panDeg, float tiltDeg, char* out, int cap) {
    LineWriter writer(out, cap);
    writer.putText("P ");
    writer.putFloat(panDeg);
    writer.putChar(' ');
    writer.putFloat(tiltDeg);
    return writer.finish();
}

int formatAck(char* out, int cap) {
    LineWriter writer(out, cap);
    writer.putChar('K');
    return writer.finish();
}

int formatReady(char* out, int cap) {
    LineWriter writer(out, cap);
    writer.putText(READY_TEXT);
    return writer.finish();
}

int formatFault(ErrCode code, uint32_t count, char* out, int cap) {
    if (code == ErrCode::NONE || code >= ErrCode::COUNT) {
        return 0;
    }
    LineWriter writer(out, cap);
    writer.putText("E ");
    writer.putInt(static_cast<uint32_t>(code));
    writer.putChar(' ');
    writer.putInt(count);
    return writer.finish();
}

}  // namespace link
