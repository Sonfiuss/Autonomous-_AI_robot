// LINK unit tests: plain asserts, no framework, so they build anywhere:
//   g++ -std=c++14 -Iinclude -Iconfig src/LINK/protocol.cpp tests/test_link.cpp -o test_link
#include <cmath>
#include <cstdio>
#include <cstring>

#include "LINK/protocol.h"

namespace {

constexpr float EPS_WIRE  = 1e-3f;   // one unit in the last formatted decimal
constexpr float EPS_ANGLE = 1e-4f;   // 0.001 deg expressed in radians
constexpr int   BUF_CAP   = 96;
constexpr int   TIGHT_CAP = 4;       // too small for any real line

int g_failures = 0;

void expectTrue(bool cond, const char* what) {
    if (!cond) {
        std::printf("  FAIL %s\n", what);
        ++g_failures;
    }
}

void expectNear(float actual, float expected, float eps, const char* what) {
    if (std::fabs(actual - expected) > eps) {
        std::printf("  FAIL %s: got %.5f expected %.5f (eps %.1e)\n", what, actual, expected, eps);
        ++g_failures;
    }
}

void expectText(const char* actual, const char* expected, const char* what) {
    if (std::strcmp(actual, expected) != 0) {
        std::printf("  FAIL %s: got \"%s\" expected \"%s\"\n", what, actual, expected);
        ++g_failures;
    }
}

// Pushes every byte of `text` and returns the number of completed lines.
int feed(link::LineAssembler* assembler, const char* text) {
    int lines = 0;
    for (const char* cursor = text; *cursor != '\0'; ++cursor) {
        if (assembler->push(*cursor)) {
            ++lines;
        }
    }
    return lines;
}

// Formats a command, pushes it through the assembler and parses it back.
bool roundTrip(const link::Command& sent, link::Command* received) {
    char buffer[BUF_CAP];
    if (link::formatCommand(sent, buffer, BUF_CAP) <= 0) {
        return false;
    }
    link::LineAssembler assembler;
    if (feed(&assembler, buffer) != 1 || assembler.overflowed()) {
        return false;
    }
    return link::parseCommand(assembler.line(), received);
}

bool parsesCommand(const char* line) {
    link::Command cmd;
    return link::parseCommand(line, &cmd);
}

bool parsesTelemetry(const char* line) {
    link::Telemetry tele;
    return link::parseTelemetry(line, &tele);
}

void testCommandRoundTrip() {
    std::printf("command round trip\n");

    link::Command velocity;
    velocity.kind = link::CmdKind::VELOCITY;
    velocity.a = 0.15f;
    velocity.b = -0.02f;
    velocity.c = 0.8f;
    link::Command back;
    expectTrue(roundTrip(velocity, &back), "M round trips");
    expectTrue(back.kind == link::CmdKind::VELOCITY, "M keeps its kind");
    expectNear(back.a, velocity.a, EPS_WIRE, "M vx");
    expectNear(back.b, velocity.b, EPS_WIRE, "M vy");
    expectNear(back.c, velocity.c, EPS_WIRE, "M omega");

    link::Command forward;
    forward.kind = link::CmdKind::FORWARD;
    forward.a = 1.5f;
    forward.b = 0.15f;
    expectTrue(roundTrip(forward, &back), "F round trips");
    expectTrue(back.kind == link::CmdKind::FORWARD, "F keeps its kind");
    expectNear(back.a, forward.a, EPS_WIRE, "F distance");
    expectNear(back.b, forward.b, EPS_WIRE, "F speed");

    // The struct is radians on both sides; the wire in between is degrees.
    link::Command turn;
    turn.kind = link::CmdKind::TURN;
    turn.a = 90.0f * rm::cfg::DEG_TO_RAD;
    turn.b = 0.8f;
    expectTrue(roundTrip(turn, &back), "T round trips");
    expectNear(back.a, turn.a, EPS_ANGLE, "T angle survives deg<->rad");

    char buffer[BUF_CAP];
    expectTrue(link::formatCommand(turn, buffer, BUF_CAP) > 0, "T formats");
    expectText(buffer, "T 90.000 0.800\n", "T is degrees on the wire");

    link::Command stop;
    stop.kind = link::CmdKind::STOP;
    expectTrue(roundTrip(stop, &back), "S round trips");
    expectTrue(back.kind == link::CmdKind::STOP, "S keeps its kind");

    link::Command reset;
    reset.kind = link::CmdKind::RESET;
    expectTrue(roundTrip(reset, &back), "R round trips");
    expectTrue(back.kind == link::CmdKind::RESET, "R keeps its kind");
}

void testCommandRejection() {
    std::printf("malformed commands\n");

    expectTrue(!parsesCommand("M 1.0 2.0"), "M with two arguments is rejected");
    expectTrue(!parsesCommand("M 1.0 2.0 3.0 4.0"), "M with four arguments is rejected");
    expectTrue(!parsesCommand("M 1.0 2.0 junk"), "M with junk is rejected");
    expectTrue(!parsesCommand("M 1.0 2.0 3.0x"), "M with a trailing character is rejected");
    expectTrue(!parsesCommand("M 99.0 0.0 0.0"), "M above the speed range is rejected");
    expectTrue(!parsesCommand("M nan 0.0 0.0"), "M with NaN is rejected");
    expectTrue(!parsesCommand("M inf 0.0 0.0"), "M with inf is rejected");
    expectTrue(!parsesCommand("F 1.0 0.0"), "F with a zero speed is rejected");
    expectTrue(!parsesCommand("F 1.0 -0.2"), "F with a negative speed is rejected");
    expectTrue(!parsesCommand("T 90.0 0.0"), "T with a zero rate is rejected");
    expectTrue(!parsesCommand("V 900.0 0.0"), "V above the servo rate is rejected");
    expectTrue(!parsesCommand("S 1"), "S with an argument is rejected");
    expectTrue(!parsesCommand("Z 1 2 3"), "an unknown command is rejected");
    expectTrue(!parsesCommand(""), "an empty line is rejected");
    expectTrue(!parsesCommand("M"), "M with no arguments is rejected");

    expectTrue(parsesCommand("M 0.1  0.2   0.3"), "extra spaces between arguments are fine");
    expectTrue(parsesCommand("M 0.1 0.2 0.3   "), "trailing spaces are fine");

    // A receiver counts a bad M as malformed but a bad Z as unknown, and this
    // is what tells the two apart.
    expectTrue(link::isCommandChar('M'), "M is a command character");
    expectTrue(link::isCommandChar('R'), "R is a command character");
    expectTrue(!link::isCommandChar('Z'), "Z is not a command character");
    expectTrue(!link::isCommandChar('O'), "telemetry characters are not commands");
}

void testAssembler() {
    std::printf("line assembler\n");

    link::LineAssembler assembler;
    expectTrue(feed(&assembler, "S\r\n") == 1, "CRLF produces exactly one line");
    expectText(assembler.line(), "S", "the carriage return is swallowed");

    assembler.reset();
    expectTrue(feed(&assembler, "M 0.1 0.2") == 0, "a line without a newline is not complete");
    expectTrue(feed(&assembler, " 0.3\n") == 1, "the line completes on the second chunk");
    link::Command cmd;
    expectTrue(link::parseCommand(assembler.line(), &cmd), "a split line still parses");
    expectNear(cmd.c, 0.3f, EPS_WIRE, "the split argument survived");

    // An over-long line must be dropped whole, and must not corrupt the next one.
    assembler.reset();
    char flood[link::cfg::MAX_LINE_LEN + 8];
    std::memset(flood, 'x', sizeof(flood));
    flood[sizeof(flood) - 2] = '\n';
    flood[sizeof(flood) - 1] = '\0';
    expectTrue(feed(&assembler, flood) == 1, "the over-long line still completes");
    expectTrue(assembler.overflowed(), "the over-long line is flagged as overflowed");
    expectTrue(assembler.length() == 0, "the over-long line is dropped, not truncated");
    expectTrue(feed(&assembler, "S\n") == 1, "the next line completes normally");
    expectTrue(!assembler.overflowed(), "the next line is not flagged");
    expectText(assembler.line(), "S", "the parser resynchronised after the overflow");

    assembler.reset();
    expectTrue(feed(&assembler, "\n") == 1, "a bare newline completes an empty line");
    expectTrue(!parsesCommand(assembler.line()), "the empty line does not parse as a command");
}

void testTelemetry() {
    std::printf("telemetry\n");

    char buffer[BUF_CAP];
    const float theta = 89.1f * rm::cfg::DEG_TO_RAD;
    expectTrue(link::formatOdom(1.234f, -0.567f, theta, buffer, BUF_CAP) > 0, "O formats");
    expectText(buffer, "O 1.234 -0.567 89.100\n", "O is metres and degrees on the wire");

    link::LineAssembler assembler;
    expectTrue(feed(&assembler, buffer) == 1, "O completes one line");
    link::Telemetry tele;
    expectTrue(link::parseTelemetry(assembler.line(), &tele), "O parses");
    expectTrue(tele.kind == link::TeleKind::ODOM, "O keeps its kind");
    expectNear(tele.a, 1.234f, EPS_WIRE, "O x");
    expectNear(tele.c, theta, EPS_ANGLE, "O theta comes back in radians");

    expectTrue(link::formatServo(12.34f, -5.67f, buffer, BUF_CAP) > 0, "P formats");
    expectText(buffer, "P 12.340 -5.670\n", "P wire text");
    expectTrue(parsesTelemetry("P 12.34 -5.67"), "P parses");

    expectTrue(link::formatAck(buffer, BUF_CAP) > 0, "K formats");
    expectText(buffer, "K\n", "K wire text");
    expectTrue(parsesTelemetry("K"), "K parses");

    expectTrue(link::formatReady(buffer, BUF_CAP) > 0, "READY formats");
    expectText(buffer, "READY\n", "READY wire text");
    expectTrue(parsesTelemetry("READY"), "READY parses");
    // READY and the reset command share a first letter; the telemetry side must
    // not mistake one for the other.
    expectTrue(!parsesTelemetry("R"), "a bare R is not telemetry");

    expectTrue(!parsesTelemetry("O 1.0 2.0"), "O with two arguments is rejected");
    expectTrue(!parsesTelemetry("Q 1 2"), "an unknown telemetry line is rejected");
}

void testFault() {
    std::printf("fault lines\n");

    char buffer[BUF_CAP];
    expectTrue(link::formatFault(link::ErrCode::QUEUE_FULL, 7, buffer, BUF_CAP) > 0, "E formats");
    expectText(buffer, "E 4 7\n", "E wire text");

    link::Telemetry tele;
    expectTrue(link::parseTelemetry("E 4 7", &tele), "E parses");
    expectTrue(tele.kind == link::TeleKind::FAULT, "E keeps its kind");
    expectTrue(tele.code == link::ErrCode::QUEUE_FULL, "E code");
    expectTrue(tele.count == 7u, "E count");

    // A float would have rounded this; the count is parsed as an integer.
    const uint32_t bigCount = 16777219u;   // 2^24 + 3
    expectTrue(link::formatFault(link::ErrCode::TX_DROPPED, bigCount, buffer, BUF_CAP) > 0,
               "a large E count formats");
    expectTrue(link::parseTelemetry("E 5 16777219", &tele), "a large E count parses");
    expectTrue(tele.count == bigCount, "a large E count keeps every digit");

    expectTrue(!parsesTelemetry("E 0 1"), "error code 0 is rejected");
    expectTrue(!parsesTelemetry("E 99 1"), "an out-of-range error code is rejected");
    expectTrue(!parsesTelemetry("E 4"), "E with one argument is rejected");
    expectTrue(link::formatFault(link::ErrCode::NONE, 1, buffer, BUF_CAP) == 0,
               "formatting code NONE fails");
}

void testFormatting() {
    std::printf("number formatting\n");

    char buffer[BUF_CAP];
    // A fractional part must keep its leading zeros: .007, never .7
    expectTrue(link::formatServo(-0.007f, 0.0f, buffer, BUF_CAP) > 0, "small value formats");
    expectText(buffer, "P -0.007 0.000\n", "leading zeros in the fraction are kept");

    expectTrue(link::formatServo(1.23456f, 0.0f, buffer, BUF_CAP) > 0, "rounding formats");
    expectText(buffer, "P 1.235 0.000\n", "the last decimal is rounded, not truncated");

    // A buffer too small must fail loudly rather than emit a half line.
    char tight[TIGHT_CAP];
    expectTrue(link::formatOdom(1.0f, 2.0f, 0.0f, tight, TIGHT_CAP) == 0,
               "a short buffer returns 0");
    expectTrue(link::formatAck(nullptr, BUF_CAP) == 0, "a null buffer returns 0");
}

void testCounters() {
    std::printf("error counters\n");

    link::ErrorCounters counters;
    expectTrue(counters.get(link::ErrCode::MALFORMED_LINE) == 0u, "counters start at zero");
    counters.bump(link::ErrCode::MALFORMED_LINE);
    counters.bump(link::ErrCode::MALFORMED_LINE);
    counters.bump(link::ErrCode::TX_DROPPED);
    expectTrue(counters.get(link::ErrCode::MALFORMED_LINE) == 2u, "a counter accumulates");
    expectTrue(counters.get(link::ErrCode::TX_DROPPED) == 1u, "counters are independent");
    counters.bump(link::ErrCode::NONE);
    expectTrue(counters.get(link::ErrCode::NONE) == 0u, "code NONE is not a slot");
}

}  // namespace

int main() {
    testCommandRoundTrip();
    testCommandRejection();
    testAssembler();
    testTelemetry();
    testFault();
    testFormatting();
    testCounters();

    if (g_failures == 0) {
        std::printf("test_link: all checks passed\n");
        return 0;
    }
    std::printf("test_link: %d check(s) FAILED\n", g_failures);
    return 1;
}
