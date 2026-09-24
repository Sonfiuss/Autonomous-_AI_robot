#include "jetson/mission_runner.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace jetson {

namespace {

constexpr int LINE_CAP = 256;

// seq::Feedback carries one slot per link::ErrCode, and so does RobotState. If
// the two ever drifted the fault copy below would read past one of them.
static_assert(static_cast<int>(link::ErrCode::COUNT) == cfg::ERR_SLOTS,
              "RobotState and seq::Feedback must agree on the number of error slots");

// Puts one sequencer command on the wire. Going through the typed setters rather
// than a raw write keeps RobotLink's keep-alive state honest: each of these
// cancels the streamed velocity, which is what stops a keep-alive restarting the
// robot the moment a leg ends.
bool sendCommand(RobotLink* link, const link::Command& command) {
    switch (command.kind) {
        case link::CmdKind::FORWARD: return link->sendForward(command.a, command.b);
        case link::CmdKind::TURN:    return link->sendTurn(command.a, command.b);
        case link::CmdKind::STOP:    return link->sendStop();
        case link::CmdKind::RESET:   return link->sendReset();
        default:
            // A plan produces no other kind, so this is a bug rather than input.
            std::fprintf(stderr, "sequencer asked for an unexpected command\n");
            return false;
    }
}

// The four primitive keywords, in the order of mc::PrimitiveType.
const char* const ROTATE_WORD  = "ROTATE";
const char* const FORWARD_WORD = "FORWARD";
const char* const MOVE_WORD    = "MOVE";
const char* const STOP_WORD    = "STOP";

bool startsWith(const char* line, const char* word) {
    return std::strncmp(line, word, std::strlen(word)) == 0;
}

// Reads one primitive out of a line, or reports that the line is not one. `bad`
// separates "this line is not a primitive" (skip it) from "this line names a
// primitive but its arguments are wrong" (fail the file).
bool parsePrimitive(const char* line, mc::Primitive* out, bool* bad) {
    *bad = false;
    while (*line == ' ' || *line == '\t') {
        ++line;
    }

    float a = 0.0f;
    float b = 0.0f;
    if (std::sscanf(line, "ROTATE %f", &a) == 1) {
        *out = mc::Primitive{mc::PrimitiveType::ROTATE, a, 0.0f};
        return true;
    }
    if (std::sscanf(line, "FORWARD %f", &a) == 1) {
        *out = mc::Primitive{mc::PrimitiveType::FORWARD, a, 0.0f};
        return true;
    }
    if (std::sscanf(line, "MOVE %f %f", &a, &b) == 2) {
        *out = mc::Primitive{mc::PrimitiveType::MOVE, a, b};
        return true;
    }
    if (startsWith(line, STOP_WORD)) {
        *out = mc::Primitive{mc::PrimitiveType::STOP, 0.0f, 0.0f};
        return true;
    }

    // A word we recognise whose arguments did not parse is a corrupt plan, not a
    // comment. Saying so beats driving a plan with a leg quietly missing.
    if (startsWith(line, ROTATE_WORD) || startsWith(line, FORWARD_WORD) ||
        startsWith(line, MOVE_WORD)) {
        *bad = true;
    }
    return false;
}

}  // namespace

int readPlanFile(const char* path, mc::Primitive* out, int capacity) {
    if (path == nullptr || out == nullptr || capacity <= 0) {
        return -1;
    }
    std::FILE* file = std::fopen(path, "r");
    if (file == nullptr) {
        std::fprintf(stderr, "cannot open plan file: %s\n", path);
        return -1;
    }

    char line[LINE_CAP];
    int  count   = 0;
    int  lineNum = 0;
    while (std::fgets(line, LINE_CAP, file) != nullptr) {
        ++lineNum;
        mc::Primitive primitive;
        bool          bad = false;
        if (!parsePrimitive(line, &primitive, &bad)) {
            if (bad) {
                std::fprintf(stderr, "%s:%d: malformed primitive\n", path, lineNum);
                std::fclose(file);
                return -1;
            }
            continue;   // a header, a coordinate row, a blank: not for us
        }
        if (count >= capacity) {
            std::fprintf(stderr, "%s: more than %d primitives\n", path, capacity);
            std::fclose(file);
            return -1;
        }
        out[count++] = primitive;
    }
    std::fclose(file);
    return count;
}

MissionRunner::MissionRunner(RobotLink* link) : link_(link), status_(seq::Status::OK) {}

seq::Feedback MissionRunner::sample(bool linkOk) const {
    const RobotState& state = link_->state();

    seq::Feedback feedback;
    feedback.nowMs   = monotonicMs();
    feedback.acks    = state.acks;
    feedback.readies = state.readies;
    feedback.linkOk  = linkOk;
    for (int slot = 0; slot < cfg::ERR_SLOTS; ++slot) {
        feedback.faults[slot] = state.errorCounts[slot];
    }
    return feedback;
}

bool MissionRunner::run(const mc::Primitive* primitives, int count, float startTheta,
                        const seq::Config& config, const volatile sig_atomic_t* interrupted) {
    seq::Sequencer sequencer;
    if (!sequencer.load(primitives, count, startTheta, config)) {
        status_ = sequencer.status();
        std::fprintf(stderr, "plan refused: %s\n", seq::statusText(status_));
        return false;
    }
    std::printf("running %d primitive(s) from heading %.1f deg\n", count,
                static_cast<double>(startTheta * rm::cfg::RAD_TO_DEG));

    bool linkOk    = true;
    bool aborted   = false;
    int  reported  = -1;
    while (sequencer.running()) {
        if (!aborted && interrupted != nullptr && *interrupted != 0) {
            std::printf("interrupted, stopping\n");
            sequencer.abort();
            aborted = true;
        }

        linkOk = link_->poll(cfg::MISSION_POLL_MS);

        const seq::Action action = sequencer.update(sample(linkOk));
        if (action.send && !sendCommand(link_, action.command)) {
            linkOk = false;   // the next update() turns this into LINK_ERROR
        }

        // The index runs one past the last primitive when the plan ends, which is
        // how the sequencer says "done" -- not a primitive to announce.
        if (sequencer.primitiveIndex() != reported && sequencer.primitiveIndex() < count) {
            reported = sequencer.primitiveIndex();
            std::printf("  primitive %d/%d | heading %.1f deg | pose %.3f %.3f\n", reported + 1,
                        count, static_cast<double>(sequencer.heading() * rm::cfg::RAD_TO_DEG),
                        static_cast<double>(link_->state().pose.x),
                        static_cast<double>(link_->state().pose.y));
        }
    }

    // abort() defers its stop to the next update(), and the loop above has just
    // left. Give the sequencer one more turn so the robot is actually told.
    const seq::Action last = sequencer.update(sample(linkOk));
    if (last.send && !sendCommand(link_, last.command)) {
        // The firmware's own watchdog is the backstop, but a stop that never
        // reached the wire is worth saying out loud.
        std::fprintf(stderr, "could not send the final stop\n");
    }

    status_ = sequencer.status();
    if (sequencer.finished()) {
        std::printf("plan complete: %d primitive(s)\n", count);
        return true;
    }
    std::fprintf(stderr, "plan failed at primitive %d/%d: %s\n", sequencer.primitiveIndex() + 1,
                 count, seq::statusText(status_));
    return false;
}

}  // namespace jetson
