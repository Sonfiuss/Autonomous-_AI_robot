// Mission_Runner — the platform half of the sequencer: it owns the clock, the
// port and the loop, and asks seq::Sequencer what to send.
//
// The split is deliberate. Everything that can be got wrong about the handshake
// (when a leg may go out, when a leg has failed, what a fault means) lives in
// SEQ, where it is tested on a PC with no robot attached. What is left here is
// the part that cannot be tested without hardware: poll the port, read the
// clock, print progress. Keeping that part this thin is the point.
#ifndef JETSON_MISSION_RUNNER_H
#define JETSON_MISSION_RUNNER_H

#include <csignal>

#include "MC/types.h"
#include "SEQ/sequencer.h"
#include "jetson/robot_link.h"

namespace jetson {

namespace cfg {

// How often the runner polls the port. Well inside the firmware's ack cadence,
// and the same interval the CLI already uses for its own loops.
constexpr int MISSION_POLL_MS = 20;

}  // namespace cfg

class MissionRunner {
public:
    // Borrows the link; it must stay connected for the whole run.
    explicit MissionRunner(RobotLink* link);

    // Drives `primitives` to completion. `startTheta` is the heading the plan was
    // planned from, in radians. Returns true only when every leg was acknowledged;
    // on failure the reason is already on stderr and the robot has been told to
    // stop. `interrupted` lets a signal handler cut the run short -- pass nullptr
    // if the caller has no such flag.
    bool run(const mc::Primitive* primitives, int count, float startTheta,
             const seq::Config& config, const volatile sig_atomic_t* interrupted);

    // Why the last run() ended.
    seq::Status status() const { return status_; }

private:
    // Everything the sequencer needs to know about the link right now.
    seq::Feedback sample(bool linkOk) const;

    RobotLink*  link_;
    seq::Status status_;
};

// Reads a primitive list from a text file. One primitive per line:
//
//     ROTATE <rad>
//     FORWARD <m>
//     MOVE <dx> <dy>
//     STOP
//
// Leading whitespace is ignored and any line that does not start with one of
// those four words is skipped, which is exactly what makes `mv_cli plan ...`
// output usable as-is: its headers and coordinate rows fall through untouched.
//
// Returns the number of primitives read, or -1 on a file that cannot be opened,
// a malformed primitive line, or more primitives than `capacity`.
int readPlanFile(const char* path, mc::Primitive* out, int capacity);

}  // namespace jetson

#endif  // JETSON_MISSION_RUNNER_H
