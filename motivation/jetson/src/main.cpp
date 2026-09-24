// Command-line driver for the ESP32 link. Enough to prove the wire works and to
// move the robot by hand; the navigation stack drives RobotLink directly.
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "jetson/mission_runner.h"
#include "jetson/robot_link.h"

namespace {

constexpr int      POLL_MS          = 20;
constexpr uint32_t ACK_TIMEOUT_MS   = 60000;
constexpr uint32_t SETTLE_MS        = 300;   // let telemetry arrive before exiting
constexpr uint32_t PRINT_PERIOD_MS  = 200;
constexpr int      MAX_PLAN_PRIMS   = seq::cfg::MAX_PRIMITIVES;

// One plan, read from a file. Static because it is far too big for the stack and
// this program runs exactly one plan.
mc::Primitive g_plan[MAX_PLAN_PRIMS];

volatile sig_atomic_t g_interrupted = 0;

void onInterrupt(int) {
    g_interrupted = 1;
}

void printUsage() {
    std::printf(
        "usage: robot_link [--port DEV] COMMAND\n"
        "  --monitor                 print telemetry until Ctrl-C\n"
        "  --vel VX VY OMEGA         stream a body-frame velocity until Ctrl-C\n"
        "  --servo PAN TILT          stream a servo rate (deg/s) until Ctrl-C\n"
        "  --forward DIST [SPEED]    drive DIST metres, wait for the ack\n"
        "  --turn DEG [RATE]         turn DEG degrees, wait for the ack\n"
        "  --stop                    stop everything\n"
        "  --reset                   zero the odometry and the servos\n"
        "  --run-plan FILE           drive an MV primitive list, leg by leg\n"
        "                            options: --start-theta DEG, --speed M_S, --yaw-rate RAD_S\n"
        "FILE holds one primitive per line (ROTATE rad | FORWARD m | MOVE dx dy | STOP);\n"
        "any other line is ignored, so `mv_cli plan ...` output can be fed in as it is.\n"
        "default port: %s\n",
        jetson::cfg::DEFAULT_DEVICE);
}

void printState(const jetson::RobotLink& link) {
    const jetson::RobotState& state = link.state();
    std::printf("pose %7.3f %7.3f %7.1fdeg | servo %6.1f %6.1f | ready %d | acks %u",
                static_cast<double>(state.pose.x), static_cast<double>(state.pose.y),
                static_cast<double>(state.pose.theta * rm::cfg::RAD_TO_DEG),
                static_cast<double>(state.panDeg), static_cast<double>(state.tiltDeg),
                state.ready ? 1 : 0, state.acks);
    for (int code = 1; code < jetson::cfg::ERR_SLOTS; ++code) {
        if (state.errorCounts[code] != 0) {
            std::printf(" | E%d=%u", code, state.errorCounts[code]);
        }
    }
    if (link.decodeErrors() != 0) {
        std::printf(" | undecodable=%u", link.decodeErrors());
    }
    std::printf("\n");
}

// Pumps the link until interrupted, printing the state a few times a second.
bool pumpUntilInterrupted(jetson::RobotLink* link) {
    uint32_t lastPrint = 0;
    while (g_interrupted == 0) {
        if (!link->poll(POLL_MS)) {
            std::fprintf(stderr, "link error\n");
            return false;
        }
        const uint32_t now = jetson::monotonicMs();
        if (now - lastPrint >= PRINT_PERIOD_MS) {
            lastPrint = now;
            printState(*link);
        }
    }
    return true;
}

// Waits for the firmware to acknowledge a move, so the tool exits when the
// robot has actually finished rather than when the line was sent.
bool waitForAck(jetson::RobotLink* link, uint32_t acksBefore) {
    const uint32_t started = jetson::monotonicMs();
    while (g_interrupted == 0 && jetson::monotonicMs() - started < ACK_TIMEOUT_MS) {
        if (!link->poll(POLL_MS)) {
            std::fprintf(stderr, "link error\n");
            return false;
        }
        if (link->state().acks != acksBefore) {
            printState(*link);
            return true;
        }
    }
    std::fprintf(stderr, "no ack from the firmware\n");
    return false;
}

// Drains telemetry for a moment so a one-shot command's effect is visible.
void settle(jetson::RobotLink* link) {
    const uint32_t started = jetson::monotonicMs();
    while (jetson::monotonicMs() - started < SETTLE_MS) {
        if (!link->poll(POLL_MS)) {
            return;
        }
    }
    printState(*link);
}

float toFloat(const char* text) {
    return static_cast<float>(std::atof(text));
}

// Finds an option anywhere in the argument list. The plan options may follow
// --run-plan in any order, so they are not read positionally like the rest.
// Leaves *out alone when the option is absent, which is how a caller keeps a
// default without having to name it twice.
void optionValue(int argc, char** argv, const char* name, float* out) {
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::strcmp(argv[index], name) == 0) {
            *out = toFloat(argv[index + 1]);
            return;
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    const char* device = jetson::cfg::DEFAULT_DEVICE;
    int         index  = 1;
    if (argc > 2 && std::strcmp(argv[1], "--port") == 0) {
        device = argv[2];
        index  = 3;
    }
    if (index >= argc) {
        printUsage();
        return 1;
    }

    std::signal(SIGINT, onInterrupt);
    std::signal(SIGTERM, onInterrupt);

    jetson::RobotLink link;
    if (!link.connect(device)) {
        return 1;
    }

    const char* command   = argv[index];
    const int   remaining = argc - index - 1;
    bool        ok        = true;

    if (std::strcmp(command, "--monitor") == 0) {
        ok = pumpUntilInterrupted(&link);
    } else if (std::strcmp(command, "--vel") == 0 && remaining >= 3) {
        ok = link.sendVelocity(toFloat(argv[index + 1]), toFloat(argv[index + 2]),
                               toFloat(argv[index + 3])) &&
             pumpUntilInterrupted(&link);
        link.sendStop();
    } else if (std::strcmp(command, "--servo") == 0 && remaining >= 2) {
        ok = link.sendServo(toFloat(argv[index + 1]), toFloat(argv[index + 2])) &&
             pumpUntilInterrupted(&link);
        link.sendStop();
    } else if (std::strcmp(command, "--forward") == 0 && remaining >= 1) {
        const float speed =
            remaining >= 2 ? toFloat(argv[index + 2]) : mc::cfg::CRUISE_SPEED_M_S;
        const uint32_t acksBefore = link.state().acks;
        ok = link.sendForward(toFloat(argv[index + 1]), speed) && waitForAck(&link, acksBefore);
    } else if (std::strcmp(command, "--turn") == 0 && remaining >= 1) {
        const float rate = remaining >= 2 ? toFloat(argv[index + 2]) : mc::cfg::YAW_RATE_RAD_S;
        const float angle = toFloat(argv[index + 1]) * rm::cfg::DEG_TO_RAD;
        const uint32_t acksBefore = link.state().acks;
        ok = link.sendTurn(angle, rate) && waitForAck(&link, acksBefore);
    } else if (std::strcmp(command, "--stop") == 0) {
        ok = link.sendStop();
        settle(&link);
    } else if (std::strcmp(command, "--reset") == 0) {
        ok = link.sendReset();
        settle(&link);
    } else if (std::strcmp(command, "--run-plan") == 0 && remaining >= 1) {
        const int count = jetson::readPlanFile(argv[index + 1], g_plan, MAX_PLAN_PRIMS);
        if (count <= 0) {
            ok = false;
        } else {
            // The plan was planned from some heading; without it every MOVE leg
            // would be placed against the wrong bearing.
            float       startThetaDeg = 0.0f;
            seq::Config planConfig;
            optionValue(argc, argv, "--start-theta", &startThetaDeg);
            optionValue(argc, argv, "--speed", &planConfig.cruiseSpeed);
            optionValue(argc, argv, "--yaw-rate", &planConfig.yawRate);

            jetson::MissionRunner runner(&link);
            ok = runner.run(g_plan, count, startThetaDeg * rm::cfg::DEG_TO_RAD, planConfig,
                            &g_interrupted);
            printState(link);
        }
    } else {
        printUsage();
        ok = false;
    }

    link.disconnect();
    return ok ? 0 : 1;
}
