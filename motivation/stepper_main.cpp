/**
 * stepper_ctrl — run 3 stepper motors by angle (degrees)
 *
 * Usage:
 *   ./stepper_ctrl <M1_deg> <M2_deg> <M3_deg> [--hz N] [--port DEV]
 *
 * Arguments:
 *   M1_deg   degrees to rotate motor W1 (60°)   — positive=forward, negative=backward
 *   M2_deg   degrees to rotate motor W2 (180°)
 *   M3_deg   degrees to rotate motor W3 (300°)
 *
 * Options:
 *   --hz     step frequency in Hz (default: 3000)
 *   --port   serial port        (default: /dev/ttyUSB0)
 *
 * Examples:
 *   ./stepper_ctrl 360 360 360       # all three motors one full revolution forward
 *   ./stepper_ctrl 90 -90 0          # M1 quarter-turn fwd, M2 quarter-turn back, M3 skip
 *   ./stepper_ctrl 720 720 720 --hz 6000
 */

#include "StepperClient.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>

// Must match PinConfig.h STEPS_PER_REV (200 steps × 64 microstep)
static constexpr int STEPS_PER_REV = 12800;

static int degreesToSteps(float deg) {
    return (int)(deg / 360.f * STEPS_PER_REV);
}

static std::atomic<bool> g_running{true};
static void onSig(int) { g_running = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSig);
    std::signal(SIGTERM, onSig);

    if (argc < 4) {
        fprintf(stderr,
            "Usage: %s <M1_deg> <M2_deg> <M3_deg> [--hz N] [--port DEV]\n"
            "  Example: %s 360 -360 180\n",
            argv[0], argv[0]);
        return 1;
    }

    float m_deg[3] = { (float)atof(argv[1]),
                       (float)atof(argv[2]),
                       (float)atof(argv[3]) };

    uint32_t    hz   = 3000;
    std::string port = "/dev/ttyUSB0";

    for (int i = 4; i < argc; ++i) {
        if (!strcmp(argv[i], "--hz")   && i + 1 < argc) hz   = (uint32_t)atoi(argv[++i]);
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
    }

    int steps[3] = { degreesToSteps(m_deg[0]),
                     degreesToSteps(m_deg[1]),
                     degreesToSteps(m_deg[2]) };

    printf("=== stepper_ctrl ===\n");
    printf("Port : %s   Hz: %u\n", port.c_str(), hz);
    printf("M1 (60°)  : %+.1f°  → %+d steps\n", m_deg[0], steps[0]);
    printf("M2 (180°) : %+.1f°  → %+d steps\n", m_deg[1], steps[1]);
    printf("M3 (300°) : %+.1f°  → %+d steps\n", m_deg[2], steps[2]);

    // If all are zero there is nothing to do
    if (steps[0] == 0 && steps[1] == 0 && steps[2] == 0) {
        printf("All angles are 0 — nothing to move.\n");
        return 0;
    }

    StepperClient sc(port);
    if (!sc.connect()) {
        fprintf(stderr, "Connection failed.\n");
        return 1;
    }

    // Wait for K (all motors idle) or Ctrl-C
    std::mutex              mtx;
    std::condition_variable cv;
    bool done = false;

    sc.setDoneCallback([&] {
        std::lock_guard<std::mutex> lk(mtx);
        done = true;
        cv.notify_one();
    });

    printf("Running...\n");

    // Send all three move commands
    for (int i = 0; i < 3; ++i)
        if (steps[i] != 0)
            sc.moveSteps(i, steps[i], hz);

    // Wait up to 60 seconds for completion
    std::unique_lock<std::mutex> lk(mtx);
    cv.wait_for(lk, std::chrono::seconds(60), [&]{ return done || !g_running; });

    if (done)
        printf("Done.\n");
    else
        printf("Timeout or interrupted — stopping motors.\n");

    sc.stopAll();
    sc.disconnect();
    return done ? 0 : 1;
}
