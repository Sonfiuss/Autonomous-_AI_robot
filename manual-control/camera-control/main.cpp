#include "serial_link.h"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <string>

// ── Configuration ─────────────────────────────────────────────────────────────
static constexpr float STEP_DEG = 1.0f;    // degrees moved per key press
static constexpr int   LOOP_HZ  = 60;       // input-poll rate
static const char*     DEFAULT_PORT = "/dev/ttyUSB0";
// ──────────────────────────────────────────────────────────────────────────────

static struct termios orig_termios;
static volatile sig_atomic_t g_running = 1;

static void enableRawMode() {
    tcgetattr(STDIN_FILENO, &orig_termios);
    struct termios raw = orig_termios;
    raw.c_lflag &= ~static_cast<unsigned>(ECHO | ICANON);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
}

static void restoreTerminal() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

static void onSignal(int) { g_running = 0; }

int main(int argc, char** argv) {
    std::string port = DEFAULT_PORT;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc)
            port = argv[++i];
    }

    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    SerialLink link(port);
    printf("Connecting to ESP32 on %s ...\n", port.c_str());
    if (!link.connect(5.0)) {
        fprintf(stderr, "Failed to open %s\n", port.c_str());
        return 1;
    }

    enableRawMode();
    printf("=== Camera Servo Control (ESP32 / PCA9685) ===\n");
    printf("  A / D  :  pan  left / right   (1° per press)\n");
    printf("  W / S  :  tilt up   / down    (1° per press)\n");
    printf("  R      :  reset to center\n");
    printf("  Q      :  quit\n");
    printf("Step: %.1f° / press | Pan [-80, 80] | Tilt [-70, 30]\n\n", STEP_DEG);

    using Clock = std::chrono::steady_clock;
    using Ms    = std::chrono::milliseconds;
    const auto tick = Ms(1000 / LOOP_HZ);

    while (g_running) {
        auto loopStart = Clock::now();

        // Each character is one key press → one step. Sum the presses that
        // arrived this tick into a single relative move.
        float dpan = 0.f, dtilt = 0.f;
        char ch;
        while (read(STDIN_FILENO, &ch, 1) == 1) {
            switch (ch) {
                case 'd': case 'D': dpan  += STEP_DEG; break;
                case 'a': case 'A': dpan  -= STEP_DEG; break;
                case 'w': case 'W': dtilt += STEP_DEG; break;
                case 's': case 'S': dtilt -= STEP_DEG; break;
                case 'r': case 'R': link.reset();      break;
                case 'q': case 'Q': case '\x03': g_running = 0; break;
            }
        }

        if (dpan != 0.f || dtilt != 0.f)
            link.step(dpan, dtilt);

        auto st = link.getState();
        printf("\rPan: %7.2f°   Tilt: %7.2f°   ", st.pan, st.tilt);
        fflush(stdout);

        auto elapsed = Clock::now() - loopStart;
        if (elapsed < tick) std::this_thread::sleep_for(tick - elapsed);
    }

    restoreTerminal();
    printf("\nStopped.\n");
    link.disconnect();
    return 0;
}
