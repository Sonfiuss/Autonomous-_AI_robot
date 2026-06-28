#include "Teleop.hpp"

#include <cstdio>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <chrono>

// ── Terminal raw-mode helpers ─────────────────────────────────────────────────

static termios g_orig_termios;
static bool    g_raw_active = false;

static void enableRaw() {
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    termios raw = g_orig_termios;
    raw.c_lflag  &= ~(tcflag_t)(ICANON | ECHO | ISIG);
    raw.c_iflag  &= ~(tcflag_t)(IXON | ICRNL);
    raw.c_cc[VMIN]  = 0;   // non-blocking: return immediately
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    g_raw_active = true;
}

static void disableRaw() {
    if (g_raw_active) {
        tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
        g_raw_active = false;
    }
}

static int readKey() {
    char c;
    return (read(STDIN_FILENO, &c, 1) == 1) ? (unsigned char)c : -1;
}

// ── Teleop ────────────────────────────────────────────────────────────────────

void runTeleop(MotionClient& mc, std::atomic<bool>& running) {
    float lin_spd  = 0.3f;    // m/s  (wheel linear)
    float ang_spd  = 1.0f;    // rad/s
    float srv_spd  = 30.0f;   // deg/s servo

    float vx = 0.f, vy = 0.f, omega = 0.f;
    float pan_v = 0.f, tilt_v = 0.f;

    printf("\n=== Keyboard Teleop ===\n");
    printf("  W/S     forward / backward       Q/E   rotate CCW / CW\n");
    printf("  A/D     strafe left / right       J/L   pan camera left / right\n");
    printf("  I/K     tilt camera up / down     +/-   increase / decrease speed\n");
    printf("  Space   emergency stop            R     reset odometry\n");
    printf("  X/ESC   quit\n\n");

    enableRaw();

    const auto period = std::chrono::milliseconds(50);   // 20 Hz

    while (running) {
        auto t0 = std::chrono::steady_clock::now();

        int ch = readKey();
        if (ch >= 'A' && ch <= 'Z') ch |= 0x20;   // to lowercase

        switch (ch) {
            // ── Wheel motion ─────────────────────────────────────────────────
            case 'w':  vx =  lin_spd; vy = 0.f;       omega = 0.f;      break;
            case 's':  vx = -lin_spd; vy = 0.f;       omega = 0.f;      break;
            case 'a':  vx = 0.f;      vy =  lin_spd;  omega = 0.f;      break;
            case 'd':  vx = 0.f;      vy = -lin_spd;  omega = 0.f;      break;
            case 'q':  vx = 0.f;      vy = 0.f;       omega =  ang_spd; break;
            case 'e':  vx = 0.f;      vy = 0.f;       omega = -ang_spd; break;

            // ── Camera servo ─────────────────────────────────────────────────
            case 'i':  tilt_v =  srv_spd; pan_v = 0.f;       break;
            case 'k':  tilt_v = -srv_spd; pan_v = 0.f;       break;
            case 'j':  pan_v  = -srv_spd; tilt_v = 0.f;      break;
            case 'l':  pan_v  =  srv_spd; tilt_v = 0.f;      break;

            // ── Stop / reset / speed ─────────────────────────────────────────
            case ' ':
                vx = vy = omega = pan_v = tilt_v = 0.f;
                mc.stop();
                mc.setServoVelocity(0.f, 0.f);
                break;

            case '+': case '=':
                lin_spd = (lin_spd < 1.4f) ? lin_spd + 0.1f : 1.5f;
                break;
            case '-': case '_':
                lin_spd = (lin_spd > 0.2f) ? lin_spd - 0.1f : 0.1f;
                break;

            case 'r':
                mc.resetOdometry();
                break;

            // ── Quit ─────────────────────────────────────────────────────────
            case 'x':
            case 27:   // ESC
                running = false;
                break;

            default: break;
        }

        // Send current velocities every tick (keeps robot moving while held)
        if (ch != 'r' && ch != ' ') {
            mc.setVelocity(vx, vy, omega);
            mc.setServoVelocity(pan_v, tilt_v);
        }

        auto odom = mc.getOdometry();
        printf("\r  spd=%.1fm/s  vel=[%+.2f %+.2f %+.2f]  "
               "odom=[%+.3f %+.3f %+.1f°]  cam=[%+.0f° %+.0f°]   ",
               lin_spd, vx, vy, omega,
               odom.x, odom.y, odom.theta,
               mc.getServoState().pan, mc.getServoState().tilt);
        fflush(stdout);

        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < period)
            std::this_thread::sleep_for(period - elapsed);
    }

    disableRaw();
    mc.stop();
    mc.setServoVelocity(0.f, 0.f);
    printf("\n[teleop] Stopped.\n");
}
