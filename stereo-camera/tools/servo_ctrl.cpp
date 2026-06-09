/**
 * servo_ctrl — debug tool for ServoController (MOVE/OK step-and-hold protocol).
 *
 * Requires servo_driver.ino flashed on the ESP32.
 *
 * Usage:
 *   ./servo_ctrl [--port /dev/ttyUSB0] D <pan> <tilt>
 *
 * Mode D  (debug / direct move):
 *   Sends MOVE <pan> <tilt>, waits for OK, prints result and elapsed time.
 *   Angles in degrees.  pan: -80..+80,  tilt: -70..+30.
 *
 * Examples:
 *   ./servo_ctrl D 0 0          ← reset to origin
 *   ./servo_ctrl D 45 -30       ← move to pan=45° tilt=-30°
 *   ./servo_ctrl --port /dev/ttyUSB1 D 0 0
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <chrono>

#include "../control/ServoController.hpp"

static void printUsage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [--port <dev>] D <pan_deg> <tilt_deg>\n"
        "  --port   Serial port (default: /dev/ttyUSB0)\n"
        "  D        Debug mode: direct move to absolute angle\n"
        "  pan      Pan  angle in degrees  (-80 .. +80)\n"
        "  tilt     Tilt angle in degrees  (-70 .. +30)\n"
        "\nExample:\n"
        "  %s D 0 0            # reset to origin\n"
        "  %s D 45 -30         # pan=45°  tilt=-30°\n",
        prog, prog, prog);
}

int main(int argc, char* argv[]) {
    std::string port = "/dev/ttyUSB0";
    int arg = 1;

    // Optional --port flag
    if (arg + 1 < argc && strcmp(argv[arg], "--port") == 0) {
        port = argv[arg + 1];
        arg += 2;
    }

    // Expect: D <pan> <tilt>
    if (argc - arg < 3) {
        printUsage(argv[0]);
        return 1;
    }

    char mode = argv[arg][0];
    if (mode != 'D' && mode != 'd') {
        fprintf(stderr, "Unknown mode '%c'. Only 'D' is supported.\n", mode);
        printUsage(argv[0]);
        return 1;
    }

    float pan  = std::atof(argv[arg + 1]);
    float tilt = std::atof(argv[arg + 2]);

    // Soft-warn if out of firmware limits
    if (pan  < -80.f || pan  > 80.f)
        fprintf(stderr, "[WARN] pan=%.1f is outside firmware limit -80..+80\n", pan);
    if (tilt < -70.f || tilt > 30.f)
        fprintf(stderr, "[WARN] tilt=%.1f is outside firmware limit -70..+30\n", tilt);

    printf("[servo_ctrl] mode=D  pan=%.2f°  tilt=%.2f°  port=%s\n",
           pan, tilt, port.c_str());

    ServoController servo(port);
    printf("[servo_ctrl] Connecting...\n");
    if (!servo.connect(3.0)) {
        fprintf(stderr, "[servo_ctrl] Failed to connect to ESP32 on %s\n",
                port.c_str());
        return 1;
    }

    auto t0 = std::chrono::steady_clock::now();
    bool ok = servo.moveTo(pan, tilt, 3.0);
    auto dt = std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - t0).count();

    if (ok) {
        printf("[servo_ctrl] OK  (%.2f s)\n", dt);
    } else {
        fprintf(stderr, "[servo_ctrl] FAILED — no OK from ESP32 within 3 s\n");
        fprintf(stderr, "             Check: servo_driver.ino is flashed? "
                        "Baud=115200? Port correct?\n");
    }

    servo.disconnect();
    return ok ? 0 : 1;
}
