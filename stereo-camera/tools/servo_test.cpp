/**
 * Servo test tool – kiểm tra servo trước khi chạy pipeline đầy đủ.
 *
 * Build:  g++ -std=c++17 -O2 -o servo_test tools/servo_test.cpp control/ServoClient.cpp
 * Run:    ./servo_test --port /dev/ttyUSB0
 *
 * Menu tương tác:
 *   p <vel>   – set pan velocity  (deg/s, âm = trái)
 *   t <vel>   – set tilt velocity (deg/s, âm = xuống)
 *   v <p> <t> – set cả hai cùng lúc
 *   s         – stop (vel = 0)
 *   r         – reset về (0°, 0°)
 *   c         – sweep tự động để kiểm tra giới hạn
 *   q         – thoát
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>

#include "../control/ServoClient.hpp"

static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running = false; }

// ── In trạng thái liên tục ────────────────────────────────────────────────────
static void statusThread(ServoClient& servo) {
    while (g_running) {
        auto s = servo.getState();
        printf("\r  [SERVO]  pan=%+7.2f°   tilt=%+7.2f°   ", s.pan, s.tilt);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── Sweep test ────────────────────────────────────────────────────────────────
static void runSweepTest(ServoClient& servo) {
    printf("\n--- SWEEP TEST ---\n");

    auto wait = [&](int ms) {
        std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    };

    printf("  Pan sweep  +20 deg/s ...\n");
    servo.setVelocity(20.f, 0.f);  wait(4000);

    printf("  Pan sweep  -20 deg/s ...\n");
    servo.setVelocity(-20.f, 0.f); wait(8000);

    printf("  Stop pan, reset ...\n");
    servo.stop(); wait(500);
    servo.reset(); wait(2000);

    printf("  Tilt sweep +10 deg/s ...\n");
    servo.setVelocity(0.f, 10.f);  wait(8000);

    printf("  Stop, reset ...\n");
    servo.stop(); wait(500);
    servo.reset(); wait(2000);

    printf("  Combined sweep pan=15, tilt=7 deg/s ...\n");
    servo.setVelocity(15.f, 7.f);  wait(10000);

    servo.stop(); wait(500);
    servo.reset();
    printf("  Sweep test DONE.\n\n");
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::signal(SIGINT, onSignal);

    std::string port = "/dev/ttyUSB0";
    for (int i = 1; i < argc - 1; ++i)
        if (strcmp(argv[i], "--port") == 0) port = argv[i + 1];

    printf("Connecting to ESP32 on %s ...\n", port.c_str());
    ServoClient servo(port);
    if (!servo.connect(5.0)) {
        fprintf(stderr, "Connection failed.\n");
        return 1;
    }

    // Print live status in background
    std::thread st(statusThread, std::ref(servo));

    printf("\nCommands: p <vel>  t <vel>  v <pan> <tilt>  s  r  c  q\n\n");

    char line[128];
    while (g_running) {
        printf("\n> ");
        fflush(stdout);

        if (!fgets(line, sizeof(line), stdin)) break;

        char cmd = line[0];
        float a = 0.f, b = 0.f;

        if (cmd == 'q') break;
        else if (cmd == 's') { servo.stop();  printf("  Stopped.\n"); }
        else if (cmd == 'r') { servo.reset(); printf("  Reset.\n");   }
        else if (cmd == 'c') { runSweepTest(servo); }
        else if (cmd == 'p' && sscanf(line + 1, "%f", &a) == 1) {
            auto s = servo.getState();
            servo.setVelocity(a, 0.f);
            printf("  Pan vel = %.1f deg/s\n", a);
        }
        else if (cmd == 't' && sscanf(line + 1, "%f", &a) == 1) {
            servo.setVelocity(0.f, a);
            printf("  Tilt vel = %.1f deg/s\n", a);
        }
        else if (cmd == 'v' && sscanf(line + 1, "%f %f", &a, &b) == 2) {
            servo.setVelocity(a, b);
            printf("  Pan=%.1f  Tilt=%.1f deg/s\n", a, b);
        }
        else {
            printf("  Unknown command.\n");
        }
    }

    g_running = false;
    st.join();

    servo.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    servo.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    servo.disconnect();

    printf("\nDone.\n");
    return 0;
}
