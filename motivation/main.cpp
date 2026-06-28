/**
 * motivation – motion controller main
 *
 * Build:  mkdir build && cd build && cmake .. && make -j$(nproc)
 * Run:    ./motivation --port /dev/ttyUSB0 [options]
 *
 * Options:
 *   --port     /dev/ttyUSB0
 *   --demo     chạy demo sequence (move 5m, turn 90°, v=3m/s)
 *   --nav      navigation mode: nhận goal từ simulation qua ZMQ
 *   --hz       tần số vòng lặp điều khiển (default: 20)
 */

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>

#include "MotionClient.hpp"
#include "SimBridge.hpp"
#include "PoseController.hpp"

static std::atomic<bool> g_running{true};
static void onSig(int) { g_running = false; }

// ── Wheel test sequence ───────────────────────────────────────────────────────
static void runWheelTest(MotionClient& mc) {
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    auto waitDone = [&](double timeout_s) {
        done = false;
        mc.setDoneCallback([&]{ std::lock_guard<std::mutex> lk(mtx); done=true; cv.notify_one(); });
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait_for(lk, std::chrono::duration<double>(timeout_s), [&]{ return done; });
        mc.setDoneCallback(nullptr);
    };

    const char* names[] = { "W1 (60°)", "W2 (180°)", "W3 (300°)" };

    for (int i = 0; i < 3; i++) {
        printf("\n[TEST] %s — quay 1 vòng thuận ...\n", names[i]);
        mc.testWheel(i, 1);
        waitDone(10.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));

        printf("[TEST] %s — quay 1 vòng nghịch ...\n", names[i]);
        mc.testWheel(i, -1);
        waitDone(10.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    }

    printf("\n[TEST] Hoàn tất. Kiểm tra chiều quay từng bánh.\n");
    printf("       Nếu bánh quay ngược → đổi dây DIR+/DIR- trên driver.\n");
}

// ── Demo sequence ─────────────────────────────────────────────────────────────
static void runDemo(MotionClient& mc) {
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    auto waitDone = [&](double timeout_s) {
        done = false;
        mc.setDoneCallback([&]{ std::lock_guard<std::mutex> lk(mtx); done=true; cv.notify_one(); });
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait_for(lk, std::chrono::duration<double>(timeout_s), [&]{ return done; });
        mc.setDoneCallback(nullptr);
    };

    printf("\n[DEMO] Move forward 5 m at 3 m/s\n");
    mc.moveForward(5.0f, 3.0f);
    waitDone(10.0);

    auto o = mc.getOdometry();
    printf("[DEMO] Odometry after move: x=%.3f y=%.3f θ=%.1f°\n", o.x, o.y, o.theta);

    printf("[DEMO] Turn 90° at 1 rad/s\n");
    mc.turn(90.0f, 1.0f);
    waitDone(5.0);

    o = mc.getOdometry();
    printf("[DEMO] Odometry after turn: x=%.3f y=%.3f θ=%.1f°\n", o.x, o.y, o.theta);

    printf("[DEMO] Move forward 2 m at 1.5 m/s\n");
    mc.moveForward(2.0f, 1.5f);
    waitDone(8.0);

    printf("[DEMO] Stop and reset\n");
    mc.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mc.resetOdometry();
}

// ── Interactive mode ──────────────────────────────────────────────────────────
static void runInteractive(MotionClient& mc) {
    printf("\nCommands:\n");
    printf("  m <vx> <vy> <omega>    – set velocity\n");
    printf("  f <dist_m> <speed_ms>  – move forward\n");
    printf("  t <angle_deg> <rads>   – turn\n");
    printf("  v <pan_v> <tilt_v>     – servo camera\n");
    printf("  s                      – stop\n");
    printf("  r                      – reset odometry\n");
    printf("  o                      – print odometry\n");
    printf("  q                      – quit\n\n");

    mc.setOdomCallback([](const MotionClient::Odometry& o) {
        printf("\r  [ODOM] x=%+7.3f m  y=%+7.3f m  θ=%+7.1f°   ", o.x, o.y, o.theta);
        fflush(stdout);
    });

    char line[128];
    while (g_running) {
        printf("\n> "); fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;
        char cmd = line[0];
        float a=0,b=0,c=0;
        if      (cmd == 'q') break;
        else if (cmd == 's') mc.stop();
        else if (cmd == 'r') mc.resetOdometry();
        else if (cmd == 'o') {
            auto o = mc.getOdometry();
            printf("  x=%.3f  y=%.3f  θ=%.1f°\n", o.x, o.y, o.theta);
        }
        else if (cmd == 'm' && sscanf(line+1,"%f %f %f",&a,&b,&c)==3)
            mc.setVelocity(a, b, c);
        else if (cmd == 'f' && sscanf(line+1,"%f %f",&a,&b)==2)
            mc.moveForward(a, b);
        else if (cmd == 't' && sscanf(line+1,"%f %f",&a,&b)==2)
            mc.turn(a, b);
        else if (cmd == 'v' && sscanf(line+1,"%f %f",&a,&b)==2)
            mc.setServoVelocity(a, b);
        else
            printf("  Unknown command.\n");
    }
}

// ── Navigation mode ───────────────────────────────────────────────────────────
static void runNavigation(MotionClient& mc, double ctrl_hz) {
    SimBridge    bridge;
    PoseController ctrl;

    if (!bridge.start()) {
        fprintf(stderr, "[nav] SimBridge failed to start\n");
        return;
    }

    PoseController::Pose target{0.f, 0.f, 0.f};
    bool has_goal = false;

    bridge.setGoalCallback([&](const SimBridge::Goal& g) {
        target    = {g.x, g.y, g.theta * (float)M_PI / 180.f};
        has_goal  = true;
        printf("\n[nav] New goal → x=%.3f  y=%.3f\n", g.x, g.y);
    });

    const auto interval = std::chrono::duration<double>(1.0 / ctrl_hz);

    printf("[nav] Waiting for goals from simulation …\n");

    while (g_running) {
        auto t0 = std::chrono::steady_clock::now();

        auto odom = mc.getOdometry();
        PoseController::Pose current{
            odom.x, odom.y,
            odom.theta * (float)M_PI / 180.f
        };

        // Gửi odometry về simulation để hiển thị
        bridge.publishOdom(odom.x, odom.y, odom.theta);

        if (has_goal) {
            auto cmd = ctrl.compute(current, target);
            if (cmd.reached) {
                mc.stop();
                has_goal = false;
                printf("\n[nav] Goal reached.\n");
            } else {
                mc.setVelocity(cmd.vx, cmd.vy, cmd.omega);
            }
        }

        auto elapsed = std::chrono::steady_clock::now() - t0;
        if (elapsed < interval)
            std::this_thread::sleep_for(interval - elapsed);
    }

    mc.stop();
    bridge.stop();
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  onSig);
    std::signal(SIGTERM, onSig);

    std::string port = "/dev/ttyUSB0";
    bool   demo = false;
    bool   nav  = false;
    bool   test = false;
    double hz   = 20.0;

    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i],"--port") && i+1<argc) port = argv[++i];
        if (!strcmp(argv[i],"--demo"))              demo = true;
        if (!strcmp(argv[i],"--nav"))               nav  = true;
        if (!strcmp(argv[i],"--test"))              test = true;
        if (!strcmp(argv[i],"--hz") && i+1<argc)    hz   = atof(argv[++i]);
    }

    printf("=== motivation ===\nPort: %s\n", port.c_str());

    MotionClient mc(port);
    if (!mc.connect()) { fprintf(stderr,"Connection failed\n"); return 1; }

    mc.setDoneCallback([]{ printf("\n  [DONE] Motion complete.\n"); });

    if (nav)
        runNavigation(mc, hz);
    else if (demo)
        runDemo(mc);
    else if (test)
        runWheelTest(mc);
    else
        runInteractive(mc);

    mc.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    mc.disconnect();
    return 0;
}
