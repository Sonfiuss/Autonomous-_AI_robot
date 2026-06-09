/*
 * test_servo.cpp – interactive servo tester
 *
 * Usage: ./test_servo <serial_port> [baud]
 *   e.g: ./test_servo /dev/ttyUSB0
 *        ./test_servo /dev/ttyACM0 115200
 *
 * Hardware: Jetson -> UART -> ESP32 -> I2C (SDA=21, SCL=17) -> PCA9685
 *   Pan  : PCA9685 channel 12
 *   Tilt : PCA9685 channel 14
 */

#include "../ServoClient.hpp"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>

static void printHelp() {
    puts("\n--- Servo Test Commands ---");
    puts("  v <pan> <tilt>   set velocity deg/s  (e.g. v 20 0)");
    puts("  s                stop both axes");
    puts("  r                reset to 0°");
    puts("  p                print current position");
    puts("  sweep pan        pan sweep ±70° at 30 deg/s (5 s)");
    puts("  sweep tilt       tilt sweep ±70° at 30 deg/s (5 s)");
    puts("  sweep all        sweep both axes simultaneously (5 s)");
    puts("  h / help         show this help");
    puts("  q / quit         stop and exit");
    puts("---------------------------\n");
}

static void runSweep(ServoClient& servo, float pan_vel, float tilt_vel, int seconds) {
    printf("  Sweeping %d s (pan_vel=%.0f, tilt_vel=%.0f deg/s) ...\n",
           seconds, pan_vel, tilt_vel);
    servo.setVelocity(pan_vel, tilt_vel);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    servo.stop();
    servo.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    printf("  Sweep done, reset to 0°\n");
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <serial_port> [baud]\n", argv[0]);
        fprintf(stderr, "  e.g: %s /dev/ttyUSB0\n", argv[0]);
        return 1;
    }

    const std::string port = argv[1];
    const int baud = (argc >= 3) ? std::stoi(argv[2]) : 115200;

    ServoClient servo(port, baud);

    // Print position feedback on the same line while user types
    servo.setPositionCallback([](const ServoClient::State& s) {
        printf("\r  [pos] pan=%+6.1f°  tilt=%+6.1f°   ", s.pan, s.tilt);
        fflush(stdout);
    });

    printf("[test_servo] Connecting to %s @ %d baud ...\n", port.c_str(), baud);
    if (!servo.connect(5.0)) {
        fprintf(stderr, "[test_servo] Connection failed.\n");
        return 1;
    }

    printHelp();

    std::string line;
    while (true) {
        printf("\n> ");
        fflush(stdout);
        if (!std::getline(std::cin, line)) break;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;

        if (cmd == "q" || cmd == "quit") {
            servo.stop();
            servo.reset();
            printf("  Bye.\n");
            break;

        } else if (cmd == "v") {
            float pv = 0.f, tv = 0.f;
            if (!(iss >> pv >> tv)) {
                puts("  Usage: v <pan_vel> <tilt_vel>");
                continue;
            }
            servo.setVelocity(pv, tv);
            printf("  Velocity → pan=%.1f  tilt=%.1f deg/s\n", pv, tv);

        } else if (cmd == "s") {
            servo.stop();
            puts("  Stopped.");

        } else if (cmd == "r") {
            servo.reset();
            puts("  Reset to 0°.");

        } else if (cmd == "p") {
            auto st = servo.getState();
            printf("  pan=%+.2f°  tilt=%+.2f°\n", st.pan, st.tilt);

        } else if (cmd == "sweep") {
            std::string axis;
            iss >> axis;
            if (axis == "pan")       runSweep(servo, 30.f,  0.f,  5);
            else if (axis == "tilt") runSweep(servo,  0.f, 30.f,  5);
            else if (axis == "all")  runSweep(servo, 30.f, 30.f,  5);
            else puts("  Usage: sweep pan | tilt | all");

        } else if (cmd == "h" || cmd == "help" || cmd.empty()) {
            printHelp();

        } else {
            printf("  Unknown command: '%s'  (type 'h' for help)\n", cmd.c_str());
        }
    }

    return 0;
}
