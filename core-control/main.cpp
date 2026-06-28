/**
 * core-control — omni-wheel kinematics resolver
 *
 * Build:  mkdir build && cd build && cmake .. && make -j$(nproc)
 *
 * Usage:
 *   ./core_control                        — interactive mode
 *   ./core_control "move left 30"         — single-shot mode
 *   ./core_control "spin right 45"
 *   ./core_control "arc forward 20 turn 30"
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "KinematicsCore.hpp"
#include "CommandParser.hpp"

static void printHelp() {
    printf("\nCommands:\n");
    printf("  move forward|back|left|right <cm>           — pure translation\n");
    printf("  move forward-left|forward-right|... <cm>    — diagonal (45°)\n");
    printf("  spin [left|right] <deg>                     — spin in place\n");
    printf("  self-round [left|right] <deg>               — same as spin\n");
    printf("  arc forward|back|left|right <cm> turn <deg> — move + rotate\n");
    printf("  help                                        — show this\n");
    printf("  quit / q                                    — exit\n\n");
}

int main(int argc, char* argv[]) {
    // Single-shot mode: argument provided on command line
    if (argc >= 2) {
        std::string input;
        for (int i = 1; i < argc; ++i) {
            if (i > 1) input += ' ';
            input += argv[i];
        }
        kinematics::Command cmd;
        std::string error;
        if (!kinematics::parseCommand(input, cmd, error)) {
            fprintf(stderr, "Parse error: %s\n", error.c_str());
            return 1;
        }
        auto wa = kinematics::compute(cmd);
        kinematics::printResult(cmd, wa);
        return 0;
    }

    // Interactive mode
    printf("=== core-control — omni kinematics ===\n");
    printf("Robot: 3-wheel omni  r=4.1cm  L=14.4cm  wheels at 150°/270°/30°\n");
    printHelp();

    char line[256];
    while (true) {
        printf("> "); fflush(stdout);
        if (!fgets(line, sizeof(line), stdin)) break;

        std::string input(line);
        // trim trailing newline/whitespace
        while (!input.empty() && (input.back() == '\n' || input.back() == '\r' || input.back() == ' '))
            input.pop_back();
        if (input.empty()) continue;
        if (input == "quit" || input == "q") break;
        if (input == "help" || input == "h") { printHelp(); continue; }

        kinematics::Command cmd;
        std::string error;
        if (!kinematics::parseCommand(input, cmd, error)) {
            printf("  Error: %s\n\n", error.c_str());
            continue;
        }

        auto wa = kinematics::compute(cmd);
        kinematics::printResult(cmd, wa);
        printf("\n");
    }

    return 0;
}
