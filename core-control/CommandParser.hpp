#pragma once

#include "KinematicsCore.hpp"
#include <string>

namespace kinematics {

/**
 * Parse a human-readable command string into a Command struct.
 *
 * Supported syntax (case-insensitive, unit suffix optional):
 *
 *   Translation:
 *     "move forward 30"          → dx=+30
 *     "move back 30"             → dx=-30
 *     "move left 30"             → dy=+30
 *     "move right 30"            → dy=-30
 *     "move forward-left 30"     → dx=+21.2, dy=+21.2  (45° diagonal)
 *     "move forward-right 30"    → dx=+21.2, dy=-21.2
 *     "move back-left 30"        → dx=-21.2, dy=+21.2
 *     "move back-right 30"       → dx=-21.2, dy=-21.2
 *
 *   Rotation in place:
 *     "spin 45"                  → rotate=+45 (CCW)
 *     "spin left 45"             → rotate=+45 (CCW)
 *     "spin right 45"            → rotate=-45 (CW)
 *     "self-round 45"            → rotate=+45 (CCW)
 *     "self-round left 45"       → rotate=+45
 *     "self-round right 45"      → rotate=-45
 *
 *   Arc (translation + rotation simultaneously):
 *     "arc forward 30 turn 45"   → dx=+30, rotate=+45
 *     "arc left 20 turn -30"     → dy=+20, rotate=-30
 *     "arc back 10 turn 90"      → dx=-10, rotate=+90
 *
 * Returns false if the string cannot be parsed.
 */
bool parseCommand(const std::string& input, Command& out, std::string& error);

} // namespace kinematics
