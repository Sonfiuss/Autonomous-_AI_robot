#pragma once

#include "MotionClient.hpp"
#include <atomic>

/**
 * Real-time keyboard teleoperation for the omni robot.
 *
 * Puts the terminal in raw mode, reads key presses at 20 Hz,
 * and streams velocity commands to the ESP32 via MotionClient.
 *
 * Key map:
 *   W/S       forward / backward
 *   A/D       strafe left / right
 *   Q/E       rotate CCW / CW
 *   J/L       pan camera left / right
 *   I/K       tilt camera up / down
 *   Space     emergency stop (zero all velocities)
 *   +/-       increase / decrease linear speed (0.1 – 1.5 m/s)
 *   R         reset odometry
 *   X / ESC   quit
 */
void runTeleop(MotionClient& mc, std::atomic<bool>& running);
