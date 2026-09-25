// Firmware configuration — every pin, rate, priority and buffer size lives
// here. Physical limits are NOT duplicated: those come from
// project/config/constants.h through the rm:: namespace.
#ifndef FW_CONFIG_H
#define FW_CONFIG_H

#include <cstdint>

#include "constants.h"

namespace fw {
namespace cfg {

// ---------------------------------------------------------------- pins
// Wiring given by the user, 2026-09-25: PUL+/DIR+ on 26/27, 12/13, 4/5, PUL-/DIR-
// on the ESP32's GND (opto inputs). WHICH WHEEL each pair drives is taken from the
// previous firmware's PinConfig.h (git a96ff99^, the Arduino build still on the
// board), the one configuration known to have turned the wheels correctly:
//   W1   0 deg  STEP 12 / DIR 13  (front)   W2 120 deg  STEP 4 / DIR 5  (back-left)
//   W3 240 deg  STEP 26 / DIR 27  (back-right)
// (Listing the pairs in the user's table order, 26/27 first, would rotate every
// motion by 120 deg.) Check on the robot: on a forward move W1 stands still and
// the robot drives towards it.
// Also on this board, so never to be used here: INMP441 mic SD 33 / WS 25 /
// SCK 32, PCA9685 I2C SDA 21 / SCL 17, MAX98357 on 5V.
// GPIO 12 is a strapping pin (flash voltage): it must be LOW at reset. The
// DM556 opto input only sinks current, so it cannot pull it high; if the board
// ever fails to boot with the drivers powered, this pin is the first suspect.
constexpr int STEP_PIN[rm::cfg::NUM_WHEELS] = {12, 4, 26};
constexpr int DIR_PIN[rm::cfg::NUM_WHEELS]  = {13, 5, 27};
// true flips a wheel's DIR level: cheaper than rewiring a driver. None are
// flipped on this robot. All three were true at first (the previous firmware
// drove DIR low for a positive IK roll), but both drive tests of 2026-09-25 only
// fit the wheel layout above with every wheel's sign reversed. Verify:
// `robot_link --forward 0.1` must move the robot towards W1 and
// `robot_link --turn 90` must turn it left; if both are reversed, set all three
// back to true.
constexpr bool DIR_INVERTED[rm::cfg::NUM_WHEELS] = {false, false, false};
// NOT the real servo wiring: the camera servos hang off a PCA9685 over I2C
// (SDA 21, SCL 17), which servo_driver.cpp does not speak. These two GPIOs are
// unconnected on this board, so the LEDC servo output is harmless but moves
// nothing. The drive demo does not use the servos.
constexpr int SERVO_PAN_PIN                 = 18;
constexpr int SERVO_TILT_PIN                = 19;
constexpr int UART_TX_PIN                   = 1;
constexpr int UART_RX_PIN                   = 3;

// ---------------------------------------------------------------- uart
constexpr int UART_PORT_NUM        = 0;        // UART_NUM_0, the USB console port
constexpr int UART_BAUD            = 115200;   // fixed by interfaces.md
constexpr int UART_RX_BUFFER_BYTES = 1024;
constexpr int UART_TX_BUFFER_BYTES = 1024;
constexpr int UART_EVENT_QUEUE_LEN = 16;
constexpr int UART_READ_CHUNK      = 64;       // bytes pulled per read call
constexpr int UART_READ_TIMEOUT_MS = 20;

// ---------------------------------------------------------------- timing
// One tick is 20 ms. The same 50 Hz MC uses, so a trajectory previewed on the
// Jetson and the motion the firmware produces are sampled identically.
constexpr int   TICK_MS  = 20;
constexpr float TICK_S   = mc::cfg::TICK_S;
constexpr int   SERVO_TICK_MS = 20;            // 50 Hz, matches the P report rate

// Status cadence, counted in its own 20 ms ticks.
constexpr int ODOM_EVERY_N_TICKS  = 5;         // O at 10 Hz
constexpr int FAULT_EVERY_N_TICKS = 50;        // E at 1 Hz

// ---------------------------------------------------------------- watchdogs
// A command older than this counts as zero. Without it the robot keeps driving
// on the last M after the cable is pulled, and resumes a stale M the moment an
// F/T leg ends. Aliased from link::cfg rather than redefined: the Jetson paces
// its keep-alive by the same numbers, so they live in the shared contract.
constexpr uint32_t MOTION_CMD_TIMEOUT_MS = link::cfg::MOTION_CMD_TIMEOUT_MS;
constexpr uint32_t SERVO_CMD_TIMEOUT_MS  = link::cfg::SERVO_CMD_TIMEOUT_MS;

// ---------------------------------------------------------------- tasks
// Motion gets a core to itself so UART parsing and servo updates can never
// jitter the step timing.
constexpr int MOTION_CORE  = 1;
constexpr int SERVICE_CORE = 0;

constexpr int MOTION_PRIORITY     = 6;
constexpr int COMM_PRIORITY       = 5;
constexpr int STATUS_PRIORITY     = 4;
constexpr int PERIPHERAL_PRIORITY = 3;

constexpr int MOTION_STACK_BYTES     = 4096;
constexpr int COMM_STACK_BYTES       = 3072;
constexpr int STATUS_STACK_BYTES     = 3072;
constexpr int PERIPHERAL_STACK_BYTES = 2560;

// ---------------------------------------------------------------- queues
// Deep enough for a short burst of moves, shallow enough that a stuck consumer
// is noticed (as an E 4) rather than hidden.
constexpr int ONE_SHOT_QUEUE_LEN = 8;

// ---------------------------------------------------------------- stepper pwm
// Each wheel owns a LEDC high-speed timer, because each needs its own
// frequency. Three wheels, three timers; the servo uses the low-speed group.
constexpr int   STEP_DUTY_BITS = 10;      // 1024 levels: good to ~78 kHz
constexpr int   STEP_DUTY_HALF = 1 << (STEP_DUTY_BITS - 1);
constexpr float STEP_MIN_HZ    = 1.0f;    // below this the channel is switched off

// ---------------------------------------------------------------- servo
constexpr float SERVO_MIN_DEG       = -90.0f;
constexpr float SERVO_MAX_DEG       = 90.0f;
constexpr float SERVO_SPAN_DEG      = SERVO_MAX_DEG - SERVO_MIN_DEG;
constexpr int   SERVO_PWM_HZ        = 50;
constexpr int   SERVO_DUTY_BITS     = 16;
constexpr int   SERVO_MIN_PULSE_US  = 500;
constexpr int   SERVO_MAX_PULSE_US  = 2500;
constexpr int   SERVO_PERIOD_US     = 1000000 / SERVO_PWM_HZ;

}  // namespace cfg
}  // namespace fw

#endif  // FW_CONFIG_H
