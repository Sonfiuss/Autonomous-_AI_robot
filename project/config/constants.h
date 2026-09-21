// RM module configuration — every physical/tuning constant lives here.
// All values are compile-time constants (constexpr) so the library has no
// runtime initialisation and can be embedded on ESP32 without setup code.
//
// Chassis values follow agent/description/project_overview.md (ESP32 firmware
// PinConfig.h / OmniKinematics.h). They OVERRIDE the assumed values listed in
// documents/software/RM/omni3wheel.md.
#ifndef RM_CONFIG_CONSTANTS_H
#define RM_CONFIG_CONSTANTS_H

#include <cstdint>

namespace rm {
namespace cfg {

// ---------------------------------------------------------------- math
constexpr int   NUM_WHEELS = 3;
constexpr float PI         = 3.14159265358979f;
constexpr float TWO_PI     = 2.0f * PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / PI;

// ---------------------------------------------------------------- chassis
constexpr float WHEEL_RADIUS_M        = 0.055f;                       // a  (⌀11 cm wheel)
constexpr float WHEEL_DIAMETER_M      = 2.0f * WHEEL_RADIUS_M;        // d
constexpr float WHEEL_CIRCUMFERENCE_M = TWO_PI * WHEEL_RADIUS_M;      // C ≈ 0.3456 m
constexpr float ROBOT_RADIUS_M        = 0.21f;                        // L  (centre → wheel contact)

// Wheel mounting angle α_i measured CCW from the robot +x (forward) axis.
// Drive direction of wheel i is the CCW tangent at α_i, so a positive ω_i
// pushes the robot CCW around its centre.
constexpr float WHEEL_ANGLE_DEG[NUM_WHEELS] = {60.0f, 180.0f, 300.0f};
constexpr float WHEEL_ANGLE_RAD[NUM_WHEELS] = {
    WHEEL_ANGLE_DEG[0] * DEG_TO_RAD,
    WHEEL_ANGLE_DEG[1] * DEG_TO_RAD,
    WHEEL_ANGLE_DEG[2] * DEG_TO_RAD};

// Wheel contact positions (dx_i, dy_i) = L·(cos α_i, sin α_i), metres.
// Kept explicit for documentation / firmware use; kinematics use the angles.
constexpr float WHEEL_POS_X_M[NUM_WHEELS] = { 0.105000f, -0.210000f,  0.105000f};
constexpr float WHEEL_POS_Y_M[NUM_WHEELS] = { 0.181865f,  0.000000f, -0.181865f};

// ---------------------------------------------------------------- stepper
constexpr int32_t STEPPER_FULL_STEPS_PER_REV = 200;                                  // motor datasheet
constexpr int32_t MICROSTEP                  = 64;                                   // driver DIP setting
constexpr int32_t STEPS_PER_REV              = STEPPER_FULL_STEPS_PER_REV * MICROSTEP; // 12800
constexpr float   RAD_PER_STEP               = TWO_PI / static_cast<float>(STEPS_PER_REV);
constexpr float   STEPS_PER_RAD              = static_cast<float>(STEPS_PER_REV) / TWO_PI;
constexpr float   METERS_PER_STEP            = WHEEL_CIRCUMFERENCE_M / static_cast<float>(STEPS_PER_REV);

// Highest pulse rate the driver/motor is trusted to follow. Placeholder —
// measure on hardware and tune. 20 kHz → 1.56 rev/s → 0.54 m/s wheel rim speed.
constexpr float MAX_PULSE_HZ         = 20000.0f;
constexpr float MAX_WHEEL_OMEGA_RAD_S = TWO_PI * MAX_PULSE_HZ / static_cast<float>(STEPS_PER_REV);
// Below this |ω| the driver outputs 0 Hz (avoids sub-Hz pulse trains / jitter).
constexpr float MIN_WHEEL_OMEGA_RAD_S = 0.01f;

// ---------------------------------------------------------------- odometry
// No encoder on the robot yet: odometry integrates the steps actually
// commanded to the driver, so one count == one microstep.
// When a real encoder is fitted set ODOM_COUNTS_PER_REV = ENCODER_PPR * ENCODER_QUADRATURE_MULT.
constexpr int32_t ENCODER_PPR             = 100;   // pulses per wheel revolution (future hardware)
constexpr int32_t ENCODER_QUADRATURE_MULT = 4;     // x4 decoding (future hardware)
constexpr int32_t ODOM_COUNTS_PER_REV     = STEPS_PER_REV;
constexpr float   RAD_PER_ODOM_COUNT      = TWO_PI / static_cast<float>(ODOM_COUNTS_PER_REV);

// ---------------------------------------------------------------- velocity profile
// Angular acceleration limits per wheel (rad/s²). 2 rad/s² ≈ 0.11 m/s² at the rim:
// conservative to avoid stepper skipping; raise after testing on hardware.
constexpr float WHEEL_ACCEL_RAD_S2 = 2.0f;
constexpr float WHEEL_DECEL_RAD_S2 = 2.0f;
// Two wheel speeds are considered equal within this tolerance.
constexpr float OMEGA_TOLERANCE_RAD_S = 1e-3f;

// ---------------------------------------------------------------- numeric guards
constexpr float MIN_DT_S           = 1e-5f;  // update() calls with dt below this are ignored
constexpr float MIN_DETERMINANT    = 1e-6f;  // kinematics matrix considered singular below this
constexpr float MIN_PROFILE_LENGTH = 1e-6f;  // trapezoid distance below this is a no-op

}  // namespace cfg
}  // namespace rm

// ============================================================================
// MV module (path planning) -- grid geometry and planner limits.
// ============================================================================
namespace mv {
namespace cfg {

// ---------------------------------------------------------------- grid
constexpr float GRID_RES_M     = 0.05f;                      // cell edge, metres
constexpr int   MAX_GRID_COLS  = 240;                        // 12 m at 0.05 m
constexpr int   MAX_GRID_ROWS  = 240;
constexpr int   MAX_CELLS      = MAX_GRID_COLS * MAX_GRID_ROWS;
constexpr int   NUM_NEIGHBOURS = 8;                          // 8-connected search
constexpr float DIAGONAL_COST  = 1.41421356f;                // sqrt(2) cells

// ---------------------------------------------------------------- robot as a point
// Obstacles and walls are inflated by robot hull radius + SAFETY_MARGIN_M.
constexpr float SAFETY_MARGIN_M = 0.03f;
// Start/goal inside an inflated cell are moved to the nearest free cell within this radius.
constexpr float SNAP_RADIUS_M   = 0.20f;

// ---------------------------------------------------------------- path
constexpr int   MAX_PATH_CELLS = 4096;   // longest cell path the planner keeps internally
constexpr float MIN_SEGMENT_M  = 1e-3f;  // shorter waypoint segments emit no FORWARD/MOVE
constexpr float MIN_ROTATE_RAD = 1e-3f;  // smaller heading changes emit no ROTATE
constexpr float LOS_EPS        = 1e-6f;  // ray-cast tie tolerance for exact corner crossings

}  // namespace cfg
}  // namespace mv

// ============================================================================
// MC module (motion executor) -- turns MV primitives into per-wheel speeds.
// Every physical limit is taken from the rm::cfg block above; only the
// executor's own timing and defaults live here.
// ============================================================================
namespace mc {
namespace cfg {

// ---------------------------------------------------------------- timing
constexpr float TICK_S = 0.02f;               // control tick, 50 Hz
// Longest trajectory a caller-owned buffer is expected to hold: 4096 ticks = 81.9 s at 50 Hz.
constexpr int   MAX_TRAJECTORY_STEPS = 4096;
// Longest primitive list the C API accepts. MV emits at most two primitives per
// path cell plus a final rotate and stop, so this covers its worst case.
constexpr int   MAX_PRIMITIVES = 2 * mv::cfg::MAX_PATH_CELLS + 8;

// ---------------------------------------------------------------- commanded speed
// Defaults when the caller passes 0. Both are well inside the wheel limit
// (rm::cfg::MAX_WHEEL_OMEGA_RAD_S allows ~0.62 m/s forward, ~0.54 m/s sideways).
constexpr float CRUISE_SPEED_M_S = 0.15f;
constexpr float YAW_RATE_RAD_S   = 0.8f;

// ---------------------------------------------------------------- numeric guards
// A direction whose fastest wheel coefficient is below this is treated as "no motion":
// it would need an unbounded chassis speed to move any wheel.
constexpr float MIN_WHEEL_COEFF   = 1e-6f;
constexpr float MIN_LEG_LENGTH_M  = 1e-4f;   // shorter FORWARD/MOVE legs are skipped
constexpr float MIN_LEG_ANGLE_RAD = 1e-4f;   // smaller ROTATE legs are skipped
constexpr float STOP_HOLD_S       = 0.10f;   // STOP emits zero-speed ticks for this long

}  // namespace cfg
}  // namespace mc

#endif  // RM_CONFIG_CONSTANTS_H
