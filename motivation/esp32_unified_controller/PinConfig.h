#pragma once

// ── I2C (PCA9685 – servo camera pan-tilt + arm) ───────────────────────────────
#define I2C_SDA          21
#define I2C_SCL          17
#define PCA9685_ADDR     0x40
#define PAN_CHANNEL      12
#define TILT_CHANNEL     14

// ── Stepper motors : STEP / DIR / EN (active LOW) ────────────────────────────
#define M1_STEP_PIN      2
#define M1_DIR_PIN       4
#define M1_EN_PIN        25

#define M2_STEP_PIN      5
#define M2_DIR_PIN       18
#define M2_EN_PIN        26

#define M3_STEP_PIN      19
#define M3_DIR_PIN       23
#define M3_EN_PIN        27

// ── FreeRTOS task stack ───────────────────────────────────────────────────────
#define TASK_STACK_SIZE  4096

// ── Robot physical parameters (chỉnh theo thực tế) ───────────────────────────
#define WHEEL_RADIUS_M   0.05f    // bán kính bánh xe (m)
#define ROBOT_RADIUS_M   0.15f    // khoảng cách tâm robot → tâm bánh (m)
#define STEPS_PER_REV    1600     // 200 steps × 8 microstep (set trên DIP switch DM556/552)
#define MAX_STEP_FREQ    20000    // tần số bước tối đa (Hz)
#define MAX_ACCEL_STEPS  40000    // gia tốc tối đa (steps/s²)
