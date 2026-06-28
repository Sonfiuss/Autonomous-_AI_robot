#pragma once

// ── I2C (PCA9685 – servo camera pan-tilt) ────────────────────────────────────
#define I2C_SDA          21
#define I2C_SCL          17
#define PCA9685_ADDR     0x40
#define PAN_CHANNEL      12
#define TILT_CHANNEL     14

// ── Stepper motors : STEP (PUL+) / DIR (DIR+) — ENA not connected ────────────
#define M1_STEP_PIN      12   // W1  60°  DM556
#define M1_DIR_PIN       13

#define M2_STEP_PIN      4    // W2 180°  DM542
#define M2_DIR_PIN       5

#define M3_STEP_PIN      26   // W3 300°  DM542
#define M3_DIR_PIN       27

// ── FreeRTOS task stack ───────────────────────────────────────────────────────
#define TASK_STACK_SIZE  4096

// ── Robot physical parameters ─────────────────────────────────────────────────
#define WHEEL_RADIUS_M   0.055f   // bán kính bánh xe: ⌀11cm / 2
#define ROBOT_RADIUS_M   0.21f    // khoảng cách tâm robot → tâm bánh
#define STEPS_PER_REV    12800    // 200 steps × 64 microstep (DM556/DM542 DIP)
#define MAX_STEP_FREQ    60000    // Hz — đủ cho ~1.5 m/s (~48 kHz tại max vel)
#define MAX_ACCEL_STEPS  200000   // steps/s²
