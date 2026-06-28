/**
 * ESP32 Unified Controller
 * Chạy đồng thời 3 nhiệm vụ trên FreeRTOS:
 *   - CommTask  (Core 0): nhận/gửi lệnh UART với Jetson
 *   - MotionTask(Core 1): điều khiển 3 stepper omni + odometry
 *   - ServoTask (Core 0): điều khiển PCA9685 (camera pan-tilt + arm)
 *
 * Protocol Jetson → ESP32 (115200 baud):
 *   C <idx> <hz>\n               – spin motor idx continuously (neg=reverse, 0=stop)
 *   M <vx> <vy> <omega>\n       – velocity liên tục (m/s, m/s, rad/s)
 *   F <dist_m> <spd_ms>\n       – di chuyển thẳng dist_m mét tốc độ spd_ms
 *   T <angle_deg> <rads>\n      – xoay angle_deg độ với vận tốc góc rads rad/s
 *   V <pan_v> <tilt_v>\n        – vận tốc servo (deg/s)
 *   MOVE <pan_deg> <tilt_deg>\n – di chuyển servo đến góc tuyệt đối, chờ ổn định
 *   RESET\n                     – reset servo về (0°, 0°), chờ ổn định
 *   S\n                         – dừng tất cả
 *   R\n                         – reset odometry về (0,0,0)
 *
 * ESP32 → Jetson:
 *   O <x> <y> <theta_deg>\n – odometry (10 Hz)
 *   P <pan> <tilt>\n        – góc servo  (50 Hz)
 *   K\n                     – motion done (đã đến đích)
 *   OK\n                    – MOVE/RESET hoàn thành (sau SERVO_SETTLE_MS)
 *   READY\n                 – boot OK
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastAccelStepper.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "OmniKinematics.h"
#include "PinConfig.h"

// ─── Servo constants ──────────────────────────────────────────────────────────
#define SERVO_FREQ_HZ    50
#define PULSE_MIN        102    // ~500 µs  → -90°
#define PULSE_MAX        512    // ~2500 µs → +90°
#define PULSE_CENTER     307    // ~1500 µs →   0°
#define ANGLE_MIN       -70.0f
#define ANGLE_MAX        70.0f
#define SERVO_DT_MS      20     // 50 Hz

// Step-and-hold limits (matches servo_driver.ino)
#define PAN_MIN         -80.0f
#define PAN_MAX          80.0f
#define TILT_MIN        -70.0f
#define TILT_MAX         30.0f
#define SERVO_SETTLE_MS  500    // settle time after absolute move

// ─── Command types ────────────────────────────────────────────────────────────
enum CmdType { CMD_VELOCITY, CMD_FORWARD, CMD_TURN, CMD_SERVO, CMD_STOP, CMD_RESET, CMD_WHEEL, CMD_SPIN };

struct Command {
    CmdType type;
    float a, b, c;
};

// Absolute servo move — separate from motor command queue
struct ServoMoveCmd {
    float pan;
    float tilt;
};

// ─── Globals ──────────────────────────────────────────────────────────────────
static QueueHandle_t     g_cmd_queue;
static QueueHandle_t     g_servo_move_queue;   // capacity 1; MOVE/RESET commands
static SemaphoreHandle_t g_move_done_sem;       // given by ServoTask when OK ready
static SemaphoreHandle_t g_uart_mutex;

// Stepper
static FastAccelStepperEngine g_engine;
static FastAccelStepper*      g_motor[3];

// Kinematics
static OmniKinematics* g_kin;

// Odometry (updated by MotionTask)
static volatile float g_odom_x     = 0.f;
static volatile float g_odom_y     = 0.f;
static volatile float g_odom_theta = 0.f;
static SemaphoreHandle_t g_odom_mutex;

// Servo
static Adafruit_PWMServoDriver g_pwm(PCA9685_ADDR);
static volatile float g_pan_angle  = 0.f;
static volatile float g_tilt_angle = 0.f;
static volatile float g_pan_vel    = 0.f;
static volatile float g_tilt_vel   = 0.f;

// ─── Helpers ──────────────────────────────────────────────────────────────────
static void uart_print(const char* msg) {
    xSemaphoreTake(g_uart_mutex, portMAX_DELAY);
    Serial.print(msg);
    xSemaphoreGive(g_uart_mutex);
}

static int angleToPulse(float deg) {
    float p = PULSE_CENTER + (deg / 90.f) * ((PULSE_MAX - PULSE_MIN) / 2.f);
    if (p < PULSE_MIN) p = PULSE_MIN;
    if (p > PULSE_MAX) p = PULSE_MAX;
    return (int)p;
}

static void setMotorVelocity(int idx, float omega_rads) {
    float hz;  bool fwd;
    OmniKinematics::toStep(omega_rads, STEPS_PER_REV, hz, fwd);

    if (hz < 1.f) {
        g_motor[idx]->stopMove();
        return;
    }
    hz = hz > MAX_STEP_FREQ ? MAX_STEP_FREQ : hz;

    g_motor[idx]->setDirectionPin(
        idx == 0 ? M1_DIR_PIN : (idx == 1 ? M2_DIR_PIN : M3_DIR_PIN),
        fwd ? false : true
    );
    g_motor[idx]->setSpeedInHz((uint32_t)hz);
    g_motor[idx]->runForward();
}

static void stopAllMotors() {
    for (int i = 0; i < 3; i++) g_motor[i]->stopMove();
}

static void applyVelocity(const RobotVel& rv) {
    WheelVel wv = g_kin->inverse(rv);
    setMotorVelocity(0, wv.w1);
    setMotorVelocity(1, wv.w2);
    setMotorVelocity(2, wv.w3);
}

// ─── CommTask ─────────────────────────────────────────────────────────────────
static void CommTask(void* pv) {
    String line;
    for (;;) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n') {
                line.trim();
                if (line.length() > 0) {
                    Command cmd{};
                    char t = line[0];

                    if (t == 'M' && sscanf(line.c_str()+2,"%f %f %f",&cmd.a,&cmd.b,&cmd.c)==3) {
                        cmd.type = CMD_VELOCITY;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'F' && sscanf(line.c_str()+2,"%f %f",&cmd.a,&cmd.b)==2) {
                        cmd.type = CMD_FORWARD;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'T' && sscanf(line.c_str()+2,"%f %f",&cmd.a,&cmd.b)==2) {
                        cmd.type = CMD_TURN;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'V' && sscanf(line.c_str()+2,"%f %f",&cmd.a,&cmd.b)==2) {
                        cmd.type = CMD_SERVO;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'C' && sscanf(line.c_str()+2,"%f %f",&cmd.a,&cmd.b)==2) {
                        // C <idx> <hz>  — continuous spin (neg hz = reverse, 0 = stop)
                        cmd.type = CMD_SPIN;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'W' && sscanf(line.c_str()+2,"%f %f %f",&cmd.a,&cmd.b,&cmd.c)>=2) {
                        // W <idx> <steps> [<hz>]  — direct wheel move by steps
                        if (cmd.c < 1.f) cmd.c = 2000.f;   // default speed
                        cmd.type = CMD_WHEEL;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'S') {
                        cmd.type = CMD_STOP;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (t == 'R' && line.length() == 1) {
                        cmd.type = CMD_RESET;
                        xQueueSend(g_cmd_queue, &cmd, 0);

                    } else if (line.startsWith("MOVE ")) {
                        // MOVE <pan> <tilt> → absolute servo move, reply OK
                        ServoMoveCmd sc;
                        if (sscanf(line.c_str()+5, "%f %f", &sc.pan, &sc.tilt) == 2) {
                            xSemaphoreTake(g_move_done_sem, 0); // drain stale signal
                            xQueueOverwrite(g_servo_move_queue, &sc);
                            // Block until ServoTask settles and signals done
                            xSemaphoreTake(g_move_done_sem, portMAX_DELAY);
                            uart_print("OK\n");
                        }

                    } else if (line == "RESET") {
                        // RESET servo to home (0, 0), reply OK
                        ServoMoveCmd sc = {0.f, 0.f};
                        xSemaphoreTake(g_move_done_sem, 0);
                        xQueueOverwrite(g_servo_move_queue, &sc);
                        xSemaphoreTake(g_move_done_sem, portMAX_DELAY);
                        uart_print("OK\n");
                    }
                }
                line = "";
            } else if (c != '\r') {
                line += c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ─── MotionTask ───────────────────────────────────────────────────────────────
static void MotionTask(void* pv) {
    int32_t prev_pos[3] = {0, 0, 0};
    TickType_t odom_tick   = xTaskGetTickCount();
    TickType_t report_tick = xTaskGetTickCount();

    for (;;) {
        Command cmd;
        while (xQueueReceive(g_cmd_queue, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case CMD_VELOCITY: {
                    RobotVel rv{cmd.a, cmd.b, cmd.c};
                    applyVelocity(rv);
                    break;
                }
                case CMD_FORWARD: {
                    float spd = fabsf(cmd.b);
                    if (spd < 0.001f) break;
                    float omega_w = spd / WHEEL_RADIUS_M;
                    float steps_per_sec = omega_w / (2.f * (float)M_PI) * STEPS_PER_REV;
                    float duration_s    = fabsf(cmd.a) / spd;
                    int32_t steps       = (int32_t)(steps_per_sec * duration_s);
                    WheelVel wv = g_kin->inverse({spd, 0.f, 0.f});
                    for (int i = 0; i < 3; i++) {
                        float hz; bool fwd;
                        float w = (i==0)?wv.w1:(i==1)?wv.w2:wv.w3;
                        OmniKinematics::toStep(w, STEPS_PER_REV, hz, fwd);
                        hz = hz > MAX_STEP_FREQ ? MAX_STEP_FREQ : hz;
                        g_motor[i]->setSpeedInHz((uint32_t)hz);
                        g_motor[i]->setAcceleration(MAX_ACCEL_STEPS);
                        int32_t s = (int32_t)(steps * (w >= 0 ? 1 : -1));
                        g_motor[i]->move(s);
                    }
                    break;
                }
                case CMD_TURN: {
                    float angle_rad = cmd.a * (float)M_PI / 180.f;
                    float omega     = fabsf(cmd.b);
                    if (omega < 0.001f) break;
                    WheelVel wv = g_kin->inverse({0.f, 0.f,
                        angle_rad > 0 ? omega : -omega});
                    float t = fabsf(angle_rad) / omega;
                    for (int i = 0; i < 3; i++) {
                        float hz; bool fwd;
                        float w = (i==0)?wv.w1:(i==1)?wv.w2:wv.w3;
                        OmniKinematics::toStep(w, STEPS_PER_REV, hz, fwd);
                        hz = hz > MAX_STEP_FREQ ? MAX_STEP_FREQ : hz;
                        int32_t s = (int32_t)(hz * t * (w>=0?1:-1));
                        g_motor[i]->setSpeedInHz((uint32_t)hz);
                        g_motor[i]->setAcceleration(MAX_ACCEL_STEPS);
                        g_motor[i]->move(s);
                    }
                    break;
                }
                case CMD_SERVO:
                    g_pan_vel  = cmd.a;
                    g_tilt_vel = cmd.b;
                    break;
                case CMD_STOP:
                    stopAllMotors();
                    g_pan_vel = g_tilt_vel = 0.f;
                    break;
                case CMD_RESET:
                    stopAllMotors();
                    xSemaphoreTake(g_odom_mutex, portMAX_DELAY);
                    g_odom_x = g_odom_y = g_odom_theta = 0.f;
                    xSemaphoreGive(g_odom_mutex);
                    for (int i=0;i<3;i++) g_motor[i]->setCurrentPosition(0);
                    break;
                case CMD_SPIN: {
                    int   idx = (int)cmd.a;
                    float hz  = cmd.b;
                    if (idx < 0 || idx > 2) break;
                    if (fabsf(hz) < 1.f) {
                        g_motor[idx]->stopMove();
                    } else {
                        uint32_t ahz = (uint32_t)fabsf(hz);
                        ahz = ahz > MAX_STEP_FREQ ? MAX_STEP_FREQ : ahz;
                        g_motor[idx]->setSpeedInHz(ahz);
                        g_motor[idx]->setAcceleration(MAX_ACCEL_STEPS);
                        if (hz > 0.f) g_motor[idx]->runForward();
                        else          g_motor[idx]->runBackward();
                    }
                    break;
                }
                case CMD_WHEEL: {
                    int      idx   = (int)cmd.a;
                    int      steps = (int)cmd.b;
                    uint32_t hz    = (cmd.c >= 1.f) ? (uint32_t)cmd.c : 2000;
                    if (idx < 0 || idx > 2 || steps == 0) break;
                    hz = hz > MAX_STEP_FREQ ? MAX_STEP_FREQ : hz;
                    g_motor[idx]->setSpeedInHz(hz);
                    g_motor[idx]->setAcceleration(MAX_ACCEL_STEPS);
                    g_motor[idx]->move(steps);
                    break;
                }
            }
        }

        // ── Odometry update (50 Hz) ───────────────────────────────────────────
        TickType_t now = xTaskGetTickCount();
        float dt = (now - odom_tick) / (float)configTICK_RATE_HZ;
        if (dt >= 0.02f) {
            odom_tick = now;
            int32_t cur[3];
            for (int i=0;i<3;i++) cur[i] = g_motor[i]->getCurrentPosition();

            float dw[3];
            for (int i=0;i<3;i++) {
                dw[i] = (cur[i] - prev_pos[i]) * 2.f*(float)M_PI / STEPS_PER_REV;
                prev_pos[i] = cur[i];
            }

            WheelVel wv{ dw[0]/dt, dw[1]/dt, dw[2]/dt };
            RobotVel rv = g_kin->forward(wv);

            xSemaphoreTake(g_odom_mutex, portMAX_DELAY);
            float th = g_odom_theta;
            g_odom_x     += (rv.vx*cosf(th) - rv.vy*sinf(th)) * dt;
            g_odom_y     += (rv.vx*sinf(th) + rv.vy*cosf(th)) * dt;
            g_odom_theta +=  rv.omega * dt;
            xSemaphoreGive(g_odom_mutex);
        }

        // ── Report odometry at 10 Hz ──────────────────────────────────────────
        if ((now - report_tick) >= pdMS_TO_TICKS(100)) {
            report_tick = now;
            float x,y,th;
            xSemaphoreTake(g_odom_mutex, portMAX_DELAY);
            x=g_odom_x; y=g_odom_y; th=g_odom_theta;
            xSemaphoreGive(g_odom_mutex);

            char buf[64];
            snprintf(buf, sizeof(buf), "O %.3f %.3f %.2f\n",
                     x, y, th * 180.f / (float)M_PI);
            uart_print(buf);
        }

        // ── Motion done check ─────────────────────────────────────────────────
        bool all_idle = true;
        for (int i=0;i<3;i++)
            if (g_motor[i]->isRunning()) { all_idle = false; break; }
        static bool was_running = false;
        if (was_running && all_idle) uart_print("K\n");
        was_running = !all_idle;

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ─── ServoTask ────────────────────────────────────────────────────────────────
static void ServoTask(void* pv) {
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SERVO_DT_MS));

        // ── Check for absolute move request ──────────────────────────────────
        ServoMoveCmd sc;
        if (xQueueReceive(g_servo_move_queue, &sc, 0) == pdTRUE) {
            // Stop velocity, snap to target
            g_pan_vel  = 0.f;
            g_tilt_vel = 0.f;
            g_pan_angle  = sc.pan  < PAN_MIN  ? PAN_MIN  : (sc.pan  > PAN_MAX  ? PAN_MAX  : sc.pan);
            g_tilt_angle = sc.tilt < TILT_MIN ? TILT_MIN : (sc.tilt > TILT_MAX ? TILT_MAX : sc.tilt);
            g_pwm.setPWM(PAN_CHANNEL,  0, angleToPulse(g_pan_angle));
            g_pwm.setPWM(TILT_CHANNEL, 0, angleToPulse(g_tilt_angle));

            // Settle: hold for SERVO_SETTLE_MS without returning to velocity loop
            vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));

            // Signal CommTask that OK can be sent
            xSemaphoreGive(g_move_done_sem);

            // Re-sync tick baseline after the long delay
            last = xTaskGetTickCount();
            continue;
        }

        // ── Normal velocity mode ──────────────────────────────────────────────
        float dt = SERVO_DT_MS / 1000.f;
        g_pan_angle  += g_pan_vel  * dt;
        g_tilt_angle += g_tilt_vel * dt;

        if (g_pan_angle  >  ANGLE_MAX) { g_pan_angle  =  ANGLE_MAX; g_pan_vel  = -fabsf(g_pan_vel);  }
        if (g_pan_angle  <  ANGLE_MIN) { g_pan_angle  =  ANGLE_MIN; g_pan_vel  =  fabsf(g_pan_vel);  }
        if (g_tilt_angle >  ANGLE_MAX) { g_tilt_angle =  ANGLE_MAX; g_tilt_vel = -fabsf(g_tilt_vel); }
        if (g_tilt_angle <  ANGLE_MIN) { g_tilt_angle =  ANGLE_MIN; g_tilt_vel =  fabsf(g_tilt_vel); }

        g_pwm.setPWM(PAN_CHANNEL,  0, angleToPulse(g_pan_angle));
        g_pwm.setPWM(TILT_CHANNEL, 0, angleToPulse(g_tilt_angle));

        // Report at 50 Hz
        char buf[40];
        snprintf(buf, sizeof(buf), "P %.2f %.2f\n", g_pan_angle, g_tilt_angle);
        uart_print(buf);
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // PCA9685
    Wire.begin(I2C_SDA, I2C_SCL);
    g_pwm.begin();
    g_pwm.setOscillatorFrequency(27000000);
    g_pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);
    g_pwm.setPWM(PAN_CHANNEL,  0, PULSE_CENTER);
    g_pwm.setPWM(TILT_CHANNEL, 0, PULSE_CENTER);

    // Kinematics
    g_kin = new OmniKinematics(WHEEL_RADIUS_M, ROBOT_RADIUS_M);

    // FastAccelStepper
    g_engine.init();
    const int step_pins[3] = {M1_STEP_PIN, M2_STEP_PIN, M3_STEP_PIN};
    const int dir_pins[3]  = {M1_DIR_PIN,  M2_DIR_PIN,  M3_DIR_PIN};
    for (int i = 0; i < 3; i++) {
        g_motor[i] = g_engine.stepperConnectToPin(step_pins[i]);
        g_motor[i]->setDirectionPin(dir_pins[i]);
        g_motor[i]->setSpeedInHz(1000);
        g_motor[i]->setAcceleration(MAX_ACCEL_STEPS);
    }

    // FreeRTOS primitives
    g_cmd_queue        = xQueueCreate(20, sizeof(Command));
    g_servo_move_queue = xQueueCreate(1,  sizeof(ServoMoveCmd));
    g_move_done_sem    = xSemaphoreCreateBinary();
    g_uart_mutex       = xSemaphoreCreateMutex();
    g_odom_mutex       = xSemaphoreCreateMutex();

    // Tasks
    xTaskCreatePinnedToCore(CommTask,   "Comm",   TASK_STACK_SIZE, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(MotionTask, "Motion", TASK_STACK_SIZE, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ServoTask,  "Servo",  TASK_STACK_SIZE, NULL, 2, NULL, 0);

    Serial.println("READY");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
