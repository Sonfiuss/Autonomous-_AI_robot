/*
 * ESP32 Servo Driver — step-and-hold, MOVE/OK protocol
 *
 * Hardware
 *   PCA9685 I2C : SDA=GPIO21, SCL=GPIO17, addr=0x40
 *   Pan  servo  : channel 12  (-80° … +80°)
 *   Tilt servo  : channel 14  (-70° … +30°)
 *
 * Serial protocol (115200 baud, LF-terminated):
 *   Jetson → ESP32 : "MOVE <pan_f> <tilt_f>\n"
 *   ESP32  → Jetson: "OK\n"           after SETTLE_MS
 *   Jetson → ESP32 : "RESET\n"
 *   ESP32  → Jetson: "OK\n"           after servos reach 0,0
 *   Jetson → ESP32 : "STOP\n"        hold current position (no reply)
 *   ESP32  → Jetson: "READY\n"       on boot
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── Hardware ──────────────────────────────────────────────────────────────────
#define I2C_SDA         21
#define I2C_SCL         17
#define PCA9685_ADDR    0x40

#define PAN_CHANNEL     12
#define TILT_CHANNEL    14

// ── Servo calibration ─────────────────────────────────────────────────────────
// At 50 Hz: 1 period = 20 ms = 4096 counts
// 500 µs → 102 counts (-90°),  2500 µs → 512 counts (+90°)
#define SERVO_FREQ_HZ   50
#define PULSE_MIN       102     // -90°
#define PULSE_MAX       512     // +90°
#define PULSE_CENTER    307     //   0°

// ── Motion limits ─────────────────────────────────────────────────────────────
#define PAN_MIN        -80.0f
#define PAN_MAX         80.0f
#define TILT_MIN       -70.0f
#define TILT_MAX        30.0f

// ── Settle time ───────────────────────────────────────────────────────────────
// Conservative: covers 160° move at ~60°/s typical servo speed.
// Reduce if your servos are faster and timing is measured.
#define SETTLE_MS       500

// ── State ─────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

float pan_angle  = 0.0f;
float tilt_angle = 0.0f;

// ── Helpers ───────────────────────────────────────────────────────────────────
static int angleToPulse(float angle) {
    float pulse = PULSE_CENTER + (angle / 90.0f) * ((PULSE_MAX - PULSE_MIN) / 2.0f);
    return (int)constrain(pulse, PULSE_MIN, PULSE_MAX);
}

static void applyAngles() {
    pwm.setPWM(PAN_CHANNEL,  0, angleToPulse(pan_angle));
    pwm.setPWM(TILT_CHANNEL, 0, angleToPulse(tilt_angle));
}

static void moveTo(float pan, float tilt) {
    pan_angle  = constrain(pan,  PAN_MIN,  PAN_MAX);
    tilt_angle = constrain(tilt, TILT_MIN, TILT_MAX);
    applyAngles();
    delay(SETTLE_MS);
    Serial.println("OK");
}

// ── Command parser ────────────────────────────────────────────────────────────
static void parseCommand(const String& cmd) {
    if (cmd.startsWith("MOVE ")) {
        // "MOVE <pan_f> <tilt_f>"
        int sp = cmd.indexOf(' ', 5);  // space between the two numbers
        float pan, tilt;
        if (sp < 0) {
            // malformed: treat second value as current
            pan  = cmd.substring(5).toFloat();
            tilt = tilt_angle;
        } else {
            pan  = cmd.substring(5, sp).toFloat();
            tilt = cmd.substring(sp + 1).toFloat();
        }
        moveTo(pan, tilt);

    } else if (cmd == "RESET") {
        moveTo(0.0f, 0.0f);

    } else if (cmd == "STOP") {
        // Hold current position — no reply
    }
    // Unrecognised commands are silently ignored
}

// ── Arduino entry points ──────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // board-specific calibration
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    // Move to home without sending OK (boot sequence)
    pan_angle = tilt_angle = 0.0f;
    applyAngles();
    delay(SETTLE_MS);

    Serial.println("READY");
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0)
            parseCommand(line);
    }
}
