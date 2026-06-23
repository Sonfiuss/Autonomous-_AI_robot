/*
 * ESP32 Camera Servo Firmware — step control via PCA9685
 *
 * Architecture:  Jetson (keypress) --serial--> ESP32 --I2C--> PCA9685 --> servos
 *
 * Hardware (matches stereo-camera project wiring):
 *   PCA9685 I2C : SDA=GPIO21, SCL=GPIO17, addr=0x40
 *   Pan  servo  : channel 12   (-80° … +80°)
 *   Tilt servo  : channel 14   (-70° … +30°)
 *
 * Serial protocol (115200 baud, LF-terminated):
 *   Jetson → ESP32 : "M <dpan> <dtilt>\n"   relative step (degrees)
 *   Jetson → ESP32 : "R\n"                    reset both axes to 0°
 *   ESP32  → Jetson: "P <pan> <tilt>\n"      position feedback after each move
 *   ESP32  → Jetson: "READY\n"               on boot
 *
 * Each key press on the Jetson sends one "M ±1 0" / "M 0 ±1". The ESP32 applies
 * the delta, CLAMPS at the limits, and reports the new absolute position.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ── Hardware ────────────────────────────────────────────────────────────────
#define I2C_SDA          21
#define I2C_SCL          17
#define PCA9685_ADDR     0x40

#define PAN_CHANNEL      12
#define TILT_CHANNEL     14

// ── Servo pulse calibration (50 Hz → 1 period = 20 ms = 4096 counts) ─────────
#define SERVO_FREQ_HZ    50
#define PULSE_MIN        102   // ~500 µs  → -90°
#define PULSE_MAX        512   // ~2500 µs → +90°
#define PULSE_CENTER     307   // ~1500 µs →   0°

// ── Motion limits (asymmetric, per project spec) ────────────────────────────
#define PAN_MIN         -80.0f
#define PAN_MAX          80.0f
#define TILT_MIN        -70.0f
#define TILT_MAX         30.0f

// ── State ───────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

float pan_angle  = 0.0f, tilt_angle = 0.0f;

// ── Helpers ─────────────────────────────────────────────────────────────────
int angleToPulse(float angle) {
    float pulse = PULSE_CENTER + (angle / 90.0f) * ((PULSE_MAX - PULSE_MIN) / 2.0f);
    return (int)constrain(pulse, PULSE_MIN, PULSE_MAX);
}

void applyAngles() {
    pwm.setPWM(PAN_CHANNEL,  0, angleToPulse(pan_angle));
    pwm.setPWM(TILT_CHANNEL, 0, angleToPulse(tilt_angle));
}

void reportPosition() {
    Serial.print("P ");
    Serial.print(pan_angle, 2);
    Serial.print(" ");
    Serial.println(tilt_angle, 2);
}

void resetPosition() {
    pan_angle = tilt_angle = 0.0f;
    applyAngles();
    reportPosition();
}

// ── Command parser ──────────────────────────────────────────────────────────
void parseCommand(const String &cmd) {
    if (cmd.length() == 0) return;
    char type = cmd.charAt(0);

    if (type == 'M') {
        // "M <dpan> <dtilt>" — relative step in degrees
        int s1 = cmd.indexOf(' ', 2);
        if (s1 < 0) return;
        float dpan  = cmd.substring(2, s1).toFloat();
        float dtilt = cmd.substring(s1 + 1).toFloat();

        pan_angle  = constrain(pan_angle  + dpan,  PAN_MIN,  PAN_MAX);
        tilt_angle = constrain(tilt_angle + dtilt, TILT_MIN, TILT_MAX);
        applyAngles();
        reportPosition();

    } else if (type == 'R') {
        resetPosition();
    }
}

// ── Arduino entry points ────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // board-specific calibration
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    pan_angle = tilt_angle = 0.0f;
    applyAngles();
    Serial.println("READY");
}

void loop() {
    while (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        parseCommand(line);
    }
}
