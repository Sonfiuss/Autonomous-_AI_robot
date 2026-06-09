/*
 * ESP32 Servo Controller via PCA9685
 * PCA9685: SDA=GPIO21, SCL=GPIO17
 * Pan servo: channel 12, Tilt servo: channel 14
 * Range: -70 to +70 degrees per axis
 *
 * Protocol (UART 115200 baud):
 *   Jetson -> ESP32: "V <pan_vel> <tilt_vel>\n"  (deg/s)
 *   Jetson -> ESP32: "R\n"                         (reset to 0,0)
 *   Jetson -> ESP32: "S\n"                         (stop)
 *   ESP32  -> Jetson: "P <pan_angle> <tilt_angle>\n" (feedback at 50Hz)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── Hardware config ──────────────────────────────────────────────────────────
#define I2C_SDA          21
#define I2C_SCL          17
#define PCA9685_ADDR     0x40

#define PAN_CHANNEL      12
#define TILT_CHANNEL     14

// ─── Servo pulse calibration (adjust per servo model) ─────────────────────────
// At 50 Hz, one period = 20 ms = 4096 counts
// Typical: 500 µs -> 102 counts, 2500 µs -> 512 counts (full 180° range)
#define SERVO_FREQ_HZ    50
#define PULSE_MIN        102   // ~500 µs  → -90°
#define PULSE_MAX        512   // ~2500 µs → +90°
#define PULSE_CENTER     307   // ~1500 µs →   0°

// ─── Motion limits ────────────────────────────────────────────────────────────
#define ANGLE_MIN       -70.0f
#define ANGLE_MAX        70.0f

// ─── Update interval ──────────────────────────────────────────────────────────
#define UPDATE_MS        20    // 50 Hz

// ─── State ───────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);

float pan_angle    = 0.0f;
float tilt_angle   = 0.0f;
float pan_vel      = 0.0f;   // deg/s
float tilt_vel     = 0.0f;   // deg/s

unsigned long last_update_ms = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────
int angleToPulse(float angle) {
    // Linear map: [-90°..+90°] -> [PULSE_MIN..PULSE_MAX]
    float pulse = PULSE_CENTER + (angle / 90.0f) * ((PULSE_MAX - PULSE_MIN) / 2.0f);
    return (int)constrain(pulse, PULSE_MIN, PULSE_MAX);
}

void writeServo(uint8_t channel, float angle) {
    pwm.setPWM(channel, 0, angleToPulse(angle));
}

void applyAngles() {
    writeServo(PAN_CHANNEL,  pan_angle);
    writeServo(TILT_CHANNEL, tilt_angle);
}

void resetPosition() {
    pan_angle  = 0.0f;
    tilt_angle = 0.0f;
    pan_vel    = 0.0f;
    tilt_vel   = 0.0f;
    applyAngles();
}

// ─── Command parser ───────────────────────────────────────────────────────────
void parseCommand(const String &cmd) {
    if (cmd.length() == 0) return;

    char type = cmd.charAt(0);

    if (type == 'V') {
        // "V <pan_vel> <tilt_vel>"
        int s1 = cmd.indexOf(' ', 2);
        if (s1 < 0) return;
        int s2 = cmd.indexOf(' ', s1 + 1);

        if (s2 < 0) {
            pan_vel  = cmd.substring(2, s1).toFloat();
            tilt_vel = cmd.substring(s1 + 1).toFloat();
        } else {
            pan_vel  = cmd.substring(2, s1).toFloat();
            tilt_vel = cmd.substring(s1 + 1, s2).toFloat();
        }

    } else if (type == 'R') {
        resetPosition();

    } else if (type == 'S') {
        pan_vel  = 0.0f;
        tilt_vel = 0.0f;
    }
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    Wire.begin(I2C_SDA, I2C_SCL);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    resetPosition();
    Serial.println("READY");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
    // --- Read commands from Jetson ---
    while (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        parseCommand(line);
    }

    // --- Update servo positions at fixed rate ---
    unsigned long now = millis();
    if (now - last_update_ms >= UPDATE_MS) {
        float dt = (now - last_update_ms) / 1000.0f;
        last_update_ms = now;

        pan_angle  += pan_vel  * dt;
        tilt_angle += tilt_vel * dt;

        // Bounce at limits
        if (pan_angle >= ANGLE_MAX) {
            pan_angle = ANGLE_MAX;
            pan_vel   = -fabsf(pan_vel);
        } else if (pan_angle <= ANGLE_MIN) {
            pan_angle = ANGLE_MIN;
            pan_vel   = fabsf(pan_vel);
        }

        if (tilt_angle >= ANGLE_MAX) {
            tilt_angle = ANGLE_MAX;
            tilt_vel   = -fabsf(tilt_vel);
        } else if (tilt_angle <= ANGLE_MIN) {
            tilt_angle = ANGLE_MIN;
            tilt_vel   = fabsf(tilt_vel);
        }

        applyAngles();

        // Send feedback to Jetson
        Serial.print("P ");
        Serial.print(pan_angle,  2);
        Serial.print(" ");
        Serial.println(tilt_angle, 2);
    }
}
