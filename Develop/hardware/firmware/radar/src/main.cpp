/**
 * Ultrasonic Radar Firmware
 * 
 * HC-SR04 ultrasonic sensor mounted on servo via PCA9685
 * Sweeps ±30° at 0.5 rad/s, outputs angle + distance for visualization
 * 
 * Output format: RADAR:angle,distance
 * Example: RADAR:15,23.45
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace RadarConfig {
#include "../config/connect.config"
}

// ============== GLOBAL VARIABLES ==============
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(RadarConfig::kPCA9685Address);

int currentAngle = RadarConfig::kMinAngle;
int sweepDirection = 1;  // 1 = increasing (CW), -1 = decreasing (CCW)

// ============== CONVERSION FUNCTIONS ==============
uint16_t angleToPWM(int angle) {
    // Convert from relative angle (-30 to +30) to actual servo angle (60 to 120)
    // Center is 90 degrees
    int actualAngle = angle + 90;
    actualAngle = constrain(actualAngle, 0, 180);
    
    // Calculate pulse width
    long pulseWidth = map(actualAngle, 0, 180, 
                          RadarConfig::kServoMinPulseUs, 
                          RadarConfig::kServoMaxPulseUs);
    
    // Convert to 12-bit PWM value (50Hz = 20000us per cycle)
    return (uint16_t)((pulseWidth * 4096L) / 20000L);
}

void moveServo(int angle) {
    angle = constrain(angle, RadarConfig::kMinAngle, RadarConfig::kMaxAngle);
    pwm.setPWM(RadarConfig::kServoChannel, 0, angleToPWM(angle));
    currentAngle = angle;
}

// ============== ULTRASONIC READING ==============
float readDistanceCm() {
    digitalWrite(RadarConfig::kTriggerPin, LOW);
    delayMicroseconds(2);
    
    digitalWrite(RadarConfig::kTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(RadarConfig::kTriggerPin, LOW);
    
    unsigned long duration = pulseIn(RadarConfig::kEchoPin, HIGH, RadarConfig::kEchoTimeoutUs);
    
    if (duration == 0) {
        return -1.0F;
    }
    
    // Speed of sound: 343 m/s = 0.0343 cm/us
    // Distance = duration * 0.0343 / 2
    float distance = (duration * 0.0343F) / 2.0F;
    
    if (distance > RadarConfig::kMaxRangeCm) {
        return -1.0F;
    }
    
    return distance;
}

// ============== SETUP ==============
void setup() {
    // Initialize ultrasonic pins
    pinMode(RadarConfig::kTriggerPin, OUTPUT);
    pinMode(RadarConfig::kEchoPin, INPUT);
    digitalWrite(RadarConfig::kTriggerPin, LOW);
    
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    // Initialize PCA9685
    Wire.begin();
    pwm.begin();
    pwm.setPWMFreq(RadarConfig::kPWMFrequency);
    
    // Move to start position
    moveServo(RadarConfig::kMinAngle);
    delay(500);
    
    Serial.println("RADAR_READY");
    Serial.print("RADAR_CONFIG:");
    Serial.print(RadarConfig::kMinAngle);
    Serial.print(",");
    Serial.print(RadarConfig::kMaxAngle);
    Serial.print(",");
    Serial.println(RadarConfig::kMaxRangeCm, 0);
}

// ============== MAIN LOOP ==============
void loop() {
    // Read distance at current angle
    float distance = readDistanceCm();
    
    // Output radar data
    Serial.print("RADAR:");
    Serial.print(currentAngle);
    Serial.print(",");
    if (distance < 0) {
        Serial.println("-1");
    } else {
        Serial.println(distance, 2);
    }
    
    // Move to next angle
    int nextAngle = currentAngle + (sweepDirection * RadarConfig::kSweepStepDeg);
    
    // Check bounds and reverse direction
    if (nextAngle > RadarConfig::kMaxAngle) {
        sweepDirection = -1;
        nextAngle = RadarConfig::kMaxAngle;
    } else if (nextAngle < RadarConfig::kMinAngle) {
        sweepDirection = 1;
        nextAngle = RadarConfig::kMinAngle;
    }
    
    moveServo(nextAngle);
    
    // Wait for servo to reach position (based on speed)
    delay(RadarConfig::kStepDelayMs);
}
