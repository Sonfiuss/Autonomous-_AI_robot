/**
 * Robot Arms Firmware - PCA9685 Servo Control
 * 
 * Control servo via PCA9685 I2C PWM Driver
 * I2C Pins: A5 (SCL), A4 (SDA) - Arduino Uno
 * 
 * Protocol: Receive commands from Python API via Serial
 * Commands:
 *   LEFT  - Rotate servo 5 degrees left
 *   RIGHT - Rotate servo 5 degrees right
 *   RESET - Return servo to origin (0 degrees)
 *   POS   - Get current position
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ============== CONFIGURATION ==============
// PCA9685 I2C Address (default 0x40)
#define PCA9685_ADDRESS 0x40

// Servo channel on PCA9685 (0-15)
#define SERVO_CHANNEL 5

// PWM frequency for servo (typically 50Hz)
#define PWM_FREQ 50

// Servo PWM pulse range (microseconds)
// Adjust according to your servo
#define SERVO_MIN_PULSE 500   // 0 degrees
#define SERVO_MAX_PULSE 2500  // 180 degrees

// Rotation step per command (degrees)
#define ROTATION_STEP 5

// Angle limits (degrees)
#define MIN_ANGLE -90
#define MAX_ANGLE 90

// ============== GLOBAL VARIABLES ==============
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Current servo position (startup position is origin 0)
int currentAngle = 0;

// Program running flag
volatile bool isRunning = true;

// ============== CONVERSION FUNCTIONS ==============
/**
 * Convert angle (degrees) to PWM value
 * Angle 0 corresponds to center position (90 degrees actual)
 * Angle -90 to +90 corresponds to 0-180 degrees actual
 */
uint16_t angleToPWM(int angle) {
    // Convert from relative angle (-90 to +90) to actual angle (0 to 180)
    int actualAngle = angle + 90;
    
    // Constrain angle
    actualAngle = constrain(actualAngle, 0, 180);
    
    // Calculate pulse width (microseconds)
    long pulseWidth = map(actualAngle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Convert to 12-bit PWM value (0-4095)
    // With 50Hz frequency, each cycle = 20000us
    // PWM value = (pulseWidth / 20000) * 4096
    uint16_t pwmValue = (uint16_t)((pulseWidth * 4096L) / 20000L);
    
    return pwmValue;
}

/**
 * Move servo to specified angle
 */
void moveServo(int angle) {
    // Constrain angle within allowed range
    angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    
    uint16_t pwmValue = angleToPWM(angle);
    pwm.setPWM(SERVO_CHANNEL, 0, pwmValue);
    currentAngle = angle;
    
    Serial.print("Servo moved to: ");
    Serial.print(currentAngle);
    Serial.println(" degrees");
}

/**
 * Rotate servo to the left (negative angle)
 */
void rotateLeft() {
    int newAngle = currentAngle - ROTATION_STEP;
    if (newAngle < MIN_ANGLE) {
        Serial.println("ERROR: Min angle reached");
        return;
    }
    moveServo(newAngle);
}

/**
 * Rotate servo to the right (positive angle)
 */
void rotateRight() {
    int newAngle = currentAngle + ROTATION_STEP;
    if (newAngle > MAX_ANGLE) {
        Serial.println("ERROR: Max angle reached");
        return;
    }
    moveServo(newAngle);
}

/**
 * Return servo to origin (0 degrees)
 */
void resetPosition() {
    moveServo(0);
    Serial.println("Servo reset to origin (0 degrees)");
}

/**
 * Stop servo and disable PWM output
 */
void stopServo() {
    // Disable PWM output for servo channel
    pwm.setPWM(SERVO_CHANNEL, 0, 0);
    isRunning = false;
    Serial.println("=====================================");
    Serial.println("Servo STOPPED - PWM disabled");
    Serial.println("Program halted. Reset Arduino to restart.");
    Serial.println("=====================================");
}

/**
 * Process command from Serial
 */
void processCommand(String command) {
    // Remove special characters (\r, \n, spaces)
    command.trim();
    
    // Remove remaining \r if any
    command.replace("\r", "");
    command.replace("\n", "");
    
    // Debug: print received command
    Serial.print("Received: [");
    Serial.print(command);
    Serial.print("] len=");
    Serial.println(command.length());
    
    // Handle empty Enter or Ctrl+C (character 3)
    if (command.length() == 0) {
        stopServo();
        return;
    }
    
    // Check for Ctrl+C or ESC
    if (command.charAt(0) == 3 || command.charAt(0) == 27) {
        stopServo();
        return;
    }
    
    command.toUpperCase();
    
    if (command == "LEFT" || command == "L") {
        rotateLeft();
    }
    else if (command == "RIGHT" || command == "R") {
        rotateRight();
    }
    else if (command == "RESET" || command == "0") {
        resetPosition();
    }
    else if (command == "POS" || command == "P") {
        Serial.print("Current position: ");
        Serial.print(currentAngle);
        Serial.println(" degrees");
    }
    else if (command.startsWith("ANGLE ") || command.startsWith("A ")) {
        // Move to specific angle: "ANGLE 45" or "A 45"
        int spaceIndex = command.indexOf(' ');
        if (spaceIndex > 0) {
            int targetAngle = command.substring(spaceIndex + 1).toInt();
            moveServo(targetAngle);
        }
    }
    else if (command == "STOP" || command == "EXIT" || command == "QUIT" || command == "Q") {
        stopServo();
    }
    else if (command == "HELP" || command == "H") {
        Serial.println("=== PCA9685 Servo Control ===");
        Serial.println("Commands:");
        Serial.println("  LEFT/L   - Rotate 5 degrees left");
        Serial.println("  RIGHT/R  - Rotate 5 degrees right");
        Serial.println("  RESET/0  - Return to origin (0 deg)");
        Serial.println("  POS/P    - Get current position");
        Serial.println("  ANGLE n  - Move to angle n (-90 to 90)");
        Serial.println("  STOP/EXIT/QUIT/Q - Stop servo & halt");
        Serial.println("  [Enter]  - Stop servo & halt");
        Serial.println("  HELP/H   - Show this help");
    }
    else {
        Serial.println("ERROR: Unknown command. Type HELP for commands.");
    }
}

// ============== SETUP ==============
void setup() {
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }
    
    Serial.println("=====================================");
    Serial.println("PCA9685 Servo Controller Starting...");
    Serial.println("=====================================");
    
    // Initialize I2C (using default SCL-A5, SDA-A4)
    Wire.begin();
    
    // Initialize PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // Internal oscillator
    pwm.setPWMFreq(PWM_FREQ);
    
    delay(100);
    
    // Set servo to origin position (0 degrees - center position)
    // Current servo position will be treated as 0 degrees
    moveServo(0);
    
    Serial.println("Servo initialized at origin (0 degrees)");
    Serial.println("Type HELP for available commands");
    Serial.println("Ready!");
    Serial.println();
}

// ============== MAIN LOOP ==============
void loop() {
    // Check if program stopped
    if (!isRunning) {
        delay(100);
        return;
    }
    
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }
}
