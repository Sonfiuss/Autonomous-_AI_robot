/**
 * Drive System Firmware
 * 
 * Điều khiển hệ thống di chuyển omni wheel với DM556 stepper drivers.
 * Tham chiếu: omni_wheel_module/ARD_DRVDM556/src/main.cpp
 * 
 * Protocol: Nhận lệnh vận tốc (vx, vy, omega) từ Python API qua Serial
 */

#include <Arduino.h>

// TODO: Import từ omni_wheel_module/ARD_DRVDM556 và refactor

void setup() {
    Serial.begin(115200);
    Serial.println("Drive Controller Ready");
}

void loop() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        // TODO: Parse velocity commands và điều khiển motors
        Serial.println("ACK: " + command);
    }
}
