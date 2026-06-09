// I2C scanner – xác nhận PCA9685 ở địa chỉ nào
// SDA=21, SCL=17 (theo config hệ thống)
#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 17

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(1000);
    Serial.println("=== I2C SCAN ===");

    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == 0x40) Serial.print("  <-- PCA9685 (default)");
            if (addr == 0x70) Serial.print("  <-- PCA9685 (all-call)");
            Serial.println();
            found++;
        }
    }
    if (found == 0) Serial.println("  No I2C devices found! Check wiring.");
    Serial.println("=== DONE ===");
}

void loop() {
    delay(3000);
    Serial.println("=== I2C SCAN ===");
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("  Found device at 0x%02X", addr);
            if (addr == 0x40) Serial.print("  <-- PCA9685 (default)");
            if (addr == 0x70) Serial.print("  <-- PCA9685 (all-call)");
            Serial.println();
            found++;
        }
    }
    if (found == 0) Serial.println("  No I2C devices found!");
    Serial.println("=== DONE ===");
}
