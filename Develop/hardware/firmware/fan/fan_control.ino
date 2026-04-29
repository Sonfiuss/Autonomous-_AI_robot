/**
 * @file    fan_control.ino
 * @brief   Fan DC 12V - Serial PWM Control + EEPROM Persist
 *
 * Model: Prism 6PRO  120×120×25mm  DC 12V
 *
 * Kết nối:
 * ┌───────────────┬──────────────────────────┬───────────────────────────┐
 * │  Arduino Uno  │  Thiết bị ngoài          │  Mô tả                    │
 * ├───────────────┼──────────────────────────┼───────────────────────────┤
 * │  Pin 5 (PWM)  │  Gate MOSFET (qua 100Ω)  │  Điều khiển tốc độ quạt  │
 * └───────────────┴──────────────────────────┴───────────────────────────┘
 *
 * Mạch MOSFET (N-channel: IRLZ44N / IRFZ44N):
 *   Arduino Pin5 ──[100Ω]── Gate
 *   Drain ──────────────── Dây GND quạt (đen)
 *   Source ─────────────── GND chung
 *   Dây 12V quạt (đỏ) ──── Nguồn 12V DC ngoài
 *
 * Serial commands (9600 baud):
 *   SPD#<0-100>  e.g. "SPD#80"  - set speed %
 *   STOP                         - stop fan
 *   RUN                          - run at last saved speed
 *   STATUS                       - print speed & PWM
 */

#include <EEPROM.h>
#include <stdlib.h>
#include "fan_config.h"

/* ── EEPROM address (same pattern as ArduinoARGBSync) ─────────── */
#define SPD_STORE   0     // stores fan speed percent 0-100

/* ── Function declarations ───────────────────────────────────────  */
void setup();
void loop();
void handleSerialInput();
void handleFanSpeed();
void eeprom_persist();
void eeprom_update();

/* ── Globals ─────────────────────────────────────────────────────  */
int  fan_speed_pct = 80;    // speed 0-100 %
bool fan_running   = true;

/* =================================================================*/
void setup() {
    eeprom_persist();              // restore last saved speed

    Serial.begin(SERIAL_BAUD);
    while (!Serial) { ; }

    pinMode(PIN_PWM, OUTPUT);

    Serial.println(F("Fan Speed Controller - Prism 6PRO 12V"));
    Serial.print(F("Restored speed: "));
    Serial.print(fan_speed_pct);
    Serial.println('%');
    Serial.println(F("Cmds: SPD#<0-100>  STOP  RUN  STATUS"));
}

/* =================================================================*/
void loop() {
    eeprom_update();
    handleSerialInput();
    handleFanSpeed();
}

/* =================================================================*/
void handleSerialInput() {
    while (Serial.available() > 0) {
        String in = Serial.readStringUntil('\n');
        in.trim();

        int  len = in.length() + 1;
        char str[len];
        in.toCharArray(str, len);

        Serial.print(F("Serial Input: "));
        Serial.println(str);

        if (strcmp(str, "STOP") == 0) {
            fan_running = false;
            Serial.println(F("Fan STOP"));

        } else if (strcmp(str, "RUN") == 0) {
            fan_running = true;
            Serial.print(F("Fan RUN at "));
            Serial.print(fan_speed_pct);
            Serial.println('%');

        } else if (strcmp(str, "STATUS") == 0) {
            int pwm = fan_running ? map(fan_speed_pct, 0, 100, 0, 255) : 0;
            if (fan_running && pwm > 0 && pwm < MIN_START_PWM) pwm = MIN_START_PWM;
            Serial.print(F("Speed: "));
            Serial.print(fan_speed_pct);
            Serial.print(F("%  PWM: "));
            Serial.print(pwm);
            Serial.print(F("/255  Running: "));
            Serial.println(fan_running ? F("YES") : F("NO"));

        } else if (in.indexOf("SPD") > -1) {
            // Parse "SPD#<value>"  — same strtok pattern as reference
            char *token = strtok(str, "#");
            token = strtok(NULL, "#");
            if (token != NULL) {
                int val = atoi(token);
                if (val < 0)   val = 0;
                if (val > 100) val = 100;
                fan_speed_pct = val;
                fan_running   = (val > 0);
                Serial.print(F("Setting speed: "));
                Serial.print(fan_speed_pct);
                Serial.println('%');
            }
        }
        // Ignore all other input (same as reference)
    }
}

/* =================================================================*/
void handleFanSpeed() {
    if (!fan_running) {
        analogWrite(PIN_PWM, 0);
        return;
    }

    // Map percent -> PWM, enforce dead-band so fan actually starts
    int pwm = map(fan_speed_pct, 0, 100, 0, 255);
    if (pwm > 0 && pwm < MIN_START_PWM) pwm = MIN_START_PWM;
    analogWrite(PIN_PWM, pwm);
}

/* =================================================================*/
void eeprom_persist() {
    int saved = EEPROM.read(SPD_STORE);
    // 0xFF (255) = first boot / erased EEPROM → use default 80%
    if (saved >= 0 && saved <= 100) {
        fan_speed_pct = saved;
    }
}

void eeprom_update() {
    EEPROM.update(SPD_STORE, (uint8_t)fan_speed_pct);
}
