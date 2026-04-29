/**
 * @file    main.cpp
 * @brief   ESP32 – MAX98357A + INMP441 audio test.
 *
 * Sequence:
 *   1. Play one pip          → signals recording is about to start
 *   2. Record into RAM       → HW_RECORD_DURATION_MS seconds
 *   3. Stream over Serial    → run voice/receive.py on the PC to save WAV
 *
 * Pin assignments: see include/hw_config.h
 */

#include <Arduino.h>
#include "Max98357A.h"
#include "INMP441.h"
#include "hw_config.h"

static const uint32_t TOTAL_SAMPLES =
    (uint32_t)HW_SAMPLE_RATE * HW_RECORD_DURATION_MS / 1000;

static int16_t *audioBuf = nullptr;

void setup() {
    Serial.begin(HW_SERIAL_BAUD);
    Serial.println("\n== ESP32 Speaker + Mic Test ==");

    // Allocate recording buffer (prefer PSRAM when available)
    audioBuf = psramFound()
        ? (int16_t *)ps_malloc(TOTAL_SAMPLES * sizeof(int16_t))
        : (int16_t *)malloc(TOTAL_SAMPLES * sizeof(int16_t));

    if (!audioBuf) {
        Serial.println("[ERROR] Cannot allocate audio buffer. Halting.");
        while (true) { delay(1000); }
    }

    spk_init();
    mic_init();

    // Pip → record → send
    Serial.println("[TEST] Playing pip...");
    spk_playPip(1);

    Serial.println("[TEST] Recording...");
    uint32_t captured = mic_record(audioBuf, TOTAL_SAMPLES);

    Serial.println("[TEST] Sending to PC...");
    mic_send_serial(audioBuf, captured);

    Serial.println("[TEST] Done.");
}

void loop() {
    // nothing
}