#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include "hw_config.h"

// ── I2S Port ──────────────────────────────────────────────────────
#define MIC_I2S_PORT    I2S_NUM_0   // RX – must differ from speaker (I2S_NUM_1)

// ── Microphone Pins (from hw_config.h) ───────────────────────────
#define MIC_SD_PIN      HW_MIC_SD_PIN    // 34 – input-only GPIO
#define MIC_SCK_PIN     HW_MIC_SCK_PIN   // 33
#define MIC_WS_PIN      HW_MIC_WS_PIN    // 32
// L/R pin → GND → LEFT channel

// ── Audio Config ──────────────────────────────────────────────────
#define MIC_SAMPLE_RATE     HW_SAMPLE_RATE     // 16 000 Hz
#define MIC_BUFFER_LEN      HW_DMA_BUF_LEN     // samples per DMA buffer

/**
 * @brief Initialize I2S for the INMP441 microphone.
 *        Call once in setup().
 */
void mic_init();

/**
 * @brief Record audio into a caller-supplied 16-bit PCM buffer.
 *
 * Software gain (×8) is applied to compensate for INMP441 low sensitivity.
 * Peak level is printed to Serial for diagnostic purposes.
 *
 * @param buf        Caller-allocated destination buffer
 * @param numSamples Number of samples to capture
 * @return           Actual samples written
 */
uint32_t mic_record(int16_t *buf, uint32_t numSamples);

/**
 * @brief Stream a recorded PCM buffer to the PC via Serial.
 *
 * Wire protocol (binary):
 *   "AUDIO_START\n"     – ASCII start marker
 *   uint32_t numSamples – 4 bytes little-endian
 *   uint32_t sampleRate – 4 bytes little-endian
 *   int16_t  samples[]  – numSamples × 2 bytes raw PCM
 *   "AUDIO_END\n"       – ASCII end marker
 *
 * Run voice/receive.py on the PC to receive and save as WAV.
 *
 * @param buf        PCM buffer recorded by mic_record()
 * @param numSamples Number of valid samples in buf
 */
void mic_send_serial(const int16_t *buf, uint32_t numSamples);
