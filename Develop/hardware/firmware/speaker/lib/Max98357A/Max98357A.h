#pragma once

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/dac.h>
#include "hw_config.h"

// ── I2S Port ──────────────────────────────────────────────────────
#define SPK_I2S_PORT    I2S_NUM_1   // TX – must differ from mic (I2S_NUM_0)

// ── Speaker Pins (from hw_config.h) ──────────────────────────────
#define SPK_DIN_PIN     HW_SPK_DIN_PIN    // 22
#define SPK_BCLK_PIN    HW_SPK_BCLK_PIN   // 19
#define SPK_LRC_PIN     HW_SPK_LRC_PIN    // 18

// ── Audio Config ──────────────────────────────────────────────────
#define SPK_SAMPLE_RATE     HW_SAMPLE_RATE    // 16 000 Hz
#define SPK_BUFFER_LEN      HW_DMA_BUF_LEN    // samples per DMA buffer

/**
 * @brief Initialize I2S for the MAX98357A amplifier.
 *        Call once in setup().
 */
void spk_init();

/**
 * @brief Play a beep tone through the speaker.
 *
 * @param count      Number of beeps
 * @param freqHz     Tone frequency in Hz (default 1000 Hz)
 * @param durationMs Duration of each beep in ms (default 150 ms)
 * @param gapMs      Silence gap between beeps in ms (default 120 ms)
 */
void spk_playPip(int count, int freqHz = 1000, int durationMs = 150, int gapMs = 120);

/**
 * @brief Play a raw 16-bit mono PCM buffer through the speaker.
 *
 * @param buf        Pointer to the PCM 16-bit mono buffer
 * @param numSamples Number of samples to play
 */
void spk_playBuffer(const int16_t *buf, uint32_t numSamples);
