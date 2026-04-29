#pragma once

/**
 * @file   hw_config.h
 * @brief  Centralized hardware pin configuration for ESP32 audio modules.
 *
 * Both modules share the same I2S clock domain but use separate I2S ports:
 *   MAX98357A (speaker) → I2S_NUM_1  (TX)
 *   INMP441   (mic)     → I2S_NUM_0  (RX)
 *
 * ── MAX98357A Wiring ─────────────────────────────────────────────
 *  ┌─────────────┬────────────┬───────────────────────────────┐
 *  │ MAX98357A   │ ESP32 GPIO │ Notes                         │
 *  ├─────────────┼────────────┼───────────────────────────────┤
 *  │ DIN         │ 22         │ I2S data out                  │
 *  │ BCLK        │ 19         │ Bit clock                     │
 *  │ LRC         │ 18         │ Word select (LRCLK)           │
 *  │ SD          │ (NC)       │ Float = always ON             │
 *  │ GAIN        │ (NC)       │ Float = 15 dB gain            │
 *  │ Vin         │ 5V         │                               │
 *  │ GND         │ GND        │                               │
 *  └─────────────┴────────────┴───────────────────────────────┘
 *
 * ── INMP441 Wiring ───────────────────────────────────────────────
 *  ┌─────────────┬────────────┬───────────────────────────────┐
 *  │ INMP441     │ ESP32 GPIO │ Notes                         │
 *  ├─────────────┼────────────┼───────────────────────────────┤
 *  │ SD          │ 34         │ Serial data (input-only GPIO) │
 *  │ SCK         │ 33         │ Bit clock                     │
 *  │ WS          │ 32         │ Word select (LRCLK)           │
 *  │ L/R         │ GND        │ IMPORTANT: selects LEFT ch    │
 *  │ VDD         │ 3.3V       │                               │
 *  │ GND         │ GND        │                               │
 *  └─────────────┴────────────┴───────────────────────────────┘
 */

// ── MAX98357A (speaker, I2S_NUM_1, TX) ───────────────────────────
#define HW_SPK_DIN_PIN      22
#define HW_SPK_BCLK_PIN     19
#define HW_SPK_LRC_PIN      18

// ── INMP441 (microphone, I2S_NUM_0, RX) ──────────────────────────
#define HW_MIC_SD_PIN       33   // input-only GPIO – ideal for data line
#define HW_MIC_SCK_PIN      32
#define HW_MIC_WS_PIN       25
// L/R pin must be tied to GND on the PCB to select LEFT channel

// ── Audio parameters ──────────────────────────────────────────────
#define HW_SAMPLE_RATE          16000   // Hz  (both modules)
#define HW_DMA_BUF_COUNT        8
#define HW_DMA_BUF_LEN          256     // samples per DMA buffer

// ── Serial transfer ───────────────────────────────────────────────
// High baud rate for fast binary audio transfer to PC.
// receive.py on the PC side must use the same value.
#define HW_SERIAL_BAUD          115200

// ── Recording defaults ────────────────────────────────────────────
// 3 s × 16000 samples/s × 2 bytes = 96 KB — safe for ESP32 heap without PSRAM
#define HW_RECORD_DURATION_MS   3000
