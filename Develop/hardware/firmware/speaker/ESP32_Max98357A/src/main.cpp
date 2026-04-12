/**
 * @file    main.cpp
 * @brief   ESP32 – Record (INMP441) & Playback (MAX98357A) Loop
 *
 * Chu kỳ hoạt động:
 *   1. "Pip" ngắn  → bắt đầu thu âm 10 giây
 *   2. "Pip Pip"   → kết thúc thu âm
 *   3. Delay 2 giây
 *   4. Phát lại đoạn vừa thu
 *   5. Delay 3 giây → lặp lại từ bước 1
 *
 * Kết nối phần cứng:
 * ┌─────────────┬────────────┬─────────────────────────────┐
 * │ MAX98357A   │ ESP32      │ Ghi chú                     │
 * ├─────────────┼────────────┼─────────────────────────────┤
 * │ DIN         │ GPIO 25    │ I2S data out                │
 * │ BCLK        │ GPIO 27    │ I2S bit clock               │
 * │ LRC         │ GPIO 26    │ I2S word select             │
 * │ SD          │ (để trống) │ Shutdown – float = ON       │
 * │ GAIN        │ (để trống) │ Float = 15 dB gain          │
 * │ Vin         │ 5V         │                             │
 * │ GND         │ GND        │                             │
 * └─────────────┴────────────┴─────────────────────────────┘
 *
 * ┌─────────────┬────────────┬─────────────────────────────┐
 * │ INMP441     │ ESP32      │ Ghi chú                     │
 * ├─────────────┼────────────┼─────────────────────────────┤
 * │ SD          │ GPIO 34    │ I2S data in                 │
 * │ SCK         │ GPIO 33    │ I2S bit clock               │
 * │ WS          │ GPIO 32    │ I2S word select             │
 * │ L/R         │ GND        │ QUAN TRỌNG: kênh Left       │
 * │ VD          │ 3.3V       │                             │
 * │ GND         │ GND        │                             │
 * └─────────────┴────────────┴─────────────────────────────┘
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/dac.h>

// ── I2S Port Assignment ───────────────────────────────────────────
#define I2S_MIC_PORT    I2S_NUM_0   // INMP441 microphone
#define I2S_SPK_PORT    I2S_NUM_1   // MAX98357A speaker

// ── Microphone Pins (INMP441) ─────────────────────────────────────
#define MIC_SD_PIN      34
#define MIC_SCK_PIN     33
#define MIC_WS_PIN      32

// ── Speaker Pins (MAX98357A) ──────────────────────────────────────
#define SPK_DIN_PIN     22
#define SPK_BCLK_PIN    19
#define SPK_LRC_PIN     18

// ── Audio Config ──────────────────────────────────────────────────
#define SAMPLE_RATE         16000       // 16 kHz
#define SAMPLE_BITS         16
#define I2S_BUFFER_LEN      1024        // samples per DMA buffer
#define RECORD_SECONDS      3    // 3s = 96KB – an toàn cho heap ESP32 không PSRAM
#define TOTAL_SAMPLES       ((uint32_t)SAMPLE_RATE * RECORD_SECONDS)
#define AUDIO_BUF_BYTES     (TOTAL_SAMPLES * sizeof(int16_t))

// ── Timing ────────────────────────────────────────────────────────
#define DELAY_BEFORE_PLAY_MS    2000
#define DELAY_AFTER_PLAY_MS     3000

// ── Audio buffer in PSRAM (or heap if no PSRAM) ───────────────────
static int16_t *audioBuf = nullptr;

// ─────────────────────────────────────────────────────────────────
void i2s_mic_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_32BIT, // INMP441 outputs 24-bit in 32-bit frame
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = I2S_BUFFER_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = false,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = MIC_SCK_PIN,
        .ws_io_num    = MIC_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = MIC_SD_PIN
    };
    esp_err_t err = i2s_driver_install(I2S_MIC_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) Serial.printf("[ERROR] MIC i2s_driver_install: %s\n", esp_err_to_name(err));
    err = i2s_set_pin(I2S_MIC_PORT, &pins);
    if (err != ESP_OK) Serial.printf("[ERROR] MIC i2s_set_pin: %s\n", esp_err_to_name(err));
    i2s_zero_dma_buffer(I2S_MIC_PORT);
}

void i2s_spk_init() {
    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = I2S_BUFFER_LEN,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };
    i2s_pin_config_t pins = {
        .bck_io_num   = SPK_BCLK_PIN,
        .ws_io_num    = SPK_LRC_PIN,
        .data_out_num = SPK_DIN_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };
    // GPIO 25 = DAC1, GPIO 26 = DAC2 – phải disable DAC trước khi I2S dùng
    dac_output_disable(DAC_CHANNEL_1);
    dac_output_disable(DAC_CHANNEL_2);
    esp_err_t err = i2s_driver_install(I2S_SPK_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) Serial.printf("[ERROR] SPK i2s_driver_install: %s\n", esp_err_to_name(err));
    err = i2s_set_pin(I2S_SPK_PORT, &pins);
    if (err != ESP_OK) Serial.printf("[ERROR] SPK i2s_set_pin: %s\n", esp_err_to_name(err));
    i2s_zero_dma_buffer(I2S_SPK_PORT);
}

// ── Phát tone pip qua loa ─────────────────────────────────────────
void playPip(int count, int freqHz = 1000, int durationMs = 150, int gapMs = 120) {
    const int samples  = SAMPLE_RATE * durationMs / 1000;
    const float period = (float)SAMPLE_RATE / freqHz;
    int16_t buf[128];
    size_t written;

    for (int p = 0; p < count; p++) {
        int idx = 0;
        for (int s = 0; s < samples; ) {
            int chunk = min(128, samples - s);
            for (int i = 0; i < chunk; i++, s++) {
                buf[i] = (int16_t)(30000.0f * sinf(2.0f * M_PI * s / period));
            }
            i2s_write(I2S_SPK_PORT, buf, chunk * sizeof(int16_t), &written, portMAX_DELAY);
        }
        if (p < count - 1) delay(gapMs);
    }
    // flush silence
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < 4; i++) {
        i2s_write(I2S_SPK_PORT, buf, sizeof(buf), &written, portMAX_DELAY);
    }
}

// ── Ghi âm vào buffer ─────────────────────────────────────────────
void recordAudio() {
    Serial.println("[REC] Recording...");
    uint32_t samplesRead = 0;
    int32_t raw32[I2S_BUFFER_LEN];
    size_t bytesRead;

    while (samplesRead < TOTAL_SAMPLES) {
        uint32_t remaining = TOTAL_SAMPLES - samplesRead;
        uint32_t toRead    = min((uint32_t)I2S_BUFFER_LEN, remaining);

        i2s_read(I2S_MIC_PORT, raw32, toRead * sizeof(int32_t), &bytesRead, portMAX_DELAY);
        uint32_t got = bytesRead / sizeof(int32_t);

        for (uint32_t i = 0; i < got && samplesRead < TOTAL_SAMPLES; i++, samplesRead++) {
            // INMP441: data is in upper 24 bits of 32-bit word, shift down to 16-bit
            audioBuf[samplesRead] = (int16_t)(raw32[i] >> 16);
        }
    }
    Serial.printf("[REC] Done. %u samples captured.\n", samplesRead);
}

// ── Phát lại từ buffer ────────────────────────────────────────────
void playbackAudio() {
    Serial.println("[PLAY] Playing back...");
    uint32_t sent = 0;
    size_t written;

    while (sent < TOTAL_SAMPLES) {
        uint32_t chunk = min((uint32_t)I2S_BUFFER_LEN, TOTAL_SAMPLES - sent);
        i2s_write(I2S_SPK_PORT, &audioBuf[sent], chunk * sizeof(int16_t), &written, portMAX_DELAY);
        sent += written / sizeof(int16_t);
    }
    Serial.println("[PLAY] Done.");
}

// ─────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("\n== ESP32 Record & Playback ==");

    // Cấp phát buffer (ưu tiên PSRAM nếu có)
    if (psramFound()) {
        audioBuf = (int16_t *)ps_malloc(AUDIO_BUF_BYTES);
        Serial.printf("PSRAM found, buffer %u bytes in PSRAM\n", AUDIO_BUF_BYTES);
    } else {
        audioBuf = (int16_t *)malloc(AUDIO_BUF_BYTES);
        Serial.printf("No PSRAM, buffer %u bytes in heap\n", AUDIO_BUF_BYTES);
    }

    if (!audioBuf) {
        Serial.println("ERROR: Cannot allocate audio buffer! Halting.");
        while (true) { delay(1000); }
    }

    i2s_mic_init();
    i2s_spk_init();

    // Speaker self-test: 3 pip để xác nhận loa hoạt động
    Serial.println("[TEST] Speaker self-test...");
    delay(500);
    playPip(3);
    Serial.println("[TEST] Speaker OK. Starting main loop.");
    delay(500);
}

void loop() {
    // ── Hardware test: phát tone 1kHz liên tục ───────────────────
    playPip(1, 1000, 500, 0);
}