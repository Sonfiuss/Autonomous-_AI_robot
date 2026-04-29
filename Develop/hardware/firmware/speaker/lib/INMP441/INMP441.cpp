#include "INMP441.h"

// ── I2S microphone initialization ────────────────────────────────
void mic_init() {
    i2s_config_t cfg = {
        // INMP441 outputs 24-bit left-justified data in a 32-bit I2S frame
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate          = MIC_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = MIC_BUFFER_LEN,
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

    Serial.println("[DEBUG] Bat dau i2s_driver_install...");
    esp_err_t err = i2s_driver_install(MIC_I2S_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) {
        Serial.printf("[MIC] i2s_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    } else {
        Serial.println("[DEBUG] i2s_driver_install OK");
    }

    Serial.println("[DEBUG] Bat dau i2s_set_pin...");
    err = i2s_set_pin(MIC_I2S_PORT, &pins);
    if (err != ESP_OK) {
        Serial.printf("[MIC] i2s_set_pin failed: %s\n", esp_err_to_name(err));
        return;
    } else {
        Serial.println("[DEBUG] i2s_set_pin OK");
    }

    i2s_zero_dma_buffer(MIC_I2S_PORT);
    Serial.println("[MIC] INMP441 ready.");
}

// ── Record into a RAM buffer ──────────────────────────────────────
uint32_t mic_record(int16_t *buf, uint32_t numSamples) {
    if (!buf || numSamples == 0) return 0;

    int32_t  raw[MIC_BUFFER_LEN];
    size_t   bytesRead;
    uint32_t written = 0;
    int32_t  peakRaw = 0;   // for diagnostic

    Serial.printf("[MIC] Recording %u samples...\n", numSamples);

    while (written < numSamples) {
        uint32_t toRead = min((uint32_t)MIC_BUFFER_LEN, numSamples - written);

        esp_err_t err = i2s_read(MIC_I2S_PORT, raw,
                                 toRead * sizeof(int32_t),
                                 &bytesRead, portMAX_DELAY);
        if (err != ESP_OK) {
            Serial.printf("[MIC] i2s_read error: %s\n", esp_err_to_name(err));
            break;
        }
        if(bytesRead == 0) {
            Serial.println("[MIC] i2s_read timeout or no data.");
            continue;
        }

        uint32_t got = bytesRead / sizeof(int32_t);
        for (uint32_t i = 0; i < got; i++) {
            int32_t sample = (raw[i] >> 16) << 5;
            if (sample >  32767) sample =  32767;
            if (sample < -32768) sample = -32768;
            buf[written + i] = (int16_t)sample;

            int32_t absVal = sample < 0 ? -sample : sample;
            if (absVal > peakRaw) peakRaw = absVal;

            // Debug: In 10 mau dau tien
            if (written + i < 10) {
                Serial.printf("[DEBUG] raw[%u]=%d, pcm=%d\n", written + i, raw[i], (int16_t)sample);
            }
        }
        written += got;
    }

    // Diagnostic: peak level as percentage of full scale
    float peakPct = (peakRaw / 32768.0f) * 100.0f;
    Serial.printf("[MIC] Captured %u samples. Peak: %d (%.1f%% FS)\n",
                  written, peakRaw, peakPct);
    if (peakPct < 0.1f)
        Serial.println("[MIC] WARNING: Signal near zero. Check wiring (L/R→GND, VDD=3.3V, SD pin).");
    else if (peakPct < 1.0f)
        Serial.println("[MIC] INFO: Signal very low. Try increasing gain (shift more than <<3).");
    else
        Serial.println("[MIC] Signal OK.");

    return written;
}

// ── Stream PCM buffer to PC via Serial ───────────────────────────
void mic_send_serial(const int16_t *buf, uint32_t numSamples) {
    if (!buf || numSamples == 0) return;

    const uint32_t sampleRate = MIC_SAMPLE_RATE;

    // Stop any debug prints that could corrupt the binary stream
    Serial.println("[MIC] Sending audio over Serial...");
    delay(10);   // flush Serial TX queue

    // ── Start marker ─────────────────────────────────────────────
    Serial.print("AUDIO_START\n");

    // ── Metadata (8 bytes, little-endian) ────────────────────────
    Serial.write((const uint8_t *)&numSamples, 4);
    Serial.write((const uint8_t *)&sampleRate, 4);

    // ── Raw PCM data ─────────────────────────────────────────────
    // Write in chunks to avoid watchdog resets on large buffers
    const uint32_t chunkSamples = 512;
    uint32_t sent = 0;
    while (sent < numSamples) {
        uint32_t chunk = min(chunkSamples, numSamples - sent);
        Serial.write((const uint8_t *)&buf[sent], chunk * sizeof(int16_t));
        sent += chunk;
    }

    // ── End marker ───────────────────────────────────────────────
    Serial.print("AUDIO_END\n");
    Serial.flush();

    Serial.println("[MIC] Transfer complete.");
}


