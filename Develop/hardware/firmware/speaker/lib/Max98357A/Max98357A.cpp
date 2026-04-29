#include "Max98357A.h"

// ── I2S speaker initialization ───────────────────────────────────
void spk_init() {
    // Disable DAC on GPIO25/26 to avoid conflict with I2S
    dac_output_disable(DAC_CHANNEL_1);
    dac_output_disable(DAC_CHANNEL_2);

    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SPK_SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = SPK_BUFFER_LEN,
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

    esp_err_t err = i2s_driver_install(SPK_I2S_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) {
        Serial.printf("[SPK] i2s_driver_install failed: %s\n", esp_err_to_name(err));
        return;
    }

    err = i2s_set_pin(SPK_I2S_PORT, &pins);
    if (err != ESP_OK) {
        Serial.printf("[SPK] i2s_set_pin failed: %s\n", esp_err_to_name(err));
        return;
    }

    i2s_zero_dma_buffer(SPK_I2S_PORT);
    Serial.println("[SPK] MAX98357A ready.");
}

// ── Beep tone output ─────────────────────────────────────────────
void spk_playPip(int count, int freqHz, int durationMs, int gapMs) {
    const int   samples = SPK_SAMPLE_RATE * durationMs / 1000;
    const float period  = (float)SPK_SAMPLE_RATE / freqHz;
    int16_t     buf[128];
    size_t      written;

    for (int p = 0; p < count; p++) {
        int s = 0;
        while (s < samples) {
            int chunk = min(128, samples - s);
            for (int i = 0; i < chunk; i++, s++) {
                buf[i] = (int16_t)(30000.0f * sinf(2.0f * M_PI * s / period));
            }
            i2s_write(SPK_I2S_PORT, buf, chunk * sizeof(int16_t), &written, portMAX_DELAY);
        }
        if (p < count - 1) {
            // flush silence between pips
            memset(buf, 0, sizeof(buf));
            int silenceSamples = SPK_SAMPLE_RATE * gapMs / 1000;
            int s2 = 0;
            while (s2 < silenceSamples) {
                int chunk = min(128, silenceSamples - s2);
                i2s_write(SPK_I2S_PORT, buf, chunk * sizeof(int16_t), &written, portMAX_DELAY);
                s2 += chunk;
            }
        }
    }

    // flush trailing silence to prevent DMA stall
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < 4; i++) {
        i2s_write(SPK_I2S_PORT, buf, sizeof(buf), &written, portMAX_DELAY);
    }
}

// ── PCM buffer playback ──────────────────────────────────────────
void spk_playBuffer(const int16_t *buf, uint32_t numSamples) {
    if (!buf || numSamples == 0) return;

    uint32_t sent = 0;
    size_t   written;

    while (sent < numSamples) {
        uint32_t chunk = min((uint32_t)SPK_BUFFER_LEN, numSamples - sent);
        i2s_write(SPK_I2S_PORT, &buf[sent], chunk * sizeof(int16_t), &written, portMAX_DELAY);
        sent += written / sizeof(int16_t);
    }
}
