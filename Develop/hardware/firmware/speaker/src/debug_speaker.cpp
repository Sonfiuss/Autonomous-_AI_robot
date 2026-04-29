/**
 * @file   debug_speaker.cpp
 * @brief  Minimal speaker-only debug sketch for MAX98357A
 *
 * MỤC ĐÍCH: Cô lập loa để xác định lỗi phần cứng hay phần mềm.
 *   - KHÔNG dùng mic, KHÔNG cấp phát buffer, KHÔNG cần PSRAM
 *   - Phát tone pip 1 kHz liên tục mỗi 2 giây
 *   - In chi tiết từng bước qua Serial (115200 baud)
 *
 * KẾT QUẢ KỲ VỌNG:
 *   Serial: "[STEP 1] i2s_driver_install ... OK"  → I2S driver load được
 *   Serial: "[STEP 2] i2s_set_pin ... OK"         → Pin map OK
 *   Serial: "[PIP] Writing tone ..."              → DMA đang ghi
 *   LỌA  : tiếng "pip" 1 kHz mỗi 2 giây          → Phần cứng OK
 *
 * CÁCH DÙNG:
 *   upload_debug.bat COM7
 *
 * KẾT NỐI MAX98357A:
 *   DIN  = GPIO 25  |  BCLK = GPIO 27  |  LRC = GPIO 26
 *   Vin  = 5V       |  GND  = GND
 *   SD   = float (not connected) = always ON
 */

#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

// ── Pins ──────────────────────────────────────────────────────────
#define SPK_DIN_PIN     25
#define SPK_BCLK_PIN    27
#define SPK_LRC_PIN     26

// ── Audio params ──────────────────────────────────────────────────
#define SAMPLE_RATE     16000
#define PIP_FREQ_HZ     1000
#define PIP_DURATION_MS 300
#define I2S_PORT        I2S_NUM_0
#define CHUNK           128

static void printSeparator() {
    Serial.println("----------------------------------------------");
}

static bool spk_init() {
    printSeparator();
    Serial.println("[STEP 1] i2s_driver_install ...");

    i2s_config_t cfg = {
        .mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate          = SAMPLE_RATE,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count        = 8,
        .dma_buf_len          = 256,
        .use_apll             = false,
        .tx_desc_auto_clear   = true,
        .fixed_mclk           = 0
    };

    esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
    if (err != ESP_OK) {
        Serial.printf("[STEP 1] FAILED: i2s_driver_install error 0x%x\n", err);
        return false;
    }
    Serial.println("[STEP 1] i2s_driver_install ... OK");

    // ── Step 2: Pin mapping ────────────────────────────────────────
    Serial.printf("[STEP 2] i2s_set_pin  DIN=%d  BCLK=%d  LRC=%d ...\n",
                  SPK_DIN_PIN, SPK_BCLK_PIN, SPK_LRC_PIN);

    i2s_pin_config_t pins = {
        .bck_io_num   = SPK_BCLK_PIN,
        .ws_io_num    = SPK_LRC_PIN,
        .data_out_num = SPK_DIN_PIN,
        .data_in_num  = I2S_PIN_NO_CHANGE
    };

    err = i2s_set_pin(I2S_PORT, &pins);
    if (err != ESP_OK) {
        Serial.printf("[STEP 2] FAILED: i2s_set_pin error 0x%x\n", err);
        return false;
    }
    Serial.println("[STEP 2] i2s_set_pin ... OK");

    // ── Step 3: Zero buffer ────────────────────────────────────────
    Serial.println("[STEP 3] i2s_zero_dma_buffer ...");
    i2s_zero_dma_buffer(I2S_PORT);
    Serial.println("[STEP 3] OK");

    return true;
}

static void playPip() {
    const int totalSamples = SAMPLE_RATE * PIP_DURATION_MS / 1000;
    const float period     = (float)SAMPLE_RATE / PIP_FREQ_HZ;
    int16_t buf[CHUNK];
    size_t written;
    int sent = 0;

    Serial.printf("[PIP] Writing %d samples @ %d Hz ...\n", totalSamples, PIP_FREQ_HZ);

    while (sent < totalSamples) {
        int chunk = min(CHUNK, totalSamples - sent);
        for (int i = 0; i < chunk; i++) {
            buf[i] = (int16_t)(20000.0f * sinf(2.0f * (float)M_PI * (sent + i) / period));
        }
        esp_err_t err = i2s_write(I2S_PORT, buf, chunk * sizeof(int16_t), &written, portMAX_DELAY);
        if (err != ESP_OK) {
            Serial.printf("[PIP] i2s_write error 0x%x\n", err);
            break;
        }
        sent += written / sizeof(int16_t);
    }

    // Flush silence to push last DMA buffer through DAC
    memset(buf, 0, sizeof(buf));
    for (int i = 0; i < 8; i++) {
        i2s_write(I2S_PORT, buf, sizeof(buf), &written, portMAX_DELAY);
    }
    Serial.println("[PIP] Done.");
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n============================================");
    Serial.println("  ESP32 MAX98357A Speaker Debug");
    Serial.println("============================================");

    if (!spk_init()) {
        Serial.println("\n>>> I2S init FAILED. Check wiring and restart. <<<");
        while (true) { delay(1000); }
    }

    printSeparator();
    Serial.println("I2S init SUCCESS.");
    Serial.println("Playing 3 test pips...");
    printSeparator();

    for (int i = 0; i < 3; i++) {
        playPip();
        delay(300);
    }

    Serial.println("\nIf you heard 3 pips -> HARDWARE OK, check main firmware.");
    Serial.println("If silent           -> Check wiring or replace MAX98357A.");
    printSeparator();
}

void loop() {
    Serial.println("[LOOP] Pip in 2s...");
    delay(2000);
    playPip();
}
