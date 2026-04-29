// Standard Arduino header for basic Arduino functions (pinMode, digitalWrite, Serial, etc.)
#include <Arduino.h>
// ESP-IDF header for using the I2S (Inter-IC Sound) peripheral driver
#include "driver/i2s.h"

// Common I2S (Inter-IC Sound) Configuration parameters
#define I2S_SAMPLE_RATE         16000 // Audio sample rate in Hz (samples per second)
#define I2S_BITS_PER_SAMPLE     I2S_BITS_PER_SAMPLE_16BIT // Number of bits per audio sample (e.g., 16-bit, 24-bit, 32-bit)
                                                          // INMP441 outputs 24-bit data, but 16-bit is often sufficient and simpler.
#define I2S_BUFFER_SIZE         1024 // Size of the audio buffer in samples. This buffer temporarily stores audio data.

// I2S Pins - INMP441 (Input Microphone)
#define I2S_MIC_SERIAL_CLOCK    14 // I2S Serial Clock (SCK) or Bit Clock (CLK) pin for the microphone
#define I2S_MIC_WORD_SELECT     15 // I2S Word Select (WS) or Left/Right Clock (LRCL) pin for the microphone
#define I2S_MIC_SERIAL_DATA     32 // I2S Serial Data (SD) or Data Out (DOUT) pin from the microphone
#define I2S_MIC_PORT            I2S_NUM_0 // ESP32 I2S peripheral port number for the microphone (ESP32 has two I2S peripherals: I2S_NUM_0 and I2S_NUM_1)

// I2S Pins - MAX98357A (Output Amplifier)
#define I2S_AMP_SERIAL_CLOCK    27 // I2S Bit Clock (BCLK) pin for the amplifier
#define I2S_AMP_WORD_SELECT     26 // I2S Left/Right Clock (LRC / WS) pin for the amplifier
#define I2S_AMP_SERIAL_DATA     25 // I2S Serial Data In (DIN) pin for the amplifier
#define I2S_AMP_PORT            I2S_NUM_1 // ESP32 I2S peripheral port number for the amplifier
#define MAX98357_SD_PIN         33 // Shutdown (SD) pin for the MAX98357A amplifier. Active LOW for shutdown.

// Audio buffer: This array will store audio samples read from the microphone or to be written to the amplifier.
// It's defined as int16_t because we're using 16-bit samples.
int16_t i2s_audio_buffer[I2S_BUFFER_SIZE];

// Function to set up the I2S peripheral for audio input from the INMP441 microphone.
void setup_i2s_input() {
    Serial.println("Configuring I2S Input (INMP441)...");

    // Configure I2S parameters for the microphone
    i2s_config_t i2s_mic_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // Set I2S mode to Master and Receiver (RX)
        .sample_rate = I2S_SAMPLE_RATE,                     // Set the audio sample rate
        .bits_per_sample = I2S_BITS_PER_SAMPLE,             // Set the number of bits per sample
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,        // Configure for mono audio, using only the left channel.
                                                            // For INMP441, connecting L/R pin to GND selects left channel data on SD.
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Use standard I2S communication format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt priority level
        .dma_buf_count = 8,                                 // Number of DMA (Direct Memory Access) buffers to use
        .dma_buf_len = 256,                                 // Size of each DMA buffer in samples
        .use_apll = false,                                  // Do not use Audio PLL (Phase-Locked Loop) as clock source (APB clock is simpler)
        .tx_desc_auto_clear = false,                        // Not applicable for RX mode
        .fixed_mclk = 0                                     // No fixed Master Clock output
    };

    // Configure I2S pins for the microphone
    i2s_pin_config_t i2s_mic_pins = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK,    // Assign the SCK/BCLK pin
        .ws_io_num = I2S_MIC_WORD_SELECT,      // Assign the WS/LRCL pin
        .data_out_num = I2S_PIN_NO_CHANGE,     // Data Out pin is not used for RX mode
        .data_in_num = I2S_MIC_SERIAL_DATA     // Assign the SD/Data In pin
    };

    esp_err_t err; // Variable to store error codes from ESP-IDF functions

    // Install and start the I2S driver for the microphone
    err = i2s_driver_install(I2S_MIC_PORT, &i2s_mic_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S RX driver: %s\n", esp_err_to_name(err));
        return; // Exit if driver installation fails
    }

    // Set the I2S pins for the microphone
    err = i2s_set_pin(I2S_MIC_PORT, &i2s_mic_pins);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S RX pins: %s\n", esp_err_to_name(err));
        return; // Exit if pin configuration fails
    }
    Serial.println("I2S Input (INMP441) initialized.");
}

// Function to set up the I2S peripheral for audio output to the MAX98357A amplifier.
void setup_i2s_output() {
    Serial.println("Configuring I2S Output (MAX98357)...");

    // Enable the MAX98357A amplifier
    pinMode(MAX98357_SD_PIN, OUTPUT);      // Set the shutdown pin as an output
    digitalWrite(MAX98357_SD_PIN, HIGH);   // Pull the shutdown pin HIGH to enable the amplifier (it's active LOW)

    // Configure I2S parameters for the amplifier
    i2s_config_t i2s_amp_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // Set I2S mode to Master and Transmitter (TX)
        .sample_rate = I2S_SAMPLE_RATE,                     // Set the audio sample rate (must match input)
        .bits_per_sample = I2S_BITS_PER_SAMPLE,             // Set the number of bits per sample (must match input)
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,        // Configure for mono audio, outputting to the left channel.
                                                            // MAX98357A typically sums L+R if stereo data is sent, or plays the single channel.
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Use standard I2S communication format
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,           // Interrupt priority level
        .dma_buf_count = 8,                                 // Number of DMA buffers
        .dma_buf_len = 256,                                 // Size of each DMA buffer in samples
        .use_apll = false,                                  // Do not use Audio PLL
        .tx_desc_auto_clear = true,                         // Automatically clear TX DMA descriptors after data is sent
        .fixed_mclk = 0                                     // No fixed Master Clock output
    };

    // Configure I2S pins for the amplifier
    i2s_pin_config_t i2s_amp_pins = {
        .bck_io_num = I2S_AMP_SERIAL_CLOCK,    // Assign the BCLK pin
        .ws_io_num = I2S_AMP_WORD_SELECT,      // Assign the LRC/WS pin
        .data_out_num = I2S_AMP_SERIAL_DATA,   // Assign the DIN/Data Out pin
        .data_in_num = I2S_PIN_NO_CHANGE       // Data In pin is not used for TX mode
    };

    esp_err_t err; // Variable to store error codes

    // Install and start the I2S driver for the amplifier
    err = i2s_driver_install(I2S_AMP_PORT, &i2s_amp_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S TX driver: %s\n", esp_err_to_name(err));
        return; // Exit if driver installation fails
    }

    // Set the I2S pins for the amplifier
    err = i2s_set_pin(I2S_AMP_PORT, &i2s_amp_pins);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S TX pins: %s\n", esp_err_to_name(err));
        return; // Exit if pin configuration fails
    }

    // Zero out the DMA buffer (transmit silence) initially to prevent noise.
    i2s_zero_dma_buffer(I2S_AMP_PORT);

    Serial.println("I2S Output (MAX98357) initialized.");
}

// Standard Arduino setup function, runs once at the beginning.
void setup() {
    // Initialize Serial communication for debugging and status messages.
    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial port to connect (especially for some ESP32 boards).

    // Initialize I2S for audio input (microphone).
    setup_i2s_input();
    // Initialize I2S for audio output (amplifier).
    setup_i2s_output();

    Serial.println("Audio passthrough setup complete. Starting loop...");
}

// Standard Arduino loop function, runs repeatedly after setup().
void loop() {
    size_t bytes_read = 0; // Variable to store the number of bytes read from I2S input.

    // Read audio data from the I2S microphone (INMP441).
    // This is a blocking call, it will wait until data is available or timeout (portMAX_DELAY means wait indefinitely).
    esp_err_t read_err = i2s_read(I2S_MIC_PORT, (void*)i2s_audio_buffer, sizeof(i2s_audio_buffer), &bytes_read, portMAX_DELAY);

    // Check if the read operation was successful and if any data was read.
    if (read_err == ESP_OK && bytes_read > 0) {
        // Optional: Print the number of bytes read for debugging.
        // Serial.printf("Read %d bytes from INMP441. ", bytes_read);

        size_t bytes_written = 0; // Variable to store the number of bytes written to I2S output.

        // Write the audio data (read from the microphone) to the I2S amplifier (MAX98357A).
        // This is also a blocking call.
        esp_err_t write_err = i2s_write(I2S_AMP_PORT, (const void*)i2s_audio_buffer, bytes_read, &bytes_written, portMAX_DELAY);

        // Check if the write operation was successful.
        if (write_err == ESP_OK) {
            // Check if all the data read was actually written.
            if (bytes_written != bytes_read) {
                Serial.printf("Partial write: %d bytes written out of %d\n", bytes_written, bytes_read);
            }
            // Optional: Print the number of bytes written for debugging.
            // else {
            //     Serial.printf("Wrote %d bytes to MAX98357.\n", bytes_written);
            // }
        } else {
            // Print an error message if I2S write failed.
            Serial.printf("I2S write error: %s\n", esp_err_to_name(write_err));
        }
    } else if (read_err != ESP_OK) {
        // Print an error message if I2S read failed.
        Serial.printf("I2S read error: %s\n", esp_err_to_name(read_err));
    }

    // No delay in the main loop for real-time passthrough.
    // The I2S read/write functions are blocking and will effectively control the loop rate
    // based on the sample rate and buffer sizes.
    // Adding a delay here would introduce latency or break the audio stream.
    // vTaskDelay(pdMS_TO_TICKS(1)); // Example: A small RTOS delay, generally not needed here.
}