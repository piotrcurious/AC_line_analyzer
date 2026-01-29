// 50 Hz sine generator using I2S -> built-in DAC (robust variant)
// Writes stereo interleaved samples and streams continuously from a task.
// Measure on GPIO25 (DAC1). Tested approach is compatible with many esp32 Arduino cores.

#include <Arduino.h>
#include <WiFi.h>
#include "driver/i2s.h"

// If the esp-idf macro for both DAC channels exists, use it.
#if defined(I2S_DAC_CHANNEL_BOTH_EN)
  #define HAVE_I2S_DAC_BOTH 1
#endif

// --- CONFIG ------------------------------------------------
const int SIGNAL_FREQ_HZ      = 50;        // target frequency
const int SAMPLES_PER_CYCLE   = 200;       // samples per cycle per channel
const uint32_t SAMPLE_RATE_HZ = SIGNAL_FREQ_HZ * SAMPLES_PER_CYCLE; // 10 kHz
const int DAC_CHANNEL         = 1;         // 1 -> GPIO25 (DAC1). 2 -> GPIO26 (DAC2)
const int CPU_FREQ_MHZ        = 240;       // fix CPU freq
// ----------------------------------------------------------

static uint16_t *i2sBuffer = nullptr;
static size_t i2sBufferSamples = 0; // number of 16-bit words (interleaved L,R)
static TaskHandle_t streamTask = NULL;

void buildI2SBuffer(float amplitude = 1.0f, float dcOffset = 0.0f) {
  // We build an interleaved buffer: [L0, R0, L1, R1, ...]
  // Each element is uint16_t where DAC 8-bit value sits in high byte (value << 8).
  if (i2sBuffer) {
    free(i2sBuffer);
    i2sBuffer = nullptr;
  }

  // We'll create two words per sample (left + right)
  i2sBufferSamples = (size_t)SAMPLES_PER_CYCLE * 2;
  i2sBuffer = (uint16_t*)malloc(sizeof(uint16_t) * i2sBufferSamples);
  if (!i2sBuffer) {
    Serial.println("Failed to allocate I2S buffer");
    i2sBufferSamples = 0;
    return;
  }

  for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
    float angle = (2.0f * PI * i) / (float)SAMPLES_PER_CYCLE;
    float s = sinf(angle); // -1..1
    float scaled = (s * amplitude * 0.5f) + (0.5f + dcOffset * 0.5f); // 0..1
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 1.0f) scaled = 1.0f;
    uint16_t dac8 = (uint16_t) roundf(scaled * 255.0f) & 0xFF;
    uint16_t word = (uint16_t)(dac8 << 8);
    // place same sample into left and right slots to maximize compatibility
    i2sBuffer[2*i + 0] = word; // left
    i2sBuffer[2*i + 1] = word; // right
  }
}

// i2s driver configure
bool configureI2SForDAC() {
  // uninstall first in case it's running
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // interleaved stereo
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256, // in samples per buffer (tweak if you want)
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("i2s_driver_install failed: ");
    Serial.println((int)err);
    return false;
  }

  // No pins needed for built-in DAC
  i2s_pin_config_t pin_config;
  memset(&pin_config, 0, sizeof(pin_config));
  i2s_set_pin(I2S_NUM_0, &pin_config);

  // enable DAC channel(s)
  #ifdef HAVE_I2S_DAC_BOTH
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN); // best compatibility
    Serial.println("i2s_set_dac_mode: BOTH");
  #else
    if (DAC_CHANNEL == 1) {
      i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
      Serial.println("i2s_set_dac_mode: LEFT (DAC1)");
    } else {
      i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
      Serial.println("i2s_set_dac_mode: RIGHT (DAC2)");
    }
  #endif

  // clear DMA buffers
  i2s_zero_dma_buffer(I2S_NUM_0);

  return true;
}

// continuous streaming task
void i2s_stream_task(void *arg) {
  Serial.println("Stream task started");
  while (1) {
    if (!i2sBuffer || i2sBufferSamples == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    size_t bytesToWrite = i2sBufferSamples * sizeof(uint16_t);
    size_t bytesWritten = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, (const char*)i2sBuffer, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.print("i2s_write error: ");
      Serial.println((int)err);
      vTaskDelay(pdMS_TO_TICKS(100));
    } else if (bytesWritten != bytesToWrite) {
      Serial.print("Partial write bytesWritten=");
      Serial.print(bytesWritten);
      Serial.print(" expected=");
      Serial.println(bytesToWrite);
      // try again after a short delay
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    // loop to write again (keeps DMA supplied)
  }
}

// Stop streaming cleanly
void stopI2SOutput() {
  if (streamTask) {
    vTaskDelete(streamTask);
    streamTask = NULL;
  }
  // write a short mid-scale burst
  const uint16_t mid = (uint16_t)(127 << 8);
  for (int i = 0; i < 64; ++i) {
    size_t bw;
    i2s_write(I2S_NUM_0, (const char*)&mid, sizeof(mid), &bw, portMAX_DELAY);
  }
  i2s_driver_uninstall(I2S_NUM_0);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_DISABLE);
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\nESP32 50 Hz sine via I2S -> built-in DAC (robust)");

  // quick DAC sanity test: plain dacWrite to ensure hardware works
  Serial.println("Sanity: dacWrite midscale (127) for 1s on GPIO25...");
  dacWrite(25, 127);
  delay(1000);
  Serial.println("Sanity test done, proceeding.");

  setCpuFrequencyMhz(CPU_FREQ_MHZ); // lock CPU freq (optional)

  // small attempt to stop WiFi to reduce interruptions (Arduino API)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(10);

  buildI2SBuffer(1.0f, 0.0f);

  if (!configureI2SForDAC()) {
    Serial.println("I2S config failed, aborting.");
    while (1) delay(1000);
  }

  // create streaming task at high priority
  BaseType_t t = xTaskCreatePinnedToCore(
    i2s_stream_task, "i2s_stream", 4096, NULL, 5, &streamTask, 1);
  if (t != pdPASS) {
    Serial.println("Failed to create stream task");
    while (1) delay(1000);
  }

  Serial.print("Sample rate: "); Serial.println(SAMPLE_RATE_HZ);
  Serial.print("Samples per cycle (per channel): "); Serial.println(SAMPLES_PER_CYCLE);
  Serial.println("Streaming... measure on GPIO25 (DAC1)");
}

void loop() {
  // Small serial UI to change amplitude/stop/restart
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') {
      buildI2SBuffer(0.5f, 0.0f);
      Serial.println("Amplitude 50%");
    } else if (c == 'A') {
      buildI2SBuffer(1.0f, 0.0f);
      Serial.println("Amplitude 100%");
    } else if (c == 's') {
      stopI2SOutput();
      Serial.println("Stopped I2S output.");
    } else if (c == 'r') {
      // reconfigure and restart
      if (configureI2SForDAC()) {
        BaseType_t t = xTaskCreatePinnedToCore(i2s_stream_task, "i2s_stream", 4096, NULL, 5, &streamTask, 1);
        Serial.println(t == pdPASS ? "Restarted streaming." : "Failed to restart streaming.");
      }
    }
  }
  delay(10);
}
