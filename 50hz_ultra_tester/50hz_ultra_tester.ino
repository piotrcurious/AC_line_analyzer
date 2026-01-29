// ESP32 I2S->DAC with software freq control + test pattern
// - Fixed I2S sample rate (48 kHz) to avoid MCLK division errors.
// - Phase-accumulator fractional stepping changes output frequency (no driver reinstall).
// - Test: +1Hz for 2s, -1Hz for 2s, baseline 1s, trapezoid 1s, restore baseline.
// Measure on GPIO25 (DAC1). Serial 115200.

#include <Arduino.h>
#include <WiFi.h>
#include "driver/i2s.h"

#if defined(I2S_DAC_CHANNEL_BOTH_EN)
  #define HAVE_I2S_DAC_BOTH 1
#endif

// ---------------- CONFIG ----------------
const int BASE_FREQ_HZ       = 50;         // baseline output frequency
const int WAVETABLE_SIZE    = 200;         // table resolution for one cycle
const int I2S_SAMPLE_RATE   = 48000;       // fixed, safe hardware sample rate
const int CHUNK_FRAMES      = 256;         // frames per i2s_write() (adjustable)
const int DAC_CHANNEL       = 1;           // 1 -> GPIO25 (DAC1)
const int CPU_FREQ_MHZ      = 240;
// -----------------------------------------

enum WaveformMode { WM_SINE = 0, WM_TRAPEZOID = 1 };

static float wavetable[WAVETABLE_SIZE];
static volatile float currentFreq = BASE_FREQ_HZ; // Hz, updated by test runner
static volatile WaveformMode waveformMode = WM_SINE;

TaskHandle_t streamTask = NULL;

// build sine table (normalized -1..1)
void buildWaveTable() {
  for (int i = 0; i < WAVETABLE_SIZE; ++i) {
    wavetable[i] = sinf((2.0f * PI * (float)i) / (float)WAVETABLE_SIZE);
  }
}

// trapezoid generator (return value -1..1) based on table index (fractional)
float trapezoidAtPhase(float phaseIndex) {
  // phaseIndex in [0, WAVETABLE_SIZE)
  float p = fmodf(phaseIndex, (float)WAVETABLE_SIZE) / (float)WAVETABLE_SIZE; // 0..1
  if (p < 0.20f) { // rise 0..0.2
    float t = p / 0.20f;
    return -1.0f + 2.0f * t; // -1 -> +1
  } else if (p < 0.50f) { // top 0.2..0.5
    return 1.0f;
  } else if (p < 0.70f) { // fall 0.5..0.7
    float t = (p - 0.50f) / 0.20f;
    return 1.0f - 2.0f * t; // +1 -> -1
  } else { // bottom plateau 0.7..1
    return -1.0f;
  }
}

// convert waveform value (-1..1) to 8-bit DAC word stored in high byte of 16-bit word
inline uint16_t waveToI2SWord(float v) {
  // map -1..1 -> 0..255
  float scaled = (v * 0.5f) + 0.5f;
  if (scaled < 0.0f) scaled = 0.0f;
  if (scaled > 1.0f) scaled = 1.0f;
  uint16_t dac8 = (uint16_t)roundf(scaled * 255.0f) & 0xFF;
  return (uint16_t)(dac8 << 8);
}

// configure I2S once at fixed sample rate
bool configureI2S_fixedRate() {
  // uninstall first
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // interleaved stereo
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 512 / 2,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.print("i2s_driver_install failed: ");
    Serial.println((int)err);
    return false;
  }

  i2s_pin_config_t pin_config;
  memset(&pin_config, 0, sizeof(pin_config));
  i2s_set_pin(I2S_NUM_0, &pin_config);

  #ifdef HAVE_I2S_DAC_BOTH
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
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

  i2s_zero_dma_buffer(I2S_NUM_0);
  return true;
}

// streaming task: build small chunk from currentFreq + waveformMode and write to DMA
void i2s_stream_task(void *arg) {
  Serial.println("Stream task started");
  const size_t wordsPerFrame = 2; // L + R
  const size_t chunkWords = (size_t)CHUNK_FRAMES * wordsPerFrame;
  uint16_t *buf = (uint16_t*)malloc(chunkWords * sizeof(uint16_t));
  if (!buf) {
    Serial.println("Stream task: failed to allocate chunk buffer");
    vTaskDelete(NULL);
    return;
  }

  double phase = 0.0; // fractional index into waveform table (0..WAVETABLE_SIZE)
  while (1) {
    // capture freq atomically
    float freq = currentFreq;
    // compute phase step = table_size * freq / i2s_rate
    double step = ((double)WAVETABLE_SIZE * (double)freq) / (double)I2S_SAMPLE_RATE;

    // fill chunk frames
    for (int n = 0; n < CHUNK_FRAMES; ++n) {
      float v;
      if (waveformMode == WM_SINE) {
        // fractional index interpolation for smoothness
        double idx = fmod(phase, (double)WAVETABLE_SIZE);
        int i0 = (int)floor(idx);
        int i1 = (i0 + 1) % WAVETABLE_SIZE;
        double frac = idx - (double)i0;
        v = (float)((1.0 - frac) * wavetable[i0] + frac * wavetable[i1]);
      } else {
        // trapezoid directly using phase
        double idx = fmod(phase, (double)WAVETABLE_SIZE);
        v = trapezoidAtPhase((float)idx);
      }

      uint16_t word = waveToI2SWord(v);
      // left then right
      buf[2*n + 0] = word;
      buf[2*n + 1] = word;

      phase += step;
      // wrap phase safely
      if (phase >= (double)WAVETABLE_SIZE) phase -= (double)WAVETABLE_SIZE * floor(phase / (double)WAVETABLE_SIZE);
      else if (phase < 0) phase += (double)WAVETABLE_SIZE;
    }

    // write to I2S (blocks until DMA accepts)
    size_t bytesToWrite = chunkWords * sizeof(uint16_t);
    size_t bytesWritten = 0;
    esp_err_t err = i2s_write(I2S_NUM_0, (const char*)buf, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (err != ESP_OK) {
      Serial.print("i2s_write error: ");
      Serial.println((int)err);
      vTaskDelay(pdMS_TO_TICKS(50));
    } else if (bytesWritten != bytesToWrite) {
      Serial.print("Partial write: ");
      Serial.print(bytesWritten);
      Serial.print(" / ");
      Serial.println(bytesToWrite);
    }
    // loop and produce another chunk
  }
  // never here
}

// Run one test pattern (non-blocking wrt scheduler, but runs with delays)
void runTestPatternOnce() {
  Serial.println("=== Test pattern: start ===");
  // +1 Hz for 2s
  Serial.println("Drift +1 Hz for 2 s");
  waveformMode = WM_SINE;
  currentFreq = (float)BASE_FREQ_HZ + 1.0f;
  vTaskDelay(pdMS_TO_TICKS(2000));

  // -1 Hz for 2s
  Serial.println("Drift -1 Hz for 2 s");
  currentFreq = (float)BASE_FREQ_HZ - 1.0f;
  vTaskDelay(pdMS_TO_TICKS(2000));

  // baseline 1s
  Serial.println("Baseline for 1 s");
  currentFreq = (float)BASE_FREQ_HZ;
  vTaskDelay(pdMS_TO_TICKS(1000));

  // trapezoid 1s
  Serial.println("Trapezoid distortion for 1 s");
  waveformMode = WM_TRAPEZOID;
  currentFreq = (float)BASE_FREQ_HZ;
  vTaskDelay(pdMS_TO_TICKS(1000));

  // restore
  Serial.println("Restoring baseline sine");
  waveformMode = WM_SINE;
  currentFreq = (float)BASE_FREQ_HZ;
  Serial.println("=== Test pattern: done ===");
}

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\nESP32 I2S->DAC with software freq control + test pattern");

  // sanity
  Serial.println("Sanity: dacWrite midscale for 200 ms on GPIO25");
  dacWrite(25, 127);
  delay(200);

  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  // try reduce interference (Arduino API)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(10);

  // build wavetable
  buildWaveTable();

  // configure I2S once at fixed safe rate
  if (!configureI2S_fixedRate()) {
    Serial.println("I2S configuration failed. Abort.");
    while (1) delay(1000);
  }

  // create streaming task
  BaseType_t t = xTaskCreatePinnedToCore(i2s_stream_task, "i2s_stream", 4096, NULL, 5, &streamTask, 1);
  if (t != pdPASS) {
    Serial.println("Failed to create stream task. Abort.");
    while (1) delay(1000);
  }

  Serial.print("I2S sample rate: "); Serial.println(I2S_SAMPLE_RATE);
  Serial.print("Wavetable size: "); Serial.println(WAVETABLE_SIZE);
  Serial.print("Chunk frames: "); Serial.println(CHUNK_FRAMES);
  Serial.println("Streaming baseline. Test pattern will run automatically in 3 s.");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // run the pattern once
  runTestPatternOnce();

  Serial.println("Ready. Send 't' over Serial to run the test pattern again.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') {
      runTestPatternOnce();
    } else if (c == 's') {
      // stop streaming (not fully uninstalling here)
      Serial.println("Stop requested (not fully uninstalling driver). Reset to fully stop.");
    } else if (c == 'r') {
      Serial.println("Resume requested (no-op if streaming task still running).");
    }
  }
vTaskDelay(pdMS_TO_TICKS(3000));runTestPatternOnce(); // crude to repeat test

  
  vTaskDelay(pdMS_TO_TICKS(50));
}
