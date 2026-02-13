// Pure-integer Bresenham-style phase stepping for ESP32 I2S->DAC
// - No per-sample float.
// - Frequency stored as integer micro-Hz (SCALE = 1'000'000).
// - Bresenham: step_numer/denom with remainder distributed by 'err' accumulator.
// - Integer linear interpolation using frac16 = (err * 65536) / denom.
// Measure on GPIO25 (DAC1). Serial 115200.

#include <Arduino.h>
#include <WiFi.h>
#include "driver/i2s.h"

#if defined(I2S_DAC_CHANNEL_BOTH_EN)
  #define HAVE_I2S_DAC_BOTH 1
#endif

// ---------------- CONFIG ----------------
const int BASE_FREQ_HZ       = 50;         // baseline output frequency
const int WAVETABLE_SIZE     = 200;        // table resolution for one cycle
const int I2S_SAMPLE_RATE    = 48000;      // fixed hardware sample rate
const int CHUNK_FRAMES       = 256;        // frames per i2s_write() (adjustable)
const int DAC_CHANNEL        = 1;          // 1 -> GPIO25 (DAC1)
const int CPU_FREQ_MHZ       = 240;
// ratio scale to represent frequency as integer micro-hertz (precision)
const uint32_t SCALE = 1000000u;           // 1e6 (microHz)
// -----------------------------------------

enum WaveformMode { WM_SINE = 0, WM_TRAPEZOID = 1 };

static int16_t wavetable[WAVETABLE_SIZE];                 // int16 table -32767..32767
static volatile float currentFreq = BASE_FREQ_HZ;         // Hz, updated by test runner
static volatile WaveformMode waveformMode = WM_SINE;

TaskHandle_t streamTask = NULL;

// build int16 sine table [-32767..+32767]
void buildWaveTable() {
  for (int i = 0; i < WAVETABLE_SIZE; ++i) {
    float f = sinf((2.0f * PI * (float)i) / (float)WAVETABLE_SIZE);
    int32_t s = (int32_t)roundf(f * 32767.0f);
    if (s > 32767) s = 32767;
    if (s < -32767) s = -32767;
    wavetable[i] = (int16_t)s;
  }
}

// integer trapezoid using integer phase components (cheap fallback)
static inline int16_t trapezoidFromPhaseIdxAndErr(uint32_t idx, uint64_t err, uint64_t denom) {
  // Compute fractional p = (idx + err/denom) / WAVETABLE_SIZE
  // We will compute p as double only inside this function since trapezoid is rare.
  double frac = ((double)idx + (double)err / (double)denom) / (double)WAVETABLE_SIZE; // 0..1
  double p = fmod(frac, 1.0);
  float out;
  if (p < 0.20f) {
    float t = p / 0.20f;
    out = -1.0f + 2.0f * t;
  } else if (p < 0.50f) {
    out = 1.0f;
  } else if (p < 0.70f) {
    float t = (p - 0.50f) / 0.20f;
    out = 1.0f - 2.0f * t;
  } else {
    out = -1.0f;
  }
  int32_t s = (int32_t)roundf(out * 32767.0f);
  return (int16_t)s;
}

// convert int16 waveform (-32767..32767) to 8-bit DAC word in high byte of 16-bit word
inline uint16_t waveToI2SWord_int16(int32_t v_signed16) {
  // map -32767..+32767 -> 0..255
  uint32_t u = (uint32_t)(v_signed16 + 32768); // 0..65535
  uint8_t dac8 = (uint8_t)(u >> 8); // top 8 bits
  return (uint16_t)(dac8 << 8);
}

// configure I2S once at fixed sample rate (kept same as original)
bool configureI2S_fixedRate() {
  i2s_driver_uninstall(I2S_NUM_0);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
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

// streaming task: pure integer Bresenham stepping + integer interpolation
void i2s_stream_task(void *arg) {
  Serial.println("Stream task started (pure-integer Bresenham)");
  const size_t wordsPerFrame = 2; // L + R
  const size_t chunkWords = (size_t)CHUNK_FRAMES * wordsPerFrame;
  uint16_t *buf = (uint16_t*)malloc(chunkWords * sizeof(uint16_t));
  if (!buf) {
    Serial.println("Stream task: failed to allocate chunk buffer");
    vTaskDelete(NULL);
    return;
  }

  // Bresenham state:
  uint32_t phase_idx = 0;   // integer wavetable index [0 .. WAVETABLE_SIZE-1]
  uint64_t err = 0;         // remainder accumulator, < denom

  // denominator (constant): I2S_SAMPLE_RATE * SCALE
  const uint64_t denom = (uint64_t)I2S_SAMPLE_RATE * (uint64_t)SCALE;

  // per-chunk step parameters:
  uint64_t step_numer = (uint64_t)WAVETABLE_SIZE * (uint64_t)((uint64_t)BASE_FREQ_HZ * (uint64_t)SCALE); // initialize
  uint64_t q = 0;
  uint64_t rem = 0;

  while (1) {
    // snapshot freq and mode
    float freq_f = currentFreq;
    WaveformMode mode = waveformMode;

    // compute integer numerator step using micro-Hz scale (one division / chunk only)
    // step_numer = WAVETABLE_SIZE * freq_microHz
    uint64_t freq_micro = (uint64_t) ( (double)freq_f * (double)SCALE + 0.5 );
    step_numer = (uint64_t)WAVETABLE_SIZE * freq_micro;

    // compute quotient and remainder for Bresenham
    q = step_numer / denom;    // integer table index increment per sample
    rem = step_numer % denom;  // remainder to accumulate (err)

    // Safety: keep rem < denom, q can be 0 for low frequencies
    // Fill chunk
    for (int n = 0; n < CHUNK_FRAMES; ++n) {
      int32_t sample_int16;

      if (mode == WM_SINE) {
        // Bresenham index stepping
        phase_idx += (uint32_t)q;

        err += rem;
        if (err >= denom) {
          phase_idx++;
          err -= denom;
        }

        // wrap phase_idx
        if (phase_idx >= (uint32_t)WAVETABLE_SIZE) phase_idx %= (uint32_t)WAVETABLE_SIZE;

        // linear interpolation fraction derived from err:
        // compute frac16 = (err * 65536) / denom  (0..65535)
        uint32_t frac16 = (uint32_t)((__uint128_t)err * 65536u / denom); // use 128 for safety if available
        // get indices
        uint32_t i0 = phase_idx;
        uint32_t i1 = (i0 + 1 >= (uint32_t)WAVETABLE_SIZE) ? 0 : (i0 + 1);
        int32_t a = (int32_t)wavetable[i0];
        int32_t b = (int32_t)wavetable[i1];
        // interp = (a * (65536 - frac) + b * frac) >> 16
        int32_t interp = (int32_t)((((int64_t)a * (65536u - frac16)) + ((int64_t)b * frac16)) >> 16);
        sample_int16 = interp;
      } else {
        // trapezoid: we need a fractional p to place the trapezoid correctly
        // use current phase_idx and err to compute trapezoid function (function internally uses double)
        sample_int16 = (int32_t)trapezoidFromPhaseIdxAndErr(phase_idx, err, denom);

        // advance phase by q/rem same as sine (stay consistent)
        phase_idx += (uint32_t)q;
        err += rem;
        if (err >= denom) {
          phase_idx++;
          err -= denom;
        }
        if (phase_idx >= (uint32_t)WAVETABLE_SIZE) phase_idx %= (uint32_t)WAVETABLE_SIZE;
      }

      // convert to I2S word
      uint16_t word = waveToI2SWord_int16(sample_int16);
      buf[2*n + 0] = word; // left
      buf[2*n + 1] = word; // right
    }

    // write to I2S (blocks until DMA accepts)
    size_t bytesToWrite = chunkWords * sizeof(uint16_t);
    size_t bytesWritten = 0;
    esp_err_t err_write = i2s_write(I2S_NUM_0, (const char*)buf, bytesToWrite, &bytesWritten, portMAX_DELAY);
    if (err_write != ESP_OK) {
      Serial.print("i2s_write error: ");
      Serial.println((int)err_write);
      vTaskDelay(pdMS_TO_TICKS(50));
    } else if (bytesWritten != bytesToWrite) {
      Serial.print("Partial write: ");
      Serial.print(bytesWritten);
      Serial.print(" / ");
      Serial.println(bytesToWrite);
    }
    // continue producing chunks
  }

  free(buf);
  vTaskDelete(NULL);
}

// Run one test pattern (same as original)
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
  Serial.println("\nESP32 I2S->DAC pure-integer Bresenham");

  Serial.println("Sanity: dacWrite midscale for 200 ms on GPIO25");
  dacWrite(25, 127);
  delay(200);

  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(10);

  buildWaveTable();

  if (!configureI2S_fixedRate()) {
    Serial.println("I2S configuration failed. Abort.");
    while (1) delay(1000);
  }

  BaseType_t t = xTaskCreatePinnedToCore(i2s_stream_task, "i2s_stream", 8192, NULL, 5, &streamTask, 1);
  if (t != pdPASS) {
    Serial.println("Failed to create stream task. Abort.");
    while (1) delay(1000);
  }

  Serial.print("I2S sample rate: "); Serial.println(I2S_SAMPLE_RATE);
  Serial.print("Wavetable size: "); Serial.println(WAVETABLE_SIZE);
  Serial.print("Chunk frames: "); Serial.println(CHUNK_FRAMES);
  Serial.println("Streaming baseline. Test pattern will run automatically in 3 s.");
  vTaskDelay(pdMS_TO_TICKS(3000));

  runTestPatternOnce();

  Serial.println("Ready. Send 't' over Serial to run the test pattern again.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 't' || c == 'T') {
      runTestPatternOnce();
    } else if (c == 's') {
      Serial.println("Stop requested (not fully uninstalling driver). Reset to fully stop.");
    } else if (c == 'r') {
      Serial.println("Resume requested (no-op if streaming task still running).");
    }
  }
  // keep the test repeating occasionally as you had
  vTaskDelay(pdMS_TO_TICKS(3000));
  runTestPatternOnce();
  vTaskDelay(pdMS_TO_TICKS(50));
}
