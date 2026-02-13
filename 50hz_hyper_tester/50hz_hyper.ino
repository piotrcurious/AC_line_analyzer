// Improved ESP32 I2S->DAC with fixed-point phase accumulator,
// Bresenham-style fractional stepping, integer interpolation,
// and triangular dithering to reduce harmonic intermodulation.
//
// Keep I2S config and test pattern logic from original sketch.
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
// Fixed point fractional bits for phase accumulator. 24 is a good tradeoff.
const int PHASE_FP = 24;                   // fractional bits
// Dither width in bits applied into low fractional bits (triangular dither)
const int DITHER_BITS = 6;                 // 0..6 recommended (smaller = less noise)
const uint32_t DITHER_MASK = ((1u << DITHER_BITS) - 1u);
// -----------------------------------------

enum WaveformMode { WM_SINE = 0, WM_TRAPEZOID = 1 };

static int16_t wavetable[WAVETABLE_SIZE];                 // int16 table -32767..32767
static volatile float currentFreq = BASE_FREQ_HZ;         // Hz, updated by test runner
static volatile WaveformMode waveformMode = WM_SINE;

TaskHandle_t streamTask = NULL;

// small xorshift PRNG for dither (32-bit state)
static uint32_t xorshift_state = 0xA5A5A5A5;
static inline uint32_t xorshift32() {
  uint32_t x = xorshift_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  xorshift_state = x;
  return x;
}

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

// integer trapezoid using fixed-point phase
// phase_fp is 64-bit fixed point with PHASE_FP fractional bits
static inline int16_t trapezoidFromPhaseFP(uint64_t phase_fp) {
  // Compute p in 0..1 using double (cheap here; trapezoid used only occasionally)
  double phase_total = (double)phase_fp / (double)((uint64_t)WAVETABLE_SIZE << PHASE_FP);
  double p = fmod(phase_total, 1.0);
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
  // add 32768 -> 1..65535, >>8 -> 0..255
  uint32_t u = (uint32_t)(v_signed16 + 32768); // 0..65535
  uint8_t dac8 = (uint8_t)(u >> 8); // top 8 bits
  return (uint16_t)(dac8 << 8);
}

// configure I2S once at fixed sample rate (same as original)
bool configureI2S_fixedRate() {
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

// streaming task: fixed-point phase acc, integer interpolation, triangular dither
void i2s_stream_task(void *arg) {
  Serial.println("Stream task started (fixed-point + dither)");
  const size_t wordsPerFrame = 2; // L + R
  const size_t chunkWords = (size_t)CHUNK_FRAMES * wordsPerFrame;
  uint16_t *buf = (uint16_t*)malloc(chunkWords * sizeof(uint16_t));
  if (!buf) {
    Serial.println("Stream task: failed to allocate chunk buffer");
    vTaskDelete(NULL);
    return;
  }

  // phase and step as fixed point (phase_fp has integer + PHASE_FP fractional)
  uint64_t phase_fp = 0; // 64-bit accumulator
  uint64_t step_fp = 0;  // fixed point step per sample (WAVETABLE_SIZE << PHASE_FP) * freq / sample_rate

  // masks for interpolation
  const uint64_t frac_mask = ((1ULL << PHASE_FP) - 1ULL);
  const int INTERP_FRAC_BITS = 16;  // use 16-bit interpolation fraction
  const int SHIFT_PHASE_TO_INTERP = PHASE_FP - INTERP_FRAC_BITS; // PHASE_FP >= INTERP_FRAC_BITS

  while (1) {
    // snapshot current frequency and waveform mode
    float freq = currentFreq;
    WaveformMode mode = waveformMode;

    // compute step_fp with double -> integer (done per chunk, cheap)
    // step_fp represents (WAVETABLE_SIZE << PHASE_FP) * freq / I2S_SAMPLE_RATE
    double step_d = ((double)WAVETABLE_SIZE * (1ULL << PHASE_FP) * (double)freq) / (double)I2S_SAMPLE_RATE;
    step_fp = (uint64_t) llround(step_d);

    // Precompute some values used per-sample
    for (int n = 0; n < CHUNK_FRAMES; ++n) {
      int32_t sample_val_int16;

      if (mode == WM_SINE) {
        // Apply triangular dithering to step's low bits to decorrelate fractional quantization
        // Triangular dithering: (rand & mask) - (rand2 & mask) yields small zero-mean integer
        uint32_t r1 = xorshift32();
        uint32_t r2 = xorshift32();
        int32_t tri = (int32_t)((r1 & DITHER_MASK) - (r2 & DITHER_MASK));
        // inject 'tri' into low fractional bits of step; keep average zero
        uint64_t step_with_dither = step_fp;
        // place tri in the low PHASE_FP bits (scale)
        if (DITHER_BITS > 0) {
          // shift tri to align with the lowest fractional bits
          int shift_into = 0; // put in lowest fractional bits
          // careful: cast to signed to add negative values correctly
          int64_t signed_step = (int64_t)step_fp + ((int64_t)tri << shift_into);
          if (signed_step < 0) signed_step = 0;
          step_with_dither = (uint64_t)signed_step;
        }

        // advance phase by integer step_with_dither (fixed point)
        phase_fp += step_with_dither;

        // index and interpolation fraction
        uint32_t idx = (uint32_t)((phase_fp >> PHASE_FP) % (uint64_t)WAVETABLE_SIZE);
        uint32_t next = idx + 1;
        if (next >= (uint32_t)WAVETABLE_SIZE) next = 0;

        uint32_t frac16 = (uint32_t)((phase_fp >> SHIFT_PHASE_TO_INTERP) & 0xFFFFu); // 0..65535

        // integer linear interpolation:
        // sample = (wavetable[idx] * (65536 - frac) + wavetable[next] * frac) >> 16
        int32_t a = (int32_t)wavetable[idx];
        int32_t b = (int32_t)wavetable[next];
        int32_t interp = (int32_t)(((int64_t)a * (65536u - frac16) + (int64_t)b * frac16) >> 16);
        sample_val_int16 = interp;
      } else {
        // trapezoid mode: compute directly using phase_fp
        sample_val_int16 = (int32_t)trapezoidFromPhaseFP(phase_fp);
        // advance phase without dithering to keep waveform deterministic (you can enable dither if you like)
        phase_fp += step_fp;
      }

      // wrap phase to avoid unlimited growth (keep phase_fp small)
      // We want phase_fp modulo WAVETABLE_SIZE<<PHASE_FP
      const uint64_t wrap_mod = ((uint64_t)WAVETABLE_SIZE << PHASE_FP);
      if (phase_fp >= wrap_mod) phase_fp -= wrap_mod * (phase_fp / wrap_mod);

      // map to I2S/DAC word
      uint16_t word = waveToI2SWord_int16(sample_val_int16);
      buf[2*n + 0] = word; // left
      buf[2*n + 1] = word; // right
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

    // next chunk
  }

  // never here, free if we ever exit
  free(buf);
  vTaskDelete(NULL);
}

// Run one test pattern (same as your original)
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
  Serial.println("\nESP32 I2S->DAC with fixed-point freq control + triangular dither");

  Serial.println("Sanity: dacWrite midscale for 200 ms on GPIO25");
  dacWrite(25, 127);
  delay(200);

  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(10);

  // build wavetable (int16)
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
