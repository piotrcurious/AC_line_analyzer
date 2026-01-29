// ESP32 50 Hz sine test signal (DAC) — Arduino (esp_timer version)
// Should compile on ESP32 Arduino core 3.3.x (and later).
// Outputs a 50 Hz sine on DAC1 (GPIO25). To use DAC2, set DAC_PIN = 26.

#include <Arduino.h>
#include "esp_timer.h"   // use esp_timer for portable periodic callbacks

const int DAC_PIN = 25;           // GPIO25 = DAC1, or 26 for DAC2
const int SAMPLES_PER_CYCLE = 200; // 200 samples -> 10 kHz sample rate
const int SIGNAL_FREQ_HZ = 50;
const uint32_t SAMPLE_RATE = SIGNAL_FREQ_HZ * SAMPLES_PER_CYCLE; // 10000 Hz
const uint64_t TIMER_PERIOD_US = 1000000ULL / SAMPLE_RATE; // 100 us per sample

volatile uint16_t sampleIndex = 0;
uint8_t sineTable[SAMPLES_PER_CYCLE];

esp_timer_handle_t periodic_timer = nullptr;

void buildSineTable(float amplitude = 1.0f, float dcOffset = 0.0f) {
  // amplitude: 0..1, dcOffset: -1..1
  for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
    float angle = (2.0f * PI * i) / (float)SAMPLES_PER_CYCLE;
    float s = sinf(angle); // -1..1
    // scale to 0..1 with amplitude and dc offset
    float scaled = (s * amplitude * 0.5f) + (0.5f + dcOffset * 0.5f);
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 1.0f) scaled = 1.0f;
    sineTable[i] = (uint8_t) roundf(scaled * 255.0f);
  }
}

static void IRAM_ATTR onTimer(void* arg) {
  // This runs in esp_timer task context. Keep it short.
  uint8_t v = sineTable[sampleIndex];
  dacWrite(DAC_PIN, v);
  sampleIndex++;
  if (sampleIndex >= SAMPLES_PER_CYCLE) sampleIndex = 0;
}

void setup() {
  Serial.begin(115200);
  delay(5);
  Serial.println();
  Serial.println("ESP32 50 Hz sine via DAC (GPIO25) using esp_timer");

  buildSineTable(1.0f, 0.0f); // full scale centered sine

  pinMode(DAC_PIN, OUTPUT);
  dacWrite(DAC_PIN, sineTable[0]);

  // Create and start periodic esp_timer
  esp_timer_create_args_t periodic_timer_args = {
    .callback = &onTimer,
    .arg = nullptr,
    .name = "sine_timer"
  };

  esp_err_t err = esp_timer_create(&periodic_timer_args, &periodic_timer);
  if (err != ESP_OK) {
    Serial.print("esp_timer_create error: ");
    Serial.println(err);
    while (1) delay(1000);
  }

  err = esp_timer_start_periodic(periodic_timer, TIMER_PERIOD_US);
  if (err != ESP_OK) {
    Serial.print("esp_timer_start_periodic error: ");
    Serial.println(err);
    while (1) delay(1000);
  }

  Serial.print("SAMPLES_PER_CYCLE: "); Serial.println(SAMPLES_PER_CYCLE);
  Serial.print("Sample rate (Hz): "); Serial.println(SAMPLE_RATE);
  Serial.print("Timer period (us): "); Serial.println(TIMER_PERIOD_US);
  Serial.println("Outputting... measure on GPIO25 (DAC1).");
}

void loop() {
  // Simple serial controls: a/A change amplitude, s/r stop/resume
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'a') {
      buildSineTable(0.5f, 0.0f);
      Serial.println("Amplitude set to 50%");
    } else if (c == 'A') {
      buildSineTable(1.0f, 0.0f);
      Serial.println("Amplitude set to 100%");
    } else if (c == 's') {
      esp_timer_stop(periodic_timer);
      dacWrite(DAC_PIN, 127); // midscale
      Serial.println("Stopped output.");
    } else if (c == 'r') {
      esp_timer_start_periodic(periodic_timer, TIMER_PERIOD_US);
      Serial.println("Resumed output.");
    } else if (c == 'd') {
      buildSineTable(1.0f, -0.2f);
      Serial.println("DC offset -0.2 applied");
    } else if (c == 'D') {
      buildSineTable(1.0f, 0.2f);
      Serial.println("DC offset +0.2 applied");
    }
  }
  delay(50);
}
