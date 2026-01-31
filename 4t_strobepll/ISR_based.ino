#include <Arduino.h>

// Config
#define PIN_ADC 36
#define BUF_SZ 384          // 128 samples * 3 cycles
#define NOMINAL_FREQ 50.0f
#define SAMPLE_RATIO 128    // Samples per cycle
#define STROBE_DIV 4        // Strobe = Grid / 4

// State Definitions
enum State { STATE_IDLE, STATE_CAPTURING, STATE_PROCESSING };

struct SysState {
  // Data
  uint16_t buf[BUF_SZ];
  uint64_t timestamps[BUF_SZ];
  volatile uint16_t idx;
  
  // Timing / Control
  hw_timer_t *timer;
  volatile bool strobe_fire;
  uint64_t sample_interval_us;
  uint64_t last_sample_us;
  
  // State
  State state;
} sys;

// ISR: Fires at beat frequency (Grid/4)
void IRAM_ATTR onStrobe() {
  sys.strobe_fire = true;
}

// Adjusts sampling and strobe rates based on estimated frequency
void setFrequencyBase(float grid_freq_hz) {
  // 1. Adjust Strobe (The "PLL" Lock)
  float strobe_hz = grid_freq_hz / STROBE_DIV;
  uint64_t alarm_val = (uint64_t)(1000000.0f / strobe_hz);
  timerAlarmWrite(sys.timer, alarm_val, true);

  // 2. Adjust Sampling Rate (The "Elastic" Buffer)
  // We want exactly SAMPLE_RATIO samples per grid cycle
  float sample_hz = grid_freq_hz * SAMPLE_RATIO;
  sys.sample_interval_us = (uint64_t)(1000000.0f / sample_hz);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  memset(&sys, 0, sizeof(sys));

  // Timer Setup (1us ticks)
  sys.timer = timerBegin(0, 80, true); 
  timerAttachInterrupt(sys.timer, &onStrobe, true);
  
  // Initialize with nominal
  setFrequencyBase(NOMINAL_FREQ);
  timerAlarmEnable(sys.timer);
}

void loop() {
  uint64_t now = esp_timer_get_time();

  // 1. State: Waiting for Strobe (Dead time)
  if (sys.state == STATE_IDLE) {
    if (sys.strobe_fire) {
      sys.strobe_fire = false;
      sys.idx = 0;
      sys.state = STATE_CAPTURING;
      // Align first sample immediate to strobe for phase consistency
      sys.last_sample_us = now - sys.sample_interval_us; 
    }
  }

  // 2. State: Capturing (High Priority)
  else if (sys.state == STATE_CAPTURING) {
    if ((now - sys.last_sample_us) >= sys.sample_interval_us) {
      sys.last_sample_us = now;
      sys.buf[sys.idx] = analogRead(PIN_ADC);
      sys.timestamps[sys.idx] = now;
      sys.idx++;

      if (sys.idx >= BUF_SZ) sys.state = STATE_PROCESSING;
    }
  }

  // 3. State: Processing (Computation Window)
  else if (sys.state == STATE_PROCESSING) {
    uint64_t t_start = esp_timer_get_time();

    // --- INSERT YOUR ALGORITHM HERE ---
    // Access sys.buf[] (raw) and sys.timestamps[]
    // float v = (sys.buf[i] - dc_offset) * scale;
    
    // Example: Simple Feedback Placeholder
    float estimated_freq = NOMINAL_FREQ; // Replace with result of algo
    
    // --- CLOSE LOOP ---
    setFrequencyBase(estimated_freq);

    // Diagnostics
    Serial.printf("Proc: %llu us | F_base: %.2f\n", 
      esp_timer_get_time() - t_start, estimated_freq);

    sys.state = STATE_IDLE; // Wait for next strobe
  }
}

