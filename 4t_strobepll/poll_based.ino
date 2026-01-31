#include <Arduino.h>

// --- Configuration ---
#define ADC_PIN             36
#define GRID_NOMINAL_FREQ   50.0f
#define SAMPLES_PER_CYCLE   128     // High oversampling
#define CYCLES_PER_FRAME    3       // Number of cycles to capture per strobe
#define STROBE_DIVISOR      4.0f    // Strobe Freq = Grid Freq / 4

// --- Auto-Calculated Constants ---
#define BUFFER_SIZE         (SAMPLES_PER_CYCLE * CYCLES_PER_FRAME)

// --- Data Structures ---
struct SamplePoint {
  uint16_t raw;       // ADC Reading
  int16_t jitter_us;  // Difference between ideal schedule and actual read time
};

class HeterodynePLL {
  public:
    SamplePoint buffer[BUFFER_SIZE];
    
    // Timing State
    uint64_t next_sample_us;
    uint64_t next_strobe_us;
    float current_sample_interval_us;
    float current_strobe_interval_us;
    
    // Process State
    uint16_t write_idx;
    bool capturing;

    void begin() {
      tune(GRID_NOMINAL_FREQ);
      next_strobe_us = esp_timer_get_time() + 10000; // Start in 10ms
      capturing = false;
      write_idx = 0;
    }

    // Call this as fast as possible in loop()
    // Returns TRUE when a buffer is full and ready for math
    bool poll() {
      uint64_t now = esp_timer_get_time();

      // 1. Strobe Logic (Trigger for new frame)
      if (!capturing) {
        if (now >= next_strobe_us) {
          capturing = true;
          write_idx = 0;
          
          // Align first sample exactly with the strobe event
          next_sample_us = next_strobe_us; 
          
          // Schedule next strobe (maintaining phase lock)
          next_strobe_us += (uint64_t)current_strobe_interval_us;
          
          // Safety: If we blew the deadline by a lot, reset phase to avoid burst
          if (now > next_sample_us + 5000) next_sample_us = now; 
        }
      }

      // 2. Sampling Logic
      if (capturing) {
        if (now >= next_sample_us) {
          // Critical: Read immediately
          buffer[write_idx].raw = analogRead(ADC_PIN);
          
          // record how late we were (observation noise for EKF)
          buffer[write_idx].jitter_us = (int16_t)(now - next_sample_us);

          // Schedule next sample based on IDEAL time (cancels drift)
          next_sample_us += (uint64_t)current_sample_interval_us;
          write_idx++;

          if (write_idx >= BUFFER_SIZE) {
            capturing = false;
            return true; // Frame Ready
          }
        }
      }
      return false;
    }

    // Adjust sampling rates based on EKF estimated grid frequency
    void tune(float grid_freq_hz) {
      // 1. Sample Rate: Elastic to match grid harmonics
      float sample_hz = grid_freq_hz * SAMPLES_PER_CYCLE;
      current_sample_interval_us = 1000000.0f / sample_hz;

      // 2. Strobe Rate: Controls the beat frequency
      float strobe_hz = grid_freq_hz / STROBE_DIVISOR;
      current_strobe_interval_us = 1000000.0f / strobe_hz;
    }
    
    // Diagnostics
    float getSamplingRate() { return 1000000.0f / current_sample_interval_us; }
};

HeterodynePLL pll;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pll.begin();
}

void loop() {
  // 1. High Speed Polling
  if (pll.poll()) {
    
    // 2. Dead Cycle Processing (The buffer is full, strobe is waiting)
    uint64_t t_start = esp_timer_get_time();
    
    // --- YOUR EKF / SOGI LOGIC HERE ---
    // Iterate over pll.buffer[]
    // buffer[i].jitter_us contains precise timing error
    
    // Simulation: Just calculate a fake frequency for testing
    float estimated_grid_freq = GRID_NOMINAL_FREQ + (sin(millis()/1000.0)*0.1); 

    // 3. Close the Loop
    pll.tune(estimated_grid_freq);

    // Diagnostics
    Serial.printf("Grid: %.3f Hz | SR: %.1f Hz | Jitter[0]: %d us | Proc: %llu us\n", 
      estimated_grid_freq, 
      pll.getSamplingRate(), 
      pll.buffer[0].jitter_us,
      esp_timer_get_time() - t_start
    );
  }
}

