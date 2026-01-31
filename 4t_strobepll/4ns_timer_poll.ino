#include <Arduino.h>
#include <hal/cpu_hal.h>

// --- Tuning ---
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SAMPLES_PER_CYCLE 128
#define CYCLES_TO_CAPTURE 3
#define STROBE_DIV 4.0f

// --- Derived Constants ---
#define BUF_SZ (SAMPLES_PER_CYCLE * CYCLES_TO_CAPTURE)

// --- Shared System State ---
struct Sample {
  uint16_t val;
  int32_t jitter_cycles; // Deviation from ideal schedule
};

struct SystemState {
  // Config / Status
  uint32_t cpu_freq;
  bool frame_ready;
  
  // Data Buffer
  Sample buf[BUF_SZ];
  
  // High-Res Timing Info for EKF
  int32_t strobe_offset_cycles; // How late the Frame started vs Phase Target
  
  // Internal PLL State
  bool capturing;
  uint16_t idx;
  uint32_t next_strobe_tick;
  uint32_t next_sample_tick;
  uint32_t cycles_per_sample;
  uint32_t cycles_per_strobe;
} sys;

// --- Core Logic ---

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  
  memset(&sys, 0, sizeof(sys));
  sys.cpu_freq = ESP.getCpuFreqMHz() * 1000000;
  
  // Initial Tune
  float base_freq = NOMINAL_FREQ;
  sys.cycles_per_sample = sys.cpu_freq / (base_freq * SAMPLES_PER_CYCLE);
  sys.cycles_per_strobe = sys.cpu_freq / (base_freq / STROBE_DIV);
  
  sys.next_strobe_tick = cpu_hal_get_cycle_count() + sys.cpu_freq; // Start in 1s
}

// 1. Precise Polling Engine
// Returns true if a frame was completed
inline bool poll_pll() {
  uint32_t now = cpu_hal_get_cycle_count();

  // A. Strobe Trigger (Start Frame)
  if (!sys.capturing) {
    // Unsigned arithmetic handles overflow correctly
    if ((int32_t)(now - sys.next_strobe_tick) >= 0) {
      sys.capturing = true;
      sys.idx = 0;
      
      // Record global phase error (Strobe Jitter)
      sys.strobe_offset_cycles = (int32_t)(now - sys.next_strobe_tick);
      
      // Align sampling grid to ideal strobe target + offset (or just now)
      // Here we align to 'now' to minimize sample jitter, but we track the 
      // start offset via strobe_offset_cycles for the math layer.
      sys.next_sample_tick = now; 
      
      // Schedule next strobe
      sys.next_strobe_tick += sys.cycles_per_strobe;
    }
  }

  // B. Sampling
  if (sys.capturing) {
    if ((int32_t)(now - sys.next_sample_tick) >= 0) {
      // 1. Read (Critical)
      sys.buf[sys.idx].val = analogRead(ADC_PIN);
      
      // 2. Record Timing Error (jitter)
      // Positive = Late, Negative = Early (unlikely in polling)
      sys.buf[sys.idx].jitter_cycles = (int32_t)(cpu_hal_get_cycle_count() - sys.next_sample_tick);
      
      // 3. Schedule next (Drift cancelling)
      sys.next_sample_tick += sys.cycles_per_sample;
      sys.idx++;

      if (sys.idx >= BUF_SZ) {
        sys.capturing = false;
        sys.frame_ready = true;
        return true;
      }
    }
  }
  return false;
}

// 2. Frequency Actuator
void tune_pll(float grid_freq) {
  // Update intervals for next cycle
  // We compute float once, then cast to int for the hot loop
  sys.cycles_per_sample = sys.cpu_freq / (grid_freq * SAMPLES_PER_CYCLE);
  sys.cycles_per_strobe = sys.cpu_freq / (grid_freq / STROBE_DIV);
}

void loop() {
  // Run polling as fast as possible
  if (poll_pll()) {
    
    // --- Dead Cycle: Signal Processing ---
    // The buffer is full. The next strobe is ~60ms away.
    // We have plenty of time for float math.
    
    // Example: Convert cycles to seconds for EKF
    double dt_strobe_error_s = (double)sys.strobe_offset_cycles / sys.cpu_freq;
    double dt_first_sample_jitter_s = (double)sys.buf[0].jitter_cycles / sys.cpu_freq;

    // ... Run EKF / SOGI here ...
    // ... Calculate new grid frequency ...
    float estimated_freq = NOMINAL_FREQ; // Placeholder

    // Apply control
    tune_pll(estimated_freq);

    // Diagnostics
    Serial.printf("Grid:%.2fHz | StrobeJit:%d cyc (%.2fus) | SampleJit:%d cyc\n", 
      estimated_freq, 
      sys.strobe_offset_cycles,
      dt_strobe_error_s * 1e6,
      sys.buf[0].jitter_cycles
    );

    sys.frame_ready = false; // Reset flag
  }
}

