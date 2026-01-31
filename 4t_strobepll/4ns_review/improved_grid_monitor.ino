#include <Arduino.h>
#include <hal/cpu_hal.h>

// =============================================================================
// Configuration Constants
// =============================================================================
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SAMPLES_PER_CYCLE 128
#define CYCLES_TO_CAPTURE 3
#define STROBE_DIV 4.0f

// Validation constants
#define MIN_FREQ 45.0f
#define MAX_FREQ 55.0f
#define MAX_JITTER_CYCLES 10000  // Reasonable threshold for warning

// =============================================================================
// Derived Constants
// =============================================================================
#define BUF_SZ (SAMPLES_PER_CYCLE * CYCLES_TO_CAPTURE)

// =============================================================================
// Data Structures
// =============================================================================

/**
 * @brief Single ADC sample with timing metadata
 */
struct Sample {
  uint16_t val;              // ADC reading (0-4095 for 12-bit)
  int32_t jitter_cycles;     // Deviation from ideal schedule (signed)
};

/**
 * @brief Complete system state for PLL-based grid monitoring
 */
struct SystemState {
  // Configuration
  uint32_t cpu_freq;         // CPU frequency in Hz
  volatile bool frame_ready; // Flag for completed frame (volatile for safety)
  
  // Data Buffer
  Sample buf[BUF_SZ];
  
  // Timing Metadata
  int32_t strobe_offset_cycles;  // Frame start deviation from ideal phase
  
  // PLL State
  volatile bool capturing;       // Currently acquiring samples
  uint16_t idx;                  // Current buffer index
  uint32_t next_strobe_tick;     // Next frame start time
  uint32_t next_sample_tick;     // Next sample acquisition time
  uint32_t cycles_per_sample;    // Sampling period in CPU cycles
  uint32_t cycles_per_strobe;    // Frame period in CPU cycles
  
  // Statistics (optional enhancement)
  uint32_t frames_captured;      // Total frames processed
  int32_t max_jitter_seen;       // Worst jitter observed
} sys;

// =============================================================================
// Function Prototypes
// =============================================================================
void setup();
void loop();
inline bool poll_pll();
void tune_pll(float grid_freq);
void print_diagnostics();
bool validate_frequency(float freq);

// =============================================================================
// Setup and Initialization
// =============================================================================

void setup() {
  Serial.begin(115200);
  
  // Wait for serial connection (helpful for debugging)
  delay(1000);
  Serial.println("\n=== Grid Frequency Monitor Initializing ===");
  
  // Configure ADC
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);  // For 0-3.3V range
  
  // Initialize system state
  memset(&sys, 0, sizeof(sys));
  sys.cpu_freq = ESP.getCpuFreqMHz() * 1000000UL;
  
  // Validate CPU frequency
  if (sys.cpu_freq == 0) {
    Serial.println("ERROR: Failed to read CPU frequency!");
    while (1) delay(1000);  // Halt
  }
  
  Serial.printf("CPU Frequency: %u MHz\n", sys.cpu_freq / 1000000);
  
  // Calculate initial timing parameters
  float base_freq = NOMINAL_FREQ;
  sys.cycles_per_sample = sys.cpu_freq / (base_freq * SAMPLES_PER_CYCLE);
  sys.cycles_per_strobe = sys.cpu_freq / (base_freq / STROBE_DIV);
  
  // Validate timing parameters
  if (sys.cycles_per_sample == 0 || sys.cycles_per_strobe == 0) {
    Serial.println("ERROR: Invalid timing calculation!");
    while (1) delay(1000);  // Halt
  }
  
  Serial.printf("Samples per cycle: %u\n", SAMPLES_PER_CYCLE);
  Serial.printf("Cycles per sample: %u (%.2f us)\n", 
                sys.cycles_per_sample, 
                (float)sys.cycles_per_sample / sys.cpu_freq * 1e6);
  Serial.printf("Cycles per strobe: %u (%.2f ms)\n", 
                sys.cycles_per_strobe,
                (float)sys.cycles_per_strobe / sys.cpu_freq * 1e3);
  
  // Schedule first strobe 1 second from now
  sys.next_strobe_tick = cpu_hal_get_cycle_count() + sys.cpu_freq;
  
  Serial.println("Initialization complete. Starting capture...\n");
}

// =============================================================================
// Core PLL Polling Engine
// =============================================================================

/**
 * @brief High-precision polling loop for phase-locked sampling
 * @return true if a complete frame was captured, false otherwise
 * 
 * This function must be called repeatedly with minimal latency.
 * Uses cycle-accurate timing to maintain phase lock with grid frequency.
 */
inline bool poll_pll() {
  const uint32_t now = cpu_hal_get_cycle_count();

  // -------------------------------------------------------------------------
  // Phase A: Strobe Trigger (Frame Start)
  // -------------------------------------------------------------------------
  if (!sys.capturing) {
    // Check if it's time to start a new frame
    // Using signed arithmetic handles 32-bit overflow correctly
    if ((int32_t)(now - sys.next_strobe_tick) >= 0) {
      sys.capturing = true;
      sys.idx = 0;
      
      // Record phase error: how late we started relative to ideal timing
      sys.strobe_offset_cycles = (int32_t)(now - sys.next_strobe_tick);
      
      // Align first sample to actual start time (minimize first sample jitter)
      sys.next_sample_tick = now; 
      
      // Schedule next strobe (maintains long-term phase alignment)
      sys.next_strobe_tick += sys.cycles_per_strobe;
    }
    return false;  // Early exit when not capturing
  }

  // -------------------------------------------------------------------------
  // Phase B: Sample Acquisition
  // -------------------------------------------------------------------------
  if ((int32_t)(now - sys.next_sample_tick) >= 0) {
    // CRITICAL SECTION: Minimize code between timing check and ADC read
    
    // 1. Acquire ADC sample
    sys.buf[sys.idx].val = analogRead(ADC_PIN);
    
    // 2. Record timing error for this sample
    // Positive = late, Negative = early (rare in polling mode)
    const uint32_t actual_read_time = cpu_hal_get_cycle_count();
    sys.buf[sys.idx].jitter_cycles = (int32_t)(actual_read_time - sys.next_sample_tick);
    
    // Track maximum jitter for diagnostics
    if (abs(sys.buf[sys.idx].jitter_cycles) > abs(sys.max_jitter_seen)) {
      sys.max_jitter_seen = sys.buf[sys.idx].jitter_cycles;
    }
    
    // 3. Schedule next sample (drift-canceling)
    sys.next_sample_tick += sys.cycles_per_sample;
    sys.idx++;

    // Check if frame is complete
    if (sys.idx >= BUF_SZ) {
      sys.capturing = false;
      sys.frame_ready = true;
      sys.frames_captured++;
      return true;
    }
  }
  
  return false;
}

// =============================================================================
// PLL Tuning and Control
// =============================================================================

/**
 * @brief Update PLL parameters based on estimated grid frequency
 * @param grid_freq Estimated grid frequency in Hz
 * 
 * This function recalculates timing intervals for the next capture cycle.
 * Should only be called between frames (during dead time).
 */
void tune_pll(float grid_freq) {
  // Validate input
  if (!validate_frequency(grid_freq)) {
    Serial.printf("WARNING: Invalid frequency %.2f Hz, keeping current setting\n", grid_freq);
    return;
  }
  
  // Recalculate timing intervals
  // Float math is acceptable here (called during dead time between frames)
  const uint32_t new_cycles_per_sample = 
    (uint32_t)(sys.cpu_freq / (grid_freq * SAMPLES_PER_CYCLE));
  const uint32_t new_cycles_per_strobe = 
    (uint32_t)(sys.cpu_freq / (grid_freq / STROBE_DIV));
  
  // Sanity check results
  if (new_cycles_per_sample == 0 || new_cycles_per_strobe == 0) {
    Serial.println("WARNING: PLL tuning resulted in zero cycles, ignoring");
    return;
  }
  
  // Apply updates atomically (prevent race conditions)
  noInterrupts();
  sys.cycles_per_sample = new_cycles_per_sample;
  sys.cycles_per_strobe = new_cycles_per_strobe;
  interrupts();
}

/**
 * @brief Validate frequency is within acceptable range
 * @param freq Frequency to validate
 * @return true if valid, false otherwise
 */
bool validate_frequency(float freq) {
  return (freq >= MIN_FREQ && freq <= MAX_FREQ);
}

// =============================================================================
// Diagnostics and Reporting
// =============================================================================

/**
 * @brief Print diagnostic information about captured frame
 */
void print_diagnostics() {
  // Convert timing data to human-readable units
  const double strobe_error_us = (double)sys.strobe_offset_cycles / sys.cpu_freq * 1e6;
  const double first_sample_jitter_us = (double)sys.buf[0].jitter_cycles / sys.cpu_freq * 1e6;
  const double max_jitter_us = (double)sys.max_jitter_seen / sys.cpu_freq * 1e6;
  
  // Basic frame info
  Serial.printf("Frame #%u | Strobe offset: %d cycles (%.2f us) | First sample jitter: %d cycles (%.2f us)\n",
    sys.frames_captured,
    sys.strobe_offset_cycles,
    strobe_error_us,
    sys.buf[0].jitter_cycles,
    first_sample_jitter_us
  );
  
  // Sample statistics
  uint16_t min_val = 4095, max_val = 0;
  uint32_t sum = 0;
  
  for (uint16_t i = 0; i < BUF_SZ; i++) {
    const uint16_t val = sys.buf[i].val;
    if (val < min_val) min_val = val;
    if (val > max_val) max_val = val;
    sum += val;
  }
  
  const uint16_t avg_val = sum / BUF_SZ;
  Serial.printf("ADC stats: min=%u, max=%u, avg=%u, pk-pk=%u\n", 
    min_val, max_val, avg_val, max_val - min_val);
  
  // Jitter warning
  if (abs(sys.max_jitter_seen) > MAX_JITTER_CYCLES) {
    Serial.printf("⚠️  WARNING: High jitter detected: %d cycles (%.2f us)\n",
      sys.max_jitter_seen, max_jitter_us);
  }
  
  Serial.println();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
  // Run polling engine as fast as possible (critical for timing)
  if (poll_pll()) {
    
    // --- DEAD TIME: Signal Processing Window ---
    // Buffer is full, next strobe is ~60ms away (for 50Hz, 3 cycles, 4x strobe)
    // Plenty of time for floating-point math and analysis
    
    // Example: Prepare timing data for Extended Kalman Filter
    const double dt_strobe_error_s = (double)sys.strobe_offset_cycles / sys.cpu_freq;
    const double dt_first_sample_jitter_s = (double)sys.buf[0].jitter_cycles / sys.cpu_freq;
    
    // --- INSERT YOUR SIGNAL PROCESSING HERE ---
    // Recommended algorithms:
    // - Second-Order Generalized Integrator (SOGI) for frequency estimation
    // - Extended Kalman Filter (EKF) for noise rejection
    // - Zero-crossing detection with parabolic interpolation
    // - DFT/FFT-based frequency estimation
    
    // Placeholder: In real implementation, compute frequency from samples
    float estimated_freq = NOMINAL_FREQ;  // TODO: Replace with actual estimation
    
    // Apply frequency correction to PLL
    tune_pll(estimated_freq);
    
    // Print diagnostic information
    print_diagnostics();
    
    // Reset frame ready flag
    sys.frame_ready = false;
  }
  
  // Optional: Add a small yield to prevent watchdog timeout
  // Only if processing is very light and loop runs very fast
  // yield();
}
