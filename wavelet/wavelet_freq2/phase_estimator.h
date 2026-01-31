#ifndef PHASE_ESTIMATOR_H
#define PHASE_ESTIMATOR_H

#include <Arduino.h>
#include <math.h>

#define PE_HISTORY_DEPTH 16      // Number of full buffers to store
#define PE_SAMPLES_PER_CYCLE 128 // Must match SAMPLES_PER_CYCLE
#define PE_CYCLES_PER_BUFFER 3   // Must match CYCLES_TO_CAPTURE

// Phase estimation states
enum PhaseEstState {
  PE_INITIALIZING,        // Filling initial history
  PE_STABLE,              // Tracking stable linear drift
  PE_NONLINEAR_DRIFT,     // Non-linear phase changes (signal variation or freq changing)
  PE_READY               // General ready state
};

// Configuration for phase estimator
struct PhaseEstConfig {
  uint16_t history_depth;          // Number of buffers to keep
  float correction_threshold_rad;  // Threshold to detect intentional correction
  float nonlinear_threshold_rad;   // Threshold for non-linear drift detection
  float stable_tolerance_rad;      // Max deviation for "stable" classification
};

// Result from phase estimation
struct PhaseEstResult {
  PhaseEstState state;
  float phase_trend[PE_HISTORY_DEPTH]; // Phase shift for each historical buffer (radians)
  float linear_drift_rate;              // Radians per buffer (if stable drift)
  float recent_phase_shift;             // Most recent phase shift (radians)
  uint8_t valid_samples;                // Number of valid trend entries
  bool correction_applied;              // True if WE applied a correction (input flag)
  bool correction_effective;            // True if applied correction shows up in trend
  float correction_magnitude;           // Size of detected correction (radians)
  float drift_variance;                 // Variance in phase drift (nonlinearity indicator)
  
  // Frequency estimation
  float estimated_frequency_error;      // Estimated frequency error (Hz)
  float estimated_frequency;            // Estimated actual grid frequency (Hz)
  bool frequency_estimate_valid;        // True if frequency estimate is reliable
};

// Frequency estimation result
struct FrequencyEstResult {
  float frequency_hz;                   // Estimated grid frequency
  float frequency_error_hz;             // Error relative to nominal (positive = grid faster)
  float confidence;                     // Confidence in estimate (0-1)
  bool valid;                           // True if estimate is reliable
  float pll_correction_hz;              // Recommended PLL frequency correction
};

class PhaseEstimator {
private:
  // Configuration
  PhaseEstConfig config;
  
  // History buffer storage
  uint16_t* history_buffers;  // Flat array: [depth][CYCLES][SAMPLES]
  uint16_t history_count;     // Number of buffers currently stored
  uint16_t history_write_idx; // Circular buffer write position
  
  // State tracking
  PhaseEstState current_state;
  float last_phase_shift;
  float phase_trend_cache[PE_HISTORY_DEPTH];
  uint8_t cache_count;
  float expected_next_shift; // Based on linear model
  float history_pll_error[PE_HISTORY_DEPTH];
  float current_pll_error;
  float strobe_cycles;
  uint32_t correction_cooldown; // Frames to wait after correction
  bool correction_was_applied;  // Tracks if we applied a correction
  uint32_t frames_since_correction; // Frames elapsed since correction
  
  // Frequency estimation
  float nominal_frequency;      // Nominal grid frequency (Hz)
  float buffer_time_interval;   // Time between buffers (seconds)
  float last_freq_estimate;     // Last frequency estimate
  uint32_t samples_per_cycle;   // Samples per nominal cycle
  
  // Internal calculation buffers
  float* correlation_buffer;
  float* residual_buffer;
  
  bool initialized;

  // Helper: get buffer pointer for a specific history index
  inline uint16_t* get_history_buffer(uint16_t idx) {
    uint16_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
    return &history_buffers[idx * buf_size];
  }
  
  // Compute phase shift between reference wavelet and target buffer using sliding correlation
  float compute_phase_shift(const uint16_t* reference, const uint16_t* target);
  
  // Analyze trend to determine state
  void analyze_trend(PhaseEstResult& result);
  
  // Fit linear model to phase trend
  void fit_linear_drift(const float* trend, uint8_t count, float& slope, float& intercept);
  
  // Calculate variance of residuals from linear fit
  float calculate_drift_variance(const float* trend, uint8_t count, float slope, float intercept);

public:
  PhaseEstimator();
  ~PhaseEstimator();
  
  // Initialize with custom configuration
  bool begin(const PhaseEstConfig* cfg = nullptr);
  
  // Set nominal frequency and timing parameters for frequency estimation
  // strobe_div_cycles is the number of nominal cycles captured between buffers (e.g. 4.0)
  void set_frequency_params(float nominal_hz, float buffer_interval_s, uint16_t samps_per_cycle, float strobe_div_cycles = 0.0f);
  
  // Add a new frame to history
  void add_frame(const uint16_t* buffer, uint16_t size);
  
  // Notify that a phase correction was applied (for tracking)
  void notify_correction_applied(float correction_rad);
  
  // Perform phase estimation analysis
  bool estimate_phase(PhaseEstResult& result);
  
  // Perform frequency estimation based on phase drift
  bool estimate_frequency(FrequencyEstResult& result);
  
  // Get current state
  PhaseEstState get_state() const { return current_state; }
  
  // Reset estimator (clear history)
  void reset();
  
  // Check if enough history is available for estimation
  bool is_ready() const { return history_count >= 3; }
};

#endif // PHASE_ESTIMATOR_H
