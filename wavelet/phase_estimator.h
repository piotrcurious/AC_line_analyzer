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
  PE_CORRECTION_DETECTED, // Phase jump detected (correction applied)
  PE_NONLINEAR_DRIFT,     // Non-linear phase changes (signal variation)
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
  bool correction_detected;             // True if phase correction was applied
  float correction_magnitude;           // Size of detected correction (radians)
  float drift_variance;                 // Variance in phase drift (nonlinearity indicator)
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
  float expected_next_shift; // Based on linear model
  uint32_t correction_cooldown; // Frames to wait after correction
  
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
  
  // Add a new frame to history
  void add_frame(const uint16_t* buffer, uint16_t size);
  
  // Perform phase estimation analysis
  bool estimate_phase(PhaseEstResult& result);
  
  // Get current state
  PhaseEstState get_state() const { return current_state; }
  
  // Reset estimator (clear history)
  void reset();
  
  // Check if enough history is available for estimation
  bool is_ready() const { return history_count >= 3; }
};

#endif // PHASE_ESTIMATOR_H
