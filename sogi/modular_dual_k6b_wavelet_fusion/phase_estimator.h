#ifndef PHASE_ESTIMATOR_H
#define PHASE_ESTIMATOR_H

#include <Arduino.h>
#include <math.h>

#define PE_HISTORY_DEPTH 16
#define PE_SAMPLES_PER_CYCLE 128
#define PE_CYCLES_PER_BUFFER 3

enum PhaseEstState {
  PE_INITIALIZING,
  PE_STABLE,
  PE_NONLINEAR_DRIFT,
  PE_READY
};

struct PhaseEstConfig {
  uint16_t history_depth;
  float correction_threshold_rad;
  float nonlinear_threshold_rad;
  float stable_tolerance_rad;
};

struct PhaseEstResult {
  PhaseEstState state;
  float phase_trend[PE_HISTORY_DEPTH];
  float linear_drift_rate;
  float recent_phase_shift;
  uint8_t valid_samples;
  bool correction_applied;
  bool correction_effective;
  float correction_magnitude;
  float drift_variance;

  float estimated_frequency_error;
  float estimated_frequency;
  bool frequency_estimate_valid;
  float absolute_phase;
};

struct FrequencyEstResult {
  float frequency_hz;
  float frequency_error_hz;
  float confidence;
  bool valid;
  float pll_correction_hz;
};

class PhaseEstimator {
private:
  PhaseEstConfig config;

  uint16_t* history_buffers;
  uint16_t history_count;
  uint16_t history_write_idx;

  PhaseEstState current_state;
  float last_phase_shift;
  float phase_trend_cache[PE_HISTORY_DEPTH];
  uint8_t cache_count;
  float expected_next_shift;
  float history_f_pll[PE_HISTORY_DEPTH];
  uint32_t history_ticks[PE_HISTORY_DEPTH];
  float history_jitter_rad[PE_HISTORY_DEPTH];
  float history_pll_error[PE_HISTORY_DEPTH];
  float current_pll_error;
  float strobe_cycles;
  uint32_t correction_cooldown;
  bool correction_was_applied;
  uint32_t frames_since_correction;

  float nominal_frequency;
  float buffer_time_interval;
  float last_freq_estimate;
  uint32_t samples_per_cycle;
  uint32_t system_cpu_hz;

  float* correlation_buffer;
  float* residual_buffer;

  bool initialized;

  inline uint16_t* get_history_buffer(uint16_t idx) {
    return &history_buffers[idx * PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE];
  }

  float compute_phase_shift(const uint16_t* reference, const uint16_t* target);
  void analyze_trend(PhaseEstResult& result);
  void fit_linear_drift(const float* trend, uint8_t count, float& slope, float& intercept);
  float calculate_drift_variance(const float* trend, uint8_t count, float slope, float intercept);

public:
  PhaseEstimator();
  ~PhaseEstimator();

  bool begin(const PhaseEstConfig* cfg = nullptr);
  void set_frequency_params(float nominal_hz, float buffer_interval_s, uint16_t samps_per_cycle, float strobe_div_cycles = 0.0f, uint32_t cpu_hz = 240000000);
  void add_frame(const uint16_t* buffer, uint16_t size, float jitter_rad = 0.0f, float current_f_pll = 0.0f, uint32_t strobe_tick = 0);
  void notify_correction_applied(float correction_rad);
  void set_pll_error(float err) { current_pll_error = err; }
  bool estimate_phase(PhaseEstResult& result);
  bool estimate_frequency(FrequencyEstResult& result);
  PhaseEstState get_state() const { return current_state; }
  void reset();
  bool is_ready() const { return history_count >= 3; }
};

#endif // PHASE_ESTIMATOR_H
