#include "phase_estimator.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

PhaseEstimator::PhaseEstimator()
  : history_buffers(nullptr),
    history_count(0),
    history_write_idx(0),
    reference_frame(nullptr),
    reference_valid(false),
    current_state(PE_INITIALIZING),
    last_phase_shift(0.0f),
    cache_count(0),
    expected_next_shift(0.0f),
    current_pll_error(0.0f),
    strobe_cycles(1.0f),
    correction_cooldown(0),
    correction_was_applied(false),
    frames_since_correction(0),
    nominal_frequency(50.0f),
    buffer_time_interval(0.02f),
    last_freq_estimate(50.0f),
    samples_per_cycle(128),
    system_cpu_hz(240000000),
    correlation_buffer(nullptr),
    extended_target(nullptr),
    initialized(false) {
  memset(&config, 0, sizeof(config));
  memset(phase_trend_cache, 0, sizeof(phase_trend_cache));
  memset(history_pll_error, 0, sizeof(history_pll_error));
  memset(history_jitter_rad, 0, sizeof(history_jitter_rad));
}

PhaseEstimator::~PhaseEstimator() {
  if (history_buffers) free(history_buffers);
  if (reference_frame) free(reference_frame);
  if (correlation_buffer) free(correlation_buffer);
  if (extended_target) free(extended_target);
}

bool PhaseEstimator::begin(const PhaseEstConfig* cfg) {
  if (cfg) {
    config = *cfg;
  } else {
    config.history_depth = PE_HISTORY_DEPTH;
    config.correction_threshold_rad = 0.3f;
    config.nonlinear_threshold_rad = 0.1f;
    config.stable_tolerance_rad = 0.02f;
  }

  uint32_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  uint32_t total_size = buf_size * config.history_depth;

  history_buffers = (float*)malloc(total_size * sizeof(float));
  if (!history_buffers) return false;

  reference_frame = (float*)malloc(buf_size * sizeof(float));
  if (!reference_frame) return false;

  correlation_buffer = (float*)malloc(PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!correlation_buffer) return false;

  extended_target = (float*)malloc(2 * PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!extended_target) return false;

  memset(history_buffers, 0, total_size * sizeof(float));

  initialized = true;
  current_state = PE_INITIALIZING;
  history_count = 0;
  history_write_idx = 0;
  reference_valid = false;

  return true;
}

void PhaseEstimator::reset() {
  history_count = 0;
  history_write_idx = 0;
  reference_valid = false;
  current_state = PE_INITIALIZING;
  last_phase_shift = 0.0f;
  cache_count = 0;
  memset(phase_trend_cache, 0, sizeof(phase_trend_cache));
  memset(history_pll_error, 0, sizeof(history_pll_error));
  memset(history_jitter_rad, 0, sizeof(history_jitter_rad));
  expected_next_shift = 0.0f;
  current_pll_error = 0.0f;
  correction_cooldown = 0;
  correction_was_applied = false;
  frames_since_correction = 0;
  last_freq_estimate = nominal_frequency;
}

void PhaseEstimator::set_frequency_params(float nominal_hz, float buffer_interval_s, uint16_t samps_per_cycle, float strobe_div_cycles, uint32_t cpu_hz) {
  nominal_frequency = nominal_hz;
  buffer_time_interval = buffer_interval_s;
  samples_per_cycle = samps_per_cycle;
  last_freq_estimate = nominal_hz;
  system_cpu_hz = cpu_hz;
  if (strobe_div_cycles <= 0.0f) {
    strobe_cycles = nominal_hz * buffer_interval_s;
  } else {
    strobe_cycles = strobe_div_cycles;
  }
}

void PhaseEstimator::notify_correction_applied(float correction_rad) {
  correction_was_applied = true;
  frames_since_correction = 0;
  expected_next_shift = correction_rad;
  current_pll_error += correction_rad;
}

void PhaseEstimator::add_frame(const float* buffer, uint16_t size, float jitter_rad, float current_f_pll, uint32_t strobe_tick) {
  if (!initialized || !buffer) return;

  uint16_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  if (size != buf_size) return;

  float* dest = get_history_buffer(history_write_idx);

  // Resample to nominal frequency if needed to keep the wavelet shape consistent
  if (fabs(current_f_pll - nominal_frequency) > 0.01f) {
      float ratio = current_f_pll / nominal_frequency;
      for (int i=0; i<buf_size; i++) {
          float src = i * ratio;
          int i0 = (int)src;
          int i1 = i0 + 1;
          float f = src - i0;
          if (i1 < buf_size) dest[i] = buffer[i0] * (1.0f - f) + buffer[i1] * f;
          else dest[i] = buffer[i0];
      }
  } else {
      memcpy(dest, buffer, buf_size * sizeof(float));
  }

  // Initialize reference with the first valid frame
  if (!reference_valid) {
      memcpy(reference_frame, dest, buf_size * sizeof(float));
      reference_valid = true;
  }

  history_f_pll[history_write_idx] = current_f_pll;
  history_ticks[history_write_idx] = strobe_tick;
  history_jitter_rad[history_write_idx] = jitter_rad;
  history_pll_error[history_write_idx] = current_pll_error;

  history_write_idx = (history_write_idx + 1) % config.history_depth;
  if (history_count < config.history_depth) {
    history_count++;
    if (history_count >= 3) {
      current_state = PE_READY;
    }
  }

  if (correction_was_applied) frames_since_correction++;
  if (correction_cooldown > 0) correction_cooldown--;
}

float PhaseEstimator::compute_phase_shift(const float* reference, const float* target) {
  const float* ref_cycle = reference + PE_SAMPLES_PER_CYCLE;
  const float* tar_cycle = target + PE_SAMPLES_PER_CYCLE;

  float ref_mean = 0.0f;
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) ref_mean += ref_cycle[i];
  ref_mean /= PE_SAMPLES_PER_CYCLE;

  float ref_std = 0.0f;
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    correlation_buffer[i] = ref_cycle[i] - ref_mean;
    ref_std += correlation_buffer[i] * correlation_buffer[i];
  }
  ref_std = sqrtf(ref_std / PE_SAMPLES_PER_CYCLE);
  if (ref_std < 1.0f) ref_std = 1.0f;

  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) correlation_buffer[i] /= ref_std;

  // Prep extended target for modulo-free inner loop
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
      extended_target[i] = (tar_cycle[i] - ref_mean) / ref_std;
      extended_target[i + PE_SAMPLES_PER_CYCLE] = extended_target[i];
  }

  float min_residual = 1e9f;
  int16_t best_offset = 0;

  for (int16_t offset = 0; offset < PE_SAMPLES_PER_CYCLE; offset++) {
    float residual_sum = 0.0f;
    const float* p_tar = extended_target + offset;
    for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
      float diff = correlation_buffer[i] - p_tar[i];
      residual_sum += diff * diff;
    }
    if (residual_sum < min_residual) {
      min_residual = residual_sum;
      best_offset = offset;
    }
  }

  float phase_rad = -(2.0f * M_PI * best_offset) / PE_SAMPLES_PER_CYCLE;
  while (phase_rad > M_PI) phase_rad -= 2.0f * M_PI;
  while (phase_rad < -M_PI) phase_rad += 2.0f * M_PI;

  return phase_rad;
}

void PhaseEstimator::fit_linear_drift(const float* trend, uint8_t count,
                                       float& slope, float& intercept) {
  if (count < 2) {
    slope = 0.0f;
    intercept = trend[0];
    return;
  }
  float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float x = (float)i * strobe_cycles;
    float y = trend[i];
    sum_x += x; sum_y += y; sum_xy += x * y; sum_xx += x * x;
  }
  float n = (float)count;
  float denominator = (n * sum_xx - sum_x * sum_x);
  if (fabs(denominator) < 1e-9f) {
    slope = 0.0f; intercept = sum_y / n;
  } else {
    slope = (n * sum_xy - sum_x * sum_y) / denominator;
    intercept = (sum_y - slope * sum_x) / n;
  }
}

float PhaseEstimator::calculate_drift_variance(const float* trend, uint8_t count,
                                                 float slope, float intercept) {
  if (count < 2) return 0.0f;
  float variance = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float x = (float)i * strobe_cycles;
    float expected = intercept + slope * x;
    float diff = trend[i] - expected;
    variance += diff * diff;
  }
  return variance / count;
}

void PhaseEstimator::analyze_trend(PhaseEstResult& result) {
  uint8_t count = result.valid_samples;
  if (count < 2) {
    current_state = PE_INITIALIZING;
    result.state = current_state;
    return;
  }
  float slope, intercept;
  fit_linear_drift(result.phase_trend, count, slope, intercept);
  result.linear_drift_rate = slope;

  float variance = calculate_drift_variance(result.phase_trend, count, slope, intercept);
  result.drift_variance = variance;

  float recent_change = 0.0f;
  if (count >= 2) recent_change = result.phase_trend[count - 1] - result.phase_trend[count - 2];

  if (correction_was_applied) {
    frames_since_correction++;
    if (frames_since_correction >= 2) {
      result.correction_magnitude = recent_change;
      result.correction_effective = true;
      if (frames_since_correction >= 3) { correction_was_applied = false; frames_since_correction = 0; }
    }
  }

  float variance_threshold = config.nonlinear_threshold_rad * config.nonlinear_threshold_rad;
  if (variance > variance_threshold) current_state = PE_NONLINEAR_DRIFT;
  else if (fabs(slope) < config.stable_tolerance_rad) current_state = PE_STABLE;
  else current_state = PE_STABLE;

  last_phase_shift = recent_change;
  result.state = current_state;
}

bool PhaseEstimator::estimate_phase(PhaseEstResult& result) {
  if (!initialized || history_count < 3 || !reference_valid) return false;

  memset(&result, 0, sizeof(result));
  result.state = PE_INITIALIZING;

  uint16_t newest_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  const float* newest_frame = get_history_buffer(newest_idx);

  // Calculate absolute phase relative to reference anchor
  float raw_phase = compute_phase_shift(reference_frame, newest_frame);

  // True signal phase deviation relative to the time reference anchor
  // We subtract the accumulated PLL corrections to get absolute deviation from nominal
  float absolute_phase = raw_phase - history_jitter_rad[newest_idx] - history_pll_error[newest_idx];
  result.absolute_phase = absolute_phase;

  // Build trend from history
  uint8_t valid_count = 0;
  for (uint16_t i = 0; i < history_count; i++) {
      uint16_t idx = (history_write_idx + config.history_depth - history_count + i) % config.history_depth;
      float ph = compute_phase_shift(reference_frame, get_history_buffer(idx));
      result.phase_trend[i] = ph - history_jitter_rad[idx] - history_pll_error[idx];
      // Unwrap trend
      if (i > 0) {
          float diff = result.phase_trend[i] - result.phase_trend[i-1];
          while (diff > M_PI) { result.phase_trend[i] -= 2.0f * M_PI; diff -= 2.0f * M_PI; }
          while (diff < -M_PI) { result.phase_trend[i] += 2.0f * M_PI; diff += 2.0f * M_PI; }
      }
      valid_count++;
  }

  result.valid_samples = valid_count;
  analyze_trend(result);

  if (buffer_time_interval > 0.0f) {
    FrequencyEstResult freq_result;
    if (estimate_frequency(freq_result)) {
      result.estimated_frequency = freq_result.frequency_hz;
      result.estimated_frequency_error = freq_result.frequency_error_hz;
      result.frequency_estimate_valid = freq_result.valid;
    }
  }

  return true;
}

bool PhaseEstimator::estimate_frequency(FrequencyEstResult& result) {
  memset(&result, 0, sizeof(result));
  result.frequency_hz = nominal_frequency;
  if (!initialized || history_count < 2 || system_cpu_hz == 0) return false;

  uint16_t r_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  uint16_t h_idx = (history_write_idx + config.history_depth - 2) % config.history_depth;

  // Measurement of phase shift between last two frames
  float delta_phi = compute_phase_shift(get_history_buffer(h_idx), get_history_buffer(r_idx));
  uint32_t dt_ticks = history_ticks[r_idx] - history_ticks[h_idx];
  float dt = (float)dt_ticks / system_cpu_hz;

  if (dt > 0.001f) {
    // f_grid = (strobe_cycles + delta_phi / 2pi) / dt
    float f_est = (strobe_cycles + delta_phi / (2.0f * M_PI)) / dt;
    result.frequency_hz = f_est;
    result.frequency_error_hz = f_est - nominal_frequency;
    result.valid = true;
    result.confidence = (current_state == PE_STABLE) ? 0.9f : 0.6f;
    result.pll_correction_hz = history_f_pll[r_idx] - f_est;
    last_freq_estimate = f_est;
    return true;
  }
  return false;
}
