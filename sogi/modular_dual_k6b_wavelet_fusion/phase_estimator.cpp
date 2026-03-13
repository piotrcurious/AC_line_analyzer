#include "phase_estimator.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

PhaseEstimator::PhaseEstimator()
  : history_buffers(nullptr),
    history_count(0),
    history_write_idx(0),
    current_state(PE_INITIALIZING),
    last_phase_shift(0.0f),
    cache_count(0),
    expected_next_shift(0.0f),
    current_pll_error(0.0f),
    strobe_cycles(4.0f),
    correction_cooldown(0),
    correction_was_applied(false),
    frames_since_correction(0),
    nominal_frequency(50.0f),
    buffer_time_interval(0.0f),
    last_freq_estimate(50.0f),
    samples_per_cycle(128),
    system_cpu_hz(240000000),
    correlation_buffer(nullptr),
    residual_buffer(nullptr),
    initialized(false) {
  memset(&config, 0, sizeof(config));
  memset(phase_trend_cache, 0, sizeof(phase_trend_cache));
  memset(history_pll_error, 0, sizeof(history_pll_error));
  memset(history_jitter_rad, 0, sizeof(history_jitter_rad));
}

PhaseEstimator::~PhaseEstimator() {
  if (history_buffers) free(history_buffers);
  if (correlation_buffer) free(correlation_buffer);
  if (residual_buffer) free(residual_buffer);
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

  history_buffers = (uint16_t*)malloc(total_size * sizeof(uint16_t));
  if (!history_buffers) return false;

  correlation_buffer = (float*)malloc(PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!correlation_buffer) return false;

  residual_buffer = (float*)malloc(PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!residual_buffer) return false;

  memset(history_buffers, 0, total_size * sizeof(uint16_t));

  initialized = true;
  current_state = PE_INITIALIZING;
  history_count = 0;
  history_write_idx = 0;

  return true;
}

void PhaseEstimator::reset() {
  history_count = 0;
  history_write_idx = 0;
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

void PhaseEstimator::add_frame(const uint16_t* buffer, uint16_t size, float jitter_rad, float current_f_pll, uint32_t strobe_tick) {
  if (!initialized || !buffer) return;

  uint16_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  if (size != buf_size) return;

  uint16_t* dest = get_history_buffer(history_write_idx);

  if (current_f_pll > 0.1f && fabs(current_f_pll - nominal_frequency) > 0.001f) {
    float ratio = current_f_pll / nominal_frequency;
    for (uint16_t i = 0; i < buf_size; i++) {
      float src_idx = (float)i * ratio;
      uint16_t i0 = (uint16_t)src_idx;
      uint16_t i1 = i0 + 1;
      float f = src_idx - (float)i0;
      if (i1 < buf_size) dest[i] = (uint16_t)((float)buffer[i0] * (1.0f - f) + (float)buffer[i1] * f);
      else dest[i] = buffer[i0];
    }
  } else {
    memcpy(dest, buffer, buf_size * sizeof(uint16_t));
  }

  history_f_pll[history_write_idx] = current_f_pll;
  history_ticks[history_write_idx] = strobe_tick;
  history_jitter_rad[history_write_idx] = jitter_rad;
  history_pll_error[history_write_idx] = current_pll_error;

  history_write_idx = (history_write_idx + 1) % config.history_depth;
  if (history_count < config.history_depth) {
    history_count++;
    if (history_count >= 3) current_state = PE_READY;
  }

  if (correction_was_applied) frames_since_correction++;
  if (correction_cooldown > 0) correction_cooldown--;
}

float PhaseEstimator::compute_phase_shift(const uint16_t* reference, const uint16_t* target) {
  const uint16_t* ref_cycle = reference + PE_SAMPLES_PER_CYCLE;
  const uint16_t* search_cycle = target + PE_SAMPLES_PER_CYCLE;

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

  float min_residual = 1e9f;
  int16_t best_offset = 0;

  for (int16_t offset = 0; offset < PE_SAMPLES_PER_CYCLE; offset++) {
    float residual_sum = 0.0f;
    for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
      uint16_t target_idx = (i + offset) % PE_SAMPLES_PER_CYCLE;
      float target_val = search_cycle[target_idx];
      float diff = correlation_buffer[i] - (target_val - ref_mean) / ref_std;
      residual_sum += diff * diff;
    }
    residual_buffer[offset] = residual_sum; // Cache for sub-sample interpolation
    if (residual_sum < min_residual) {
      min_residual = residual_sum;
      best_offset = offset;
    }
  }

  // Sub-sample Quadratic Interpolation
  float sub_offset = (float)best_offset;
  if (best_offset > 0 && best_offset < PE_SAMPLES_PER_CYCLE - 1) {
      float r_prev = residual_buffer[best_offset - 1];
      float r_curr = residual_buffer[best_offset];
      float r_next = residual_buffer[best_offset + 1];
      float denom = r_prev - 2.0f * r_curr + r_next;
      if (fabs(denom) > 1e-6f) {
          sub_offset += 0.5f * (r_prev - r_next) / denom;
      }
  } else if (best_offset == 0) {
      float r_prev = residual_buffer[PE_SAMPLES_PER_CYCLE - 1];
      float r_curr = residual_buffer[0];
      float r_next = residual_buffer[1];
      float denom = r_prev - 2.0f * r_curr + r_next;
      if (fabs(denom) > 1e-6f) {
          sub_offset += 0.5f * (r_prev - r_next) / denom;
      }
  }

  float phase_rad = -(2.0f * M_PI * sub_offset) / PE_SAMPLES_PER_CYCLE;
  while (phase_rad > M_PI) phase_rad -= 2.0f * M_PI;
  while (phase_rad < -M_PI) phase_rad += 2.0f * M_PI;

  return phase_rad;
}

void PhaseEstimator::fit_linear_drift(const float* trend, uint8_t count,
                                       float& slope, float& intercept) {
  if (count < 2) { slope = 0.0f; intercept = trend[0]; return; }
  float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float x = (float)i;
    float y = trend[i];
    sum_x += x; sum_y += y; sum_xy += x * y; sum_xx += x * x;
  }
  float n = (float)count;
  float denominator = (n * sum_xx - sum_x * sum_x);
  if (fabs(denominator) < 1e-9f) { slope = 0.0f; intercept = sum_y / n; }
  else { slope = (n * sum_xy - sum_x * sum_y) / denominator; intercept = (sum_y - slope * sum_x) / n; }
}

float PhaseEstimator::calculate_drift_variance(const float* trend, uint8_t count,
                                                 float slope, float intercept) {
  if (count < 2) return 0.0f;
  float variance = 0.0f;
  for (uint8_t i = 0; i < count; i++) {
    float expected = intercept + slope * i;
    float diff = trend[i] - expected;
    variance += diff * diff;
  }
  return variance / count;
}

void PhaseEstimator::analyze_trend(PhaseEstResult& result) {
  uint8_t count = result.valid_samples;
  if (count < 2) { current_state = PE_INITIALIZING; result.state = current_state; return; }
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
  if (!initialized || history_count < 3) return false;
  memset(&result, 0, sizeof(result));
  result.state = PE_INITIALIZING;

  uint16_t ref_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  const uint16_t* reference = get_history_buffer(ref_idx);

  float last_phase = 0.0f;
  for (uint16_t i = 0; i < history_count - 1; i++) {
    uint16_t hist_offset = i + 1;
    uint16_t hist_idx = (history_write_idx + config.history_depth - hist_offset - 1) % config.history_depth;
    const uint16_t* target = get_history_buffer(hist_idx);

    float raw_phase = compute_phase_shift(reference, target);
    float jitter_ref = history_jitter_rad[ref_idx];
    float jitter_target = history_jitter_rad[hist_idx];
    float corr_ref = history_pll_error[ref_idx];
    float corr_target = history_pll_error[hist_idx];

    float accumulated_phase = raw_phase - (jitter_target - jitter_ref) - (corr_target - corr_ref);
    if (i > 0) {
      float diff = accumulated_phase - last_phase;
      while (diff > M_PI) { accumulated_phase -= 2.0f * M_PI; diff = accumulated_phase - last_phase; }
      while (diff < -M_PI) { accumulated_phase += 2.0f * M_PI; diff = accumulated_phase - last_phase; }
    }
    uint8_t store_idx = (history_count - 2) - i;
    result.phase_trend[store_idx] = accumulated_phase;
    last_phase = accumulated_phase;
  }

  result.valid_samples = history_count - 1;
  result.recent_phase_shift = (result.valid_samples >= 2) ? (result.phase_trend[result.valid_samples - 1] - result.phase_trend[result.valid_samples - 2]) : 0.0f;

  // We define absolute_phase as the signal phase relative to the Anchor (oldest stored frame)
  // This allows EKF to track absolute deviation.
  result.absolute_phase = result.phase_trend[result.valid_samples - 1];

  cache_count = result.valid_samples;
  for (uint8_t i = 0; i < cache_count; i++) phase_trend_cache[i] = result.phase_trend[i];

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

  uint8_t n = (history_count > 4) ? 4 : history_count;
  float sum_f_est = 0.0f;
  uint8_t count = 0;
  for (uint8_t i = 0; i < n - 1; i++) {
    uint16_t r_idx = (history_write_idx + config.history_depth - 1 - i) % config.history_depth;
    uint16_t h_idx = (history_write_idx + config.history_depth - 2 - i) % config.history_depth;
    float delta_phi = compute_phase_shift(get_history_buffer(r_idx), get_history_buffer(h_idx));
    uint32_t dt_ticks = history_ticks[r_idx] - history_ticks[h_idx];
    float dt = (float)dt_ticks / system_cpu_hz;
    if (dt > 0.001f) {
      float f_est = (strobe_cycles - delta_phi / (2.0f * M_PI)) / dt;
      sum_f_est += f_est;
      count++;
    }
  }
  if (count == 0) return false;
  float estimated_freq = sum_f_est / count;
  result.frequency_hz = estimated_freq;
  result.frequency_error_hz = estimated_freq - nominal_frequency;
  result.valid = true;
  result.confidence = (current_state == PE_STABLE) ? 0.9f : 0.6f;
  last_freq_estimate = estimated_freq;
  return true;
}
