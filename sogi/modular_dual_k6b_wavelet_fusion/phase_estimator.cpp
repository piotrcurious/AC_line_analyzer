#include "phase_estimator.h"
#include <string.h>

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
    norm_ref_3c(nullptr),
    extended_tar(nullptr),
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
  if (norm_ref_3c) free(norm_ref_3c);
  if (extended_tar) free(extended_tar);
}

bool PhaseEstimator::begin(const PhaseEstConfig* cfg) {
  // Set default or user config
  if (cfg) {
    config = *cfg;
  } else {
    config.history_depth = PE_HISTORY_DEPTH;
    config.correction_threshold_rad = 0.3f;      // ~17 degrees - clear correction
    config.nonlinear_threshold_rad = 0.1f;       // ~5.7 degrees - signal changes
    config.stable_tolerance_rad = 0.02f;         // ~1.1 degrees - stable drift tolerance
  }

  // Allocate history buffers
  uint32_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  uint32_t total_size = buf_size * config.history_depth;

  history_buffers = (float*)malloc(total_size * sizeof(float));
  if (!history_buffers) return false;

  // Allocate correlation working buffer (one cycle for sliding window)
  correlation_buffer = (float*)malloc(PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!correlation_buffer) {
    free(history_buffers);
    history_buffers = nullptr;
    return false;
  }

  // Allocate residual buffer (for correlation calculations)
  residual_buffer = (float*)malloc(PE_SAMPLES_PER_CYCLE * sizeof(float));
  if (!residual_buffer) {
    free(history_buffers);
    free(correlation_buffer);
    history_buffers = nullptr;
    correlation_buffer = nullptr;
    return false;
  }

  norm_ref_3c = (float*)malloc(PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE * sizeof(float));
  extended_tar = (float*)malloc(2 * PE_SAMPLES_PER_CYCLE * sizeof(float));

  if (!norm_ref_3c || !extended_tar) {
      if (norm_ref_3c) free(norm_ref_3c);
      if (extended_tar) free(extended_tar);
      free(history_buffers);
      free(correlation_buffer);
      free(residual_buffer);
      return false;
  }

  memset(history_buffers, 0, total_size * sizeof(float));

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
  // If strobe_div_cycles is not provided, estimate from interval
  if (strobe_div_cycles <= 0.0f) {
    strobe_cycles = nominal_hz * buffer_interval_s;
  } else {
    strobe_cycles = strobe_div_cycles;
  }
}

void PhaseEstimator::notify_correction_applied(float correction_rad) {
  correction_was_applied = true;
  frames_since_correction = 0;
  expected_next_shift = correction_rad; // Expected immediate change
  current_pll_error += correction_rad;
}

void PhaseEstimator::pre_normalize_frame(const float* frame, float* dest_norm_3c) {
    for (uint8_t c = 0; c < PE_CYCLES_PER_BUFFER; c++) {
        const float* cycle = frame + c * PE_SAMPLES_PER_CYCLE;
        float* dest = dest_norm_3c + c * PE_SAMPLES_PER_CYCLE;

        float mean = 0.0f;
        for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) mean += cycle[i];
        mean /= PE_SAMPLES_PER_CYCLE;

        float std = 0.0f;
        for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
            float diff = cycle[i] - mean;
            dest[i] = diff;
            std += diff * diff;
        }
        std = sqrtf(std / PE_SAMPLES_PER_CYCLE);
        if (std < 1.0f) std = 1.0f;
        float inv_std = 1.0f / std;
        for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) dest[i] *= inv_std;
    }
}

void PhaseEstimator::add_frame(const float* buffer, uint16_t size, float jitter_rad, float current_f_pll, uint32_t strobe_tick) {
  if (!initialized || !buffer) return;

  uint16_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  if (size != buf_size) return; // Size mismatch

  // Normalize and store in history
  float* dest = get_history_buffer(history_write_idx);
  pre_normalize_frame(buffer, dest);

  // Store the state for this frame
  history_f_pll[history_write_idx] = current_f_pll;
  history_ticks[history_write_idx] = strobe_tick;
  history_jitter_rad[history_write_idx] = jitter_rad;
  history_pll_error[history_write_idx] = current_pll_error;

  // Update circular buffer index
  history_write_idx = (history_write_idx + 1) % config.history_depth;

  // Update count (saturate at depth)
  if (history_count < config.history_depth) {
    history_count++;
    if (history_count >= 3) {
      current_state = PE_READY; // Minimum data for estimation
    }
  }

  // Decrement cooldown
  if (correction_cooldown > 0) correction_cooldown--;
}

float PhaseEstimator::compute_phase_shift_norm(const float* reference, const float* target) {
  float sum_phase = 0.0f;
  for (uint8_t c = 0; c < PE_CYCLES_PER_BUFFER; c++) {
    sum_phase += compute_phase_shift_cycle_norm(reference + c * PE_SAMPLES_PER_CYCLE,
                                                target + c * PE_SAMPLES_PER_CYCLE);
  }
  return sum_phase / (float)PE_CYCLES_PER_BUFFER;
}

float PhaseEstimator::compute_phase_shift_cycle_norm(const float* norm_ref_cycle, const float* norm_tar_cycle) {
  // extended_tar is already normalized, just need to double it for sliding window
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    extended_tar[i] = norm_tar_cycle[i];
    extended_tar[i + PE_SAMPLES_PER_CYCLE] = norm_tar_cycle[i];
  }

  float max_dot = -1e9f;
  int16_t best_offset = 0;
  float residuals[PE_SAMPLES_PER_CYCLE];

  for (int16_t offset = 0; offset < PE_SAMPLES_PER_CYCLE; offset++) {
    float dot = 0.0f;
    const float* tar_ptr = extended_tar + offset;
    for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
      dot += norm_ref_cycle[i] * tar_ptr[i];
    }

    residuals[offset] = dot;

    if (dot > max_dot) {
      max_dot = dot;
      best_offset = offset;
    }
  }

  float fine_offset = (float)best_offset;
  if (best_offset > 0 && best_offset < PE_SAMPLES_PER_CYCLE - 1) {
    float y1 = residuals[best_offset - 1];
    float y2 = residuals[best_offset];
    float y3 = residuals[best_offset + 1];
    float denom = (2.0f * y2 - y1 - y3);
    if (fabs(denom) > 1e-6f) {
        fine_offset = (float)best_offset + (y3 - y1) / (2.0f * denom);
    }
  }

  float phase_rad = -(2.0f * (float)M_PI * fine_offset) / PE_SAMPLES_PER_CYCLE;
  while (phase_rad > (float)M_PI) phase_rad -= 2.0f * (float)M_PI;
  while (phase_rad < -(float)M_PI) phase_rad += 2.0f * (float)M_PI;

  return phase_rad;
}

void PhaseEstimator::fit_linear_drift(const float* trend, uint8_t count,
                                       float& slope, float& intercept) {
  if (count < 2) {
    slope = 0.0f;
    intercept = trend[0];
    return;
  }

  // Simple linear regression using cycles as X-axis
  float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;

  for (uint8_t i = 0; i < count; i++) {
    float x = (float)i * strobe_cycles;
    float y = trend[i];
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }

  float n = (float)count;
  float denominator = (n * sum_xx - sum_x * sum_x);

  if (fabs(denominator) < 1e-9f) {
    slope = 0.0f;
    intercept = sum_y / n;
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
    float expected = intercept + slope * ((float)i * strobe_cycles);
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

  // Fit linear model to phase trend
  float slope, intercept;
  fit_linear_drift(result.phase_trend, count, slope, intercept);
  result.linear_drift_rate = slope;

  // Calculate variance from linear fit
  float variance = calculate_drift_variance(result.phase_trend, count, slope, intercept);
  result.drift_variance = variance;

  // Get most recent phase change (derivative of phase trend)
  float recent_change = 0.0f;
  if (count >= 2) {
    recent_change = result.phase_trend[count - 1] - result.phase_trend[count - 2];
  }

  // === CORRECTION TRACKING (SIMPLIFIED) ===
  // If we applied a correction, just measure and report the actual change
  if (correction_was_applied) {
    frames_since_correction++;

    if (frames_since_correction >= 2) {
      // Measure actual phase change after correction
      result.correction_magnitude = recent_change;
      result.correction_effective = true; // We measured it

      // Clear tracking after reporting
      if (frames_since_correction >= 3) {
        correction_was_applied = false;
        frames_since_correction = 0;
      }
    } else {
      result.correction_magnitude = 0.0f;
      result.correction_effective = false;
    }
  }

  // === STATE CLASSIFICATION ===
  float variance_threshold = config.nonlinear_threshold_rad * config.nonlinear_threshold_rad;

  // Classify based on variance (trend linearity)
  if (variance > variance_threshold) {
    // High variance = non-linear (frequency changing or noise)
    current_state = PE_NONLINEAR_DRIFT;
  }
  else if (fabs(slope) < config.stable_tolerance_rad) {
    // Very low drift = locked
    current_state = PE_STABLE;
  }
  else {
    // Linear drift = stable tracking
    current_state = PE_STABLE;
  }

  last_phase_shift = recent_change;
  result.state = current_state;
}

bool PhaseEstimator::estimate_phase(PhaseEstResult& result) {
  if (!initialized || history_count < 3) {
    return false; // Need at least 3 buffers for meaningful analysis
  }

  // Clear result
  memset(&result, 0, sizeof(result));
  result.state = PE_INITIALIZING;
  result.correction_applied = correction_was_applied;
  result.correction_effective = false;

  // Get most recent buffer as reference (contains the wavelet to search for)
  uint16_t ref_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  const float* reference = get_history_buffer(ref_idx);

  // Calculate CUMULATIVE phase error for each historical buffer
  uint8_t valid_count = 0;
  float last_phase = 0.0f;

  for (uint16_t i = 0; i < history_count - 1; i++) {
    uint16_t hist_offset = i + 1;
    uint16_t hist_idx = (history_write_idx + config.history_depth - hist_offset - 1) % config.history_depth;

    const float* target = get_history_buffer(hist_idx);

    // Both target and reference are already normalized in history
    float raw_phase = compute_phase_shift_norm(target, reference);

    // Compensate for:
    // 1. Jitter in capture timing (positive jitter rad means captured LATE -> leading signal)
    // 2. Discrete PLL phase corrections applied between target and reference
    float jitter_ref = history_jitter_rad[ref_idx];
    float jitter_target = history_jitter_rad[hist_idx];
    float corr_ref = history_pll_error[ref_idx];
    float corr_target = history_pll_error[hist_idx];

    // Accumulated phase relative to sampling clock:
    // phi_true = measured_phase - jitter - correction
    // Trend[h] = phi_true_ref - phi_true_target
    //          = (phi_r - jitter_r - corr_r) - (phi_t - jitter_t - corr_t)
    //          = raw_phase - (jitter_ref - jitter_target) - (corr_ref - corr_target)
    float accumulated_phase = raw_phase - (jitter_ref - jitter_target) - (corr_ref - corr_target);

    // Unwrap phase discontinuities
    if (i > 0) {
      float diff = accumulated_phase - last_phase;
      while (diff > (float)M_PI) {
        accumulated_phase -= 2.0f * (float)M_PI;
        diff = accumulated_phase - last_phase;
      }
      while (diff < -(float)M_PI) {
        accumulated_phase += 2.0f * (float)M_PI;
        diff = accumulated_phase - last_phase;
      }
    }

    // Store in chronological order: [0] = oldest, [n-1] = newest
    uint8_t store_idx = (history_count - 2) - i;
    result.phase_trend[store_idx] = accumulated_phase;
    last_phase = accumulated_phase;
    valid_count++;
  }

  result.valid_samples = valid_count;
  result.recent_phase_shift = (valid_count >= 2) ?
    (result.phase_trend[valid_count - 1] - result.phase_trend[valid_count - 2]) : 0.0f;

  // Cache the trend for frequency estimation
  cache_count = valid_count;
  for (uint8_t i = 0; i < valid_count; i++) {
    phase_trend_cache[i] = result.phase_trend[i];
  }

  // Analyze trend and update state machine
  analyze_trend(result);

  // Add frequency estimation if parameters are set
  if (buffer_time_interval > 0.0f) {
    FrequencyEstResult freq_result;
    // Pass the phase result so it can use the trend data
    if (estimate_frequency(freq_result)) {
      result.estimated_frequency = freq_result.frequency_hz;
      result.estimated_frequency_error = freq_result.frequency_error_hz;
      result.frequency_estimate_valid = freq_result.valid;
    }
  }

  return true;
}

bool PhaseEstimator::estimate_frequency(FrequencyEstResult& result) {
  // Clear result first to avoid garbage values
  memset(&result, 0, sizeof(result));
  result.frequency_hz = nominal_frequency;
  result.valid = false;

  if (!initialized || history_count < 3 || system_cpu_hz == 0 || cache_count < 2) {
    return false;
  }

  // OPTIMIZED: Use the trend slope from estimate_phase instead of re-calculating correlations.
  // slope is radians per cycle.
  // f_grid = f_pll * (1 - slope / 2pi)

  uint16_t newest_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  float current_f_pll = history_f_pll[newest_idx];

  // We need to find the slope. It was calculated in analyze_trend which is called by estimate_phase.
  // We'll use the cached linear_drift_rate if valid.
  float slope, intercept;
  fit_linear_drift(phase_trend_cache, cache_count, slope, intercept);

  float estimated_freq = current_f_pll * (1.0f - slope / (2.0f * (float)M_PI));

  // PLL error for correction is f_pll - f_grid
  float pll_error_hz = current_f_pll - estimated_freq;

  // Confidence based on current state
  float confidence = 0.0f;
  switch(current_state) {
    case PE_STABLE:
      confidence = 0.9f;
      break;
    case PE_NONLINEAR_DRIFT:
      confidence = 0.7f; // Still useful during frequency changes
      break;
    case PE_READY:
      confidence = 0.6f;
      break;
    default:
      confidence = 0.3f;
      break;
  }

  // Sanity checks - be more permissive for rapid changes
  bool freq_reasonable = (fabs(pll_error_hz) < 15.0f); // Max ±15 Hz error
  bool confidence_ok = (confidence > 0.2f);

  if (freq_reasonable && confidence_ok) {
    result.frequency_hz = estimated_freq;
    result.frequency_error_hz = estimated_freq - nominal_frequency;
    result.confidence = confidence;
    result.valid = true;
    result.pll_correction_hz = pll_error_hz;

    last_freq_estimate = estimated_freq;
  } else {
    // Fall back to last estimate
    result.frequency_hz = last_freq_estimate;
    result.frequency_error_hz = last_freq_estimate - nominal_frequency;
    result.confidence = 0.0f;
    result.valid = false;
    result.pll_correction_hz = last_freq_estimate - current_f_pll;
  }

  return result.valid;
}
