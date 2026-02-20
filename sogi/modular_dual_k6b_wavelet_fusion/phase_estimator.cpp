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
    reference_frame(nullptr),
    reference_valid(false),
    ref_jitter_rad(0.0f),
    ref_pll_error(0.0f),
    initialized(false) {
  memset(&config, 0, sizeof(config));
  memset(history_pll_error, 0, sizeof(history_pll_error));
  memset(history_jitter_rad, 0, sizeof(history_jitter_rad));
  memset(phase_history, 0, sizeof(phase_history));
}

PhaseEstimator::~PhaseEstimator() {
  if (history_buffers) free(history_buffers);
  if (correlation_buffer) free(correlation_buffer);
  if (residual_buffer) free(residual_buffer);
  if (norm_ref_3c) free(norm_ref_3c);
  if (extended_tar) free(extended_tar);
  if (reference_frame) free(reference_frame);
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
  reference_frame = (float*)malloc(PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE * sizeof(float));

  if (!norm_ref_3c || !extended_tar || !reference_frame) {
      if (norm_ref_3c) free(norm_ref_3c);
      if (extended_tar) free(extended_tar);
      if (reference_frame) free(reference_frame);
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
  reference_valid = false;
  current_state = PE_INITIALIZING;
  last_phase_shift = 0.0f;
  cache_count = 0;
  memset(history_pll_error, 0, sizeof(history_pll_error));
  memset(history_jitter_rad, 0, sizeof(history_jitter_rad));
  memset(phase_history, 0, sizeof(phase_history));
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

  // Initialize reference if needed
  if (!reference_valid) {
    memcpy(reference_frame, dest, buf_size * sizeof(float));
    ref_jitter_rad = jitter_rad;
    ref_pll_error = current_pll_error;
    reference_valid = true;
  }

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
    return compute_phase_shift_cycle_hierarchical(norm_ref_cycle, norm_tar_cycle);
}

float PhaseEstimator::compute_phase_shift_cycle_hierarchical(const float* norm_ref_cycle, const float* norm_tar_cycle) {
  // Pre-fill extended target (2x length for wrapping)
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    extended_tar[i] = norm_tar_cycle[i];
    extended_tar[i + PE_SAMPLES_PER_CYCLE] = norm_tar_cycle[i];
  }

  // Pass 1: Coarse search
  const int coarse_stride = 8;
  float max_dot = -1e9f;
  int16_t coarse_best = 0;

  for (int16_t offset = 0; offset < PE_SAMPLES_PER_CYCLE; offset += coarse_stride) {
    float dot = 0.0f;
    const float* tar_ptr = extended_tar + offset;
    for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
      dot += norm_ref_cycle[i] * tar_ptr[i];
    }
    if (dot > max_dot) {
      max_dot = dot;
      coarse_best = offset;
    }
  }

  // Pass 2: Fine search around coarse_best
  int16_t fine_best = coarse_best;
  float residuals[coarse_stride * 2 + 1]; // for interpolation
  int16_t search_start = coarse_best - coarse_stride;

  max_dot = -1e9f;
  for (int i = 0; i <= coarse_stride * 2; i++) {
    int16_t offset = (search_start + i + PE_SAMPLES_PER_CYCLE) % PE_SAMPLES_PER_CYCLE;
    float dot = 0.0f;
    const float* tar_ptr = extended_tar + offset;
    for (uint16_t j = 0; j < PE_SAMPLES_PER_CYCLE; j++) {
      dot += norm_ref_cycle[j] * tar_ptr[j];
    }
    residuals[i] = dot;
    if (dot > max_dot) {
      max_dot = dot;
      fine_best = offset;
    }
  }

  // Find index of fine_best in residuals array
  int16_t best_idx_in_res = -1;
  for (int i = 0; i <= coarse_stride * 2; i++) {
      int16_t offset = (search_start + i + PE_SAMPLES_PER_CYCLE) % PE_SAMPLES_PER_CYCLE;
      if (offset == fine_best) {
          best_idx_in_res = i;
          break;
      }
  }

  // Sub-sample interpolation (parabolic fit)
  float fine_offset = (float)fine_best;
  if (best_idx_in_res > 0 && best_idx_in_res < coarse_stride * 2) {
    float y1 = residuals[best_idx_in_res - 1];
    float y2 = residuals[best_idx_in_res];
    float y3 = residuals[best_idx_in_res + 1];
    float denom = (2.0f * y2 - y1 - y3);
    if (fabs(denom) > 1e-6f) {
        fine_offset = (float)fine_best + (y3 - y1) / (2.0f * denom);
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
  if (!initialized || history_count < 3 || !reference_valid) {
    return false;
  }

  // Clear result
  memset(&result, 0, sizeof(result));
  result.state = PE_INITIALIZING;
  result.correction_applied = correction_was_applied;
  result.correction_effective = false;

  // We compute one NEW phase point for the newest frame relative to persistent reference
  uint16_t newest_idx = (history_write_idx + config.history_depth - 1) % config.history_depth;
  const float* newest_frame = get_history_buffer(newest_idx);

  // Correlate: newest_frame (target) vs persistent reference
  // raw_phase = phi_ref - phi_newest
  float raw_phase = compute_phase_shift_norm(newest_frame, reference_frame);

  float jitter_newest = history_jitter_rad[newest_idx];
  float corr_newest = history_pll_error[newest_idx];

  // Accumulated phase relative to sampling clock:
  // phi_true = measured_phase - jitter - correction
  // phi_true_newest = raw_phase_absolute = (phi_ref_true) - raw_phase - jitter_newest - corr_newest
  // We use the persistent reference as our temporal anchor (phi_ref_true = jitter_ref + corr_ref)
  float absolute_phase = (ref_jitter_rad + ref_pll_error) - raw_phase - jitter_newest - corr_newest;

  // Store in phase_history (this is already unwrapped implicitly by linear growth, but let's be careful)
  // We'll use a local history array for regression
  phase_history[newest_idx] = absolute_phase;
  history_ticks_buf[newest_idx] = history_ticks[newest_idx];
  history_f_pll_buf[newest_idx] = history_f_pll[newest_idx];

  // Collect history in chronological order for result.phase_trend
  uint8_t valid_count = 0;
  float last_val = 0.0f;
  for (uint16_t i = 0; i < history_count; i++) {
    uint16_t idx = (history_write_idx + config.history_depth - history_count + i) % config.history_depth;
    float val = phase_history[idx];

    // Unwrap relative to previous
    if (valid_count > 0) {
        float diff = val - last_val;
        while (diff > (float)M_PI) { val -= 2.0f * (float)M_PI; diff = val - last_val; }
        while (diff < -(float)M_PI) { val += 2.0f * (float)M_PI; diff = val - last_val; }
        phase_history[idx] = val; // Store back unwrapped
    }

    result.phase_trend[i] = val;
    last_val = val;
    valid_count++;
  }

  result.valid_samples = valid_count;
  result.recent_phase_shift = (valid_count >= 2) ?
    (result.phase_trend[valid_count - 1] - result.phase_trend[valid_count - 2]) : 0.0f;

  cache_count = valid_count;

  // Analyze trend and update state machine
  analyze_trend(result);

  // Add frequency estimation if parameters are set
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

  // Note: result.linear_drift_rate is already updated by analyze_trend in estimate_phase
  // which is usually called just before estimate_frequency.
  // But since we don't pass result in here, we use a local regression.
  float slope, intercept;
  // Use the trend data we just filled in estimate_phase
  // Wait, estimate_phase fills result.phase_trend.
  // Let's just do a quick regression on phase_history elements
  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
  for (uint8_t i = 0; i < cache_count; i++) {
      uint16_t idx = (history_write_idx + config.history_depth - cache_count + i) % config.history_depth;
      float x = i * strobe_cycles;
      float y = phase_history[idx];
      sum_x += x; sum_y += y; sum_xy += x * y; sum_xx += x * x;
  }
  float n = (float)cache_count;
  float denom = (n * sum_xx - sum_x * sum_x);
  if (fabs(denom) < 1e-9f) slope = 0;
  else slope = (n * sum_xy - sum_x * sum_y) / denom;

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
