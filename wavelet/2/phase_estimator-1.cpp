#include "phase_estimator.h"
#include <string.h>

PhaseEstimator::PhaseEstimator() 
  : history_buffers(nullptr),
    correlation_buffer(nullptr),
    residual_buffer(nullptr),
    history_count(0),
    history_write_idx(0),
    current_state(PE_INITIALIZING),
    last_phase_shift(0.0f),
    expected_next_shift(0.0f),
    correction_cooldown(0),
    correction_was_applied(false),
    frames_since_correction(0),
    nominal_frequency(50.0f),
    buffer_time_interval(0.0f),
    last_freq_estimate(50.0f),
    samples_per_cycle(128),
    initialized(false) {
}

PhaseEstimator::~PhaseEstimator() {
  if (history_buffers) free(history_buffers);
  if (correlation_buffer) free(correlation_buffer);
  if (residual_buffer) free(residual_buffer);
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
  
  history_buffers = (uint16_t*)malloc(total_size * sizeof(uint16_t));
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
  expected_next_shift = 0.0f;
  correction_cooldown = 0;
  correction_was_applied = false;
  frames_since_correction = 0;
  last_freq_estimate = nominal_frequency;
}

void PhaseEstimator::set_frequency_params(float nominal_hz, float buffer_interval_s, uint16_t samps_per_cycle) {
  nominal_frequency = nominal_hz;
  buffer_time_interval = buffer_interval_s;
  samples_per_cycle = samps_per_cycle;
  last_freq_estimate = nominal_hz;
}

void PhaseEstimator::notify_correction_applied(float correction_rad) {
  correction_was_applied = true;
  frames_since_correction = 0;
  expected_next_shift = correction_rad; // Expected immediate change
}

void PhaseEstimator::add_frame(const uint16_t* buffer, uint16_t size) {
  if (!initialized || !buffer) return;
  
  uint16_t buf_size = PE_CYCLES_PER_BUFFER * PE_SAMPLES_PER_CYCLE;
  if (size != buf_size) return; // Size mismatch
  
  // Copy frame to history buffer
  uint16_t* dest = get_history_buffer(history_write_idx);
  memcpy(dest, buffer, buf_size * sizeof(uint16_t));
  
  // Update circular buffer index
  history_write_idx = (history_write_idx + 1) % config.history_depth;
  
  // Update count (saturate at depth)
  if (history_count < config.history_depth) {
    history_count++;
    if (history_count >= 3) {
      current_state = PE_READY; // Minimum data for estimation
    }
  }
  
  // Track frames since correction
  if (correction_was_applied) {
    frames_since_correction++;
  }
  
  // Decrement cooldown
  if (correction_cooldown > 0) correction_cooldown--;
}

float PhaseEstimator::compute_phase_shift(const uint16_t* reference, const uint16_t* target) {
  // Extract center cycle from reference (cycle 2 of 3 cycles)
  const uint16_t* ref_cycle = reference + PE_SAMPLES_PER_CYCLE; // Middle cycle
  
  // We'll search in the center cycle of target buffer for best match
  const uint16_t* search_cycle = target + PE_SAMPLES_PER_CYCLE;
  
  // Normalize reference wavelet
  float ref_mean = 0.0f;
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    ref_mean += ref_cycle[i];
  }
  ref_mean /= PE_SAMPLES_PER_CYCLE;
  
  float ref_std = 0.0f;
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    correlation_buffer[i] = ref_cycle[i] - ref_mean;
    ref_std += correlation_buffer[i] * correlation_buffer[i];
  }
  ref_std = sqrtf(ref_std / PE_SAMPLES_PER_CYCLE);
  if (ref_std < 1.0f) ref_std = 1.0f; // Prevent division by zero
  
  for (uint16_t i = 0; i < PE_SAMPLES_PER_CYCLE; i++) {
    correlation_buffer[i] /= ref_std;
  }
  
  // Sliding window correlation to find minimum residual (best match)
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
    
    if (residual_sum < min_residual) {
      min_residual = residual_sum;
      best_offset = offset;
    }
  }
  
  // Convert offset to phase (radians)
  // Positive offset means target is ahead of reference
  float phase_rad = (2.0f * M_PI * best_offset) / PE_SAMPLES_PER_CYCLE;
  
  // Normalize to [-π, π]
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
  
  // Simple linear regression
  float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
  
  for (uint8_t i = 0; i < count; i++) {
    float x = (float)i;
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
    float expected = intercept + slope * i;
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
  
  // Fit linear model
  float slope, intercept;
  fit_linear_drift(result.phase_trend, count, slope, intercept);
  result.linear_drift_rate = slope;
  
  // Calculate variance
  float variance = calculate_drift_variance(result.phase_trend, count, slope, intercept);
  result.drift_variance = variance;
  
  // Get most recent phase change
  float recent_change = 0.0f;
  if (count >= 2) {
    recent_change = result.phase_trend[count - 1] - result.phase_trend[count - 2];
  }
  
  // STATE MACHINE LOGIC
  
  // Check if we're tracking a correction we applied
  if (correction_was_applied && frames_since_correction <= 5) {
    // We applied a correction - check if it shows up in the trend
    float expected_change = expected_next_shift;
    float deviation = fabs(recent_change - expected_change);
    
    if (frames_since_correction >= 2) { // Give it time to show up
      if (deviation < config.correction_threshold_rad * 0.5f) {
        // Correction is effective - it shows up in trend as expected
        result.correction_effective = true;
        current_state = PE_STABLE; // Return to stable tracking
        correction_was_applied = false; // Clear flag
      } else {
        // Correction not yet visible or ineffective
        result.correction_effective = false;
        current_state = PE_CORRECTION_DETECTED; // Still tracking
      }
    } else {
      // Too early to tell
      result.correction_effective = false;
      current_state = PE_CORRECTION_DETECTED;
    }
    result.correction_magnitude = recent_change;
  }
  else {
    // Normal operation - no correction tracking
    result.correction_effective = false;
    
    // Detect unexpected corrections (should not happen if notify is used properly)
    float expected_change = slope; // Expected change based on linear drift
    float deviation = fabs(recent_change - expected_change);
    
    if (correction_cooldown == 0 && deviation > config.correction_threshold_rad) {
      // Large sudden change detected - unexpected correction or disturbance
      current_state = PE_CORRECTION_DETECTED;
      result.correction_magnitude = recent_change;
      correction_cooldown = 3;
      expected_next_shift = recent_change + slope;
    }
    else if (variance > config.nonlinear_threshold_rad * config.nonlinear_threshold_rad) {
      // High variance - non-linear drift (signal changes, noise, etc.)
      current_state = PE_NONLINEAR_DRIFT;
    }
    else if (fabs(slope) < config.stable_tolerance_rad && 
             variance < config.stable_tolerance_rad * config.stable_tolerance_rad) {
      // Low slope and low variance - stable lock
      current_state = PE_STABLE;
    }
    else {
      // Stable linear drift
      current_state = PE_STABLE;
    }
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
  const uint16_t* reference = get_history_buffer(ref_idx);
  
  // Calculate phase shifts for all historical buffers (oldest to newest, excluding reference)
  uint8_t valid_count = 0;
  
  for (uint16_t i = 0; i < history_count - 1; i++) {
    // Calculate historical index (going backwards from reference)
    uint16_t hist_offset = history_count - 1 - i; // 1, 2, 3, ... (newest to oldest)
    uint16_t hist_idx = (history_write_idx + config.history_depth - hist_offset - 1) % config.history_depth;
    
    const uint16_t* target = get_history_buffer(hist_idx);
    
    // Compute phase shift between reference and this historical buffer
    float phase_shift = compute_phase_shift(reference, target);
    
    // Store in result (reverse order so [0] is oldest)
    result.phase_trend[history_count - 2 - i] = phase_shift;
    valid_count++;
  }
  
  result.valid_samples = valid_count;
  result.recent_phase_shift = (valid_count > 0) ? result.phase_trend[valid_count - 1] : 0.0f;
  
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
  if (!initialized || history_count < 5 || buffer_time_interval <= 0.0f) {
    result.valid = false;
    return false;
  }
  
  // Clear result
  memset(&result, 0, sizeof(result));
  result.frequency_hz = nominal_frequency;
  result.valid = false;
  
  // Only estimate frequency in stable conditions
  if (current_state != PE_STABLE && current_state != PE_READY) {
    return false;
  }
  
  // Use phase trend to estimate frequency error
  // Phase accumulates over time due to frequency mismatch
  // phase_drift_rate (rad/buffer) = 2π × freq_error × buffer_interval
  // freq_error = phase_drift_rate / (2π × buffer_interval)
  
  // Get recent trend for frequency estimation (use last 8 samples for good balance)
  uint8_t trend_length = (history_count - 1 < 8) ? (history_count - 1) : 8;
  uint8_t start_idx = (history_count - 1) - trend_length;
  
  // Create subset of phase trend for frequency estimation
  float phase_trend_subset[8];
  for (uint8_t i = 0; i < trend_length; i++) {
    phase_trend_subset[i] = last_phase_shift; // Initialize (will be computed properly below)
  }
  
  // Compute phase differences (phase accumulation over time)
  // We need to look at the total phase accumulation, not individual shifts
  float total_phase_accumulation = 0.0f;
  float time_span = trend_length * buffer_time_interval;
  
  // Calculate cumulative phase drift over the trend window
  // Use linear fit slope which represents rad/buffer
  float slope, intercept;
  fit_linear_drift(phase_trend_subset, trend_length, slope, intercept);
  
  // The slope tells us how many radians of phase drift per buffer
  // Convert to frequency error: freq_error = phase_rate / (2π × time_per_buffer)
  float phase_drift_rate_rad_per_sec = slope / buffer_time_interval;
  float freq_error_hz = phase_drift_rate_rad_per_sec / (2.0f * M_PI);
  
  // Estimated actual frequency
  float estimated_freq = nominal_frequency + freq_error_hz;
  
  // Calculate variance for confidence estimation
  float variance = calculate_drift_variance(phase_trend_subset, trend_length, slope, intercept);
  
  // Confidence based on variance (lower variance = higher confidence)
  // Also consider how linear the trend is
  float confidence = 1.0f / (1.0f + variance * 50.0f); // Scaling factor tuned for typical variance
  confidence = fmaxf(0.0f, fminf(1.0f, confidence)); // Clamp to [0,1]
  
  // Sanity checks
  bool freq_reasonable = (fabs(freq_error_hz) < 5.0f); // Max ±5 Hz error seems reasonable
  bool confidence_ok = (confidence > 0.2f); // Minimum confidence threshold
  bool enough_data = (trend_length >= 4);
  
  if (freq_reasonable && confidence_ok && enough_data) {
    result.frequency_hz = estimated_freq;
    result.frequency_error_hz = freq_error_hz;
    result.confidence = confidence;
    result.valid = true;
    
    // Recommended PLL correction
    // If grid is faster than nominal (positive error), PLL needs to speed up
    result.pll_correction_hz = freq_error_hz;
    
    last_freq_estimate = estimated_freq;
  } else {
    // Fall back to last estimate if current one fails checks
    result.frequency_hz = last_freq_estimate;
    result.frequency_error_hz = last_freq_estimate - nominal_frequency;
    result.confidence = 0.0f;
    result.valid = false;
    result.pll_correction_hz = 0.0f;
  }
  
  return result.valid;
}

