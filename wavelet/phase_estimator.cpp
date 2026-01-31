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
  
  // Check for phase correction (sudden jump outside normal drift pattern)
  float expected_change = slope; // Expected change based on linear drift
  float deviation = fabs(recent_change - expected_change);
  
  if (correction_cooldown == 0 && deviation > config.correction_threshold_rad) {
    // Large sudden change detected - likely intentional correction
    current_state = PE_CORRECTION_DETECTED;
    result.correction_detected = true;
    result.correction_magnitude = recent_change;
    correction_cooldown = 3; // Wait a few frames before next detection
    expected_next_shift = recent_change + slope; // Update expectation
  }
  else if (variance > config.nonlinear_threshold_rad * config.nonlinear_threshold_rad) {
    // High variance - non-linear drift (signal changes, noise, etc.)
    current_state = PE_NONLINEAR_DRIFT;
    result.correction_detected = false;
  }
  else if (fabs(slope) < config.stable_tolerance_rad && 
           variance < config.stable_tolerance_rad * config.stable_tolerance_rad) {
    // Low slope and low variance - stable lock
    current_state = PE_STABLE;
    result.correction_detected = false;
  }
  else {
    // Stable linear drift
    current_state = PE_STABLE;
    result.correction_detected = false;
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
  
  return true;
}
