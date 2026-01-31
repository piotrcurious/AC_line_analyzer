# Phase Estimator Documentation

## Overview

The Phase Estimator is a wavelet-based phase analysis system designed for grid synchronization and phase-locked loop (PLL) correction. It captures historical signal buffers, performs wavelet correlation analysis, and provides intelligent state machine logic to distinguish between:

1. **Stable linear phase drift** - Due to slight PLL frequency mismatch
2. **Intentional phase corrections** - Applied by external control logic
3. **Non-linear phase changes** - Due to signal variations or noise

## Architecture

### Key Components

1. **History Buffer**: Stores up to 16 (configurable) complete waveform captures
2. **Wavelet Correlator**: Uses sliding window correlation to find phase shifts
3. **State Machine**: Classifies phase behavior and guides control decisions
4. **Trend Analyzer**: Fits linear models and detects anomalies

### Data Flow

```
ADC Samples → Buffer → Phase Estimator → Analysis → State Machine → Control Decision
                ↓
          History Storage
          (16 buffers)
```

## Wavelet Analysis Method

### Reference Wavelet Extraction
- Extracts the **center cycle** (cycle 2 of 3) from the most recent buffer
- This becomes the "wavelet template" to search for in historical data

### Phase Shift Calculation
For each historical buffer:
1. Extract center cycle as search target
2. Normalize both reference and target (zero mean, unit variance)
3. Perform sliding window correlation across all possible phase shifts (0-360°)
4. Find offset with **minimum residual** (best match)
5. Convert offset to phase in radians

### Mathematical Formula
```
phase_shift = (2π × best_offset) / SAMPLES_PER_CYCLE
```

The phase shift represents how much the historical signal leads/lags the current reference.

## State Machine

### States

#### 1. PE_INITIALIZING
- **Condition**: Less than 3 buffers collected
- **Behavior**: No control action, accumulating data
- **Transition**: Automatically moves to PE_READY when 3+ buffers available

#### 2. PE_STABLE
- **Condition**: 
  - Low drift variance (< threshold²)
  - Consistent linear trend
- **Behavior**: Apply corrections based on linear drift rate
- **Control**: `phase_correction = -linear_drift_rate`

#### 3. PE_CORRECTION_DETECTED
- **Condition**: 
  - Sudden phase jump > correction_threshold (default: 0.3 rad / 17°)
  - Deviation from expected linear trend
- **Behavior**: 
  - **DO NOT apply counter-correction** (would fight the intentional correction)
  - Enter cooldown period (3 frames)
  - Update expected drift model
- **Control**: `phase_correction = 0` (no action)

#### 4. PE_NONLINEAR_DRIFT
- **Condition**: High variance in phase trend (signal changes, noise)
- **Behavior**: Conservative correction to avoid instability
- **Control**: `phase_correction = -recent_shift × 0.5` (gentle damping)

#### 5. PE_READY
- **Condition**: Generic ready state with insufficient data for classification
- **Behavior**: No control action
- **Control**: `phase_correction = 0`

### State Transitions

```
INITIALIZING → READY (when 3+ buffers)
READY → STABLE (low variance, linear trend)
STABLE → CORRECTION_DETECTED (sudden jump > threshold)
STABLE → NONLINEAR_DRIFT (high variance)
CORRECTION_DETECTED → STABLE (after cooldown)
NONLINEAR_DRIFT → STABLE (variance decreases)
```

## Configuration Parameters

```cpp
struct PhaseEstConfig {
  uint16_t history_depth;          // Default: 16 buffers
  float correction_threshold_rad;  // Default: 0.3 rad (17°)
  float nonlinear_threshold_rad;   // Default: 0.1 rad (5.7°)
  float stable_tolerance_rad;      // Default: 0.02 rad (1.1°)
};
```

### Parameter Tuning

- **history_depth**: More history = better trend analysis but slower response
- **correction_threshold_rad**: Higher = less sensitive to corrections
- **nonlinear_threshold_rad**: Controls transition to nonlinear state
- **stable_tolerance_rad**: Tighter = stricter stability requirements

## Usage Example

```cpp
// Initialize
PhaseEstimator phase_est;
PhaseEstConfig config;
config.history_depth = 16;
config.correction_threshold_rad = 0.3f;
phase_est.begin(&config);

// In main loop after capturing buffer
uint16_t raw_buffer[BUFFER_SIZE];
// ... fill buffer with ADC samples ...

phase_est.add_frame(raw_buffer, BUFFER_SIZE);

PhaseEstResult result;
if (phase_est.estimate_phase(result)) {
  switch(result.state) {
    case PE_STABLE:
      // Compensate linear drift
      apply_phase_correction(-result.linear_drift_rate);
      break;
      
    case PE_CORRECTION_DETECTED:
      // Don't fight the correction!
      apply_phase_correction(0.0f);
      break;
      
    case PE_NONLINEAR_DRIFT:
      // Gentle correction
      apply_phase_correction(-result.recent_phase_shift * 0.5f);
      break;
  }
}
```

## Phase Correction Logic

### Critical Design Principle

**NEVER apply counter-correction when PE_CORRECTION_DETECTED state is active!**

This is essential because:
1. External control logic may apply intentional phase corrections
2. Immediately counter-correcting would undo the intended correction
3. This creates a "fight" between systems

### Proper Handling

```cpp
if (state == PE_CORRECTION_DETECTED) {
  // Correction was intentional - acknowledge and adapt
  phase_err_rad = 0.0f;
  
  // Update drift model to new baseline
  expected_next_shift = correction_magnitude + linear_drift_rate;
  
  // Wait cooldown period before new corrections
  cooldown = 3; // frames
}
```

## Output Data Structure

```cpp
struct PhaseEstResult {
  PhaseEstState state;                      // Current state
  float phase_trend[PE_HISTORY_DEPTH];     // Phase shifts (rad) for each buffer
  float linear_drift_rate;                  // Slope of linear fit (rad/buffer)
  float recent_phase_shift;                 // Most recent shift (rad)
  uint8_t valid_samples;                    // Number of valid trend entries
  bool correction_detected;                 // True if correction detected
  float correction_magnitude;               // Size of correction (rad)
  float drift_variance;                     // Variance in drift (nonlinearity)
};
```

## Performance Characteristics

### Memory Usage
- History buffers: `16 × 3 × 128 × 2 bytes = 12,288 bytes`
- Working buffers: `~512 bytes`
- **Total: ~13 KB RAM**

### Computation Time
- Phase shift calculation: O(N²) where N = SAMPLES_PER_CYCLE
- Per buffer comparison: ~128 × 128 = 16,384 operations
- For 16 buffers: ~260,000 operations
- **Estimated: 5-10ms on ESP32 @ 240MHz**

### Recommendations
- Run phase estimation every 10-20 frames (not every frame)
- Use buffer decimation for faster processing if needed
- Increase correction_threshold if false detections occur

## Troubleshooting

### Problem: False correction detections
**Solution**: Increase `correction_threshold_rad` to 0.4-0.5 rad

### Problem: System oscillates
**Solution**: 
- Verify cooldown mechanism is working
- Reduce correction gains in nonlinear state
- Check for proper state transition logic

### Problem: Poor phase tracking
**Solution**:
- Increase history depth for better trend fitting
- Verify signal quality (adequate SNR)
- Check that center cycle extraction is aligned properly

### Problem: High CPU usage
**Solution**:
- Reduce analysis frequency (every N frames)
- Reduce history depth
- Implement correlation decimation

## Future Enhancements

1. **Adaptive thresholds**: Adjust thresholds based on signal quality
2. **Multi-scale wavelet**: Use multiple cycle lengths for robustness
3. **Kalman filtering**: Apply to phase trend for noise reduction
4. **Frequency estimation**: Extract frequency drift from phase trend
5. **FFT-based correlation**: Faster phase shift calculation using FFT

## References

- Wavelet correlation: Cross-correlation in time domain
- Linear regression: Least squares fitting for drift rate
- State machine: Finite state automaton for control logic
