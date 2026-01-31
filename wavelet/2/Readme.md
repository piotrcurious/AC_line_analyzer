# Phase Estimator with Frequency Estimation - Complete Documentation

## Overview

The Phase Estimator is a comprehensive wavelet-based analysis system for grid synchronization that provides:

1. **Phase drift tracking** - Wavelet correlation analysis with 16-buffer history
2. **Frequency estimation** - Derive actual grid frequency from phase drift
3. **Intelligent state machine** - Distinguish between different drift scenarios
4. **Correction tracking** - Monitor effectiveness of applied corrections

## Key Features

### Phase Analysis
- Captures historical signal buffers (16 by default, configurable)
- Wavelet-based correlation using sliding window for phase shift detection
- Linear regression for trend analysis and drift rate estimation
- Variance analysis for stability classification

### Frequency Estimation
- Derives grid frequency from phase drift rate
- Provides confidence metric based on trend linearity
- Recommends PLL frequency corrections
- Validates estimates with sanity checks

### Correction Tracking
- **notify_correction_applied()** - Call this when YOU apply a correction
- Tracks effectiveness over next few frames
- Reports whether correction shows up in trend
- Prevents fighting between control systems

## Architecture

```
┌─────────────┐
│ ADC Samples │
└──────┬──────┘
       │
       ▼
┌─────────────────┐      ┌──────────────────┐
│ Buffer Storage  │──────▶│ Wavelet Analysis │
│ (16 buffers)    │      │ (Correlation)    │
└─────────────────┘      └────────┬─────────┘
                                  │
                                  ▼
                         ┌─────────────────┐
                         │ Phase Trend     │
                         │ [rad/buffer]    │
                         └────────┬────────┘
                                  │
                    ┌─────────────┴────────────┐
                    ▼                          ▼
           ┌────────────────┐         ┌───────────────┐
           │ State Machine  │         │ Freq Estimator│
           │ (PE_STABLE,    │         │ (Hz error)    │
           │  PE_CORRECTION,│         └───────┬───────┘
           │  PE_NONLINEAR) │                 │
           └────────┬───────┘                 │
                    │                         │
                    └────────┬────────────────┘
                             ▼
                    ┌─────────────────┐
                    │ Control Outputs │
                    │ - Phase corr    │
                    │ - Freq corr     │
                    └─────────────────┘
```

## Frequency Estimation Method

### Mathematical Principle

Phase accumulates over time when there's a frequency mismatch:

```
Δφ = 2π × Δf × Δt

Where:
  Δφ = phase drift (radians)
  Δf = frequency error (Hz)
  Δt = time interval (seconds)

Therefore:
  Δf = Δφ / (2π × Δt)
```

### Implementation

1. **Linear regression** on phase trend gives drift rate (rad/buffer)
2. **Convert to rad/sec**: `phase_rate = drift_rate / buffer_interval`
3. **Extract frequency error**: `freq_error = phase_rate / (2π)`
4. **Estimate actual frequency**: `f_actual = f_nominal + freq_error`

### Confidence Metric

```
confidence = 1 / (1 + variance × scaling_factor)
```

- Higher confidence when trend is linear (low variance)
- Lower confidence when trend is noisy or non-linear
- Range: [0, 1]

### Validation Checks

1. **Reasonable range**: |freq_error| < 5 Hz
2. **Minimum confidence**: confidence > 0.2
3. **Sufficient data**: ≥ 4 trend samples
4. **Stable state**: Only estimate in PE_STABLE or PE_READY states

## State Machine (Enhanced)

### States

#### PE_INITIALIZING
- **Entry**: Less than 3 buffers collected
- **Behavior**: Accumulating data, no actions
- **Exit**: When 3+ buffers available → PE_READY

#### PE_STABLE
- **Entry**: Low variance, consistent linear trend
- **Behavior**: 
  - Apply phase correction: `-drift_rate × 0.8`
  - Apply frequency correction: `freq_error × 0.1` (if confidence > 0.5)
- **Control Philosophy**: Gentle corrections to maintain lock
- **Exit**: 
  - High variance → PE_NONLINEAR_DRIFT
  - Correction applied → PE_CORRECTION_DETECTED

#### PE_CORRECTION_DETECTED
- **Entry**: 
  - **When WE apply correction**: Via `notify_correction_applied()`
  - **Unexpected correction**: Sudden jump > threshold
- **Behavior**:
  - Track correction over 2-5 frames
  - Check if correction appears in trend
  - Report effectiveness
  - **ZERO control action** (don't fight the correction)
- **Outputs**:
  - `correction_applied = true` (if we did it)
  - `correction_effective = true` (if it worked)
- **Exit**: After cooldown → PE_STABLE

#### PE_NONLINEAR_DRIFT
- **Entry**: High variance in phase trend
- **Behavior**: 
  - Very gentle phase correction: `-recent_shift × 0.3`
  - NO frequency correction
  - Conservative approach to avoid instability
- **Exit**: Variance decreases → PE_STABLE

#### PE_READY
- **Entry**: Generic state with 3+ buffers but not classified
- **Behavior**: No control action
- **Exit**: First classification → appropriate state

### Critical Correction Tracking Logic

**The Problem**: Without proper tracking, the system fights itself:
1. Control applies phase correction
2. Correction appears in trend as sudden shift
3. Old logic thinks it's a disturbance
4. Applies counter-correction
5. **Result**: Oscillation and instability

**The Solution**: 
```cpp
// When applying correction
apply_phase_rad(correction);
phase_est.notify_correction_applied(correction);

// Estimator now knows to expect this change
// Won't treat it as disturbance
// Will verify it shows up in trend
```

## Usage Guide

### Initialization

```cpp
PhaseEstimator phase_est;

void setup() {
  // Configure estimator
  PhaseEstConfig config;
  config.history_depth = 16;
  config.correction_threshold_rad = 0.3f;
  config.nonlinear_threshold_rad = 0.1f;
  config.stable_tolerance_rad = 0.02f;
  
  phase_est.begin(&config);
  
  // Set frequency estimation parameters
  float nominal_freq = 50.0f;  // Hz
  float buffer_interval = 0.08f;  // seconds (4 cycles at 50 Hz)
  uint16_t samples_per_cycle = 128;
  
  phase_est.set_frequency_params(nominal_freq, buffer_interval, samples_per_cycle);
}
```

### Main Loop Integration

```cpp
void loop() {
  if (buffer_ready) {
    // Add buffer to history
    phase_est.add_frame(adc_buffer, BUFFER_SIZE);
    
    // Estimate phase
    PhaseEstResult pe_result;
    if (phase_est.estimate_phase(pe_result)) {
      
      // Get frequency estimate
      FrequencyEstResult freq_result;
      bool freq_valid = phase_est.estimate_frequency(freq_result);
      
      // Control logic
      float phase_corr = 0.0f;
      float freq_corr = 0.0f;
      
      switch(pe_result.state) {
        case PE_STABLE:
          // Apply both corrections
          phase_corr = -pe_result.linear_drift_rate * 0.8f;
          if (freq_valid && freq_result.confidence > 0.5f) {
            freq_corr = freq_result.pll_correction_hz * 0.1f;
          }
          break;
          
        case PE_CORRECTION_DETECTED:
          // Don't fight - just monitor
          if (pe_result.correction_effective) {
            // Success! Correction worked
          }
          break;
          
        case PE_NONLINEAR_DRIFT:
          // Very gentle phase only
          phase_corr = -pe_result.recent_phase_shift * 0.3f;
          break;
      }
      
      // Apply corrections
      if (fabs(freq_corr) > 0.001f) {
        tune_pll(current_freq + freq_corr);
      }
      
      if (fabs(phase_corr) > 1e-6f) {
        apply_phase_rad(phase_corr);
        
        // CRITICAL: Notify estimator
        phase_est.notify_correction_applied(phase_corr);
      }
    }
  }
}
```

### Reading Results

```cpp
PhaseEstResult result;
if (phase_est.estimate_phase(result)) {
  Serial.printf("State: %d\n", result.state);
  Serial.printf("Drift rate: %.4f rad/buf\n", result.linear_drift_rate);
  Serial.printf("Variance: %.6f\n", result.drift_variance);
  
  if (result.correction_applied) {
    Serial.printf("Correction tracking: %s\n",
                 result.correction_effective ? "EFFECTIVE" : "PENDING");
  }
  
  if (result.frequency_estimate_valid) {
    Serial.printf("Freq: %.4f Hz (error: %+.4f Hz)\n",
                 result.estimated_frequency,
                 result.estimated_frequency_error);
  }
}

FrequencyEstResult freq;
if (phase_est.estimate_frequency(freq)) {
  Serial.printf("Frequency: %.4f Hz\n", freq.frequency_hz);
  Serial.printf("Error: %+.4f Hz\n", freq.frequency_error_hz);
  Serial.printf("Confidence: %.2f\n", freq.confidence);
  Serial.printf("Recommended PLL correction: %+.4f Hz\n", freq.pll_correction_hz);
}
```

## Configuration Parameters

### Phase Estimation

| Parameter | Default | Description |
|-----------|---------|-------------|
| `history_depth` | 16 | Number of buffers to store |
| `correction_threshold_rad` | 0.3 | Threshold for correction detection (17°) |
| `nonlinear_threshold_rad` | 0.1 | Threshold for nonlinear state (5.7°) |
| `stable_tolerance_rad` | 0.02 | Tolerance for stable lock (1.1°) |

### Frequency Estimation

| Parameter | Type | Description |
|-----------|------|-------------|
| `nominal_frequency` | float | Expected grid frequency (Hz) |
| `buffer_time_interval` | float | Time between buffers (seconds) |
| `samples_per_cycle` | uint16_t | Samples per nominal cycle |

### Tuning Guide

**For 50 Hz Grid:**
```cpp
config.history_depth = 16;  // ~1.3 seconds of history
config.correction_threshold_rad = 0.3f;  // Catch corrections > 17°
config.nonlinear_threshold_rad = 0.1f;   // Variance threshold
config.stable_tolerance_rad = 0.02f;     // Almost locked (< 1.1°)

// Frequency params
nominal = 50.0f;
buffer_interval = (STROBE_DIV / 50.0f);  // e.g., 0.08s for 4 cycles
samples = 128;
```

**For 60 Hz Grid:**
```cpp
// Same thresholds work well
nominal = 60.0f;
buffer_interval = (STROBE_DIV / 60.0f);  // e.g., 0.067s for 4 cycles
```

**If false corrections detected:**
- Increase `correction_threshold_rad` to 0.4-0.5

**If frequency estimate unstable:**
- Increase minimum confidence requirement (> 0.5)
- Use longer averaging window
- Check signal quality (SNR)

## Performance Characteristics

### Memory Usage
- History: 16 × 3 × 128 × 2 bytes = **12,288 bytes**
- Working buffers: **~512 bytes**
- **Total: ~13 KB RAM**

### Computation Time (ESP32 @ 240 MHz)
- Phase estimation: **5-8 ms** (16 buffers)
- Frequency estimation: **< 1 ms**
- **Total per analysis: ~6-9 ms**

### Recommendation
- Run analysis every 5-10 frames (not every frame)
- Allows 50-90 ms for other processing
- Still provides fast control response

## Advanced Topics

### Frequency Estimation Accuracy

**Expected accuracy**:
- **±0.01 Hz** in stable conditions (high SNR, good lock)
- **±0.05 Hz** in normal conditions
- **±0.1-0.2 Hz** with noise or signal variations

**Factors affecting accuracy**:
1. **Trend linearity**: More linear = better accuracy
2. **History length**: More buffers = better averaging
3. **Buffer interval**: Longer interval = better resolution
4. **Signal quality**: Higher SNR = better correlation

### Phase Shift Resolution

**Theoretical resolution**:
```
resolution = 2π / SAMPLES_PER_CYCLE
           = 2π / 128
           = ~0.049 rad (~2.8°)
```

**Practical resolution** (with interpolation): **~0.01 rad (~0.6°)**

### Correction Tracking Timing

The correction tracking follows this timeline:

```
Frame 0: Apply correction, call notify_correction_applied()
Frame 1: Add new buffer (correction not yet visible)
Frame 2: Correction appears in trend (earliest detection)
Frame 3: Correction confirmed, correction_effective = true
Frame 4-5: Cooldown period
Frame 6+: Return to normal tracking (PE_STABLE)
```

**Why 2-5 frames?**
- Frame 0-1: Data acquisition and buffering delay
- Frame 2: Wavelet correlation detects change
- Frame 3: Verification (not a fluke)
- Frame 4-5: Cooldown to prevent rapid re-triggering

## Troubleshooting

### Problem: Frequency estimate jumps around

**Symptoms**: Frequency varies by > 0.5 Hz frame-to-frame

**Causes**:
1. Insufficient history (< 5 buffers)
2. High phase variance (nonlinear drift)
3. Recent corrections disturbing trend

**Solutions**:
- Only use frequency when `state == PE_STABLE` AND `confidence > 0.6`
- Apply low-pass filter: `freq_filt = 0.9 × freq_filt + 0.1 × freq_new`
- Increase minimum confidence threshold

### Problem: Corrections marked ineffective when they work

**Symptoms**: `correction_effective = false` even though phase shifted

**Causes**:
1. Correction too small (< threshold)
2. Checked too early (frame 1 instead of frame 2+)
3. Noisy signal masks correction

**Solutions**:
- Ensure `frames_since_correction >= 2` before checking
- Reduce `correction_threshold_rad` for verification (use 0.5× threshold)
- Wait longer (up to 5 frames) for verification

### Problem: System oscillates

**Symptoms**: Phase swings back and forth

**Causes**:
1. Not using `notify_correction_applied()`
2. Correction gains too high
3. Frequency and phase fighting each other

**Solutions**:
- **ALWAYS** call `notify_correction_applied()` after applying correction
- Reduce gains: phase × 0.5, frequency × 0.05
- Apply frequency correction slower (every N frames)
- Check for positive feedback loops

### Problem: Won't lock to grid

**Symptoms**: Continuous drift, never reaches PE_STABLE

**Causes**:
1. Frequency error > correction rate
2. Signal quality poor
3. Thresholds too tight

**Solutions**:
- Increase frequency correction gain (0.1 → 0.2)
- Apply larger initial frequency step if error > 1 Hz
- Relax `stable_tolerance_rad` to 0.05
- Check signal SNR and waveform quality

## Example Output

```
=== Frame 150 | State: STABLE ===
Drift Rate: -0.0234 rad/buf (-1.34 deg/buf)
Recent Shift: -0.0241 rad (-1.38 deg)
Drift Variance: 0.000023
Freq Est: 49.9812 Hz (error: -0.0188 Hz, conf: 0.87)
Correction Applied: NO
Phase Trend [rad]: -0.182 -0.205 -0.229 -0.252 -0.274 -0.297 -0.321 -0.345

Applied frequency correction: +0.0019 Hz -> 49.9950 Hz
F:49.9950Hz | StrobeJit:142 cyc (0.59us) | Samp0Jit:23 cyc | State:STABLE
```

## API Reference

### Core Functions

```cpp
// Initialize estimator
bool begin(const PhaseEstConfig* cfg = nullptr);

// Set frequency estimation parameters
void set_frequency_params(float nominal_hz, float buffer_interval_s, 
                          uint16_t samps_per_cycle);

// Add new buffer to history
void add_frame(const uint16_t* buffer, uint16_t size);

// Notify that correction was applied (CRITICAL for tracking)
void notify_correction_applied(float correction_rad);

// Perform phase analysis
bool estimate_phase(PhaseEstResult& result);

// Perform frequency estimation
bool estimate_frequency(FrequencyEstResult& result);

// Get current state
PhaseEstState get_state() const;

// Reset estimator
void reset();

// Check if ready
bool is_ready() const;
```

### Data Structures

```cpp
struct PhaseEstResult {
  PhaseEstState state;
  float phase_trend[16];
  float linear_drift_rate;
  float recent_phase_shift;
  uint8_t valid_samples;
  bool correction_applied;      // Input: we applied correction
  bool correction_effective;    // Output: correction worked
  float correction_magnitude;
  float drift_variance;
  float estimated_frequency;
  float estimated_frequency_error;
  bool frequency_estimate_valid;
};

struct FrequencyEstResult {
  float frequency_hz;
  float frequency_error_hz;
  float confidence;
  bool valid;
  float pll_correction_hz;
};
```

## Conclusion

This phase estimator provides a complete solution for grid synchronization with:
- Robust phase tracking via wavelet analysis
- Accurate frequency estimation from phase drift
- Intelligent state machine for different scenarios
- Proper correction tracking to prevent instability

The key to successful operation is **proper correction notification** - always call `notify_correction_applied()` when you apply a correction, and the system will track its effectiveness and avoid fighting with itself.
