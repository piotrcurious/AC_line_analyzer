# Phase Estimator v3 - Fixed for Rapid Frequency Changes

## Critical Fixes Applied

### 1. **Phase Calculation Corrected**
**Problem**: Phase trend showed jumps and wasn't accumulating correctly
**Fix**: Properly calculate cumulative phase error going backwards in time

```cpp
// OLD (wrong): compared historical to reference, stored randomly
// NEW (correct): track cumulative phase drift, unwrap, store chronologically
for (i = 0; i < history_count - 1; i++) {
  hist_offset = i + 1;  // 1 = newest, N = oldest
  accumulated_phase = compute_phase_shift(reference, target);
  // Unwrap discontinuities
  // Store chronologically: [0] = oldest
}
```

### 2. **Simplified Correction Tracking**
**Old behavior**: Complex validation logic, tried to verify if correction "worked"
**New behavior**: Simple measurement and reporting

```cpp
if (correction_was_applied && frames_since_correction >= 2) {
  result.correction_magnitude = actual_phase_change_measured;
  result.correction_effective = true;  // Just report it
  // Clear after 3 frames
}
```

**Purpose**: Provides feedback on correction magnitude for adaptive gain tuning

### 3. **Removed PE_CORRECTION_DETECTED State**
**Why**: Was causing system to get stuck, not applying corrections during tracking
**Result**: Only two active states:
- **PE_STABLE**: Linear phase trend (low variance)
- **PE_NONLINEAR_DRIFT**: Non-linear trend (high variance) = frequency changing

### 4. **Aggressive Gains for Rapid Convergence**

| State | Phase Gain | Freq Gain | Use Case |
|-------|------------|-----------|----------|
| PE_STABLE | 0.9 | 0.25 | Fast correction when locked |
| PE_NONLINEAR | 0.4 | 0.30 | **Aggressive freq tracking** for changes |
| PE_READY | 0.5 | 0.15 | Startup |

**Key**: NONLINEAR gets highest freq gain (0.30) because that's when frequency is changing!

### 5. **Increased Frequency Error Limits**
Changed from ±5 Hz to **±15 Hz** to handle:
- 50 Hz → 51 Hz (+1 Hz)
- 51 Hz → 49 Hz (-2 Hz)
- Trapezoid (varying)

## Expected Behavior with Test Signal

### Test Signal Pattern
```
Second 0-1:   50 Hz (nominal)
Second 1-2:   51 Hz (+1 Hz change)
Second 2-3:   49 Hz (-2 Hz change)
Second 3-4:   Trapezoid (varying frequency)
Repeat...
```

### System Response Expectations

#### Phase 1: 50 Hz (Startup)
```
Frame 1-10:  State: PE_INITIALIZING → PE_READY → PE_STABLE
             PLL: 50.00 Hz → 50.00 Hz
             Phase drift: ~0 rad/buf
             Variance: Low
             Action: Locking to nominal
```

#### Phase 2: Transition to 51 Hz
```
Frame 11-15: State: PE_NONLINEAR (variance spikes)
             PLL: 50.00 Hz
             Grid: 51.00 Hz (detected)
             Error: +1.00 Hz
             Action: freq_corr = +1.00 × 0.30 = +0.30 Hz
             PLL: 50.00 → 50.30 Hz

Frame 16-20: State: PE_NONLINEAR → PE_STABLE
             PLL: 50.30 Hz
             Grid: 51.00 Hz
             Error: +0.70 Hz
             Action: freq_corr = +0.70 × 0.30 = +0.21 Hz
             PLL: 50.30 → 50.51 Hz

Frame 21-25: State: PE_STABLE
             PLL: 50.51 Hz
             Grid: 51.00 Hz
             Error: +0.49 Hz
             Action: freq_corr = +0.49 × 0.25 = +0.12 Hz
             PLL: 50.51 → 50.63 Hz

Frame 26-30: Continuing to converge...
             PLL: 50.75 Hz → 50.85 Hz → 50.93 Hz → 50.98 Hz
```

**Convergence time**: 15-20 frames (~1.2-1.6 seconds) to get within 0.1 Hz

#### Phase 3: Transition to 49 Hz
```
Frame 31-35: State: PE_NONLINEAR (sudden -2 Hz change detected)
             PLL: 50.98 Hz
             Grid: 49.00 Hz
             Error: -1.98 Hz
             Action: freq_corr = -1.98 × 0.30 = -0.59 Hz
             PLL: 50.98 → 50.39 Hz

Frame 36-50: Similar rapid convergence
             PLL: 50.39 → 49.80 → 49.45 → 49.22 → 49.08 → 49.02
```

#### Phase 4: Trapezoid (Varying Frequency)
```
State: PE_NONLINEAR throughout
Action: Continuous aggressive frequency tracking
Variance: High (non-linear phase accumulation)
PLL: Tracks frequency ramp up/down with some lag
```

## Diagnostic Checks

### 1. Phase Trend Should Be Smooth
```
Good: -1.234 -0.987 -0.745 -0.498 -0.251 -0.008  (linear decrease)
Good: 0.123 0.456 0.789 1.122 1.455 1.788        (linear increase)
Bad:  -1.234 -0.987 2.145 -2.847 0.123 -1.456    (jumps = broken unwrap)
```

### 2. Frequency Estimate Should Track Signal
```
At 50 Hz:  Est = 50.00 ± 0.1 Hz
At 51 Hz:  Est = 51.00 ± 0.1 Hz (after convergence)
At 49 Hz:  Est = 49.00 ± 0.1 Hz (after convergence)
```

### 3. State Transitions
```
Stable frequency → PE_STABLE (low variance)
Frequency change → PE_NONLINEAR (high variance) → PE_STABLE (after converge)
```

### 4. Variance During Transitions
```
Stable: Variance < 0.01 (threshold = 0.1² = 0.01)
Change: Variance > 0.01 → triggers NONLINEAR
```

## Tuning for Different Test Patterns

### If Convergence Too Slow
```cpp
// Increase frequency gain
case PE_STABLE:     freq_gain = 0.35f;  // was 0.25
case PE_NONLINEAR:  freq_gain = 0.40f;  // was 0.30
```

### If System Oscillates
```cpp
// Decrease gains
case PE_STABLE:     freq_gain = 0.15f;
                    phase_gain = 0.6f;
case PE_NONLINEAR:  freq_gain = 0.20f;
```

### If False NONLINEAR Triggers
```cpp
// Increase variance threshold
config.nonlinear_threshold_rad = 0.15f;  // was 0.1
// OR
config.nonlinear_threshold_rad = 0.20f;  // very tolerant
```

## Debugging Output Interpretation

### Good Convergence Example (50 → 51 Hz)
```
Frame 10: State: STABLE | PLL: 50.00 Hz | Est: 50.02 Hz | Err: +0.02 Hz
Phase trend: -0.123 -0.098 -0.073 -0.048 -0.023 0.002 0.027 0.052

[Frequency changes to 51 Hz]

Frame 15: State: NONLINEAR | PLL: 50.05 Hz | Est: 51.23 Hz | Err: +1.18 Hz
Phase trend: 0.052 0.234 0.567 1.123 1.876 2.534
Freq: 50.05 Hz | Corr: +0.35 Hz | Err: +1.18 Hz | Conf: 0.70

Frame 20: State: NONLINEAR | PLL: 50.40 Hz | Est: 51.12 Hz | Err: +0.72 Hz
Phase trend: 1.234 1.876 2.345 2.987 3.456 3.987
Freq: 50.40 Hz | Corr: +0.22 Hz | Err: +0.72 Hz | Conf: 0.70

Frame 25: State: STABLE | PLL: 50.78 Hz | Est: 51.03 Hz | Err: +0.25 Hz
Phase trend: 2.987 3.456 3.890 4.234 4.567 4.890
Freq: 50.78 Hz | Corr: +0.06 Hz | Err: +0.25 Hz | Conf: 0.90

Frame 30: State: STABLE | PLL: 50.94 Hz | Est: 50.99 Hz | Err: +0.05 Hz
Phase trend: 4.234 4.567 4.876 5.123 5.345 5.543
```

**Key indicators of success**:
1. Phase trend is smooth (no jumps)
2. Frequency estimate tracks actual (51 Hz)
3. Error decreases steadily
4. State changes NONLINEAR → STABLE after convergence
5. Variance decreases as PLL locks

### Bad Behavior Signs
```
// Sign 1: Phase jumping
Phase trend: 1.234 -2.145 0.987 -2.678  ← BROKEN UNWRAP

// Sign 2: Frequency stuck
Frame 30: PLL: 50.05 Hz | Est: 51.00 Hz | Err: +0.95 Hz
Frame 40: PLL: 50.07 Hz | Est: 51.00 Hz | Err: +0.93 Hz  ← NOT CONVERGING

// Sign 3: Stuck in NONLINEAR
Frame 10-50: State: NONLINEAR  ← Should transition to STABLE

// Sign 4: Wild frequency estimates
Est: 51.00 Hz → 47.23 Hz → 53.45 Hz  ← UNSTABLE
```

## Mathematical Verification

### Phase Accumulation Check
With frequency error Δf and buffer interval Δt:
```
Expected phase drift per buffer = 2π × Δf × Δt

Example: Δf = +1 Hz, Δt = 0.08s (4 cycles @ 50Hz)
Expected: 2π × 1 × 0.08 = 0.503 rad/buffer

If you see phase trend:
0.000, 0.503, 1.006, 1.509, 2.012, 2.515, 3.018
          ↑ slope = 0.503 rad/buf ✓ CORRECT
```

### Frequency Extraction
```
Measured slope = 0.503 rad/buffer
freq_error = slope / (2π × buffer_interval)
           = 0.503 / (2π × 0.08)
           = 0.503 / 0.503
           = 1.00 Hz ✓ CORRECT
```

## Common Issues and Solutions

### Issue: System doesn't track 51 Hz
**Check**:
1. Is frequency estimate valid? (freq_valid = true?)
2. Is freq_gain being applied? (should be 0.30 in NONLINEAR)
3. Is PLL actually changing? (check tune_pll() is called)

**Debug**:
```cpp
Serial.printf("Freq valid: %d, Gain: %.2f, Corr: %.4f, New: %.4f\n",
             freq_valid, freq_gain, freq_corr, new_freq);
```

### Issue: Phase trend has jumps
**Cause**: Unwrapping broken
**Fix**: Verify compute_phase_shift() returns values in [-π, +π]

### Issue: Converges to wrong frequency
**Cause**: Buffer interval wrong
**Check**:
```cpp
float buffer_interval = STROBE_DIV / NOMINAL_FREQ;
// For STROBE_DIV = 4, NOMINAL_FREQ = 50:
// buffer_interval = 4/50 = 0.08 seconds ✓
```

### Issue: Oscillates around target
**Cause**: Gains too high
**Fix**: Reduce freq_gain to 0.15-0.20

## Success Criteria

For test signal (50→51→49→trapezoid):

✓ Phase trend is smooth (no discontinuities)
✓ Frequency estimate tracks within 0.2 Hz after 20 frames
✓ PLL converges to within 0.1 Hz of signal frequency
✓ State alternates STABLE ↔ NONLINEAR appropriately
✓ Variance correctly indicates linear vs non-linear drift
✓ System handles all transitions without diverging

Expected total convergence for 1 Hz step: **15-25 frames (1-2 seconds)**
Expected tracking lag for trapezoid: **2-5 frames** behind frequency ramp
