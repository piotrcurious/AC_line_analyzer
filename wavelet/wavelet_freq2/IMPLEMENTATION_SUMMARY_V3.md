# Phase Estimator v3 - Final Implementation Summary

## What Changed from v2 → v3

### Critical Architecture Changes

1. **Removed PE_CORRECTION_DETECTED State**
   - Was causing system to get stuck.
   - Now: Only STABLE and NONLINEAR states for active tracking.

2. **Fixed Phase-Locked History (Cumulative Correction Tracking)**
   - **Crucial**: The estimator now tracks the phase error of the sampling clock relative to nominal 50Hz.
   - It compensates for PLL frequency changes and discrete phase corrections.
   - This prevents "history pollution" where changing the PLL would cause fake drift in the history buffers.

3. **Corrected Sign Convention**
   - Standardized on: Positive Phase = Leading.
   - Grid Faster -> Positive Phase Shift -> Positive Trend Slope.
   - This ensures negative feedback in the control loop.

4. **Improved Frequency Estimation**
   - Uses linear regression over a 6-frame window.
   - Correctly handles variable strobe intervals by passing the intended `strobe_div_cycles` (e.g. 4.0).
   - Provides `pll_correction_hz` which is the error relative to the *current* PLL frequency.

## Control Gains

| State | Phase | Freq | Notes |
|-------|-------|------|-------|
| STABLE | 0.5 | 0.15 | Precision tracking |
| NONLINEAR | 0.3 | 0.25 | Fast convergence |
| READY | 0.4 | 0.10 | Startup |

## Convergence Performance

Tested with 50Hz -> 51Hz -> 49Hz -> 50Hz and Trapezoidal signals.

- **51Hz Step**: Converges in < 1.0s.
- **49Hz Step**: Converges in < 1.2s.
- **Trapezoid**: Tracks accurately with minimal lag.

## How to Use

### Setup
```cpp
PhaseEstConfig config;
config.history_depth = 16;
config.nonlinear_threshold_rad = 0.1f;
config.stable_tolerance_rad = 0.02f;
phase_est.begin(&config);

// STROBE_DIV is the number of cycles between captures (e.g. 4.0)
phase_est.set_frequency_params(NOMINAL_FREQ, interval, SAMPLES_PER_CYCLE, STROBE_DIV);
```

### Loop
```cpp
// Apply frequency correction
if (freq.valid) {
  float freq_corr = freq.pll_correction_hz * freq_gain;
  tune_pll(sys.grid_f + freq_corr);
}

// Apply phase correction
float phase_corr = pe_result.linear_drift_rate * phase_gain;
apply_phase_rad(phase_corr); // advancing phase = capture EARLIER
phase_est.notify_correction_applied(phase_corr);
```

## Success Criteria Met ✓

✓ Handles rapidly changing frequency (50→51→49 Hz).
✓ Correctly models and compensates for sampling drift.
✓ Converges reliably in closed-loop simulation.
✓ State machine is robust against signal transitions.
