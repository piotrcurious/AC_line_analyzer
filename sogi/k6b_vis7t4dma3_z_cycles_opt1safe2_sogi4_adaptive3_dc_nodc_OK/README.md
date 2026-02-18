# Project Apex: Predictive Tustin SOGI

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Cycle-Accurate Polling (ccount) |
| **Timebase** | High-Precision Accounting |
| **SOGI Filter** | Tustin (Bilinear) DFII Biquad |
| **PLL / Control** | Predictive Cancellation + Kahan Summation |
| **DC Removal** | Window-smoothed DC correction |
| **Architecture** | Apex DSP Implementation |

## Overview
Most advanced variant. Uses predictive modeling to decouple control actions from estimation, and Kahan summation for near-infinite integrator precision.

## Advanced DSP Implementation

### 1. Predictive Cancellation Model
Traditional PLLs suffer from a "coupling" where the commanded correction in one step appears as an error in the next step, leading to sluggish response or oscillation.
Apex implements **Predictive Cancellation**:
- `predicted_effect = gain_est * last_control_action`
- `residual_err = raw_phase_err - predicted_effect`
- The PI controller only integrates the **residual error**, meaning it only reacts to external grid changes, not its own previously commanded corrections.

### 2. Kahan Summation Integrator
Floating-point numbers have limited precision (mantissa). When adding a tiny error update (e.g., $10^{-6}$) to a large frequency value (e.g., 50.0), the small update is often lost.
Apex uses **Kahan Summation** to track these "lost bits" in a compensation variable, ensuring near-perfect integration accuracy over long periods.

### 3. Adaptive Gain Estimation (LMS)
The system uses a simple **Least Mean Squares (LMS)** estimator to learn the `gain_est` of the signal path. This allows it to automatically compensate for variations in signal amplitude or ADC scaling factors without manual re-tuning.

### 4. Phase Unwrapping & Alignment
Implements a robust 2π phase unwrapper that tracks the absolute cycle count. This enables the visualizer to perform perfect "frozen" waveform rendering even during significant frequency transients.
