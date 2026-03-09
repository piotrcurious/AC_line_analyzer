# Phase 7 Numerical and Quantization Error Analysis

## 1. Virtual Grid Quantization
- **Issue**: In Phase 3, the virtual grid used integer ticks, leading to jitter.
- **Improvement**: Phase 7 uses `double` relative offsets (`next_sample_offset_d`) and sub-sample accurate box-filter integration. This eliminates the 1-tick jitter ($1/240\text{MHz}$) and handles the CPU clock wrap-around naturally.

## 2. Adaptive Sampling Density (Variable N)
- **Issue**: Fixed $N$ causes the virtual sampling rate to vary with frequency, affecting filter precision and potentially CPU load.
- **Improvement**: $N \in [100, 256]$ maintains $f_s \approx 6400\text{Hz}$. This keeps the SOGI coefficients in their optimal range ($\omega T_s \approx 0.05\text{rad}$), minimizing discretization errors. A 2-sample hysteresis prevents rapid $N$ toggling due to noise.

## 3. Accumulation and State Drift
- **Issue**: Recursive filters (SOGI, PLL Integrator) using `float` accumulate truncation errors over millions of samples.
- **Improvement**:
  - All recursive delay registers (SOGI states `wz1`, `wz2`, PLL `integral`) upgraded to `double`.
  - **Kahan Summation** used for the PLL integral to recover LSBs during accumulation of small `error * ts` values.

## 4. Beating and Aliasing
- **Issue**: Non-synchronous sampling causes phase-beating, leading to low-frequency ripples in frequency/magnitude estimates.
- **Improvement**: Virtual Synchronous Grid. By adjusting the virtual step size $T_s = 1/(f \cdot N)$ every cycle, samples are anchored to the same phase positions, effectively "locking" the sampling grid to the fundamental and its harmonics.

## 5. FLL Gain Stability
- **Issue**: FLL loop gain varies with signal magnitude.
- **Improvement**: NLMS-based gain estimation and normalization. The control action is normalized by `|gain_est|`, ensuring constant damping factor and convergence speed regardless of input level.

## 6. DC Estimator Bias
- **Issue**: DC offset can bias the SOGI resonant state.
- **Improvement**: DC removal ($u - v_{dc}$) before the SOGI projection. The DC estimate is tracked using a slow `double` precision IIR.

## 7. Numerical Efficiency and State Stability
- **Improvement**: Consolidated SOGI state registers. Since the alpha and beta sections of the SOGI filter share the same resonant poles (the same denominator), they can share a single Direct-Form II state ($w_{z1}, w_{z2}$). This reduces state-update arithmetic and memory overhead.
- **Improvement**: Unified `FrequencyAdaptivePLL` and `AdaptivePLL` to share the same high-precision Kahan summation logic, ensuring consistent frequency integration across variants.
