# Variable Samples per Cycle (N) and Error Minimization

To achieve a robust 50Hz AC tracking system with minimal numerical errors and quantization noise, the following design choices were made in Phase 4:

## 1. Variable Samples per Cycle (N)
The system adaptively adjusts the number of virtual samples per cycle $N \in [100, 256]$.
- **Target Rate:** The system targets a constant virtual sampling frequency $f_v \approx 6400\text{ Hz}$.
- **Adaptation:** $N = \lfloor f_v / \hat{f} + 0.5 \rfloor$. This ensures that as the grid frequency $\hat{f}$ shifts, the angular resolution of the SOGI remains approximately constant ($\Delta \phi \approx 0.05\text{ rad/sample}$), keeping the filter's $Q$ factor and transient response stable.
- **Update Synchronization:** $N$ is only updated at cycle boundaries to prevent discontinuities in the SOGI state advancement.

## 2. Identified Sources of Numerical Errors & Mitigations

### A. Temporal Quantization (Timing Jitter)
- **Source:** The ESP32 `ccount` register has a resolution of ~4.16ns (at 240MHz). ADC DMA interrupts have variable latency (jitter).
- **Mitigation:**
    - **Hardware Anchoring:** DMA frame end-timestamps are used to reconstruct a continuous hardware time axis.
    - **Double-Precision Timeline:** The virtual timeline (`next_sample_int` and `next_sample_frac`) uses `double` precision. This prevents the accumulation of "sampling phase noise" over time, which can manifest as artificial frequency modulation (FM noise) in the PLL.
    - **Fractional Box-Filter:** Instead of nearest-neighbor resampling, the resampler integrates the signal over the exact fractional virtual period.

### B. Floating-Point Accumulation Errors
- **Source:** Summing many ADC samples into an accumulator (`acc_v`) can lead to loss of precision as the sum grows much larger than the individual samples.
- **Mitigation:**
    - **Double-Precision Accumulators:** `acc_v`, `acc_i`, and `acc_weight` are now `double` precision. This ensures that the integration of the box filter maintains 12-bit ADC fidelity even at high oversampling rates.

### C. Harmonic Leakage & Distortion
- **Source:** Standard SOGI filters leak harmonic energy into the fundamental estimate, causing frequency rippling. Square waves contain significant energy in high-order odd harmonics (3, 5, 7, 9, 11...).
- **Mitigation:**
    - **High-Order Structural Decoupling:** Expanded the recursive isolation structure to include the 3rd, 5th, 7th, 9th, and 11th harmonics. Each SOGI stage $H_n$ is driven by $u_n = u - \sum_{i \neq n} v_{\alpha i}$, effectively creating a notch-filter effect for all other modeled harmonics without adding group delay.
    - **Active Phase Distortion Correction:** Replaced the "damping" approach with a "Verification" logic. The FLL fused error is $e_{fused} = e_{fll} \cdot A + e_{rot} \cdot (1-A)$, where $A$ is the "Agreement" factor between SOGI phase-error ($e_{fll}$) and instantaneous rotation rate ($e_{rot}$). This prioritizes the physically-anchored period measurement during rapid waveform shape transitions.

### D. SOGI Numerical Stability
- **Source:** Recursive IIR filters are sensitive to coefficient quantization and state precision. Small errors in the state can accumulate, raising the noise floor and preventing high-precision frequency lock.
- **Mitigation:**
    - **Double-Precision Recursive State:** The SOGI delay registers ($w_{z1}, w_{z2}$) now use `double` precision. This provides 53 bits of mantissa, ensuring that the resonant accumulation of signal energy does not lose precision even when oversampled at 250kHz.
    - **Shared State Implementation:** A single Direct-Form II state pair is used for both $\alpha$ and $\beta$ outputs.

### E. PLL Convergence and Steady-State Error
- **Source:** A standard Type-1 PLL has zero steady-state error for phase but non-zero for frequency if the integrator is missing.
- **Mitigation:**
    - **Kahan Summation:** The FLL frequency integrator uses Kahan compensated summation to preserve precision during long-duration integration of small errors.
    - **Adaptive Loop Gain (NLMS):** The system identifies the signal's loop gain online (gain_est), allowing it to remain stable even when signal amplitude or harmonic distortion levels change.

### F. DC Offset Bias
- **Source:** Persistent DC bias in the ADC input creates a stationary error in the SOGI projection.
- **Mitigation:**
    - **Single-Pole IIR Tracker:** A slow IIR (TC $\approx$ 1s) tracks the signal mean, which is then subtracted before the SOGI stage.
