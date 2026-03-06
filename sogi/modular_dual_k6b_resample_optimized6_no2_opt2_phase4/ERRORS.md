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
- **Source:** Standard SOGI filters leak harmonic energy into the fundamental estimate, causing frequency rippling.
- **Mitigation:**
    - **Full Structural Decoupling:** Implemented a recursive isolation structure ($u_1 = u - v_{\alpha 3} - v_{\alpha 5}$). By subtracting the estimated 3rd and 5th harmonics from the fundamental's input, the "clean" fundamental is isolated without high-order filters.
    - **Distortion Verification:** A fused error logic compares the SOGI phase-error estimate with the instantaneous phase rotation rate. Updates are gated by an "Agreement" factor, preventing waveform shape changes (e.g., Sine -> Square) from being misidentified as frequency shifts.

### D. SOGI Numerical Stability
- **Source:** Recursive IIR filters are sensitive to coefficient quantization and state precision.
- **Mitigation:**
    - **Tustin Transform:** The SOGI coefficients are derived via the bilinear transform, ensuring stability and mapping the analog resonance perfectly to the digital domain.
    - **Shared State Implementation:** Using a single Direct-Form II state pair $(w_{z1}, w_{z2})$ for both $\alpha$ and $\beta$ outputs reduces the total number of operations and prevents numerical divergence between the quadrature components.

### E. PLL Convergence and Steady-State Error
- **Source:** A standard Type-1 PLL has zero steady-state error for phase but non-zero for frequency if the integrator is missing.
- **Mitigation:**
    - **Kahan Summation:** The FLL frequency integrator uses Kahan compensated summation to preserve precision during long-duration integration of small errors.
    - **Adaptive Loop Gain (NLMS):** The system identifies the signal's loop gain online (gain_est), allowing it to remain stable even when signal amplitude or harmonic distortion levels change.

### F. DC Offset Bias
- **Source:** Persistent DC bias in the ADC input creates a stationary error in the SOGI projection.
- **Mitigation:**
    - **Single-Pole IIR Tracker:** A slow IIR (TC $\approx$ 1s) tracks the signal mean, which is then subtracted before the SOGI stage.
