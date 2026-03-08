# Phase 6 Numerical and Quantization Error Analysis

## 1. Virtual Grid Quantization Noise (T_resamp)
- **Problem**: In previous versions, the virtual grid was an integer multiple of the CPU clock, but rounding would cause 1-tick jitter ($1 / 240\text{MHz} \approx 4.1\text{ns}$).
- **Mitigation**: Using relative `double next_sample_offset_d` and sub-sample accurate box-filter integration.
- **Effect**: Accumulates area under the signal even when a virtual boundary falls between two hardware samples. Jitter is eliminated from the virtual timeline. By using relative offsets instead of absolute timestamps, the system is immune to 32-bit clock wrap-around (which occurs every ~17.9s).

## 2. Dynamic Sampling (Adaptive N)
- **Problem**: Fixed $N$ (samples per cycle) means as frequency $f$ increases, the virtual sampling rate $f_s = f \cdot N$ increases, potentially aliasing or overtaxing the CPU. Conversely, at low $f$, $f_s$ drops, increasing discretization error.
- **Mitigation**: Adaptive $N \in [100, 256]$ targets $f_s \approx 6400\text{Hz}$.
- **Impact**: At $50\text{Hz}$, $N = 128$. At $64\text{Hz}$, $N = 100$. At $25\text{Hz}$, $N = 256$.
- **Precision**: Maintaining a nearly constant $f_s$ keeps the SOGI coefficients within a stable operating range ($w \cdot T_s \approx 0.05\text{rad}$).

## 3. Synchronous Sampling and Beating Effects
- **Problem**: "Beating" occurs when the sampling frequency is not an exact multiple of the signal frequency. This causes the phase of the samples to "slide" across cycles, creating low-frequency oscillations in the estimated parameters (amplitude/frequency).
- **Mitigation**: The system implements a **Virtual Synchronous Grid**. By adjusting the virtual sample step $T_s = 1 / (f \cdot N)$ every cycle based on the FLL's frequency estimate, the samples are placed at nearly the same phase positions in every cycle.
- **Result**: This minimizes the aliasing of high-frequency harmonics into the fundamental estimation band, significantly reducing the "hunting" behavior of the FLL in distorted conditions.

## 4. Truncation and Recursive Drift
- **Problem**: Recursive filters (SOGI, FLL integrator, DC estimator) accumulate error over time when using `float`.
- **Mitigation**:
  - All recursive delay registers upgraded to `double`.
  - **Kahan Summation** used for the FLL frequency integrator to recover lost precision in the LSBs.
  - SOGI state is unified (Direct Form II) to eliminate phase-lag differences between the alpha and beta outputs.

## 5. FLL Ripple Rejection and Gain Normalization
- **Problem**: The $2\omega$ ripple in the phase error discriminator causes frequency "hunting." Furthermore, FLL dynamics vary with signal magnitude.
- **Mitigation**:
  - **Cycle-averaging**: Buffer of length $N$. As $N$ changes, the buffer is dynamically resized using an area-preserving mean redistribution to prevent step transients in the frequency estimate.
  - **Online Gain Identification**: An NLMS learner estimates the sensitivity of the phase error to frequency changes. The FLL integral step is divided by this `|gain_est|` to ensure consistent loop dynamics (constant damping) regardless of signal amplitude or grid conditions.

## 6. DC Estimator Sensitivity
- **Problem**: Slow-moving DC drift can bias the SOGI outputs.
- **Mitigation**: 2 rad/s recursive average estimator (`double` precision) tracks the mean. Subtraction before SOGI projection ($u - v_{dc}$) ensures the bandpass stage is zero-centered.
