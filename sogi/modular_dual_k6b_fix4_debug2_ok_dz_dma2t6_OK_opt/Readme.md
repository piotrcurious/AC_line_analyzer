# Deep Technical Analysis: Optimized Modular Dual SOGI-PLL

This directory contains a specialized ESP32 implementation of a Second-Order Generalized Integrator (SOGI) PLL. Unlike standard implementations that rely on synchronous polling or low-rate timers, this version implements an asynchronous bridge between high-rate DMA acquisition and a phase-synchronous DSP pipeline.

---

## 1. Signal Acquisition & The Asynchronous Bridge

### 1.1 DMA Pipeline & ISR Jitter
Acquisition is handled by the ESP32 `adc_continuous` hardware, configured for a 200kHz aggregate rate.
- **Hardware Layer**: The DMA controller fills 32-byte frames (16 samples of Type 1 ADC data).
- **Temporal Uncertainty**: The `ccount` (CPU cycle counter) is captured at the start of the `adc_conv_done_callback`. While `ccount` has 4.16ns resolution (@240MHz), the entry latency into the ISR introduces a jitter $\Delta t_{isr} \approx 1\text{--}5\mu\text{s}$.
- **Impact**: At the 100kHz per-channel rate, the interval between samples is $10\mu\text{s}$. A $2\mu\text{s}$ jitter represents a 20% uncertainty in the frame's temporal anchor, which is the primary source of noise in the resampled stream.

### 1.2 Linear Interpolation Math
To reconstruct a uniform timebase for the PLL, the system solves for $V(t_{target})$ using bounded frames:
$$V(t) = V_n + (t - t_n) \cdot \frac{V_{n+1} - V_n}{t_{n+1} - t_n}$$
This interpolation acts as a first-order low-pass filter on the quantization noise, providing a theoretical $\approx 3\text{dB}$ SNR improvement at the midpoint of samples ($\alpha=0.5$), but it cannot recover information lost to ISR jitter.

---

## 2. Numerical Signal Processing

### 2.1 Tustin (Bilinear) Discretization
The SOGI filter is discretized using the Tustin transform, which maps the $s$-domain to the $z$-domain via $s = \frac{2}{T} \frac{z-1}{z+1}$.
- **Pole Sensitivity**: At a 6.4kHz internal sampling rate (128 samples/cycle @ 50Hz), the system poles are located at $z \approx 0.98 \pm 0.045j$.
- **Numerical Limits**: At higher oversampling (e.g., >100kHz), the poles move to $z \approx 0.999 \pm 0.003j$. In FP32 (24-bit mantissa), the rounding errors in the $a_1$ and $a_2$ coefficients ($a_1 \to -2, a_2 \to 1$) can lead to limit-cycle oscillations or catastrophic cancellation. This implementation avoids this by fixing the internal DSP rate via Bresenham scheduling.

### 2.2 Kahan Summation Integrator
To maintain phase lock over long durations, the frequency integrator uses Kahan Summation to combat floating-point truncation.
- **Quantization**: When adding a small error $\epsilon \approx 10^{-6}$ to an angular frequency $\omega \approx 314.159$, the low bits are lost.
- **Correction**: By maintaining a separate `integral_err_c` variable to store the "lost" residue, the effective precision is extended, preventing long-term frequency drift that would otherwise occur in naive FP32 accumulation.

---

## 3. Advanced Control Theory

### 3.1 Predictive Cancellation Model
Standard PLLs suffer from a "command-to-feedback" lag caused by the SOGI filter's group delay.
- **Concept**: The system implements an internal model to predict the phase shift caused by its own frequency corrections.
- **Decoupling**: $e_{residual} = e_{raw} - (\gamma \cdot u_{last})$. By subtracting the predicted effect of the last control action ($u_{last}$) from the observed error ($e_{raw}$), the integrator only sees the *external* disturbance. This allows for significantly higher loop gains ($K_p$) without inducing self-excited oscillations.

### 3.2 LMS Gain Adaptation
The plant gain $\gamma$ (the transfer function from frequency command to phase error) is not constant; it depends on grid impedance and signal magnitude.
- **Normalized LMS**: The system uses an NLMS update law:
  $$\hat{\gamma}_{n+1} = \hat{\gamma}_n + \mu \frac{x_n (d_n - \hat{\gamma}_n x_n)}{\|x_n\|^2 + \epsilon}$$
- **Stability**: With $\mu = 0.1001$, the gain estimator is highly damped, ensuring stability even in the presence of 12-bit quantization noise and ADC non-linearities.

---

## 4. Hardware & Memory Orchestration

### 4.1 Bresenham-style Sample Scheduling
To ensure the PLL maintains exactly $N$ samples per cycle without drifting against the CPU clock, a first-order Delta-Sigma approach is used for timing.
- **Accumulator**: `bresenham_acc` tracks the fractional CPU cycles per sample.
- **Zero-Drift**: This ensures that over $M$ samples, the total elapsed cycles exactly matches $M \times (\text{CPU\_FREQ} / \text{TARGET\_FREQ})$, maintaining long-term phase-synchronicity with the grid even if the division is not an integer.

### 4.2 Frame Buffer Depth & Latency
- **Buffer Capacity**: The `FRAME_BUFFER_SIZE` is 400 frames. At 80µs/frame, this represents 32ms of raw history.
- **Cleanup Threshold**: The `CLEANUP_FRAMES_DIVIDER` (200) limits the active search window to $\approx 5\text{ms}$. If the main loop is blocked by Serial I/O or other high-priority tasks for more than 5ms, the interpolation engine will lose its temporal anchor, reverting to nearest-sample fallback and inducing phase jitter.

### 4.3 Memory & Cache Efficiency
- **IRAM-Resident Code**: Critical DSP paths and the ADC ISR are marked `IRAM_ATTR` to prevent cache-miss latency spikes during Flash MMU activity.
- **DMA RAM**: The `TileManager` utilizes 8KB of `MALLOC_CAP_DMA` internal RAM. This allows the SPI controller to transmit OLED updates in the background without CPU intervention, preserving cycles for the SOGI window processing.

### 4.3 Double-Dirty Tiling Logic
To maximize the effective refresh rate of the SSD1306 (limited by the 40MHz SPI bus), the system divides the screen into 512 tiles ($4 \times 4$ pixels).
- **The "Tail" Problem**: A moving waveform leaves a residue on the screen.
- **Double-Buffering**: Each tile maintains `dirty_curr` and `dirty_prev`. A tile is cleared and redrawn if it was dirty in either the current frame (new data) or the previous frame (needs erasure of old data). This minimizes the "SPI airtime" to only the active pixels and their recent paths.

---

## 5. Technical Specifications & Quantization

| Dimension | Specification | Notes |
| :--- | :--- | :--- |
| **ADC Quantization** | 12-bit (4096 levels) | Effective ENOB $\approx 10.5$ due to ESP32 INL/DNL. |
| **Oversampling Gain** | $\approx 15\text{dB}$ | 200kHz acquisition vs 6.4kHz DSP loop. |
| **Arithmetic** | FP32 (IEEE 754) | Optimized via Kahan Summation and Tustin pre-computation. |
| **Loop Latency** | $160\mu\text{s}$ | Combined DMA frame buffering and ISR entry. |
| **Jitter Margin** | $< 10\mu\text{s}$ | Limited by FreeRTOS interrupt latency. |

---
*Technical analysis for the SOGI-PLL Grid Synchronization Project.*
