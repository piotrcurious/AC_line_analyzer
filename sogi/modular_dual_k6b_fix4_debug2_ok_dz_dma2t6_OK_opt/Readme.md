# Optimized Modular Dual SOGI-PLL (DMA + Tile Visualizer)

This directory contains the flagship high-performance implementation of a Dual-Channel Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) for the ESP32. It is engineered for industrial-grade AC line analysis, featuring asynchronous DMA-based acquisition, cycle-accurate timing, and a micro-optimized visualization system.

## 🚀 Key Features

- **Asynchronous DMA Acquisition**: Leverages the ESP32 `adc_continuous` driver at 200kHz (100kHz per channel), minimizing CPU overhead.
- **Tustin-Discretized SOGI**: Implements the Bilinear (Tustin) transform for superior phase and amplitude mapping compared to simpler Forward Euler methods.
- **Adaptive PLL with Predictive Cancellation**: A self-tuning loop filter that estimates plant gain via LMS and decouples control commands from state estimation.
- **Bresenham-style Sample Scheduling**: Ensures zero-drift sampling intervals across varying grid frequencies and CPU clock conditions.
- **Interpolated Re-sampling**: Reconstructs signal values at exact virtual timestamps by searching and interpolating between timestamped DMA buffer frames.
- **Tile-based "Dirty-Rect" Rendering**: A low-latency graphics engine for SSD1306 OLEDs that only updates 4x4 pixel regions that have changed.
- **Kahan Summation Integrator**: Maintains high numerical precision in the frequency integrator to prevent floating-point rounding errors.

---

## ⚖️ Architectural Analysis: Pros & Cons

| Feature | Implementation | Pros | Cons |
| :--- | :--- | :--- | :--- |
| **Acquisition** | **DMA Continuous** | Zero CPU load for sampling; high aggregate rate (200kHz); rock-solid hardware timing. | High complexity; requires DMA-aligned memory; sensitive to ISR latency; frame latency (~160µs). |
| **Resampling** | **Linear Interp** | Decouples acquisition jitter from DSP; reconstructs "ideal" sample points from asynchronous data. | Computational cost of search & interpolate; depends on cache size (`FRAME_BUFFER_SIZE`); requires high-res `ccount`. |
| **Filter** | **Tustin (QSG)** | Precise phase/magnitude response near Nyquist; superior stability at high frequency. | More multiplications per step than Euler; requires careful state management (DFII). |
| **PLL** | **Adaptive + Predictive** | Extremely fast locking; immune to amplitude variations; avoids self-excitation oscillations. | More sensitive to gain estimation parameters; complex feedback loop logic. |
| **Rendering** | **TileManager** | Minimizes SPI bus contention; allows high-frame-rate OLED updates during heavy DSP. | Increased RAM usage for tile buffers; complex "double-dirty" clearing logic. |

---

## 📐 Deep Dive: Algorithmic Innovations

### 1. SOGI Discretization (Tustin Transform)
Traditional implementations often use Forward Euler ($s \approx \frac{z-1}{T}$). This version uses the Tustin Transform ($s \approx \frac{2}{T} \frac{z-1}{z+1}$), which provides a more faithful mapping of the continuous-time SOGI response into the discrete domain.
- **Stability**: Tustin ensures that the left half of the S-plane is mapped exactly into the unit circle of the Z-plane.
- **Implementation**: The filter is realized in **Direct Form II (DFII)**, which uses only two state variables ($w_{z1}, w_{z2}$) per orthogonal path, minimizing memory footprint.

### 2. Adaptive PLL with Predictive Cancellation
A common failure mode of PLLs is "coupling" between the command and the estimate. If the PLL commands a +0.1Hz shift, the filter sees this shift slightly later, leading to overshoot.
- **The Solution**: The loop filter calculates the **Residual Error**:
  $$e_{residual} = e_{raw} - (\gamma \cdot u_{last})$$
  Where $\gamma$ is the estimated plant gain. By subtracting the expected effect of our *own* previous command, the integrator only reacts to *external* grid deviations, leading to much cleaner frequency tracking.
- **LMS Gain Estimator**: The system continuously updates $\gamma$ by comparing predicted phase shift vs actual phase shift over a history window of 8 samples.

### 3. Kahan Summation for Precision
Floating-point numbers (FP32) have a 24-bit mantissa. When adding a tiny increment ($10^{-7}$) to a large accumulator ($50.0$), the low bits are truncated.
- **Effect**: Over hours of operation, this "lost precision" causes frequency drift.
- **Kahan Algorithm**:
  ```cpp
  float y = (ki * error) - compensation;
  float t = accumulator + y;
  compensation = (t - accumulator) - y;
  accumulator = t;
  ```
  The `compensation` variable tracks the discarded bits, re-injecting them into the next summation.

---

## 🏗 Data Acquisition & Timing

### The "Virtual Timebase" Concept
Unlike standard systems that sample *now* and process *now*, this version maintains a **Virtual Timebase**:
1.  **DMA Producer**: ADC hardware fills buffers and triggers an ISR. The ISR captures the CPU `ccount` at the exact moment the buffer is full.
2.  **DSP Consumer**: The loop calculates where the sample *should* have been in time.
3.  **The Bridge**: `interpolateSampleAtTime` searches backwards through the `TimestampedFrame` buffer (400 frames deep). It finds the two hardware samples that "bracket" the target virtual time and performs linear interpolation.

### Bresenham Interval Calculation
To maintain exactly 128 samples per cycle regardless of the grid frequency, we must step by a fractional number of cycles:
$Interval = \frac{CPU\_Freq}{Freq \cdot 128}$
Since we can only wait for an integer number of CPU cycles, we use a Bresenham accumulator to track the fractional part ($remainder$) and "tick" an extra cycle whenever the accumulator overflows.

---

## 📺 Visualization: Tile-based Compositor

Traditional full-screen OLED updates ($128 \times 64 = 8192$ bits) are slow (several milliseconds).
- **The Tiled Approach**: The screen is divided into 4x4 pixel tiles.
- **Dirty Tracking**: Each tile has a `dirty_curr` and `dirty_prev` flag.
- **The Clear-to-Draw Cycle**:
  1.  If a pixel is written to a tile, it is marked `dirty_curr`.
  2.  During the `flush()`, only tiles marked `dirty_curr` or `dirty_prev` are sent to the hardware.
  3.  **Why `dirty_prev`?** When a waveform moves from Tile A to Tile B, Tile A must be redrawn one more time (cleared) to erase the old segment. Without `dirty_prev`, the "tail" of the waveform would never be erased.

---

## 📂 Comparison with other Variants in Repository

| Version | Complexity | Sampling | PLL Type | Primary Use Case |
| :--- | :--- | :--- | :--- | :--- |
| **Production** | Low | Timer ISR | Standard PI | Basic grid-tie, low CPU usage. |
| **EKF** | Very High | Polling | Kalman Filter | High-noise environments, lab analysis. |
| **Resampled** | Medium | Polling | Adaptive PI | Frequency-variable systems, simple UI. |
| **This Version (Opt)**| **Maximum** | **DMA + Interp** | **Predictive** | **High-precision analysis, high UI responsiveness.** |

---
*Developed for the SOGI-PLL Grid Synchronization Project. Optimized for ESP32 and ESP32-S3.*
