# Technical Analysis: Optimized Dual SOGI-PLL (DMA + Tile Rendering)

This directory implements an advanced, high-performance Grid Synchronization system using a Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) on the ESP32. It features a sophisticated asynchronous acquisition pipeline and a low-overhead tile-based visualization system.

---

## 🏗️ System Architecture

The system is designed around a **Hardware-Software Co-Design** philosophy, decoupling high-rate hardware acquisition from phase-synchronous DSP processing.

### 1. The Data Acquisition Pipeline (Asynchronous Bridge)
Unlike standard implementations that poll the ADC or use low-rate timers, this system utilizes the ESP32's **Continuous ADC DMA** unit.

*   **Acquisition (Producer)**: The ADC1 unit samples at an aggregate rate of **200 kHz** (100 kHz per channel for Voltage and Current). Data is moved via DMA into 32-byte frames.
*   **Temporal Anchoring**: An ISR (`adc_conv_done_callback`) captures the high-resolution CPU `ccount` (4.16ns resolution) at the exact moment a DMA frame is ready. This timestamp provides a "temporal anchor" for the entire frame.
*   **Buffer Depth**: A `FRAME_BUFFER_SIZE` of 400 frames maintains a rolling history of ~32ms of raw signal, allowing the DSP consumer to "look back" in time.

### 2. The Virtual Timebase & Resampling
The DSP pipeline does not process data at the 200kHz rate. Instead, it maintains a **Virtual Timebase** synchronous to the estimated grid frequency.

*   **Bresenham Timing Engine**: To ensure exactly 128 samples per AC cycle, the system uses a Bresenham-style accumulator to calculate the ideal `target_time` in CPU cycles. This eliminates cumulative drift even when the `CPU_FREQ / (FREQ * 128)` ratio is fractional.
*   **Linear Interpolation**: For every virtual sample point, the `interpolateSampleAtTime` engine searches the DMA frame buffer. It locates the two hardware samples that bracket the target time and performs linear interpolation:
    $$V(t) = V_n + (t - t_n) \frac{V_{n+1} - V_n}{t_{n+1} - t_n}$$
*   **Oversampling Gain**: By reconstructing the signal from a 200kHz stream for a ~6.4kHz DSP loop (128 samples/cycle @ 50Hz), the system achieves a theoretical SNR gain of:
    $$10 \log_{10}\left(\frac{200\text{kHz}}{6.4\text{kHz}}\right) \approx 14.9 \text{ dB}$$

---

## 📐 Numerical Signal Processing

### 1. SOGI Discretization (Tustin Transform)
The Second-Order Generalized Integrator is discretized using the **Tustin (Bilinear) Transform**. This transform is superior to Forward Euler for power electronics because it maps the entire $s$-plane into the unit circle of the $z$-plane, ensuring stability and better matching the frequency response near the Nyquist frequency.

*   **Filter Path**: The implementation uses **Direct Form II (DFII)** to realize both the Alpha (Band-Pass) and Beta (Quadrature Low-Pass) paths. This minimizes state memory and provides better numerical properties than Direct Form I.
*   **State Management**: Orthogonal signals are processed in windows to maintain phase coherence during frequency transients.

### 2. Adaptive PLL with Predictive Cancellation
The PLL is a sophisticated self-tuning observer that decouples control actions from state estimation.

*   **Predictive Cancellation**: To prevent the loop filter from oscillating due to the SOGI group delay, the system calculates the **Residual Error**:
    $$e_{residual} = e_{raw} - (\gamma \cdot u_{last})$$
    It subtracts the "predicted effect" of the last frequency shift command from the observed phase error.
*   **LMS Gain Estimation**: The "plant gain" ($\gamma$) is not assumed; it is estimated in real-time using a Least Mean Squares (LMS) update law. This allows the PLL to stay locked even if the grid signal amplitude or impedance changes drastically.
*   **Kahan Summation**: To achieve sub-milliHz frequency resolution, the frequency integrator uses Kahan Summation. This technique stores the floating-point truncation error in a separate compensation variable, effectively extending the precision of the accumulation beyond the standard 24-bit mantissa of FP32.

---

## 📺 Visualization: Tile-based "Dirty-Rect" Rendering

Driving an SPI OLED (SSD1306) at high frame rates typically consumes significant CPU and SPI bandwidth. This implementation uses a **TileManager** to optimize throughput.

*   **Tiling**: The 128x64 screen is divided into 4x4 pixel tiles.
*   **Double-Dirty Logic**: Each tile maintains `dirty_curr` and `dirty_prev` flags. A tile is only refreshed if it was dirty in the *current* frame (new drawing) or the *previous* frame (needs erasure). This ensures the "tail" of a moving waveform is cleared without requiring a full-screen `memset`.
*   **DMA Transfers**: Tile buffers are allocated in DMA-accessible internal RAM (`MALLOC_CAP_DMA`). Updates are pushed over a **40 MHz SPI bus**, allowing the CPU to return to DSP tasks while the hardware handles the display update.

---

## ⚖️ Design Trade-offs & Hardware Limits

| Dimension | DMA/Interpolation (This Version) | Standard Polling / ISR |
| :--- | :--- | :--- |
| **CPU Load** | Very Low (Hardware handles sampling) | High (CPU handles every sample) |
| **Jitter Resistance**| High (Asynchronous bridge) | Low (Directly tied to OS latency) |
| **Numerical Stability**| High (Tustin + Kahan) | Medium (Euler / Naive Integrator) |
| **Complexity** | Very High (Requires Buffer Search/Interp) | Low |
| **Latency** | 1 Frame (~160µs) | Near Zero |

### Quantization and ENOB
While the ESP32 ADC has a 12-bit resolution, its Effective Number of Bits (ENOB) is closer to 10.5 due to non-linearity. The **Oversampling + Interpolation** strategy effectively acts as a low-pass filter, recovering some of this lost precision and providing cleaner signals to the SOGI integrator.

---

## 📂 File Analysis

- `modular_dual_..._opt.ino`: Orchestrates the DMA-ISR-Loop bridge and the Bresenham timing engine.
- `SOGI.cpp/h`: Contains the core mathematical implementation of the Tustin SOGI and Adaptive PLL.
- `SOGIvisualizer.cpp/h`: Implements the high-speed tile rendering engine.
- `FastMathToolkit.h`: Auxiliary ring-table based multipliers for future off-loading of floating-point divisions.
- `analog.cpp/h`: Legacy wrappers for one-shot ADC calibration (maintained for reference).

---
*Developed as part of the SOGI-PLL Grid Synchronization Project.*
