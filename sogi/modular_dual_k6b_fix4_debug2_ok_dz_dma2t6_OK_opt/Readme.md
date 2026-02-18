# Deep Technical Analysis: Optimized Modular Dual SOGI-PLL

This directory contains a specialized, high-performance ESP32 implementation of a Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) for dual-channel (Voltage/Current) grid synchronization. It represents a state-of-the-art approach to embedded AC analysis, combining asynchronous hardware acquisition with a phase-synchronous DSP pipeline.

---

## 1. Data Acquisition: The Asynchronous Resampling Bridge

The core architectural innovation is the decoupling of hardware sampling from the DSP integration step.

### 1.1 High-Rate DMA Acquisition
The system utilizes the ESP32 `adc_continuous` driver at an aggregate rate of **200 kHz** (100 kHz per channel).
- **Hardware Layer**: Data is moved via DMA into 32-byte "Type 1" frames.
- **Temporal Anchoring**: In the `adc_conv_done_callback` (ISR), the CPU cycle counter (`ccount`) is captured immediately. This timestamp anchors the end of the frame with 4.16ns resolution.
- **Buffer Depth**: A 400-frame circular buffer stores ~32ms of raw history. This capacity is critical to survive FreeRTOS task-switching latencies or Serial I/O blocking.

### 1.2 The Interpolation Engine
Because hardware samples do not align with the PLL's phase-synchronous "ideal" timestamps, the system implements a **Linear Resampler**:
- **Search Logic**: It searches backwards through timestamped frames (up to `MAX_SEARCH=8`) to find the two hardware samples that "bracket" the target virtual time.
- **Interpolation Math**:
  $$V(t) = V_n + (t - t_n) \cdot \frac{V_{n+1} - V_n}{t_{n+1} - t_n}$$
- **Result**: This provides a jitter-free, uniform timebase for the SOGI integrator, effectively converting hardware-level temporal jitter into a negligible amplitude error.

---

## 2. DSP & Numerical Signal Processing

### 2.1 Tustin (Bilinear) Discretization
The SOGI filter is discretized using the **Tustin Transform** ($s \approx \frac{2}{T} \frac{z-1}{z+1}$), which preserves the continuous-time stability properties in the discrete domain more accurately than the simpler Forward Euler method.
- **Architecture**: Realized in **Direct Form II (DFII)** to minimize state variables and reduce the probability of numerical overflow/limit cycles.
- **Precision Limits**: At the internal sampling rate of ~6.4kHz, poles reside at $z \approx 0.98 \pm 0.045j$. The system remains stable in FP32, as the coefficients are recalculated per-cycle to adapt to frequency shifts.

### 2.2 Numerical Stability: Kahan Summation
The frequency integrator uses **Kahan Summation** to maintain sub-milliHz precision.
- **The Problem**: Adding a tiny error $\epsilon$ to a large frequency $\omega$ (e.g., $314.159...$) leads to catastrophic cancellation as bits are shifted out of the 24-bit FP32 mantissa.
- **The Solution**: A separate compensation variable tracks the "lost bits," ensuring that over long durations, the PLL does not suffer from quantization-induced frequency drift.

---

## 3. Control Theory: Adaptive Predictive Observer

### 3.1 Predictive Cancellation Model
A standard SOGI-PLL has an inherent phase lag caused by the filter's group delay.
- **The Innovation**: The system subtracts the "predicted effect" of its own last control action from the observed phase error:
  $$e_{residual} = e_{raw} - (\gamma \cdot u_{last})$$
- **Effect**: This "active cancellation" allows the PLL to use higher gains ($K_p=1.25$) for faster locking while maintaining the stability profile of a much slower loop.

### 3.2 Normalized LMS (NLMS) Gain Estimation
The "plant gain" $\gamma$ (the phase sensitivity to frequency shifts) is estimated in real-time.
- **Update Law**: Uses a damped NLMS algorithm with $\mu = 0.1001$.
- **Robustness**: This makes the PLL immune to variations in signal magnitude or grid impedance, as it "learns" the response of the SOGI filter dynamically.

---

## 4. Hardware Orchestration & Resource Management

### 4.1 Resource Profiling
- **RAM Usage**: ~35.6 KB total static RAM (15.6 KB for DMA frames, 16 KB for Display Tiles, 4 KB for Signal Buffers).
- **CPU Load**: The interpolation engine consumes ~1.3% of the 240MHz CPU. SOGI window processing takes ~30-50µs per cycle.

### 4.2 Visualization: Tile-based "Dirty-Rect" Engine
The `SOGIVisualizer` drives the SSD1306 OLED via a 40MHz SPI bus using **Tile-based rendering**.
- **Tiling**: The screen is divided into 512 tiles ($4 \times 4$ pixels).
- **Double-Dirty Logic**: Tiles are cleared and redrawn only if they were modified in the current *or* previous frame. This maximizes SPI throughput by only updating active waveform regions.
- **DMA Alignment**: All graphics buffers are allocated with `MALLOC_CAP_DMA`, allowing the SPI hardware to handle transmission in the background.

---

## 5. Signal Conditioning & Quantization

### 5.1 Oversampling SNR Gains
The aggregate 200kHz acquisition rate provides a Significant Oversampling Ratio (OSR) relative to the 6.4kHz DSP loop (OSR $\approx 31.25$).
- **Process Gain**: The linear interpolation process acts as a decimation filter, providing $\approx 15\text{dB}$ of SNR gain.
- **Effective Resolution**: While the hardware ADC is 12-bit, the oversampled and interpolated signal fed to the SOGI integrator has an effective resolution (ENOB) approaching **14.5 bits**.

### 5.2 Hybrid DC Estimation
The system tracks the 1.65V mid-scale bias using a dual-stage approach:
1.  **Windowed Mean**: Each AC cycle calculates the arithmetic mean of all samples in the window.
2.  **EMA Smoothing**: This mean is fed into an EMA filter with $\alpha=0.2$.
    $$V_{dc} = 0.2 \cdot V_{mean\_win} + 0.8 \cdot V_{dc\_prev}$$
- **Rationale**: This prevents DC-tracking from phase-shifting the fundamental frequency (high rejection at 50Hz) while remaining agile enough to track thermal bias drift in the ADC front-end.

---

## 6. Critical Review & Implementation Assessment

### 6.1 Architectural Strengths
- **Decoupled Timing**: The use of a "Virtual Timebase" with linear interpolation is an elite-level solution for handling non-uniform hardware sampling in a phase-sensitive DSP environment.
- **Numerical Rigor**: Implementation of Kahan summation and Tustin discretization demonstrates a deep understanding of the precision limits of IEEE 754 single-precision floats in long-running integrators.
- **Hardware Exploitation**: The tile-based rendering engine effectively bypasses the bottleneck of SPI-based display updates, preserving CPU cycles for high-priority mathematical tasks.

### 6.2 Areas for Improvement (Technical Debt)
- **God-Function Pattern**: The `loop()` function is monolithic, mixing acquisition scheduling, DSP logic, and visualization orchestration. This should be refactored into a clear state-machine or distinct task-oriented functions.
- **Coefficient Redundancy**: SOGI coefficient calculation is duplicated in `SOGI::step` and `SOGI::processWindow`. This "DRY" (Don't Repeat Yourself) violation increases the risk of algorithmic divergence if only one is updated.
- **Global State Proliferation**: Extensive use of static globals and file-scope variables (e.g., `g_tiled_canvas`, `last_v_min`) complicates unit testing and prevents re-entrancy.
- **Hardcoded Configuration**: Hardware pin definitions and SPI host selections are embedded deep within the class implementations (e.g., in `LGFX_SOGI`), rather than being passed via dependency injection or a centralized configuration header.

---

## 📂 Design Rationale: Why this Version?

This "opt" (Optimized) version represents the convergence of three separate technical axes:
1.  **Asynchronous Sampling**: DMA-based acquisition eliminates the "hard-real-time" requirement for the main loop.
2.  **Theoretical Rigor**: Tustin discretization and Kahan summation ensure high-fidelity estimation.
3.  **UI Performance**: Tile-based rendering allows a high-fidelity 50fps visualization without interrupting the DSP pipeline.

---
*Technical analysis for the SOGI-PLL Grid Synchronization Project.*
