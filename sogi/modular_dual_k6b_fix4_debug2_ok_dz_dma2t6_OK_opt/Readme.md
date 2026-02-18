# Optimized Modular Dual SOGI-PLL (DMA + Tile Visualizer)

This directory contains a high-performance implementation of a Dual-Channel Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) for the ESP32. It is engineered for industrial-grade AC line analysis, featuring asynchronous DMA-based acquisition, cycle-accurate timing, and a micro-optimized visualization system.

## 🚀 Key Features

- **Asynchronous DMA Acquisition**: Leverages the ESP32 `adc_continuous` driver at 200kHz (100kHz per channel).
- **Tustin Discretized SOGI**: Uses the Bilinear (Tustin) transform for superior phase and amplitude accuracy compared to Forward Euler.
- **Adaptive PLL with Predictive Cancellation**: A self-tuning loop filter that estimates plant gain via LMS and decouples control actions from state estimation.
- **Bresenham-style Sample Scheduling**: Ensures zero-drift sampling intervals even when the CPU frequency is not a perfect multiple of the sampling rate.
- **Interpolated Re-sampling**: Reconstructs signal values at exact virtual timestamps by interpolating between jittery DMA buffer frames.
- **Tile-based "Dirty-Rect" Rendering**: Minimizes SPI overhead for the SSD1306 OLED by only updating modified 4x4 pixel regions.
- **Kahan Summation Integrator**: Eliminates floating-point rounding errors in the PLL's frequency integrator.

---

## 📐 Mathematical Foundation

### 1. SOGI Discretization (Tustin Transform)
The Second-Order Generalized Integrator is implemented using the Tustin (Bilinear) transform to map the continuous-time transfer functions to the Z-domain:
- **Alpha (Band-Pass)**: $H_\alpha(s) = \frac{k\omega s}{s^2 + k\omega s + \omega^2}$
- **Beta (Quadrature Low-Pass)**: $H_\beta(s) = \frac{k\omega^2}{s^2 + k\omega s + \omega^2}$

The discrete coefficients are computed at runtime as:
- $det = \frac{1}{4 + 2k(\omega T) + (\omega T)^2}$
- $a_{b0} = 2k(\omega T) \cdot det, \quad a_{b2} = -2k(\omega T) \cdot det$
- $a_{a1} = 2((\omega T)^2 - 4) \cdot det, \quad a_{a2} = (4 - 2k(\omega T) + (\omega T)^2) \cdot det$

### 2. Adaptive PLL & Predictive Cancellation
The PLL uses an **Adaptive Observer** model to improve stability and locking speed:
- **Gain Estimation (LMS)**: The system learns the "plant gain" ($\gamma$) by observing how control actions ($\Delta u$) affect the phase error ($\Delta \phi$):
  $\gamma_{new} = \gamma_{old} + \mu \frac{\Delta u \cdot (\Delta \phi - \gamma_{old} \Delta u)}{\Delta u^2 + \epsilon}$
- **Predictive Cancellation**: To prevent the loop filter from reacting to its own commands, the "predicted effect" of the last control action is subtracted from the raw phase error before integration:
  $e_{residual} = e_{raw} - (\gamma \cdot u_{last})$

---

## 🏗 Data Acquisition Pipeline

### 1. DMA & Frame Timestamping
The ADC runs in continuous mode, filling 32-byte frames. Each frame completion triggers an ISR (`adc_conv_done_callback`) which:
1.  Captures the high-resolution CPU `ccount` (240MHz).
2.  Performs a fast `memcpy` of raw ADC data into a circular `TimestampedFrame` buffer.
3.  Marks the frame for later processing.

### 2. Linear Interpolation Engine
Because DMA frames arrive asynchronously to the PLL's target sampling moments, the system uses an interpolation engine:
- It calculates the exact `ccount` timestamp for every individual sample within a DMA frame using the formula:
  $T_{sample} = T_{frame\_end} - (N_{samples\_from\_end} \cdot CyclesPerSample)$
- If the target PLL time falls between two DMA samples, it performs linear interpolation:
  $V_{interp} = V_n + \alpha(V_{n+1} - V_n)$

### 3. Bresenham Timing
To maintain a perfect 128 samples per AC cycle, the system uses a Bresenham-style accumulator to handle the fractional remainder of `CPU_FREQ / (FREQ * 128)`:
```cpp
bresenham_acc += ticks_remainder;
if (bresenham_acc >= SAMPLES_PER_CYCLE) {
    bresenham_acc -= SAMPLES_PER_CYCLE;
    current_interval += 1;
}
```

---

## 📺 Visualization System

The `SOGIVisualizer` uses a specialized `TileManager` to drive the SSD1306 display at high effective frame rates:
- **Tiling**: The screen is divided into 32x16 tiles of 4x4 pixels.
- **Double-Dirty Logic**: A tile is cleared and redrawn if it was "dirty" in the *current* OR *previous* frame. This ensures that a moving waveform correctly erases its previous tail without requiring a full screen clear.
- **DMA-Aware Allocation**: Tile buffers are allocated using `MALLOC_CAP_DMA` for maximum SPI throughput.

---

## 📂 File Structure

- `modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt.ino`: Main timing loop, DMA ISR, and interpolation logic.
- `SOGI.h/cpp`: High-precision SOGI and AdaptivePLL class implementations.
- `SOGIvisualizer.h/cpp`: Tile-based rendering engine and phase-aligned plotting.
- `FastMathToolkit.h`: Auxiliary ring-table based multipliers for future fixed-point acceleration.
- `analog.h/cpp`: Legacy one-shot ADC driver (retained for fallback/calibration reference).

## 🛠 Technical Specifications

| Parameter | Value |
| :--- | :--- |
| **ADC Sampling Rate** | 200,000 Hz (Aggregate) |
| **SOGI Discretization** | Tustin (Bilinear) |
| **PLL Type** | Adaptive LMS + Predictive Cancellation |
| **Rendering Algorithm** | Tile-based Dirty-Rect (4x4 Tiles) |
| **Timing Precision** | 1 CPU Cycle (~4.16ns @ 240MHz) |
| **DC Removal** | Windowed Average (Hybrid EMA) |

---
*Developed as part of the SOGI-PLL Grid Synchronization Project.*
