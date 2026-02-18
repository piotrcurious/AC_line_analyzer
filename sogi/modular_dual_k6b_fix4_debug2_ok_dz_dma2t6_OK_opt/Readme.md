# Optimized Modular Dual SOGI-PLL (DMA + Tile Visualizer)

This directory contains a highly optimized implementation of a Dual-Channel Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) for the ESP32. It is designed for high-precision AC line analysis, featuring asynchronous DMA-based acquisition and a low-overhead tile-based visualization system.

## 🚀 Key Features

- **Asynchronous DMA Acquisition**: Utilizes the ESP32 `adc_continuous` driver to sample at 200kHz (oversampled), offloading the CPU from individual sample handling.
- **Bresenham-based Scheduling**: Employs a Bresenham-style algorithm to distribute PLL samples evenly across the estimated AC cycle, ensuring long-term phase stability.
- **Linear Interpolation**: Reconstructs precise voltage and current values at exact target times by interpolating between timestamped DMA buffer frames.
- **Adaptive PLL with Predictive Cancellation**: A sophisticated loop filter that estimates plant gain in real-time and decouples control actions from state estimation for faster, more stable locking.
- **Tile-based Visualizer**: A "dirty-rect" rendering engine for SSD1306 OLEDs that minimizes SPI traffic by only updating modified 4x4 pixel tiles.
- **High Precision**: Implements Kahan summation in the PLL integrator to prevent floating-point truncation errors during long-term operation.

## 🏗 Architecture

### 1. Data Acquisition & Resampling
The system decouples hardware sampling from DSP processing:
- **DMA Buffer**: The ADC continuously fills frames (32 bytes each). Each frame is timestamped using the CPU `ccount` register in an ISR.
- **Target Timing**: The `loop()` calculates the next "ideal" sample time based on the current frequency estimate.
- **Interpolation Engine**: The `interpolateSampleAtTime` function searches through recent DMA frames to find the samples bounding the target time and performs linear interpolation.

### 2. DSP Pipeline (SOGI-PLL)
- **Modular SOGI**: The `SOGI` class implements a discrete-time band-pass filter (Alpha) and quadrature low-pass filter (Beta). It supports both single-step and windowed batch processing.
- **Adaptive Loop Filter**: The `AdaptivePLL` class extends the standard PI loop with an LMS-based gain estimator. It uses a **Predictive Cancellation Model** where it subtracts the expected effect of previous control actions from the current error before integration.
- **DC Offset Tracking**: Dynamically tracks and removes DC bias from both voltage and current channels using windowed averages.

### 3. Visualization System
The `SOGIVisualizer` provides high-frame-rate feedback:
- **TileManager**: Divides the 128x64 screen into 4x4 tiles. Only tiles that have changed are transmitted over SPI using LovyanGFX.
- **Phase Alignment**: Waveforms are automatically aligned to the detected phase, providing a stable "oscilloscope" view even as frequency varies.
- **Dual Trace**: Displays Voltage (solid line) and Current (dotted line) simultaneously.

## 📂 File Structure

- `modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt.ino`: Core orchestration, DMA setup, and timing logic.
- `SOGI.h` / `SOGI.cpp`: Modular implementation of SOGI filters and Adaptive PLL logic.
- `SOGIvisualizer.h` / `SOGIvisualizer.cpp`: Tiled rendering engine and waveform plotting logic.
- `FastMathToolkit.h`: Optimized math utilities, including a ring-table based float multiplier (currently auxiliary).
- `analog.h` / `analog.cpp`: Legacy one-shot ADC driver and calibration (maintained for compatibility/reference).

## 🛠 Technical Specifications

| Parameter | Value |
| :--- | :--- |
| **Nominal Frequency** | 50.0 Hz |
| **ADC Sampling Rate** | 200 kHz (Continuous DMA) |
| **Samples per Cycle** | 128 |
| **CPU Frequency** | 240 MHz (Recommended) |
| **Display** | SSD1306 (SPI, 128x64) |
| **SOGI Gain (K)** | 0.7071 |
| **PLL Gains** | Kp=1.25, Ki=0.00 |

## 📈 Performance Metrics
As per the serial debug output, the system typically achieves:
- **Core Processing Time**: ~30-50µs per cycle processing window.
- **Jitter**: Low-microsecond range due to `ccount` based scheduling.
- **DMA Buffer Utilization**: Monitored in real-time to ensure no frame loss.

---
*Part of the SOGI-PLL Grid Synchronization Project.*
