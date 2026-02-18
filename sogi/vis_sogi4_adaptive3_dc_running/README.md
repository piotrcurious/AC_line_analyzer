# Resampled Tustin SOGI Visualizer

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Loop Polling + Frame buffering |
| **Timebase** | Virtual Timebase with Confidence Weighting |
| **SOGI Filter** | Tustin (Bilinear) Transformation |
| **PLL / Control** | Standard PI |
| **DC Removal** | EMA with 'Coasting' logic |
| **Architecture** | Resampling-based Modular architecture |

## Overview
This implementation addresses the problem of non-uniform sampling (jitter) by decoupling hardware acquisition from the DSP pipeline. It features a "Virtual Timebase" that reconstructs a perfect 50Hz-relative sample clock.

## Key Innovation: Resampling with Confidence
The core logic in `3/process_sogi_resampled.cpp` implements a sophisticated interpolation loop:
1. **Linear Interpolation**: It finds the bounding ADC samples for a target `virtual_time` and interpolates the value.
2. **Confidence Metric**: It calculates how "trustworthy" the sample is based on the actual measured gap between ADC reads.
3. **Resonant Coasting**: If a data gap is detected (confidence < 1.0), the SOGI filter is fed a mixture of the measured value and its own previous output (`v_alpha`). At zero confidence, the filter effectively "coasts" on its own internal resonance, maintaining phase synchronization through CPU hiccups or signal drops.

## 📊 High-Performance Visualization
Includes the `TileManager` system which minimizes SPI traffic to the SSD1306 display by only updating "dirty" regions of the screen. This allows for high-frame-rate waveform plotting without starving the CPU of cycles needed for the PLL.
