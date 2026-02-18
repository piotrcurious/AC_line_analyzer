# Modular Dual-Channel SOGI-PLL

## Overview
This version introduces a class-based modular architecture, supporting concurrent SOGI instances for fundamental and harmonic tracking. It implements dual-channel sampling for both Voltage and Current.

## Key Features
- Modular `SOGI` and `FrequencyAdaptivePLL` classes.
- Dual-channel sampling (GPIO 36 and GPIO 39).
- Phase unwrapping and alignment for visualization.
- Real-time OLED visualization using `SOGIVisualizer`.
