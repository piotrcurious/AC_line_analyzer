# Adaptive SOGI with DC Offset Compensation

## Overview
A high-fidelity visualization version that implements a Tustin-transformed SOGI filter and real-time DC offset tracking using an EMA filter.

## Key Features
- Tustin-transformed SOGI (Biquad DFII implementation).
- EMA-based DC offset tracking (DC_ALPHA = 0.0002).
- High-performance OLED visualization with `SOGIVisualizer`.
- Frequency-adaptive timing (SAMPLES_PER_CYCLE = 512).
