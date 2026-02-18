# Jitter-Free Timer-Driven SOGI-PLL

## Overview
Uses ESP32 hardware timers (ISR-driven) to perform sampling at exactly 10kHz, eliminating the timing jitter caused by FreeRTOS task switching.

## Key Features
- Hardware timer interrupt-driven sampling.
- Elimination of task-switching jitter.
- Predictive control model integration.
- Hardened timing logic for high-precision applications.
