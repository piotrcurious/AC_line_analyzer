# SOGI-PLL Paradigm Shift: Timer-Driven (No Jitter)

This version represents a shift from software-loop sampling to precise hardware-timer-driven sampling.

## Features
- **Hardware Timers**: Uses ESP32 hardware timers for ADC sampling at exactly 10kHz.
- **Zero Jitter**: Eliminates sampling jitter caused by FreeRTOS task scheduling or other background tasks.
- **High Precision**: Significantly improves frequency estimation accuracy.

## Status
- **Maturity**: Optimized / Advanced
- **Architectural Shift**: Core timing logic moved to ISR (Interrupt Service Routine).
