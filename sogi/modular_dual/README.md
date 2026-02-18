# Modular Dual-Channel SOGI-PLL

A major architectural refactor introducing C++ classes for SOGI and PLL components.

## Features
- **Object-Oriented Design**: `SOGI` and `FrequencyAdaptivePLL` classes for modularity.
- **Dual-Channel Sampling**: Simultaneously tracks Voltage (V) and Current (I) signals.
- **SSD1306 Visualization**: Real-time display of both waveforms.

## Status
- **Maturity**: Functional / Refactored
- **Architectural Shift**: First move from monolithic `.ino` to class-based structure.
