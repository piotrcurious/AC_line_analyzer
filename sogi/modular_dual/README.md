# Modular Dual-Channel SOGI

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Loop Polling (ccount) |
| **Timebase** | CPU Cycles (ccount) |
| **SOGI Filter** | Class-based SOGI (Euler) |
| **PLL / Control** | Class-based FrequencyAdaptivePLL (PI) |
| **DC Removal** | Windowed Average (Recalculated per cycle) |
| **Architecture** | Modular classes (SOGI.h/PLL.h) |

## Overview
Simultaneous tracking of Voltage and Current channels. Optimized for modularity and harmonic monitoring.

## Description
Detailed documentation for this specific implementation axis. This version represents a specialized approach to grid synchronization, focusing on the specific combination of the technical parameters listed above.
