# DMA-Backed Adaptive SOGI-PLL

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | DMA (Continuous Mode) + ISR Timestamps |
| **Timebase** | Bresenham scheduling + Linear Interpolation |
| **SOGI Filter** | Class-based SOGI (Euler) |
| **PLL / Control** | AdaptivePLL (LMS Gain Estimation) |
| **DC Removal** | EMA-Smoothed Windowed Average |
| **Architecture** | DMA-driven Modular architecture |

## Overview
Decouples ADC acquisition from processing. Linear interpolation reconstructs a perfect timebase from jittery DMA frames.

## Description
Detailed documentation for this specific implementation axis. This version represents a specialized approach to grid synchronization, focusing on the specific combination of the technical parameters listed above.
