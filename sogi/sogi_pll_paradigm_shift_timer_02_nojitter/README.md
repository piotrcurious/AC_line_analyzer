# Jitter-Compensated Tustin SOGI

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Loop Polling (ccount) |
| **Timebase** | Dynamic dt (Elapsed CPU cycles per step) |
| **SOGI Filter** | Tustin (Bilinear) Transformation |
| **PLL / Control** | Standard PI |
| **DC Removal** | EMA |
| **Architecture** | Monolithic optimized loop |

## Overview
Eliminates OS-induced jitter errors by measuring exact time between samples and feeding it into the discrete integrator.

## Description
Detailed documentation for this specific implementation axis. This version represents a specialized approach to grid synchronization, focusing on the specific combination of the technical parameters listed above.
