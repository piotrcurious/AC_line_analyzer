# Grid-Tie Inverter Firmware

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Hardware Timer ISR (10kHz) |
| **Timebase** | esp_timer (Hardware-backed) |
| **SOGI Filter** | Forward Euler SOGI |
| **PLL / Control** | Standard PI (Park Transform) |
| **DC Removal** | Simple EMA |
| **Architecture** | Monolithic, RTOS-integrated |

## Overview
Includes H-Bridge/Push-Pull PWM control with hardware dead-time and safety checks (ROCOF, Over-voltage).

## Description
Detailed documentation for this specific implementation axis. This version represents a specialized approach to grid synchronization, focusing on the specific combination of the technical parameters listed above.
