# Grid-Tie Inverter Firmware for ESP32

This project provides a robust, real-time firmware solution for a single-phase grid-tie inverter (GTI) based on the ESP32 microcontroller. It features a high-performance Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL) for accurate grid synchronization and compile-time configuration for two common power stage topologies: **H-Bridge** and **Push-Pull**.

## Features

*   **Dual-Topology Support:** Compile-time selection between H-Bridge and Push-Pull power stage layouts.
*   **Real-Time Control:** Utilizes FreeRTOS tasks and `esp_timer` for precise, high-frequency control loops (10 kHz sampling rate).
*   **Grid Synchronization:** Implements a SOGI-PLL for fast and accurate tracking of grid voltage phase and frequency.
*   **Safety & Protection:** Includes basic safety checks for grid voltage, frequency limits, and Rate of Change of Frequency (ROCOF) for anti-islanding detection.
*   **Hardware Abstraction:** Uses ESP32's dedicated Motor Control PWM (MCPWM) peripheral for high-resolution PWM generation with hardware dead-time insertion.

## Topology Selection

The firmware is configured via a single `#define` in the `grid_tie_inverter.ino` file. **Only one option should be uncommented.**

| Define | Topology | Description |
| :--- | :--- | :--- |
| `#define TOPOLOGY_H_BRIDGE` | H-Bridge (Full-Bridge) | Requires four switching devices (MOSFETs/IGBTs). Provides bipolar output voltage. Used for full-bridge inverters. |
| `#define TOPOLOGY_PUSH_PULL` | Push-Pull | Requires two switching devices and a center-tapped transformer. Provides unipolar output voltage. |

## Hardware Configuration

The following pins are configured for the control and sensing:

| Function | Pin Name | ESP32 Pin | Notes |
| :--- | :--- | :--- | :--- |
| **PWM A** | `PIN_PWM_A` | 18 | High side 1 (H-Bridge) or Switch 1 (Push-Pull) |
| **PWM B** | `PIN_PWM_B` | 19 | High side 2 (H-Bridge) or Switch 2 (Push-Pull) |
| **PWM C** | `PIN_PWM_C` | 21 | Low side 1 (H-Bridge) - **H-Bridge Only** |
| **PWM D** | `PIN_PWM_D` | 22 | Low side 2 (H-Bridge) - **H-Bridge Only** |
| **Grid Voltage Sense** | `ADC_GRID_V_CHANNEL` | ADC1_CHANNEL_0 (GPIO 36) | Requires external voltage divider/isolation circuit. |
| **DC Bus Voltage Sense** | `ADC_BUS_V_CHANNEL` | ADC1_CHANNEL_3 (GPIO 39) | Currently commented out in the control loop but defined for future use. |
| **Output Current Sense** | `ADC_CURR_CHANNEL` | ADC1_CHANNEL_6 (GPIO 34) | Currently commented out in the control loop but defined for future use. |

## Control Parameters

The control loop is tuned for a 50 Hz grid. Adjust the following constants in `grid_tie_inverter.ino` as needed:

| Constant | Value (Default) | Description |
| :--- | :--- | :--- |
| `PWM_FREQ_HZ` | 20000.0f | PWM switching frequency (Hz). |
| `SAMPLING_FREQ_HZ` | 10000.0f | Control loop sampling frequency (Hz). |
| `GRID_FREQ_NOMINAL` | 50.0f | Nominal grid frequency (Hz). |
| `PLL_KP`, `PLL_KI` | 2.0f, 15.0f | Proportional and Integral gains for the PLL's loop filter. |
| `ROCOF_LIMIT` | 2.0f | Rate of Change of Frequency limit (Hz/s) for anti-islanding. |
| `V_GRID_NOMINAL_RMS` | 230.0f | Nominal RMS grid voltage (V). |
| `DEAD_TIME_NS` | 500.0f | Hardware dead-time insertion (nanoseconds). **Crucial for power stage safety.** |

## Disclaimer

**This code is provided for educational and experimental purposes only.** Grid-tie inverters deal with high voltages and require extensive safety testing, regulatory compliance, and specialized hardware design. **DO NOT** connect this firmware to a live grid without proper isolation, protection circuitry, and professional expertise. The control loop provided is a simplified sine-wave generator synchronized to the grid; a production-ready GTI requires a sophisticated current control loop (e.g., Proportional-Resonant controller) to regulate injected current and meet power quality standards.

---
*Firmware provided by Manus AI.*
