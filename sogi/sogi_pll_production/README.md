# Production-Hardened SOGI-PLL

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | Hardware Timer ISR (10kHz) |
| **Timebase** | Hardware Timer Alarm |
| **SOGI Filter** | Forward Euler |
| **PLL / Control** | Standard PI |
| **DC Removal** | Stable EMA |
| **Architecture** | RTOS Task with Mutex & Watchdog |

## Overview
Designed for maximum reliability in industrial environments. Features multi-core data sharing, stack monitoring, and an IDF compatibility layer for both ESP32 Arduino Core 2.x and 3.x.

## 📋 Quick Start

### What You'll Need
- **Hardware**: ESP32 dev board, signal source (0-3.3V range).
- **Software**: Arduino IDE 2.x or PlatformIO.

### Installation & Run
1. Open `sogi_pll_production.ino`.
2. Select your ESP32 board.
3. Upload and open Serial Monitor (115200 baud).
4. Connect signal to **GPIO36**; output is on **GPIO25**.

## ⚙️ Key Safety Features
- **Watchdog Timer**: Auto-restarts system if the high-priority SOGI task hangs.
- **Mutex Protection**: Ensures thread-safe communication between the processing core and the monitoring loop.
- **Anti-Windup**: Prevents frequency integrator saturation during signal loss or transient grid faults.
- **Stack High-Water Mark Monitoring**: Periodically reports remaining stack space to prevent silent overflows.

## Description
This implementation is the "Golden Snapshot" for deployment. It prioritizes stability and deterministic timing over advanced experimental estimation techniques. It uses the standard Forward Euler discretization which is highly stable at the 10kHz sampling rate provided by the hardware timer.
