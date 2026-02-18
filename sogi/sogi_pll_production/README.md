# SOGI-PLL Production Hardened Version

## Overview
The stable, production-ready snapshot of the SOGI-PLL system. Optimized for reliability, it features thread-safe data sharing, comprehensive diagnostics, and a watchdog timer. This version is designed for mission-critical grid synchronization tasks where stability and fault tolerance are paramount.

## 📋 Quick Start

### What You'll Need

**Hardware:**
- ESP32 development board (any variant with ADC1_CH0 and DAC_CH1)
- Grid signal source (50/60 Hz AC, scaled to 0-3.3V range)
- USB cable for programming

**Software:**
- Arduino IDE 2.x or PlatformIO
- ESP32 board support package

### Installation
1. Open `sogi_pll_production.ino` in Arduino IDE.
2. Select your ESP32 board from Tools → Board.
3. Upload the sketch.

### First Run
1. Open Serial Monitor (115200 baud).
2. Connect your grid signal to **GPIO36** (ADC input).
3. Monitor synchronized output on **GPIO25** (DAC output).

## ⚙️ Configuration

### Basic Setup
Edit `sogi_config.h` to customize for your application:
- `GRID_FREQ_NOMINAL`: Set to 50.0f or 60.0f.
- `PLL_KP` / `PLL_KI`: Adjust for tracking speed.
- `SAMPLING_FREQ_HZ`: Default is 10000.0f (10kHz).

### Pin Connections
| Function | ESP32 Pin |
|----------|-----------|
| ADC Input | GPIO36 |
| DAC Output| GPIO25 |

## 📊 Monitoring
The system provides real-time status via Serial:
`Freq: 50.023 Hz | Theta: 0.123 V | Out: 2.234 V | DC: 1.651 V | Errors: 0 [OK]`

## Key Features
- RTOS-based architecture (dedicated high-priority task).
- Hardware timer-driven ISR for fixed 10kHz sampling.
- Thread-safe data sharing with mutexes.
- Compatibility layer for ESP32 Arduino Core 2.x and 3.x.
- Extensive diagnostics and monitoring (stack, heap, error counts).
- **Anti-windup PI controller**: Prevents integrator saturation.
- **Watchdog Timer**: Auto-restarts on system hang.
