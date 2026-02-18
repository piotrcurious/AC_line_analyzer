# Modular Dual with Optimized Analog Driver

## Overview
A refinement of the modular series focusing on ADC performance. It introduces a legacy ADC wrapper to bypass overhead in the standard Arduino `analogReadMillivolts`.

## Key Features
- Optimized `analog.cpp/h` wrapper.
- Fast ADC reads with cached calibration data.
- OneShot ADC driver fixes for ESP32 Arduino Core 3.0+ compatibility.
- Supports dual-channel synchronous-like sampling.
