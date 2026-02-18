# SOGI-PLL Project Evolution

This directory contains the evolution of the Second-Order Generalized Integrator (SOGI) Phase-Locked Loop (PLL) implementation for ESP32. It ranges from early monolithic experiments to modular, multi-channel production-ready systems.

## 🏗️ Architectural Evolution

The project has transitioned through four major architectural stages:

### 1. Monolithic / Experimental (`vis_...`)
- **Characteristics:** All-in-one `.ino` files, tight coupling between sampling, DSP, and visualization.
- **Key Versions:** `vis_sogi4_adaptive3`, `vis_sogi4_adaptive3_dc_running`.
- **Improvements:** Added dynamic DC offset tracking (EMA filters) and optimized cycle boundary detection.

### 2. Modular & Dual-Channel (`modular_dual_...`)
- **Architectural Shift:** DSP logic moved into C++ classes (`SOGI`, `FrequencyAdaptivePLL`).
- **New Features:** Support for simultaneous Voltage (V) and Current (I) sampling.
- **Key Versions:** `modular_dual`, `modular_dual_k6b_...`.
- **Bugfixes:** Resolved ADC one-shot driver compatibility issues with ESP32 Arduino Core 3.x.

### 3. Advanced Estimators & Jitter Reduction (`sogi_pll_paradigm_shift_...`, `sogi_ekf_...`)
- **Architectural Shift:** Transition from software loops to hardware-timer-driven sampling to eliminate jitter.
- **New Features:** Integration of Extended Kalman Filters (EKF) for more robust state estimation in noisy environments.
- **Key Versions:** `sogi_pll_paradigm_shift_timer_02_nojitter`, `sogi_ekf10_tuned_refined_new_ekf`.

### 4. Production Ready (`sogi_pll_production...`)
- **Characteristics:** Refined, standardized code following best practices.
- **Focus:** Stability, comprehensive documentation, error handling, and ease of use.
- **Key Versions:** `sogi_pll_production`.

---

## 📂 Directory Map

### 🚀 Production & Refined
- **[sogi_pll_production](./sogi_pll_production)**: The primary stable version for single-channel grid synchronization.
- **[sogi_pll_production3c](./sogi_pll_production3c)**: Refined multi-channel/advanced production candidate.
- **[grid_tie_inverter](./grid_tie_inverter)**: Application-specific implementation for inverter control.

### 🧩 Modular & Dual-Channel
- **[modular_dual](./modular_dual)**: Initial modular refactor.
- **[modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune](./modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune)**: The most advanced and tuned modular version.

### 🧪 Advanced & Experimental
- **[sogi_ekf10_tuned_refined_new_ekf](./sogi_ekf10_tuned_refined_new_ekf)**: High-performance EKF implementation.
- **[sogi_pll_paradigm_shift_timer_02_nojitter](./sogi_pll_paradigm_shift_timer_02_nojitter)**: Cleanest timing implementation using hardware timers.

### 📜 Legacy & Development
- **[vis_sogi4_adaptive3_dc_running](./vis_sogi4_adaptive3_dc_running)**: Baseline for early visualization.
- **[vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2](./vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2)**: Highly optimized legacy version.

---

## 🗺️ Future Roadmap

The next major architectural phase focuses on **Remote Monitoring and Grid Management**:

1.  **UDP Multicast Frame Broadcasting**: Currently, the "frames" of data (waveforms and PLL state) are sent to a local SSD1306 visualizer. The roadmap includes broadcasting these frames over UDP multicast.
2.  **Distributed Visualization**: This will allow multiple clients (PCs, tablets, or other ESP32s) to visualize the grid state simultaneously without impacting the DSP core.
3.  **Grid Management Integration**: By broadcasting data, the SOGI-PLL system can act as a high-speed sensor node for a larger grid management system, providing real-time frequency, phase, and harmonic data over the network.

---

## 📖 Key Documentation
- [SOGI_PLL_DOCUMENTATION.md](SOGI_PLL_DOCUMENTATION.md): Deep dive into the algorithm and tuning.
- [COMPATIBILITY_NOTES.md](COMPATIBILITY_NOTES.md): Guidance on ESP32 Arduino Core versions (2.x vs 3.x).
- [sogi_pll_tests](./sogi_pll_tests): System validation and unit tests.
