# SOGI-PLL Project Evolution

This directory tracks the development of Second-Order Generalized Integrator (SOGI) Phase-Locked Loop (PLL) systems on the ESP32. The project has evolved from simple monolithic scripts to advanced, modular, and production-ready architectures.

## 🏗️ Architectural Stages

The following stages represent the major milestones in the project's development.

### Stage 0: Baseline Monolithic
The simplest implementations focusing on core mathematical correctness.
*   **[classic.ino](./classic.ino)**: The minimal ISR-based implementation. It uses a hardware timer to drive a 10kHz sampling loop, performing all DSP in the interrupt context.
*   **[sogi_pll_tests](./sogi_pll_tests)**: A validation suite for verifying core algorithm behavior.

### Stage 1: Monolithic Research & Visualization (`vis_...`)
Early research into frequency adaptation and real-time visualization on SSD1306 displays. These versions typically use software-timed loops for sampling.
*   **[vis_sogi4_adaptive3](./vis_sogi4_adaptive3)**: Initial frequency-adaptive sampling.
*   **[vis_sogi4_adaptive3_dc_running](./vis_sogi4_adaptive3_dc_running)**: Added dynamic DC offset tracking via EMA filters.
*   **[vis4_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK](./vis4_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK)**: Optimization for power-of-two buffer sizes and the introduction of `SOGIResampler`.
*   **[vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2](./vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2)**: The pinnacle of the monolithic research branch, featuring high-performance visualization and detailed CPU timing accounting.

### Stage 2: Modular Transformation (`modular_dual_...`)
A major architectural shift moving DSP logic into reusable C++ classes (`SOGI`, `FrequencyAdaptivePLL`). This stage introduced simultaneous Voltage and Current (V/I) monitoring.
*   **[modular_dual](./modular_dual)**: The first refactor into a class-based structure.
*   **[modular_dual_k6b_analogread_oneshot_fix4_debug2_ok_dz_dma2_ok](./modular_dual_k6b_analogread_oneshot_fix4_debug2_ok_dz_dma2_ok)**: Implementation of legacy ADC wrappers to bypass modern Arduino overhead and introduction of Bresenham-style sample scheduling.
*   **[modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune](./modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune)**: The most advanced modular version, featuring `AdaptivePLL` with predictive cancellation and Kahan summation for high precision.

### Stage 3: Hardened Production (`sogi_pll_production`)
Focusing on reliability, safety, and multi-core execution for industrial use.
*   **[sogi_pll_production](./sogi_pll_production)**: **Version 2.0.1**. This is the recommended version for most control applications. It uses a high-priority FreeRTOS task, hardware-timer-driven interrupts, and comprehensive error handling (Watchdog, Mutexes).
*   **[sogi_pll_production3c](./sogi_pll_production3c)**: A variant using `esp_timer` for compatibility with different ESP32 Arduino Core toolchains.

### Stage 4: Specialized Research & Paradigm Shifts
Exploring radical new approaches and advanced state estimation.
*   **[sogi_pll_paradigm_shift_timer_02_nojitter](./sogi_pll_paradigm_shift_timer_02_nojitter)**: A focus on zero-jitter sampling in software loops through precise cycle-count bookkeeping.
*   **[sogi_ekf10_tuned_refined_new_ekf](./sogi_ekf10_tuned_refined_new_ekf)**: Research into Extended Kalman Filters (EKF) for grid synchronization in extremely noisy environments.

---

## 📂 Version Map Summary

| Version Category | Key Feature | Representative Directory |
| :--- | :--- | :--- |
| **Monolithic** | Easy to read, single file | `vis_sogi4_adaptive3_dc_running` |
| **Modular** | Dual-channel (V/I), OO-Design | `modular_dual_k6b_..._tune` |
| **Production** | Reliable, RTOS, Mutex, WDT | `sogi_pll_production` |
| **Advanced** | EKF, Paradigm Timing | `sogi_ekf10_...` |

---

## 🗺️ Future Roadmap

The next phase of architectural evolution moves beyond local visualization:

1.  **UDP Multicast Frame Broadcasting**: The visualization data frames (waveform buffers and PLL metrics) will be broadcast over the network using UDP Multicast.
2.  **Remote Visualization**: Decoupling UI from the DSP core, allowing multiple remote monitors (PC/Web/Mobile) to visualize the grid state without adding latency to the control loop.
3.  **Grid Management Integration**: Facilitating the use of the SOGI-PLL system as a high-speed sensor node for larger-scale distributed grid monitoring and management systems.

---

## 📖 Related Documentation
*   [SOGI_PLL_DOCUMENTATION.md](SOGI_PLL_DOCUMENTATION.md): Detailed algorithm theory and tuning guidelines.
*   [COMPATIBILITY_NOTES.md](COMPATIBILITY_NOTES.md): Guidance on ESP32 Arduino Core versions (2.x vs 3.x).
