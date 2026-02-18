# SOGI-PLL Project Lineage

This directory documents the technical evolution of SOGI-PLL grid synchronization on the ESP32. Unlike standard versioning, the project's history is best understood through its transition from application-specific code to advanced modular and visualization-focused architectures.

## 📈 Technical Chronology

The project has evolved through five distinct eras, as verified by file metadata and code complexity.

### Era 1: Application Origins (`...808ns`)
Focused on direct power electronics control using legacy ADC drivers.
*   **[grid_tie_inverter](./grid_tie_inverter)**: The oldest ancestor. Uses Forward Euler discretization and simple PI frequency tracking.

### Era 2: Modular Transformation (`...820ns - 844ns`)
The primary architectural shift from monolithic scripts to reusable C++ classes.
*   **[modular_dual](./modular_dual)**: Introduced the `SOGI` and `FrequencyAdaptivePLL` classes.
*   **[modular_dual_k6b_analogread_oneshot_fix4_debug2_ok](./modular_dual_k6b_analogread_oneshot_fix4_debug2_ok)**: Solved ESP32 Core 3.x ADC "oneshot" driver overhead by implementing a custom `analog.h` legacy wrapper.
*   **[modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune](./modular_dual_k6b_fix4_debug2_ok_dz_dma2t6_OK_opt_tune)**: The math peak of the modular branch, featuring **AdaptivePLL** with predictive gain estimation and Kahan summation for high precision.

### Era 3: Advanced Math & Precision (`...844ns - 848ns`)
Experimental branches exploring radical improvements in state estimation and timing.
*   **[sogi_ekf10_tuned_refined_new_ekf](./sogi_ekf10_tuned_refined_new_ekf)**: Replaced standard PLLs with a **Heterodyne Extended Kalman Filter (EKF)** for robust lock in high-noise environments.
*   **[sogi_pll_paradigm_shift_timer_02_nojitter](./sogi_pll_paradigm_shift_timer_02_nojitter)**: Solved software-loop sampling jitter through precise CPU cycle-count bookkeeping and jitter-compensated integration.

### Era 4: The Production Snapshot (`...852ns - 856ns`)
A stabilized mid-project milestone focused on system hardening rather than DSP advancement.
*   **[sogi_pll_production](./sogi_pll_production)**: Standardizes the Era 2/3 logic into a hardened FreeRTOS environment with Watchdog protection, Mutex-guarded data sharing, and comprehensive error handling. *Note: Uses simpler Forward Euler math for maximum reliability/determinism.*

### Era 5: High-Performance Visualization (`...864ns - 880ns`)
The current project peak, optimizing for real-time waveform monitoring and grid diagnostics.
*   **[vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2](./vis7t4dma3_cycles_opt1safe2_sogi4_adaptive3_dc_running_OK_acc2)**: Introduced microsecond-level CPU timing accounting and high-speed waveform accumulation.
*   **[vis_sogi4_adaptive3_dc_running](./vis_sogi4_adaptive3_dc_running)**: **The Latest Evolution.** Features the "Adaptive3" PLL and a sophisticated dual-stage DC tracking system (separating sampling and processing offsets) to eliminate jitter in zero-crossing detection.

---

## 📂 Architectural Comparison

| Feature | Production (Era 4) | Advanced Vis (Era 5) | Modular Tune (Era 2) |
| :--- | :--- | :--- | :--- |
| **Math** | Forward Euler | Tustin (Trapezoidal) | Tustin + Adaptive Gain |
| **Timing** | Hardware Timer ISR | Software Jitter-Compensated | Bresenham Distributed |
| **ADC** | Legacy Direct | Arduino Standard | Optimized Legacy Wrapper |
| **Focus** | Stability/Uptime | Visualization Fidelity | Modular Flexibility |

---

## 🗺️ Roadmap: Distributed Monitoring

The Era 5 (`vis_sogi4`) architecture provides the foundation for the next major milestone:

1.  **UDP Multicast Frame Broadcasting**: Waveform "frames" currently used for local SSD1306 rendering will be broadcast over the network.
2.  **Remote Visualization**: Decoupling the UI from the DSP core, allowing external systems to monitor grid quality without impacting synchronization stability.
3.  **Grid Management Node**: Transitioning the system from a local controller to a networked sensor node for distributed grid management.

---

## 📖 Essential Documentation
*   [SOGI_PLL_DOCUMENTATION.md](SOGI_PLL_DOCUMENTATION.md): Deep dive into the SOGI-PLL math and tuning.
*   [COMPATIBILITY_NOTES.md](COMPATIBILITY_NOTES.md): Guidance on ESP32 Core 2.x/3.x compatibility layers.
