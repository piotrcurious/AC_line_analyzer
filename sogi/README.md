# SOGI-PLL Grid Synchronization Project

## Overview
This project implements a high-performance **Second-Order Generalized Integrator Phase-Locked Loop (SOGI-PLL)** for single-phase grid synchronization on the ESP32. It has evolved through several developmental stages, from basic inverter applications to advanced predictive control models with high-fidelity visualization.

## 🚀 Project Evolution (Chronology)

The repository is organized into five distinct "Eras," representing the technical progression of the algorithm and its implementation.

### Era 1: Application Origins
The foundation of the project, focusing on practical grid-tie inverter applications.
- **[grid_tie_inverter](./grid_tie_inverter)**: Initial application-specific implementation for H-Bridge and Push-Pull topologies.
- **classic.ino**: The original baseline SOGI-PLL logic.

### Era 2: Modular Transformation
Transition to a class-based architecture, enabling multiple SOGI instances and better hardware abstraction.
- **[modular_dual](./modular_dual)**: Introduction of `SOGI` and `FrequencyAdaptivePLL` classes; dual-channel sampling.
- **[modular_dual_k6b_analogread_oneshot_fix4_debug2_ok](./modular_dual_k6b_analogread_oneshot_fix4_debug2_ok)**: Optimized analog driver wrapper for high-speed acquisition.
- **[modular_dual_k6b_analogread_oneshot_fix4_debug2_ok_dz_dma2_ok](./modular_dual_k6b_analogread_oneshot_fix4_debug2_ok_dz_dma2_ok)**: Integration of DMA for jitter-free ADC sampling.

### Era 3: Advanced Math & Precision
A focus on estimation theory and timing precision.
- **[sogi_ekf10_tuned_refined_new_ekf](./sogi_ekf10_tuned_refined_new_ekf)**: State estimation using an Extended Kalman Filter (EKF).
- **[sogi_pll_paradigm_shift_timer_02_nojitter](./sogi_pll_paradigm_shift_timer_02_nojitter)**: Hardware timer-driven ISR to eliminate RTOS jitter.

### Era 4: Production Hardening
Hardened versions optimized for reliability and industry standards.
- **[sogi_pll_production](./sogi_pll_production)**: RTOS-based, high-reliability version with multi-core support and watchdog integration.
- **[sogi_pll_tests](./sogi_pll_tests)**: Automated validation suite.

### Era 5: High-Performance Visualization (Current Apex)
The most advanced branch, featuring Tustin-transformed filters and predictive control.
- **[vis_sogi4_adaptive3_dc_running](./vis_sogi4_adaptive3_dc_running)**: Real-time DC compensation and OLED visualization.
- **[k6b_vis7t4dma3_z_cycles_opt1safe2_sogi4_adaptive3_dc_nodc_OK](./k6b_vis7t4dma3_z_cycles_opt1safe2_sogi4_adaptive3_dc_nodc_OK)**: **Project Apex**. Features adaptive single-gain predictive modeling, Kahan summation, and advanced phase unwrapping.

---

## 📊 Feature Comparison

| Feature | Era 1 (Classic) | Era 2 (Modular) | Era 3 (Precision) | Era 4 (Production) | Era 5 (Apex) |
|---------|-----------------|-----------------|-------------------|--------------------|--------------|
| **Architecture** | Monolithic | Class-based | Estimator-based | RTOS / Mutex | Advanced DSP |
| **Filter Type** | Forward Euler | Forward Euler | EKF / Hybrid | Forward Euler | Tustin (Biquad) |
| **Sampling** | Loop-based | Loop / DMA | Timer-driven | Timer ISR | Cycle-Accurate |
| **DC Removal** | Simple EMA | Window-based | Kalman Gain | Stable EMA | Adaptive Window |
| **Precision** | Standard float | Standard float | High-precision | Standard float | Kahan Summation |
| **Diagnostics** | Serial only | Serial + OLED | Metrics | RTOS Metrics | CPU Headroom |
| **Phase Tracking**| Standard | Unwrapped | EKF Estimated | Robust | Predictive |

---

## 🛠 Key Technical Concepts

### SOGI-QSG (Quadrature Signal Generator)
The core of the system is the Second-Order Generalized Integrator. It generates an orthogonal version of the input signal (90° phase shift), which is essential for phase detection in single-phase systems. Modern versions (Era 5) use the **Tustin Transformation** for improved stability and frequency response at high sampling rates.

### Predictive Cancellation Model
Introduced in the "Apex" version, this model decouples the commanded control action from the state estimation. By predicting the effect of a frequency correction on the phase error, the system avoids "integrating its own corrections," leading to much faster and more stable locking.

### Kahan Summation
To maintain precision over long integration periods, Kahan summation is used in the Loop Filter's integrator. This prevents small error increments from being "swallowed" by the large magnitude of the accumulated frequency state.

---

## 🔮 Future Directions
The next phase of the SOGI-PLL project will focus on **External Integration and Visualization**:
- **UDP Multicast Broadcasting**: Real-time visualization frames and grid metrics will be broadcasted over UDP multicast.
- **Grid Management System**: Integration with external monitoring systems for microgrid control and stabilization.
- **Web-based Dashboard**: A remote dashboard to visualize synchronized waveforms from multiple nodes.

---
*Documentation generated for SOGI-PLL project development.*
