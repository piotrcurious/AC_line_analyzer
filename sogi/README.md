# SOGI-PLL Grid Synchronization Project

## Overview
This repository contains a diverse collection of Single-Phase Phase-Locked Loop (PLL) implementations based on the Second-Order Generalized Integrator (SOGI). Rather than a simple linear progression, these versions represent different architectural approaches and combinations of sampling, timing, and control strategies.

## 📊 Technical Taxonomy & Feature Matrix

The following table categorizes the different implementations across five key technical axes:

| Version / Folder | Sampling | Timebase | Filter | PLL / Feedback | DC Removal |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **[Apex (k6b)](./k6b_vis7t4dma3_z_cycles_opt1safe2_sogi4_adaptive3_dc_nodc_OK)** | Polling (ccount) | Cycle-Accurate | Tustin DFII | Predictive + Kahan | Windowed |
| **[Production](./sogi_pll_production)** | Timer ISR | Hardware Timer | Forward Euler | Standard PI | EMA |
| **[DMA-Backed](./modular_dual_k6b_analogread_oneshot_fix4_debug2_ok_dz_dma2_ok)** | DMA (Continuous) | Bresenham/Interp | Forward Euler | Adaptive (LMS) | EMA + Window |
| **[Resampled](./vis_sogi4_adaptive3_dc_running)** | Polling/Buffering | Virtual/Confidence | Tustin | Standard PI | Coasting EMA |
| **[No-Jitter](./sogi_pll_paradigm_shift_timer_02_nojitter)** | Polling (ccount) | Dynamic dt | Tustin | Standard PI | EMA |
| **[EKF](./sogi_ekf10_tuned_refined_new_ekf)** | Polling/ISR | Standard | EKF Observer | EKF Estimated | Internal |
| **[Dual-Channel](./modular_dual)** | Polling (ccount) | CPU Cycles | Forward Euler | Standard PI | Windowed |
| **[Inverter](./grid_tie_inverter)** | Timer ISR | esp_timer | Forward Euler | Standard PI | EMA |

---

## 📂 Variant Clusters

The repository contains several experimental branches and tuning variants:

- **Modular Variants (`modular_dual_*`)**: Focus on class-based encapsulation and multi-channel handling. Includes fixes for different ESP32 Arduino Core versions (e.g., `oneshot_fix`).
- **Visualization Variants (`vis_*`, `k4_`, `k6_`)**: Focused on high-fidelity OLED rendering. These explore different "dirty-rect" algorithms (TileManager) and phase-aligned plotting.
- **Paradigm Shift Variants**: Explore moving away from fixed-rate sampling toward cycle-accurate dynamic `dt` integration.

---

## 🛠 Architectural Approaches

### 1. Sampling & Acquisition
- **Polling (ccount)**: Low-latency access to the ESP32 CPU cycle counter allows for sub-microsecond timing accuracy without ISR overhead.
- **Hardware Timer ISR**: Provides a rock-solid, fixed-frequency sampling clock (e.g., 10kHz), ideal for production stability.
- **DMA (Continuous Mode)**: Offloads ADC acquisition to hardware. Critical for high-bandwidth applications, but requires re-sampling/interpolation to align with the PLL's phase clock.

### 2. Timebase Management
- **Dynamic dt**: Instead of assuming a fixed sample rate, the system measures the exact cycles elapsed between samples and feeds this delta into the discrete integrator, making it immune to OS-induced jitter.
- **Bresenham Scheduling**: Distributes samples with integer cycle counts while maintaining a perfect long-term average frequency.
- **Interpolation / Resampling**: Reconstructs an ideal uniform timebase from jittery or asynchronous hardware samples.

### 3. Filter Discretization
- **Forward Euler**: Simple and computationally cheap. Effective at high oversampling ratios.
- **Tustin (Bilinear) Transformation**: Provides superior phase and amplitude mapping near the Nyquist frequency. Often implemented in Direct Form II (DFII) to minimize state memory.

### 4. PLL Loop Dynamics
- **Standard PI**: The industry-standard approach. Robust and well-understood.
- **Adaptive PLL (LMS)**: Uses Least Mean Squares to estimate the "plant gain" in real-time, allowing the PLL to stay tuned even as signal amplitude or noise floor changes.
- **Predictive Cancellation**: Models the expected phase shift from a frequency correction. By subtracting this "predicted effect" from the error, the system avoids self-excitation and achieves faster locking.
- **EKF (Extended Kalman Filter)**: The most advanced observer. It models the entire system as a non-linear state space, providing optimal estimation of frequency, phase, and amplitude simultaneously.

### 5. Precision & Stability
- **Kahan Summation**: Used in the Loop Filter's integrator to preserve precision of tiny error updates that would otherwise be lost to floating-point truncation.
- **Confidence Weighting**: In resampled versions, it allows the PLL to "coast" through data gaps by trusting its internal resonance more than a missing or noisy sample.

---

## 🔮 Future Integration
Future versions of this system will focus on networked grid management:
- **UDP Multicast Visualization**: Broadcasting high-fidelity waveform frames over the network for remote monitoring.
- **Grid Management API**: Exposing synchronized frequency and phase data to higher-level microgrid control systems.
- **Time-Synchronized Meshes**: Coordinating multiple ESP32 nodes to perform distributed grid analysis.

---
*Documentation generated for SOGI-PLL project development.*
