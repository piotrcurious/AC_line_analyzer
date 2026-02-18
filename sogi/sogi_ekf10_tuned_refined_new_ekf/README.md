# Extended Kalman Filter (EKF) SOGI

## Technical Specification

| Axis | Implementation |
| :--- | :--- |
| **Sampling** | High-speed polling/ISR |
| **Timebase** | Standard |
| **SOGI Filter** | Unified EKF State Observer |
| **PLL / Control** | EKF Frequency Estimation |
| **DC Removal** | Internal to EKF State |
| **Architecture** | Monolithic math-heavy implementation |

## Overview
Superior noise rejection and harmonic tracking compared to PI-loops. Optimal for highly distorted grid signals.

## Implementation Details
This version replaces the traditional SOGI filter and PI-PLL with a unified **Extended Kalman Filter (EKF)** state observer.

### Key Advantages:
- **Optimal State Estimation**: The EKF simultaneously estimates the grid frequency, phase, and amplitude by minimizing the mean square error of the signal model.
- **Harmonic Immunity**: Because it models the grid signal as a non-linear state space, it is naturally robust against higher-order harmonics and DC bias without needing separate pre-filters.
- **Fast Convergence**: Adjusts its gain dynamically (Kalman Gain) based on the estimated covariance of the signal and noise, allowing for faster locking than a fixed-gain PI controller.

### State Space Model:
The system models three states:
1. $x_1$: Estimated grid voltage.
2. $x_2$: Orthogonal grid voltage (quadrature).
3. $x_3$: Instantaneous grid frequency.

The transition matrix $f(x)$ uses the discrete dynamics of a sine-wave oscillator, and the Jacobian is calculated every step to linearize the system around the current state.
