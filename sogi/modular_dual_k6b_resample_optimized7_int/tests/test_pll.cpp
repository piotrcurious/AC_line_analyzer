#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_pll_direction() {
    std::cout << "Testing PLL frequency direction..." << std::endl;
    // kp = 10, nominal = 50.
    // raw_p_err is what?
    // vb_f * inv_mag.
    // In SOGI, at resonance, beta = -A * cos(theta_sig)
    // alpha = A * sin(theta_sig).
    // mag = A.
    // error = -cos(theta_sig).

    // THIS IS WRONG! A frequency-locked loop (FLL) should use the SOGI quadrature output
    // to determine the frequency error.
    // The standard SOGI-FLL frequency error is e = (u - v_alpha) * v_beta.
    // If u - v_alpha is the SOGI error, and v_beta is quadrature...

    // But this code is a SOGI-PLL.
    // In a SOGI-PLL, v_alpha and v_beta are transformed to a reference frame.
    // v_alpha * cos(theta_pll) + v_beta * sin(theta_pll) ? No.

    // Actually, the simple SOGI-PLL uses atan2(v_alpha, -v_beta) to get theta_sig.
    // But this code's update(alpha, beta) doesn't use theta_pll!

    // wait...
    // void IRAM_ATTR FrequencyAdaptivePLL::update(q16_t v_alpha, q16_t v_beta, float ts)
    // {
    //    ...
    //    float raw_p_err = vb_f * inv_mag;
    //    float control = kp * raw_p_err + integral_f;
    //    freq = nominal + control;
    // }

    // This is not a PLL. It's some kind of... thing.
    // If freq = 50 + kp * (-cos(theta)), the frequency will oscillate at 50Hz!
    // IT MUST BE SMOOTHED or integrated.

    // Let's re-read the original SOGI.cpp (the float version)
    // it was the same!
    // It used raw_p_err = v_beta * inv_mag.

    // WAIT! In a SOGI, the "PLL" usually refers to the thing that tracks the phase.
    // Here, it seems to be trying to adjust the SOGI'S OWN frequency to match the grid.
    // If it's a SOGI-FLL, the error is (u - v_alpha) * v_beta.

    // Let's see the original float version's update again.
    // raw_p_err = v_beta * inv_mag;
    // control = kp * raw_p_err + integral;

    // If v_beta = -A cos(theta), and you add it to frequency, it's definitely going to oscillate.
    // UNLESS it's being used as an FLL error.
    // In some SOGI-FLLs, the error is indeed proportional to v_beta when there's a phase shift.

    // Actually, look at the original code's AdaptivePLL:
    // dy = raw_p_err - prev_phase;
    // err_gain = dy - gain_est * prev_control;

    // This looks like it's trying to estimate the GAIN, not the frequency?
    // No, it sets freq = nominal + control.

    // THE BUG IS LIKELY HERE: In the original code, maybe it worked because of some property I'm missing,
    // or maybe it was always a bit shaky.

    // Let's try to implement a proper SOGI-FLL error.
    // e_f = (u - v_alpha) * v_beta.
    // But we don't have 'u' in the update(alpha, beta) call!

    // Wait, in SOGI, (u - v_alpha) = (1/k) * d(v_beta)/dt / omega ?

    // Let's check a standard SOGI-FLL reference.
    // The frequency update law is d(omega)/dt = -gamma * v_beta * (u - v_alpha).

    // If we only have v_alpha and v_beta...
    // At resonance, u = v_alpha. So error is 0.
    // If omega_sig > omega_sogi, v_alpha lags u, so u - v_alpha has a component in phase with v_beta?

    // Let's test the SOGI error (u - v_alpha) * v_beta.
}

int main() {
    test_pll_direction();
    return 0;
}
