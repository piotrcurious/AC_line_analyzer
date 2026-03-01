#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_atan2_pll() {
    std::cout << "Testing Atan2 PLL..." << std::endl;
    SOGI sogi(0.7071f);
    float ts = 1.0f / 6400.0f;
    float pll_freq = 50.0f;
    float pll_phase = 0;

    float grid_freq = 52.0f;
    float grid_phase = 0;

    // PI Controller
    float kp = 2.0f;
    float ki = 20.0f;
    float integral = 0;

    for (int i = 0; i < 6400 * 2; ++i) {
        grid_phase += 2.0 * M_PI * grid_freq * ts;
        float u = 1000.0f * sin(grid_phase);

        sogi.step(FLOAT_TO_Q16(u), 2.0 * M_PI * pll_freq, ts);
        float alpha = Q16_TO_FLOAT(sogi.v_alpha);
        float beta = Q16_TO_FLOAT(sogi.v_beta);

        // Estimated phase of grid
        float grid_phase_est = atan2(alpha, -beta);

        // We also track the PLL phase
        pll_phase += 2.0 * M_PI * pll_freq * ts;
        // wrap pll_phase to [-pi, pi]
        pll_phase = fmod(pll_phase + M_PI, 2.0 * M_PI);
        if (pll_phase < 0) pll_phase += 2.0 * M_PI;
        pll_phase -= M_PI;

        float err = grid_phase_est - pll_phase;
        if (err > M_PI) err -= 2.0 * M_PI;
        if (err < -M_PI) err += 2.0 * M_PI;

        integral += err * ts;
        pll_freq = 50.0f + kp * err + ki * integral;

        if (i % 2000 == 0)
            std::cout << "i=" << i << " GridPhaseEst=" << grid_phase_est << " PLLPhase=" << pll_phase << " Freq=" << pll_freq << std::endl;
    }
    std::cout << "Final Freq=" << pll_freq << std::endl;
}

int main() {
    test_atan2_pll();
    return 0;
}
