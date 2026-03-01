#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_sogi_quadrature() {
    std::cout << "Testing SOGI quadrature output..." << std::endl;
    const float freq = 50.0f;
    const float omega = 2.0 * M_PI * freq;
    const float ts = 1.0f / 6400.0f;
    const float amplitude = 1000.0f;
    SOGI sogi(0.7071f);

    for (int i = 0; i < 6400; ++i) {
        float val = amplitude * sin(omega * i * ts);
        sogi.step(FLOAT_TO_Q16(val), omega, ts);
    }

    float alpha = Q16_TO_FLOAT(sogi.v_alpha);
    float beta = Q16_TO_FLOAT(sogi.v_beta);
    float mag = sqrt(alpha*alpha + beta*beta);

    std::cout << "  Final Alpha: " << alpha << " Beta: " << beta << " Mag: " << mag << std::endl;
    assert(std::abs(mag - amplitude) < amplitude * 0.05);
    std::cout << "  SOGI Quadrature Test Passed!" << std::endl;
}

void test_pll_frequency_tracking() {
    std::cout << "Testing PLL frequency tracking..." << std::endl;
    const float nominal_freq = 50.0f;
    const float target_freq = 52.0f;
    const float ts = 1.0f / 6400.0f;
    AdaptivePLL pll(nominal_freq, 5.0f, 50.0f);
    SOGI sogi(0.7071f);

    float phase = 0;
    for (int i = 0; i < 6400 * 2; ++i) {
        phase += 2.0 * M_PI * target_freq * ts;
        float val = 1000.0f * sin(phase);
        sogi.step(FLOAT_TO_Q16(val), pll.omega, ts);
        pll.update(sogi.v_alpha, sogi.v_beta, ts);

        if (i % 2000 == 0) {
            std::cout << "  i=" << i << " Freq=" << pll.freq << std::endl;
        }
    }

    std::cout << "  Final Freq: " << pll.freq << std::endl;
    assert(std::abs(pll.freq - target_freq) < 0.2f);
    std::cout << "  PLL Tracking Test Passed!" << std::endl;
}

int main() {
    try {
        test_sogi_quadrature();
        test_pll_frequency_tracking();
        std::cout << "All tests passed!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
