#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_deterministic_pll() {
    std::cout << "Testing DeterministicPLL tracking..." << std::endl;
    const float nominal_freq = 50.0f;
    const float target_freq = 51.5f;
    const float ts = 1.0f / 6400.0f;
    DeterministicPLL pll(nominal_freq, 0, 0);
    SOGI sogi(0.7071f);

    float phase = 0;
    for (int i = 0; i < 6400 * 2; ++i) {
        phase += 2.0 * M_PI * target_freq * ts;
        float val = 1000.0f * sin(phase);

        sogi.step(FLOAT_TO_Q16(val), 2.0 * M_PI * nominal_freq, ts);
        pll.update(sogi.v_alpha, sogi.v_beta, ts);
    }

    std::cout << "  Final Freq: " << pll.freq << std::endl;
    assert(std::abs(pll.freq - target_freq) < 0.2f);
    std::cout << "  Deterministic PLL Test Passed!" << std::endl;
}

int main() {
    try {
        test_deterministic_pll();
        std::cout << "All tests passed!" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
