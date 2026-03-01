#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_process_window() {
    std::cout << "Testing SOGI::processWindow..." << std::endl;
    const float freq = 50.0f;
    const float omega = 2.0 * M_PI * freq;
    const float ts = 1.0f / 6400.0f;
    const float amplitude = 1000.0f;

    SOGI sogi_step(0.7071f);
    SOGI sogi_window(0.7071f);

    std::vector<q16_t> input(128);
    for (int i = 0; i < 128; ++i) {
        input[i] = FLOAT_TO_Q16(amplitude * sin(omega * i * ts));
    }

    // Process using step
    for (int i = 0; i < 128; ++i) {
        sogi_step.step(input[i], omega, ts);
    }

    // Process using processWindow
    sogi_window.processWindow(input.data(), 128, 0, 128, omega, ts);

    std::cout << "  Step: Alpha=" << Q16_TO_FLOAT(sogi_step.v_alpha) << " Beta=" << Q16_TO_FLOAT(sogi_step.v_beta) << std::endl;
    std::cout << "  Window: Alpha=" << Q16_TO_FLOAT(sogi_window.v_alpha) << " Beta=" << Q16_TO_FLOAT(sogi_window.v_beta) << std::endl;

    assert(std::abs(sogi_step.v_alpha - sogi_window.v_alpha) <= 1);
    assert(std::abs(sogi_step.v_beta - sogi_window.v_beta) <= 1);

    std::cout << "  SOGI::processWindow Test Passed!" << std::endl;
}

int main() {
    test_process_window();
    return 0;
}
