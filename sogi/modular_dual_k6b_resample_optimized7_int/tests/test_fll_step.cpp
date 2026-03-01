#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_fll_step() {
    std::cout << "Testing FLL step with gamma..." << std::endl;
    SOGI sogi(0.7071f);
    float ts = 1.0f / 6400.0f;
    float omega_sogi = 2.0 * M_PI * 50.0f;
    float gamma = 50.0f;

    // Grid at 52Hz
    float phase = 0;
    for (int i = 0; i < 6400 * 2; ++i) {
        phase += 2.0 * M_PI * 52.0f * ts;
        float u = 1000.0f * sin(phase);
        sogi.step(FLOAT_TO_Q16(u), omega_sogi, ts);

        float u_f = Q16_TO_FLOAT(sogi.last_u);
        float va_f = Q16_TO_FLOAT(sogi.v_alpha);
        float vb_f = Q16_TO_FLOAT(sogi.v_beta);

        // FLL update: d(omega)/dt = -gamma * v_beta * (u - v_alpha)
        float err = (u_f - va_f) * vb_f;
        omega_sogi -= gamma * err * ts;

        if (i % 1000 == 0)
            std::cout << "i=" << i << " Freq=" << omega_sogi / (2.0 * M_PI) << std::endl;
    }
}

int main() {
    test_fll_step();
    return 0;
}
