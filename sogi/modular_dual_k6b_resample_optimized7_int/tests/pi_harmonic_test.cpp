#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void test_pi_integrator() {
    std::cout << "Testing PI integrator convergence..." << std::endl;
    const float nominal_freq = 50.0f;
    const float kp = 2.6f;
    const float ki = 50.0f;
    AdaptivePLL pll(nominal_freq, kp, ki);

    float ts = 1.0f / 6400.0f;

    for (int i = 0; i < 6400; ++i) {
        float grid_p = pll.phase + 0.1f;

        float va = 1000.0f * sin(grid_p);
        float vb = -1000.0f * cos(grid_p);

        pll.update(FLOAT_TO_Q16(va), FLOAT_TO_Q16(vb), ts);
    }

    std::cout << "  Final Freq after integration: " << pll.freq << std::endl;
}

void test_harmonic_rejection() {
    std::cout << "Testing harmonic rejection logic..." << std::endl;
    const float SOGI_K = 0.7071f;
    const float omega = 2.0 * M_PI * 50.0f;
    const float ts = 1.0f / 6400.0f;

    SOGI sogi_v(SOGI_K);
    SOGI sogi_v3(SOGI_K);

    float harmonic_mag1_smooth = 1e-6f;
    float harmonic_mag3_smooth = 1e-6f;
    const float alpha = 0.99f;

    for (int i = 0; i < 6400 * 2; ++i) {
        float t = i * ts;
        // Signal with 30% 3rd harmonic
        float u = 1000.0f * sin(omega * t) + 300.0f * sin(3.0 * omega * t);

        sogi_v.step(FLOAT_TO_Q16(u), omega, ts);
        sogi_v3.step(FLOAT_TO_Q16(u), 3.0f * omega, ts);

        float v1a = Q16_TO_FLOAT(sogi_v.v_alpha);
        float v1b = Q16_TO_FLOAT(sogi_v.v_beta);
        float v3a = Q16_TO_FLOAT(sogi_v3.v_alpha);
        float v3b = Q16_TO_FLOAT(sogi_v3.v_beta);

        float mag1 = sqrtf(v1a*v1a + v1b*v1b);
        float mag3 = sqrtf(v3a*v3a + v3b*v3b);

        if (i % 1000 == 0) {
            std::cout << "  i=" << i << " mag1=" << mag1 << " mag3=" << mag3 << std::endl;
        }

        harmonic_mag1_smooth = (1.0f - alpha) * harmonic_mag1_smooth + alpha * mag1;
        harmonic_mag3_smooth = (1.0f - alpha) * harmonic_mag3_smooth + alpha * mag3;
    }

    float ratio = harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f);
    std::cout << "  Final Ratio: " << ratio << " (Expected ~0.3)" << std::endl;
}

int main() {
    test_pi_integrator();
    test_harmonic_rejection();
    return 0;
}
