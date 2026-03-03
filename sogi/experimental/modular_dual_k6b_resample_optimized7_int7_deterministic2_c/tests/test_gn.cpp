#include "../SOGI.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstdio>

int main() {
    float nominal_f = 50.0f;
    int K = 3;
    int N = 128;
    SlidingGNAnalyzer gn(nominal_f, K, N);

    float fs = 6400.0f;
    float ts = 1.0f / fs;
    float true_f = 50.2f;
    float true_phase = 0.5f;
    float true_offset = 0.1f;
    float true_mag1 = 1.0f;
    float true_mag3 = 0.3f;
    float true_phase3 = -1.2f;

    std::cout << "Final Verification of GN Solver..." << std::endl;

    int samples_per_cycle = 128;
    for (int cycle = 0; cycle < 10; ++cycle) {
        for (int s = 0; s < samples_per_cycle; ++s) {
            float t = (cycle * samples_per_cycle + s) * ts;
            float signal = true_offset
                         + true_mag1 * std::cos(2.0 * M_PI * true_f * t + true_phase)
                         + true_mag3 * std::cos(2.0 * M_PI * 3.0 * true_f * t + true_phase3);

            q16_t s_q16 = FLOAT_TO_Q16(signal);
            gn.addSample(s_q16, ts);
        }

        bool converged = gn.solve(12);

        float m1 = sqrtf(gn.ReC()[0]*gn.ReC()[0] + gn.ImC()[0]*gn.ImC()[0]);
        float m3 = sqrtf(gn.ReC()[2]*gn.ReC()[2] + gn.ImC()[2]*gn.ImC()[2]);
        printf("Cycle %d: F=%.4f, Off=%.4f, M1=%.4f, M3=%.4f, Conv=%d\n",
            cycle, gn.grid_freq, gn.offset, m1, m3, converged);
    }

    if (std::abs(gn.grid_freq - true_f) < 0.01) {
        std::cout << "TEST PASSED" << std::endl;
        return 0;
    } else {
        std::cout << "TEST FAILED" << std::endl;
        return 1;
    }
}
