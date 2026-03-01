#include "../SOGI.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

int main() {
    const float NOMINAL_FREQ = 50.0f;
    const float SOGI_K = 0.7071f;
    const float PLL_KP = 2.55f;
    const float PLL_KI = 0.0f;
    const int SAMPLES_PER_CYCLE = 128;
    const float TS = 1.0f / (NOMINAL_FREQ * SAMPLES_PER_CYCLE);
    const float DC_OFFSET_TRUE = 1650.0f;
    const float AMPLITUDE = 1000.0f;

    SOGI sogi(SOGI_K);
    AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI);

    // Initial guess: 1500.0f
    q16_t v_dc_offset = FLOAT_TO_Q16(1500.0f);

    std::ofstream csv("test_results.csv");
    csv << "time,input,v_alpha,v_beta,dc_offset,freq\n";

    for (int i = 0; i < SAMPLES_PER_CYCLE * 200; ++i) {
        float t = i * TS;
        float input_f = DC_OFFSET_TRUE + AMPLITUDE * sin(2.0 * M_PI * NOMINAL_FREQ * t);
        q16_t input_q = FLOAT_TO_Q16(input_f);

        // This simulates the FIXED code from .ino
        // The .ino code updates every SAMPLES_PER_CYCLE, but for per-sample test we use a slower alpha
        // If it updates every 128 samples by 1/50, it's roughly 1/6400 per sample.
        v_dc_offset += (input_q - v_dc_offset) / 6400;

        sogi.step(input_q - v_dc_offset, pll.omega, TS);
        pll.update(sogi.v_alpha, sogi.v_beta, TS);

        if (i % 5000 == 0) {
            std::cout << "Step " << i << ": Input=" << input_f
                      << " DC_Est=" << Q16_TO_FLOAT(v_dc_offset) << std::endl;
        }
    }

    csv.close();
    std::cout << "Final DC_Est: " << Q16_TO_FLOAT(v_dc_offset) << " (True: " << DC_OFFSET_TRUE << ")" << std::endl;
    return 0;
}
