#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

int main() {
    const float k = 0.7071f;
    const float ts = 1.0f / 6400.0f;
    const float omega_sogi = 2.0f * M_PI * 50.0f;
    const float omega_sig = 2.0f * M_PI * 52.0f;

    SOGI sogi(k);

    float theta = 0;
    for (int i = 0; i < 2000; ++i) {
        theta += omega_sig * ts;
        float u = 1000.0f * sin(theta);
        sogi.step(FLOAT_TO_Q16(u), omega_sogi, ts);

        float alpha = Q16_TO_FLOAT(sogi.v_alpha);
        float beta = Q16_TO_FLOAT(sogi.v_beta);

        // FLL error: (u - alpha) * beta
        float err = (u - alpha) * beta;

        if (i % 200 == 0) {
            std::cout << "i=" << i << " u=" << u << " alpha=" << alpha << " beta=" << beta << " err=" << err << std::endl;
        }
    }
    return 0;
}
