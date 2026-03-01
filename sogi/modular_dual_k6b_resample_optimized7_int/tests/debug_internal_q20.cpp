#include "SOGI.h"
#include <iostream>
#include <cmath>

int main() {
    const float k = 0.7071f;
    const float omega = 2.0f * M_PI * 50.0f;
    const float ts = 1.0f / 6400.0f;

    float wts  = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    float a_b0_f =  2.0f * k_wts * det;
    float a_a1_f =  2.0f * (wts2 - 4.0f) * det;
    float a_a2_f =  (4.0f - 2.0f * k_wts + wts2) * det;

    q30_t a_b0 = FLOAT_TO_Q30(a_b0_f);
    q30_t a_a1 = FLOAT_TO_Q30(a_a1_f);
    q30_t a_a2 = FLOAT_TO_Q30(a_a2_f);

    q20_t wz1 = 0, wz2 = 0;
    q16_t u = FLOAT_TO_Q16(1000.0f);
    q20_t u_20 = Q16_TO_Q20(u);

    for(int i=0; i<5; i++) {
        // Corrected signs and check logic
        float expected_in = 1000.0f - (a_a1_f * (float)wz1/(1<<20)) - (a_a2_f * (float)wz2/(1<<20));
        q20_t in = u_20 - (q20_t)(((int64_t)wz1 * a_a1) >> 30) - (q20_t)(((int64_t)wz2 * a_a2) >> 30);
        q20_t out = (q20_t)(((int64_t)in * a_b0) >> 30) - (q20_t)(((int64_t)wz2 * a_b0) >> 30);
        wz2 = wz1;
        wz1 = in;
        std::cout << "i=" << i << " in=" << (float)in/(1<<20) << " exp_in=" << expected_in << " out=" << (float)out/(1<<20) << std::endl;
    }

    return 0;
}
