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
    float a_a1_f = 2.0f * (wts2 - 4.0f) * det;

    q16_t u = FLOAT_TO_Q16(1000.0f);
    q30_t a_a1 = FLOAT_TO_Q30(a_a1_f);
    q20_t wz1 = 0;

    std::cout << "a_a1_f: " << a_a1_f << std::endl;
    std::cout << "a_a1_q30: " << a_a1 << std::endl;

    wz1 = Q16_TO_Q20(u);
    std::cout << "u: " << u << " wz1: " << wz1 << std::endl;

    int64_t prod = (int64_t)wz1 * a_a1;
    std::cout << "prod: " << prod << std::endl;
    int32_t res = (int32_t)(prod >> 30);
    std::cout << "res: " << res << std::endl;

    float res_f = (float)res / (1<<20);
    std::cout << "res_f: " << res_f << " expected: " << a_a1_f * 1000.0f << std::endl;

    return 0;
}
