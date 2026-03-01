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

    float a_b0 =  2.0f * k_wts * det;
    float a_a1 =  2.0f * (wts2 - 4.0f) * det;
    float a_a2 =  (4.0f - 2.0f * k_wts + wts2) * det;

    std::cout << "wts: " << wts << " wts2: " << wts2 << " k_wts: " << k_wts << std::endl;
    std::cout << "det: " << det << std::endl;
    std::cout << "a_b0: " << a_b0 << " a_a1: " << a_a1 << " a_a2: " << a_a2 << std::endl;
    std::cout << "a_b0 (Q30): " << (int32_t)(a_b0 * (1<<30)) << std::endl;
    std::cout << "a_a1 (Q30): " << (int32_t)(a_a1 * (1<<30)) << std::endl;
    std::cout << "a_a2 (Q30): " << (int32_t)(a_a2 * (1<<30)) << std::endl;

    return 0;
}
