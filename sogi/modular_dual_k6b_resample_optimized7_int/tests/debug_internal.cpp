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

    std::cout << "a_b0: " << a_b0 << " a_a1: " << a_a1 << " a_a2: " << a_a2 << std::endl;

    // Internal state simulation
    float wz1 = 0, wz2 = 0;
    float u = 1000.0f;

    for(int i=0; i<5; i++) {
        float in = u - a_a1 * wz1 - a_a2 * wz2;
        float out = a_b0 * in - a_b0 * wz2;
        wz2 = wz1;
        wz1 = in;
        std::cout << "i=" << i << " in=" << in << " out=" << out << std::endl;
    }

    return 0;
}
