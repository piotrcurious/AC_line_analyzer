#include "SOGI.h"
#include <iostream>
#include <cmath>

int main() {
    const float k = 0.7071f;
    const float ts = 1.0f / 6400.0f;
    const float omega1 = 2.0 * M_PI * 50.0f;
    const float omega3 = 2.0 * M_PI * 150.0f;

    // Transfer function for v_alpha (Band Pass)
    auto bp_gain = [&](float target_w, float signal_w) {
        float h = signal_w / target_w;
        float h2 = h*h;
        return (k*h) / sqrt( pow(1.0f-h2, 2) + pow(k*h, 2) );
    };

    // Signal: 1000*sin(w1*t) + 300*sin(3*w1*t)

    // SOGI1 (tuned to w1)
    float sogi1_mag_at_w1 = 1000.0 * bp_gain(omega1, omega1);
    float sogi1_mag_at_w3 = 300.0 * bp_gain(omega1, omega3);
    float total1 = sqrt(sogi1_mag_at_w1*sogi1_mag_at_w1 + sogi1_mag_at_w3*sogi1_mag_at_w3);

    // SOGI3 (tuned to w3)
    float sogi3_mag_at_w1 = 1000.0 * bp_gain(omega3, omega1);
    float sogi3_mag_at_w3 = 300.0 * bp_gain(omega3, omega3);
    float total3 = sqrt(sogi3_mag_at_w1*sogi3_mag_at_w1 + sogi3_mag_at_w3*sogi3_mag_at_w3);

    std::cout << "SOGI1 response: at w1=" << sogi1_mag_at_w1 << " at w3=" << sogi1_mag_at_w3 << " total=" << total1 << std::endl;
    std::cout << "SOGI3 response: at w1=" << sogi3_mag_at_w1 << " at w3=" << sogi3_mag_at_w3 << " total=" << total3 << std::endl;
    std::cout << "Theoretical Ratio: " << total3 / total1 << std::endl;

    return 0;
}
