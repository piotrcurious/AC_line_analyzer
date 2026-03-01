#include "SOGI.h"
#include <iostream>
#include <cmath>

int main() {
    const float k = 0.7071f;
    const float ts = 1.0f / 6400.0f;
    const float omega = 2.0 * M_PI * 50.0f;

    auto sogi_mag = [&](float h) {
        float h2 = h*h;
        return (k*h) / sqrt( pow(1.0f-h2, 2) + pow(k*h, 2) );
    };

    // Correcting ratio calculation in my head:
    // mag1 = 1000 * gain(1.0) + 300 * gain(3.0)  <-- WRONG, they are at different frequencies
    // mag1_sogi sees:
    //   50Hz signal with gain 1.0 -> 1000
    //   150Hz signal with gain gain(3.0) -> 300 * 0.256 = 76.8
    //   Total mag approx sqrt(1000^2 + 76.8^2) = 1003

    // mag3_sogi (tuned to 150Hz) sees:
    //   50Hz signal with gain gain(1/3) -> 1000 * 0.256 = 256
    //   150Hz signal with gain gain(1) -> 300
    //   Total mag approx sqrt(256^2 + 300^2) = 394

    // Ratio = 394 / 1003 = 0.39

    // Why did I get 0.56?
    // Maybe SOGI beta (low pass) has different gain.

    auto sogi_beta_mag = [&](float h) {
        float h2 = h*h;
        return k / sqrt( pow(1.0f-h2, 2) + pow(k*h, 2) );
    };

    std::cout << "v_alpha gains: h=1: " << sogi_mag(1.0) << " h=3: " << sogi_mag(3.0) << " h=1/3: " << sogi_mag(1.0/3.0) << std::endl;
    std::cout << "v_beta gains:  h=1: " << sogi_beta_mag(1.0) << " h=3: " << sogi_beta_mag(3.0) << " h=1/3: " << sogi_beta_mag(1.0/3.0) << std::endl;

    return 0;
}
