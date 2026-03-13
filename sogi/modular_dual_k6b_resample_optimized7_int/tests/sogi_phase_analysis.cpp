#include "SOGI.h"
#include <iostream>
#include <cmath>

int main() {
    const float f_grid = 52.0f;
    const float f_nom = 50.0f;
    const float k = 0.7071f;

    // SOGI BP transfer function phase shift
    auto bp_phi = [&](float fg, float fn) {
        float h = fg / fn;
        return atan2( k * h, 1.0f - h*h );
    };

    std::cout << "SOGI Phase shift at 50Hz: " << bp_phi(50.0, 50.0) * 180.0 / M_PI << " deg" << std::endl;
    std::cout << "SOGI Phase shift at 51Hz: " << bp_phi(51.0, 50.0) * 180.0 / M_PI << " deg" << std::endl;
    std::cout << "SOGI Phase shift at 52Hz: " << bp_phi(52.0, 50.0) * 180.0 / M_PI << " deg" << std::endl;

    return 0;
}
