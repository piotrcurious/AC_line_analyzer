#include "SOGI.h"
#include <iostream>
#include <cmath>

int main() {
    q16_t u = FLOAT_TO_Q16(1000.0f);
    q30_t a_a1 = FLOAT_TO_Q30(-1.96354);
    q20_t wz1 = 0;

    int64_t prod = (int64_t)wz1 * a_a1;
    int32_t res = (int32_t)(prod >> 30);

    std::cout << "u: " << u << std::endl;
    std::cout << "a_a1: " << a_a1 << std::endl;

    wz1 = Q16_TO_Q20(u);
    prod = (int64_t)wz1 * a_a1;
    res = (int32_t)(prod >> 30);
    std::cout << "wz1: " << wz1 << " prod: " << prod << " res: " << res << std::endl;

    return 0;
}
