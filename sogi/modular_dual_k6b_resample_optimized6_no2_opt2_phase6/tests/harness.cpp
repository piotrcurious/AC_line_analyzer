#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../SOGI.h"

int main() {
    UnifiedSOGIAnalyzer analyzer(50.0f, 0.7071f, 2500.0f);

    double v_raw, i_raw;
    const double fs_hw = 250000.0;
    const double ts_hw = 1.0 / fs_hw;

    uint32_t samples_per_cycle = 128;
    const double target_virtual_rate = 6400.0;

    double f = 50.0;
    double ts_v = 1.0 / (f * (double)samples_per_cycle);
    double next_virt_offset = ts_v;
    double acc_v = 0;
    double current_hw_offset = 0;

    while (std::cin >> v_raw >> i_raw) {
        double next_hw_offset = current_hw_offset + ts_hw;

        while (next_virt_offset <= next_hw_offset) {
            double win_start = (current_hw_offset > (next_virt_offset - ts_v)) ?
                                current_hw_offset : (next_virt_offset - ts_v);
            double d_win = next_virt_offset - win_start;
            if (d_win > 0.0) {
                acc_v += v_raw * d_win;
            }

            float v_virt = (float)(acc_v / ts_v);

            analyzer.process(v_virt, (float)ts_v, samples_per_cycle);

            std::cout << "DATA," << v_virt << "," << analyzer.getFreq() << ","
                      << analyzer.getVAlpha() << "," << (v_virt - analyzer.getDC()) << ","
                      << analyzer.getGainEst() << "," << analyzer.getDC() << ","
                      << samples_per_cycle << "\n";

            acc_v = 0;

            // Adaptive parameters
            f = analyzer.getFreq();
            if (f < 10.0) f = 10.0;
            samples_per_cycle = (uint32_t)std::round(target_virtual_rate / f);
            samples_per_cycle = std::max(100u, std::min(256u, samples_per_cycle));

            ts_v = 1.0 / (f * (double)samples_per_cycle);
            next_virt_offset += ts_v;
        }

        double win_start = (current_hw_offset > (next_virt_offset - ts_v)) ?
                            current_hw_offset : (next_virt_offset - ts_v);
        double d_rem = next_hw_offset - win_start;
        if (d_rem > 0.0) {
            acc_v += v_raw * d_rem;
        }

        current_hw_offset = next_hw_offset;

        // Prevent drift and eventual precision loss in simulation
        if (current_hw_offset > 1.0) {
            current_hw_offset -= 1.0;
            next_virt_offset -= 1.0;
        }
    }

    return 0;
}
