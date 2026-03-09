#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include "../SOGI.h"

int main() {
    AdaptivePLL pll(50.0f, 2.55f, 10.0f, 0.1001f);
    SOGI sogi_v(0.7071f);
    SOGI sogi_v3(0.7071f);

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

    double v_dc_offset = 1650.0;
    double harmonic_mag1_smooth = 1e-6;
    double harmonic_mag3_smooth = 1e-6;
    const double HARMONIC_SMOOTH_ALPHA = 0.05;

    while (std::cin >> v_raw >> i_raw) {
        double next_hw_offset = current_hw_offset + ts_hw;

        while (next_virt_offset <= next_hw_offset) {
            double win_start = (current_hw_offset > (next_virt_offset - ts_v)) ?
                                current_hw_offset : (next_virt_offset - ts_v);
            double d_win = next_virt_offset - win_start;
            if (d_win > 0.0) {
                acc_v += v_raw * d_win;
            } else if (acc_v == 0.0) {
                 acc_v = v_dc_offset * ts_v;
            }

            float v_virt = (float)(acc_v / ts_v);
            float u = v_virt - (float)v_dc_offset;

            sogi_v.step(u, pll.omega, (float)ts_v);
            sogi_v3.step(u, 3.0f * pll.omega, (float)ts_v);

            double mag1 = sqrt((double)sogi_v.v_alpha * sogi_v.v_alpha + (double)sogi_v.v_beta * sogi_v.v_beta);
            double mag3 = sqrt((double)sogi_v3.v_alpha * sogi_v3.v_alpha + (double)sogi_v3.v_beta * sogi_v3.v_beta);
            harmonic_mag1_smooth = (1.0 - HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + HARMONIC_SMOOTH_ALPHA * mag1;
            harmonic_mag3_smooth = (1.0 - HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + HARMONIC_SMOOTH_ALPHA * mag3;

            float ratio = (float)(harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9));
            float damp_factor = 1.0f;
            float learn_att   = 1.0f;

            if (ratio > 0.1f) {
                float overshoot = (ratio - 0.1f) / 0.3f;
                if (overshoot > 1.0f) overshoot = 1.0f;
                damp_factor = 1.0f - overshoot * (1.0f - 0.01f);
                learn_att   = 1.0f - overshoot;
                if (learn_att < 0.0f) learn_att = 0.0f;
            }
            pll.setDistortionDamping(damp_factor, learn_att);
            pll.update(sogi_v.v_alpha * damp_factor, sogi_v.v_beta * damp_factor, (float)ts_v);

            // Simple DC tracking
            v_dc_offset = 0.9995 * v_dc_offset + 0.0005 * (double)v_virt;

            std::cout << "DATA," << v_virt << "," << pll.freq << ","
                      << sogi_v.v_alpha << "," << u << ","
                      << pll.gain_est << "," << v_dc_offset << ","
                      << samples_per_cycle << "\n";

            acc_v = 0;

            f = pll.freq;
            if (f < 10.0) f = 10.0;
            uint32_t target_N = (uint32_t)std::round(target_virtual_rate / f);
            target_N = std::max(100u, std::min(256u, target_N));

            if (abs((int)target_N - (int)samples_per_cycle) > 2) {
                samples_per_cycle = target_N;
            }

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

        if (current_hw_offset > 1.0) {
            current_hw_offset -= 1.0;
            next_virt_offset -= 1.0;
        }
    }

    return 0;
}
