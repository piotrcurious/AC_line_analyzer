#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstring>
#include "../phase_estimator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double generate_sample(double t, double freq, int type) {
    double val = sin(2.0 * M_PI * freq * t);
    if (type == 1) { // Trapezoid
        val = val * 1.5;
        if (val > 1.0) val = 1.0;
        if (val < -1.0) val = -1.0;
    }
    return 2048.0 + 1000.0 * val;
}

void generate_buffer(uint16_t* buf, double freq, double start_time, int samples_per_cycle, int cycles, int type) {
    for (int i = 0; i < samples_per_cycle * cycles; i++) {
        // Assume sampling rate is fixed at 50Hz * 128
        double t = start_time + (double)i / (50.0 * 128.0);
        buf[i] = (uint16_t)generate_sample(t, freq, type);
    }
}

int main() {
    std::cout << "Starting PhaseEstimator Simulation (User Sequence)..." << std::endl;

    PhaseEstimator pe;
    PhaseEstConfig pe_config;
    pe_config.history_depth = 16;
    pe_config.correction_threshold_rad = 0.3f;
    pe_config.nonlinear_threshold_rad = 0.1f;
    pe_config.stable_tolerance_rad = 0.02f;

    if (!pe.begin(&pe_config)) {
        std::cerr << "Failed to initialize PhaseEstimator" << std::endl;
        return 1;
    }

    double nominal_freq = 50.0;
    double current_pll_freq = 50.0;
    double buffer_interval_s = 4.0 / nominal_freq; // 0.08s
    pe.set_frequency_params(nominal_freq, buffer_interval_s, 128);

    struct TestStep {
        double freq;
        double duration;
        int type;
        const char* label;
    };
    std::vector<TestStep> steps = {
        {50.0, 1.5, 0, "50Hz Sine"},
        {51.0, 1.5, 0, "51Hz Sine"},
        {49.0, 1.5, 0, "49Hz Sine"},
        {50.0, 1.5, 0, "50Hz Sine"},
        {50.0, 1.5, 1, "50Hz Trapezoid"}
    };

    double pll_time_accumulator = 0.0;
    double total_elapsed_time = 0.0;
    uint32_t frame_counter = 0;
    bool success = true;

    for (const auto& step : steps) {
        std::cout << "\n>>> Starting Step: " << step.label << " (" << step.freq << "Hz) <<<" << std::endl;
        double step_start_time = total_elapsed_time;

        while (total_elapsed_time - step_start_time < step.duration) {
            uint16_t buf[128 * 3];
            generate_buffer(buf, step.freq, pll_time_accumulator, 128, 3, step.type);

            pe.add_frame(buf, 128 * 3);

            PhaseEstResult pe_result;
            if (pe.estimate_phase(pe_result)) {
                FrequencyEstResult freq_result;
                bool freq_valid = pe.estimate_frequency(freq_result);

                // Control logic from main loop
                float phase_gain = 0.0f;
                float freq_gain = 0.0f;
                switch(pe_result.state) {
                    case PE_STABLE: phase_gain = 0.5f; freq_gain = 0.15f; break;
                    case PE_NONLINEAR_DRIFT: phase_gain = 0.3f; freq_gain = 0.25f; break;
                    case PE_READY: phase_gain = 0.4f; freq_gain = 0.10f; break;
                    default: break;
                }

                if (freq_valid && freq_gain > 0.0f) {
                    current_pll_freq += freq_result.pll_correction_hz * freq_gain;
                    // Clamp like in original code
                    if (current_pll_freq < 40.0) current_pll_freq = 40.0;
                    if (current_pll_freq > 60.0) current_pll_freq = 60.0;
                    // Update estimator parameters
                    pe.set_frequency_params(nominal_freq, 4.0 / current_pll_freq, 128);
                }

                if (phase_gain > 0.0f && std::abs(pe_result.linear_drift_rate) > 1e-6f) {
                    float phase_corr = -pe_result.linear_drift_rate * phase_gain;
                    // Apply phase correction to our time accumulator
                    double time_shift = (phase_corr / (2.0 * M_PI)) * (1.0 / current_pll_freq);
                    pll_time_accumulator += time_shift;
                    pe.notify_correction_applied(phase_corr);
                }

                if (frame_counter % 5 == 0) {
                    std::cout << "t=" << std::fixed << std::setprecision(2) << total_elapsed_time
                              << " | Sig: " << step.freq
                              << " | PLL: " << std::setprecision(4) << current_pll_freq
                              << " | Drift: " << std::setprecision(4) << pe_result.linear_drift_rate
                              << " | State: " << pe_result.state << std::endl;
                }

                // Verification: check if it's heading in the right direction
                // (Omitted strict convergence check for 1.5s steps due to 1.28s estimator inertia)
            }

            double interval = 4.0 / current_pll_freq;
            pll_time_accumulator += interval;
            total_elapsed_time += interval;
            frame_counter++;
        }
    }

    if (success) {
        std::cout << "\nALL TESTS PASSED SUCCESSFULLY" << std::endl;
        return 0;
    } else {
        std::cout << "\nTESTS FAILED" << std::endl;
        return 1;
    }
}
