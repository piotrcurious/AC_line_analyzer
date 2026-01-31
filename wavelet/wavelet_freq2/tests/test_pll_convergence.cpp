#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstring>
#include "../phase_estimator.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Simulation parameters
const double NOMINAL_FREQ = 50.0;
const double STROBE_DIV = 4.0;
const int SAMPLES_PER_CYCLE = 128;
const int CYCLES_TO_CAPTURE = 3;
const int BUF_SZ = SAMPLES_PER_CYCLE * CYCLES_TO_CAPTURE;

double generate_sample(double t, double freq, int type) {
    double val = std::sin(2.0 * M_PI * freq * t);
    if (type == 1) { // Trapezoid
        val = val * 1.5;
        if (val > 1.0) val = 1.0;
        if (val < -1.0) val = -1.0;
    }
    return 2048.0 + 1000.0 * val;
}

int main() {
    std::cout << "Starting Robust PLL Convergence Test..." << std::endl;

    PhaseEstimator pe;
    PhaseEstConfig pe_config;
    pe_config.history_depth = 16;
    pe_config.correction_threshold_rad = 0.3f;
    pe_config.nonlinear_threshold_rad = 0.1f;
    pe_config.stable_tolerance_rad = 0.02f;
    pe.begin(&pe_config);

    double current_pll_freq = 50.0;
    double current_pll_time = 0.0;

    // Set initial params
    pe.set_frequency_params(NOMINAL_FREQ, STROBE_DIV / NOMINAL_FREQ, SAMPLES_PER_CYCLE, STROBE_DIV);

    struct TestStep {
        double freq;
        double duration;
        int type;
        const char* label;
    };
    std::vector<TestStep> steps = {
        {50.0, 1.0, 0, "50Hz Sine"},
        {51.0, 2.0, 0, "51Hz Sine"},
        {49.0, 2.0, 0, "49Hz Sine"},
        {50.0, 1.0, 0, "50Hz Sine"},
        {50.0, 1.0, 1, "50Hz Trapezoid"}
    };

    double total_time = 0.0;
    uint32_t frame_counter = 0;
    bool all_success = true;

    for (const auto& step : steps) {
        std::cout << "\n>>> STEP: " << step.label << " (" << step.freq << "Hz) <<<" << std::endl;
        double step_start_time = total_time;

        while (total_time - step_start_time < step.duration) {
            // Simulate buffer capture
            // The buffer starts at current_pll_time
            // Sampling rate is current_pll_freq * SAMPLES_PER_CYCLE
            uint16_t buf[BUF_SZ];
            double dt = 1.0 / (current_pll_freq * SAMPLES_PER_CYCLE);
            for (int i = 0; i < BUF_SZ; i++) {
                buf[i] = (uint16_t)generate_sample(current_pll_time + i * dt, step.freq, step.type);
            }

            // Update estimator with current conditions
            double interval = STROBE_DIV / current_pll_freq;
            pe.set_frequency_params(NOMINAL_FREQ, interval, SAMPLES_PER_CYCLE, STROBE_DIV);

            pe.add_frame(buf, BUF_SZ);

            PhaseEstResult pe_result;
            if (pe.estimate_phase(pe_result)) {
                FrequencyEstResult freq_result;
                bool freq_valid = pe.estimate_frequency(freq_result);

                float phase_gain = 0.0f;
                float freq_gain = 0.0f;
                switch(pe_result.state) {
                    case PE_STABLE: phase_gain = 0.5f; freq_gain = 0.15f; break;
                    case PE_NONLINEAR_DRIFT: phase_gain = 0.3f; freq_gain = 0.25f; break;
                    case PE_READY: phase_gain = 0.4f; freq_gain = 0.10f; break;
                    default: break;
                }

                // Apply frequency correction
                if (freq_valid && freq_gain > 0.0f) {
                    current_pll_freq += freq_result.pll_correction_hz * freq_gain;
                }

                // Apply phase correction
                if (phase_gain > 0.0f && std::abs(pe_result.linear_drift_rate) > 1e-4f) {
                    float phase_corr = pe_result.linear_drift_rate * phase_gain;
                    // Advancing phase means moving current_pll_time BACKWARDS relative to signal
                    current_pll_time -= (phase_corr / (2.0 * M_PI)) * (1.0 / current_pll_freq);
                    pe.notify_correction_applied(phase_corr);
                }

                if (frame_counter % 5 == 0) {
                    std::cout << "t=" << std::fixed << std::setprecision(2) << total_time
                              << " | Sig=" << step.freq
                              << " | PLL=" << std::setprecision(3) << current_pll_freq
                              << " | GridEst=" << freq_result.frequency_hz
                              << " | Drift=" << std::setprecision(4) << pe_result.linear_drift_rate
                              << " | State=" << pe_result.state << std::endl;
                }
            }

            // Advance PLL time to next strobe
            current_pll_time += STROBE_DIV / current_pll_freq;
            total_time += STROBE_DIV / current_pll_freq;
            frame_counter++;
        }

        // Verify convergence at end of step
        if (std::abs(current_pll_freq - step.freq) > 0.1) {
            std::cerr << "FAIL: Step " << step.label << " did not converge. Got " << current_pll_freq << std::endl;
            all_success = false;
        }
    }

    if (all_success) {
        std::cout << "\nSUCCESS: All test cases converged!" << std::endl;
        return 0;
    } else {
        std::cout << "\nFAILURE: Some test cases failed." << std::endl;
        return 1;
    }
}
