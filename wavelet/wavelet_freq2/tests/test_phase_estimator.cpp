#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstring>
#include "../phase_estimator.h"

// Mock Arduino types and constants if needed
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void generate_buffer(uint16_t* buf, double freq, double start_time, int samples_per_cycle, int cycles) {
    for (int i = 0; i < samples_per_cycle * cycles; i++) {
        // Assume sampling rate is 50Hz * 128 samples/cycle
        double t = start_time + (double)i / (50.0 * 128.0);
        double val = 2048.0 + 1000.0 * sin(2.0 * M_PI * freq * t);
        buf[i] = (uint16_t)val;
    }
}

int main() {
    std::cout << "Starting PhaseEstimator Unit Tests..." << std::endl;

    PhaseEstimator pe;
    PhaseEstConfig config;
    config.history_depth = 16;
    config.correction_threshold_rad = 0.3f;
    config.nonlinear_threshold_rad = 0.1f;
    config.stable_tolerance_rad = 0.02f;

    if (!pe.begin(&config)) {
        std::cerr << "Failed to initialize PhaseEstimator" << std::endl;
        return 1;
    }

    double nominal_freq = 50.0;
    double actual_freq = 51.0;
    double buffer_interval = 4.0 / nominal_freq; // 0.08s
    pe.set_frequency_params(nominal_freq, buffer_interval, 128);

    std::cout << "Simulating " << actual_freq << "Hz signal (nominal 50Hz)" << std::endl;
    std::cout << "Buffer interval: " << buffer_interval << "s" << std::endl;

    uint16_t buf[128 * 3];
    bool success = true;

    for (int frame = 0; frame < 20; frame++) {
        double start_time = frame * buffer_interval;
        generate_buffer(buf, actual_freq, start_time, 128, 3);
        pe.add_frame(buf, 128 * 3);

        if (pe.is_ready()) {
            PhaseEstResult pe_result;
            if (pe.estimate_phase(pe_result)) {
                FrequencyEstResult freq_result;
                pe.estimate_frequency(freq_result);

                std::cout << "Frame " << std::setw(2) << frame
                          << " | Drift: " << std::fixed << std::setprecision(4) << std::setw(7) << pe_result.linear_drift_rate
                          << " | Freq Err: " << std::setw(7) << freq_result.frequency_error_hz
                          << " | Est Freq: " << std::setw(8) << freq_result.frequency_hz
                          << " | State: " << pe_result.state << std::endl;

                // Verification after 10 frames of stability
                if (frame > 10) {
                    if (std::abs(freq_result.frequency_hz - actual_freq) > 0.1) {
                        std::cerr << "ERROR: Frequency estimate out of range! Expected ~" << actual_freq << ", got " << freq_result.frequency_hz << std::endl;
                        success = false;
                    }
                    if (pe_result.linear_drift_rate <= 0) {
                        std::cerr << "ERROR: Drift rate should be positive for freq > nominal!" << std::endl;
                        success = false;
                    }
                }
            }
        }
    }

    if (success) {
        std::cout << "TEST PASSED SUCCESSFULLY" << std::endl;
        return 0;
    } else {
        std::cout << "TEST FAILED" << std::endl;
        return 1;
    }
}
