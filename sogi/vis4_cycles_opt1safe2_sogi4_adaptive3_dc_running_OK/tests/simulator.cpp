#include "mock_arduino.h"
#include <iostream>
#include <vector>

uint32_t simulated_cycles = 0;
float current_test_freq = 50.0f;
float test_phase = 0.0f;

MockESP ESP;
MockSerial Serial;

int analogRead(int pin) {
    // Generate a sine wave around 2048 (12-bit ADC midpoint)
    // Signal magnitude: approx 1.0V -> (1.0 / 3.3) * 4095 = 1241 units peak
    float magnitude = 1200.0f;
    float val = 2048.0f + magnitude * sinf(test_phase);
    return (int)val;
}

void analogReadResolution(int res) {}

// To avoid redefinition when including sim_main.cpp
#define main arduino_main
#include "sim_main.cpp"
#undef main

int main() {
    setup();

    const uint32_t CPU_FREQ = 240000000;

    // Test scenarios: 50Hz for 1s, 51Hz for 2s, 49Hz for 2s
    struct Scenario {
        float freq;
        float seconds;
    };
    std::vector<Scenario> scenarios = {
        {50.0f, 1.0f},
        {51.0f, 2.0f},
        {49.0f, 2.0f}
    };

    std::cout << "Starting Simulation..." << std::endl;
    std::cout << "Time(s), TargetFreq(Hz), EstimatedFreq(Hz), Magnitude" << std::endl;

    for (const auto& s : scenarios) {
        current_test_freq = s.freq;
        uint32_t start_cycles = simulated_cycles;
        uint32_t duration_cycles = (uint32_t)(s.seconds * (float)CPU_FREQ);

        uint32_t last_report_cycles = simulated_cycles;
        uint32_t report_interval_cycles = CPU_FREQ / 10; // Report every 0.1s

        while ((simulated_cycles - start_cycles) < duration_cycles) {
            // loop() handles catching up with multiple samples if needed.
            // We advance time by some amount each iteration.
            uint32_t step = 500; // Skip 500 cycles (approx 2us)

            // Update phase for the time jump
            test_phase += 2.0f * PI * current_test_freq * ((float)step / (float)CPU_FREQ);
            if (test_phase > 2.0f * PI) test_phase -= 2.0f * PI;

            simulated_cycles += step;
            loop();

            if ((simulated_cycles - last_report_cycles) >= report_interval_cycles) {
                float t = (float)simulated_cycles / (float)CPU_FREQ;
                // Note: Serial.printf in loop() will also output to stdout
                // but we want a clean CSV-like output here.
                printf("OUT: %.2f, %.1f, %.4f, %.3f\n", t, current_test_freq, sogi.freq, sogi.mag_smooth);
                last_report_cycles = simulated_cycles;
            }
        }
    }

    std::cout << "Simulation Finished." << std::endl;
    return 0;
}
