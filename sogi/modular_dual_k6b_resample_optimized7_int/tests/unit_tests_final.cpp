#include "../SOGI.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>

void test_triple_sogi_analyzer() {
    std::cout << "Testing TripleSOGIAnalyzer deterministic tracking..." << std::endl;
    const float nominal_freq = 50.0f;
    const float target_freq = 51.5f;
    const float ts = 1.0f / 6400.0f;

    TripleSOGIAnalyzer analyzer(nominal_freq, 0.7071f);

    float phase = 0;
    for (int i = 0; i < 6400 * 5; ++i) {
        phase += 2.0 * M_PI * target_freq * ts;
        // Signal with 3rd harmonic
        float val = 1000.0f * sin(phase) + 300.0f * sin(3.0 * phase);
        analyzer.process(FLOAT_TO_Q16(val), ts);
    }

    std::cout << "  Final Freq: " << analyzer.grid_freq << std::endl;
    std::cout << "  Final H3Ratio: " << analyzer.h3_ratio << std::endl;

    assert(std::abs(analyzer.grid_freq - target_freq) < 0.2f);
    assert(analyzer.h3_ratio > 0.1f);

    std::cout << "  TripleSOGIAnalyzer Test Passed!" << std::endl;
}

int main() {
    test_triple_sogi_analyzer();
    return 0;
}
