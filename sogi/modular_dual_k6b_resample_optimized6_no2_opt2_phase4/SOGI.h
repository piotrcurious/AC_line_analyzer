#ifndef SOGI_H
#define SOGI_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <cstring>
#include <cstdint>
#define IRAM_ATTR
#endif

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    // Process a single sample
    void step(float input, float omega, float ts);

    // Process a window of samples (for batch processing)
    void processWindow(const float* buffer, int bufLen, int startIdx, int count, float omega, float ts, float offset = 0.0f);

    float v_alpha;
    float v_beta;
    float k;

private:
    // States
    float wz1, wz2; // Shared states for alpha and beta filters

    // Cached coefficients (computed when omega/ts change)
    float a_b0, a_b2;
    float a_a1, a_a2;
    float b_b0, b_b1, b_b2;

    bool coeff_valid;
    float last_omega;
    float last_ts;
    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class SOGIFLL {
public:
    SOGIFLL(float nominal_freq, float gamma);
    void init();
    void update(float u, float v_alpha, float v_beta, float ts);

    float freq;
    float omega;
    float nominal_freq;
    float gamma;
    float mag_smooth;

    // Compatibility for main loop
    float gamma_scale = 1.0f;
    inline void setDistortionDamping(float new_p_scale, float new_learn_scale) {
        (void)new_learn_scale;
        if (new_p_scale < 0.0f) new_p_scale = 0.0f;
        if (new_p_scale > 1.0f) new_p_scale = 1.0f;
        gamma_scale = new_p_scale;
    }
};

#endif // SOGI_H
