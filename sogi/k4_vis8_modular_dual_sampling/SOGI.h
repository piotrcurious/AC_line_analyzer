#ifndef SOGI_H
#define SOGI_H

#include <Arduino.h>

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
    float wz1_a, wz2_a; // States for alpha filter
    float wz1_b, wz2_b; // States for beta filter
};

class FrequencyAdaptivePLL {
public:
    FrequencyAdaptivePLL(float nominal_freq, float kp, float ki);
    void init();
    void update(float v_alpha, float v_beta, float ts);

    float freq;
    float omega;
    float mag_smooth;
    float integral;
    float integral_err_c; // Kahan summation correction
    float kp, ki;
    float nominal_freq;

private:
    static constexpr float MAG_ALPHA = 0.2f;
};

#endif // SOGI_H
