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

protected:
    static constexpr float MAG_ALPHA = 1.0f;

};


#define SOGI_HIST_LEN 8
const float PHASE_DEADBAND = 0.01f;  // Adjust based on your noise level

struct DFTResult {
    float mag;
    float phase;
    float real;
    float imag;
};

class DFTAnalyzer {
public:
    DFTAnalyzer();
    void analyze(const float* buffer, int bufLen, int startIdx, int count, float freq, float ts, float offset = 0.0f);

    DFTResult fundamental;
    DFTResult h3; // 3rd harmonic
    DFTResult h5; // 5th harmonic
    float thd_approx;
};

class AdaptivePLL : public FrequencyAdaptivePLL {
public:
    AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate = 0.1001f);
    void init();
    void update(float v_alpha, float v_beta, float ts);
    void updateWithDFT(float v_alpha, float v_beta, const DFTAnalyzer& dft, float ts);

    float integral_state;
    float i_term;
    float last_control_action;
    float gain_est;
    float learn_rate;

    float control_hist[SOGI_HIST_LEN];
    float phase_hist[SOGI_HIST_LEN];
    uint8_t hist_idx;

    float prev_dft_phase;
    bool dft_initialized;
};

#endif // SOGI_H
