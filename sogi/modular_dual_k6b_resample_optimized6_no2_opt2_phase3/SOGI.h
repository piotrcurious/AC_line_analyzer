#ifndef SOGI_H
#define SOGI_H

#include <Arduino.h>

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    // Process a single sample
    void IRAM_ATTR step(float input, float omega, float ts);

    // Process a window of samples (for batch processing)
    void IRAM_ATTR processWindow(const float* buffer, int bufLen, int startIdx, int count, float omega, float ts, float offset = 0.0f);

    float v_alpha;
    float v_beta;
    float k;

private:
    // States
    float wz1_a, wz2_a; // States for alpha filter
    float wz1_b, wz2_b; // States for beta filter

    // Cached coefficients (computed when omega/ts change)
    float a_b0, a_b2;
    float a_a1, a_a2;
    float b_b0, b_b1, b_b2;

    bool coeff_valid;
    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class FrequencyAdaptivePLL {
public:
    FrequencyAdaptivePLL(float nominal_freq, float kp, float ki);
    void init();
    void IRAM_ATTR update(float v_alpha, float v_beta, float ts);

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
const float PHASE_DEADBAND = 0.001f;  // Adjust based on your noise level

class AdaptivePLL : public FrequencyAdaptivePLL {
public:
    AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate = 0.1001f);
    void init();
    void IRAM_ATTR update(float v_alpha, float v_beta, float ts);

    float integral_state;
    float i_term;
    float last_control_action;
    float gain_est;
    float learn_rate;

    float control_hist[SOGI_HIST_LEN];
    float phase_hist[SOGI_HIST_LEN];
    uint8_t hist_idx;
// --- new members (public or protected as you prefer)
float p_scale = 1.0f;        // multiplies kp inside update()
float learn_scale = 1.0f;    // multiplies effective learn_rate inside update()

inline void setDistortionDamping(float new_p_scale, float new_learn_scale) {
    // clamp sensible ranges
    if (new_p_scale < 0.0f) new_p_scale = 0.0f;
    if (new_p_scale > 1.0f) new_p_scale = 1.0f;
    if (new_learn_scale < 0.0f) new_learn_scale = 0.0f;
    if (new_learn_scale > 1.0f) new_learn_scale = 1.0f;
    p_scale = new_p_scale;
    learn_scale = new_learn_scale;
}
    
};

#endif // SOGI_H
