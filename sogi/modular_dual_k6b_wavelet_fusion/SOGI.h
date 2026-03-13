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

class AdaptivePLL : public FrequencyAdaptivePLL {
public:
    AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate = 0.1001f);
    void init();

    // Core EKF Fusion
    void updateFused(float v_alpha, float v_beta, float wavelet_drift_rate, float wavelet_freq, float confidence, float dt);

    // Legacy support
    void update(float v_alpha, float v_beta, float ts);

    float getFusedPhase() const;

    // EKF State Vector: [Phase Error (rad), Frequency Deviation (rad/s), SOGI Bias (rad)]
    float x_theta;
    float x_omega;
    float x_beta;

    float P[3][3];      // State Covariance
    float Q[3][3];      // Process Noise Covariance

    float last_control_action;
    float gain_est;
    float learn_rate;

    float control_hist[SOGI_HIST_LEN];
    float phase_hist[SOGI_HIST_LEN];
    uint8_t hist_idx;

private:
    void predict(float dt);
    void sequentialUpdate(const float z[], const float h[], const float R[], const float H[][3], int num_measurements);
};

#endif // SOGI_H
