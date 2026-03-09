#ifndef SOGI_H
#define SOGI_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <cstring>
#include <cstdint>
#define IRAM_ATTR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif

/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK (UTF) — Phase 7
 * =============================================================================
 *
 * This framework expresses signal processing as a composition of operators:
 *   T_resamp : Hardware-to-Virtual Grid Mapping (Variable N in [100, 256])
 *   T_dc     : DC Offset Removal
 *   P_sogi   : Orthogonal Projection onto Quadrature Space {sin, cos}
 *   Φ_fll    : Frequency/Phase Tracking
 *
 * Improvements for Phase 7:
 *   - Variable N (100-256) to maintain constant virtual rate and minimize quantization.
 *   - Double precision for all recursive state variables to eliminate drift.
 *   - Kahan summation for frequency integration.
 *   - Area-preserving resampler (box-filter) for noise reduction.
 */

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    void IRAM_ATTR step(float input, float omega, float ts);
    void IRAM_ATTR processWindow(const float* __restrict buffer, int bufLen, int startIdx, int count, float omega, float ts, float offset = 0.0f);

    float v_alpha;
    float v_beta;
    float k;

private:
    // Using double for states to prevent resonant magnitude drift.
    // alpha and beta sections share the same denominator (Direct-Form II).
    double wz1, wz2;

    float a_b0, a_b2;
    float a_a1, a_a2;
    float b_b0, b_b1, b_b2;

    bool coeff_valid;
    float last_omega;
    float last_ts;

    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class FrequencyAdaptivePLL {
public:
    FrequencyAdaptivePLL(float nominal_freq, float kp, float ki);
    void init();
    void IRAM_ATTR update(float v_alpha, float v_beta, float ts);

    float freq;
    float omega;
    double integral;
    double integral_err_c; // Kahan summation correction
    float kp, ki;
    float nominal_freq;
};

#define SOGI_HIST_LEN 8
const float PHASE_DEADBAND = 0.001f;

class AdaptivePLL : public FrequencyAdaptivePLL {
public:
    AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate = 0.1001f);
    void init();
    void IRAM_ATTR update(float v_alpha, float v_beta, float ts);

    float i_term;
    float last_control_action;
    float gain_est;
    float learn_rate;

    float control_hist[SOGI_HIST_LEN];
    float phase_hist[SOGI_HIST_LEN];
    uint8_t hist_idx;

    float p_scale = 1.0f;
    float learn_scale = 1.0f;

    inline void setDistortionDamping(float new_p_scale, float new_learn_scale) {
        if (new_p_scale < 0.0f) new_p_scale = 0.0f;
        if (new_p_scale > 1.0f) new_p_scale = 1.0f;
        if (new_learn_scale < 0.0f) new_learn_scale = 0.0f;
        if (new_learn_scale > 1.0f) new_learn_scale = 1.0f;
        p_scale = new_p_scale;
        learn_scale = new_learn_scale;
    }
};

#endif // SOGI_H
