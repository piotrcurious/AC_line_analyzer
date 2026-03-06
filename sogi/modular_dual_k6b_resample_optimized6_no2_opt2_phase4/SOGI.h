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
 * UNIFIED TRANSFORM FRAMEWORK — Phase 4 Refined
 * =============================================================================
 */

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    void IRAM_ATTR step(float input, float omega, float ts);

    float v_alpha;
    float v_beta;
    float k;

    float getRotationRate() const { return w_rot; }
    float getFllError(float input) const;

private:
    float wz1, wz2;
    float v_alpha_prev, v_beta_prev;
    float w_rot;
    bool  has_prev_phi;

    float a_b0, a_b2;
    float a_a1, a_a2;
    float b_b0, b_b1, b_b2;

    bool coeff_valid;
    float last_omega;
    float last_ts;

    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class AdaptiveFLL {
public:
    AdaptiveFLL(float nominal_freq, float gamma, float learn_rate = 0.1f);
    void init();

    void update(float fll_err, float rot_err, float ts);
    void setDistortionDamping(float p_scale, float learn_scale);

    float freq;
    float omega;
    float nominal_freq;
    float gamma;
    float gain_est;

private:
    float integral_err_c;
    float p_scale_factor;
    float learn_scale_factor;
    float learn_rate;

    static constexpr int HIST_LEN = 16;
    float phase_hist[HIST_LEN];
    uint8_t hist_idx;
};

#endif // SOGI_H
