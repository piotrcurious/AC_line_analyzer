#ifndef SOGI_H
#define SOGI_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#include <cmath>
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#endif

// Fixed-point definitions
typedef int32_t q16_t;
#define Q16_SHIFT 16
#define Q16_ONE (1 << Q16_SHIFT)
#define FLOAT_TO_Q16(x) ((q16_t)((x) * Q16_ONE))
#define Q16_TO_FLOAT(x) ((float)(x) / Q16_ONE)

typedef int64_t q40_t;
#define Q40_SHIFT 40
#define Q40_ONE (1LL << Q40_SHIFT)
#define Q16_TO_Q40(x) (((int64_t)(x)) << (Q40_SHIFT - Q16_SHIFT))
#define Q40_TO_Q16(x) ((int32_t)((x) >> (Q40_SHIFT - Q16_SHIFT)))

typedef int32_t q30_t;
#define Q30_SHIFT 30
#define Q30_ONE (1 << Q30_SHIFT)
#define FLOAT_TO_Q30(x) ((q30_t)((x) * Q30_ONE))

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    void IRAM_ATTR step(q16_t input, float omega, float ts);
    void IRAM_ATTR processWindow(const q16_t* buffer, int bufLen, int startIdx, int count, float omega, float ts, q16_t offset = 0);

    q16_t v_alpha;
    q16_t v_beta;
    float k;

private:
    q40_t wz1_a, wz2_a;
    q40_t wz1_b, wz2_b;

    q30_t a_b0, a_b2;
    q30_t a_a1, a_a2;
    q30_t b_b0, b_b1, b_b2;

    bool coeff_valid;
    float last_omega;
    float last_ts;

    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class FrequencyAdaptivePLL {
public:
    FrequencyAdaptivePLL(float nominal_freq, float kp, float ki);
    virtual ~FrequencyAdaptivePLL() {}
    virtual void init();
    virtual void IRAM_ATTR update(q16_t v_alpha, q16_t v_beta, float ts);

    float freq;
    float omega;
    float mag_smooth;
    float phase;
    int64_t integral_q32;
    float kp, ki;
    float nominal_freq;

protected:
    static constexpr float MAG_ALPHA = 1.0f;
};

// New Deterministic PLL using phase rotation rate
class DeterministicPLL : public FrequencyAdaptivePLL {
public:
    DeterministicPLL(float nominal_freq, float kp, float ki);
    void init() override;
    void IRAM_ATTR update(q16_t v_alpha, q16_t v_beta, float ts) override;

    float prev_phase;
    float freq_filtered;

    // Cycle-averaging buffer
    static constexpr int BUF_LEN = 128;
    float rate_buf[BUF_LEN];
    int buf_idx;
    float rate_sum;
};

#define SOGI_HIST_LEN 8
const float PHASE_DEADBAND = 0.001f;

class AdaptivePLL : public FrequencyAdaptivePLL {
public:
    AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate = 0.1001f);
    void init() override;
    void IRAM_ATTR update(q16_t v_alpha, q16_t v_beta, float ts) override;

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
