#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float TWO_PI_F = 2.0f * M_PI;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

// High-precision multiplication: Q24.40 * Q2.30 -> Q26.70 -> Q24.40
static inline int64_t q40_30_mul(int64_t a, int32_t b) {
    int64_t a_hi = a >> 30;
    uint64_t a_lo = (uint64_t)a & 0x3FFFFFFF;
    return (a_hi * (int64_t)b) + ((int64_t)(a_lo * (int64_t)b) >> 30);
}

// ==============================
// ========  SOGI  ==============
// ==============================

SOGI::SOGI(float k_) : k(k_) {
    init();
}

void SOGI::init() {
    reset();
    coeff_valid = false;
    last_omega = 0.0f;
    last_ts = 0.0f;
}

void SOGI::reset() {
    v_alpha = 0;
    v_beta  = 0;
    wz1_a = wz2_a = 0;
    wz1_b = wz2_b = 0;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts)
{
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;

    float wts  = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;

    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    a_b0 =  FLOAT_TO_Q30(2.0f * k_wts * det);
    a_b2 = -a_b0;
    a_a1 =  FLOAT_TO_Q30(2.0f * (wts2 - 4.0f) * det);
    a_a2 =  FLOAT_TO_Q30((4.0f - 2.0f * k_wts + wts2) * det);

    float b_b0_f = k * wts2 * det;
    b_b0 = FLOAT_TO_Q30(b_b0_f);
    b_b1 = FLOAT_TO_Q30(2.0f * b_b0_f);
    b_b2 = b_b0;

    last_omega = omega;
    last_ts = ts;
    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(q16_t u, float omega, float ts)
{
    if (!coeff_valid || fabsf(omega - last_omega) > 0.1f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    q40_t u_40 = Q16_TO_Q40(u);

    // Alpha filter
    q40_t in_a = u_40 - q40_30_mul(wz1_a, a_a1) - q40_30_mul(wz2_a, a_a2);
    q40_t va_40 = q40_30_mul(in_a, a_b0) + q40_30_mul(wz2_a, a_b2);
    v_alpha = Q40_TO_Q16(va_40);
    wz2_a = wz1_a;
    wz1_a = in_a;

    // Beta filter
    q40_t in_b = u_40 - q40_30_mul(wz1_b, a_a1) - q40_30_mul(wz2_b, a_a2);
    q40_t vb_40 = q40_30_mul(in_b, b_b0) + q40_30_mul(wz1_b, b_b1) + q40_30_mul(wz2_b, b_b2);
    v_beta = Q40_TO_Q16(vb_40);
    wz2_b = wz1_b;
    wz1_b = in_b;
}

void IRAM_ATTR SOGI::processWindow(
    const q16_t* __restrict buffer,
    int bufLen,
    int startIdx,
    int count,
    float omega,
    float ts,
    q16_t offset)
{
    if (count <= 0) return;
    if (!coeff_valid || fabsf(omega - last_omega) > 0.1f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    q40_t l_wz1_a = wz1_a;
    q40_t l_wz2_a = wz2_a;
    q40_t l_wz1_b = wz1_b;
    q40_t l_wz2_b = wz2_b;
    q40_t l_v_alpha = 0;
    q40_t l_v_beta = 0;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        q40_t u = Q16_TO_Q40(buffer[i] - offset);

        q40_t in_a = u - q40_30_mul(l_wz1_a, a_a1) - q40_30_mul(l_wz2_a, a_a2);
        l_v_alpha = q40_30_mul(in_a, a_b0) + q40_30_mul(l_wz2_a, a_b2);
        l_wz2_a = l_wz1_a;
        l_wz1_a = in_a;

        q40_t in_b = u - q40_30_mul(l_wz1_b, a_a1) - q40_30_mul(l_wz2_b, a_a2);
        l_v_beta = q40_30_mul(in_b, b_b0) + q40_30_mul(l_wz1_b, b_b1) + q40_30_mul(l_wz2_b, b_b2);
        l_wz2_b = l_wz1_b;
        l_wz1_b = in_b;
    };

    if (endIdx <= bufLen) {
        for (int i = startIdx; i < endIdx; ++i) process(i);
    } else {
        for (int i = startIdx; i < bufLen; ++i) process(i);
        int remaining = endIdx - bufLen;
        for (int i = 0; i < remaining; ++i) process(i);
    }

    wz1_a = l_wz1_a; wz2_a = l_wz2_a;
    wz1_b = l_wz1_b; wz2_b = l_wz2_b;
    v_alpha = Q40_TO_Q16(l_v_alpha);
    v_beta = Q40_TO_Q16(l_v_beta);
}

// ==============================
// ===== Frequency PLL ==========
// ==============================

FrequencyAdaptivePLL::FrequencyAdaptivePLL(float nf, float kp_, float ki_)
    : nominal_freq(nf), kp(kp_), ki(ki_) { init(); }

void FrequencyAdaptivePLL::init() {
    freq = nominal_freq;
    omega = TWO_PI_F * freq;
    mag_smooth = 1.0f;
    phase = 0;
    integral_q32 = 0;
}

void IRAM_ATTR FrequencyAdaptivePLL::update(q16_t v_alpha, q16_t v_beta, float ts)
{
    float va_f = Q16_TO_FLOAT(v_alpha);
    float vb_f = Q16_TO_FLOAT(v_beta);

    float grid_phase_est = atan2f(va_f, -vb_f);

    phase += TWO_PI_F * freq * ts;
    while (phase > M_PI) phase -= TWO_PI_F;
    while (phase < -M_PI) phase += TWO_PI_F;

    float err = grid_phase_est - phase;
    if (err > M_PI) err -= TWO_PI_F;
    if (err < -M_PI) err += TWO_PI_F;

    int64_t delta_i = (int64_t)(ki * err * ts * ((double)(1LL << 32)));
    integral_q32 += delta_i;

    const int64_t I_MAX = (int64_t)10 * (1LL << 32);
    if (integral_q32 > I_MAX) integral_q32 = I_MAX;
    else if (integral_q32 < -I_MAX) integral_q32 = -I_MAX;

    float integral_f = (float)integral_q32 / (float)(1LL << 32);
    float control = kp * err + integral_f;

    float f_new = nominal_freq + control;
    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    if (f_new > f_max) f_new = f_max;
    else if (f_new < f_min) f_new = f_min;

    freq = f_new;
    omega = TWO_PI_F * freq;
}

// ==============================
// ==== Deterministic PLL =======
// ==============================

DeterministicPLL::DeterministicPLL(float nf, float kp_, float ki_)
    : FrequencyAdaptivePLL(nf, kp_, ki_) { init(); }

void DeterministicPLL::init() {
    FrequencyAdaptivePLL::init();
    prev_phase = 0.0f;
    freq_filtered = 0.0f;
    buf_idx = 0;
    rate_sum = 0.0f;
    for (int i = 0; i < BUF_LEN; i++) rate_buf[i] = 0.0f;
}

void IRAM_ATTR DeterministicPLL::update(q16_t v_alpha, q16_t v_beta, float ts)
{
    float va_f = Q16_TO_FLOAT(v_alpha);
    float vb_f = Q16_TO_FLOAT(v_beta);

    float current_phase = atan2f(va_f, -vb_f);

    float dp = current_phase - prev_phase;
    while (dp > M_PI) dp -= TWO_PI_F;
    while (dp < -M_PI) dp += TWO_PI_F;

    float freq_inst = dp / (TWO_PI_F * ts);

    rate_sum -= rate_buf[buf_idx];
    rate_buf[buf_idx] = freq_inst;
    rate_sum += freq_inst;
    buf_idx = (buf_idx + 1) % BUF_LEN;

    freq_filtered = rate_sum / BUF_LEN;

    // Grid Frequency is exactly the observed phase rotation speed
    freq = freq_filtered;
    omega = TWO_PI_F * freq;

    phase = current_phase;
    prev_phase = current_phase;
}

// ==============================
// ===== Adaptive PLL ===========
// ==============================

AdaptivePLL::AdaptivePLL(float nf, float kp_, float ki_, float lr)
    : FrequencyAdaptivePLL(nf, kp_, ki_), learn_rate(lr) { init(); }

void AdaptivePLL::init() {
    FrequencyAdaptivePLL::init();
    last_control_action = 0.0f;
    gain_est = 0.1f;
    hist_idx = 0;
    for (int i = 0; i < SOGI_HIST_LEN; i++) {
        control_hist[i] = 0.0f;
        phase_hist[i] = 0.0f;
    }
}

void IRAM_ATTR AdaptivePLL::update(q16_t v_alpha, q16_t v_beta, float ts)
{
    float va_f = Q16_TO_FLOAT(v_alpha);
    float vb_f = Q16_TO_FLOAT(v_beta);

    float grid_phase_est = atan2f(va_f, -vb_f);

    phase += TWO_PI_F * freq * ts;
    while (phase > M_PI) phase -= TWO_PI_F;
    while (phase < -M_PI) phase += TWO_PI_F;

    float raw_p_err = grid_phase_est - phase;
    if (raw_p_err > M_PI) raw_p_err -= TWO_PI_F;
    if (raw_p_err < -M_PI) raw_p_err += TWO_PI_F;

    uint8_t prev_idx = (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);
    float prev_phase   = phase_hist[prev_idx];
    float prev_control = control_hist[prev_idx];

    // GAIN ADAPTATION
    float dy = raw_p_err - prev_phase;
    float denom = prev_control * prev_control + 1e-6f;
    float err_gain = dy - gain_est * prev_control;
    float eff_learn = learn_rate * learn_scale;
    gain_est += eff_learn * (prev_control * err_gain) / denom;
    if (gain_est < 0.01f) gain_est = 0.01f;
    if (gain_est > 5.0f)  gain_est = 5.0f;

    // Frequency integration
    int64_t delta_i = (int64_t)(ki * raw_p_err * ts * ((double)(1LL << 32)));
    integral_q32 += delta_i;
    const int64_t I_MAX_Q32 = (int64_t)10 * (1LL << 32);
    if (integral_q32 > I_MAX_Q32) integral_q32 = I_MAX_Q32;
    else if (integral_q32 < -I_MAX_Q32) integral_q32 = -I_MAX_Q32;

    float p_term = 0.0f;
    if (fabsf(raw_p_err) > PHASE_DEADBAND) {
        p_term = (kp * p_scale) * raw_p_err;
    }

    float integral_f = (float)integral_q32 / (float)(1LL << 32);
    float control = p_term + integral_f;

    float f_new = nominal_freq + control;
    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    if (f_new > f_max) f_new = f_max;
    else if (f_new < f_min) f_new = f_min;

    freq = f_new;
    omega = TWO_PI_F * freq;

    phase_hist[hist_idx] = raw_p_err;
    control_hist[hist_idx] = control;
    hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    last_control_action = control;
}
