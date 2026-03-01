#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float TWO_PI_F = 2.0f * M_PI;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

// State is Q32.32 (64-bit), Coeff is Q2.30 (32-bit).
// Product is Q34.62 (96-bit effectively).
// We want Q32.32 back, so shift by 30.
// This requires __int128 if we want to be absolutely safe, but since SOGI states are bounded,
// we can use a trick or just use double for intermediate.
// Wait, the user asked for INTEGER math.
// Let's use 64-bit with a smaller shift if needed, but Q30 is a large shift.
// If state is 1000 (2^10) * 2^32 = 2^42. Coeff is 2^30. Product 2^72. Definitely overflows 64-bit.
// BUT, on ESP32, we don't have __int128.
// We can split the multiplication: (A_hi*2^32 + A_lo) * B = A_hi*B*2^32 + A_lo*B.
#define Q32_30_MUL(a, b) ((((a) >> 30) * (b)) + ((((a) & 0x3FFFFFFF) * (b)) >> 30))

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
    if (!coeff_valid || fabsf(omega - last_omega) > 0.01f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    q32_t u_32 = Q16_TO_Q32(u);

    // Direct Form II
    q32_t in_a = u_32 - Q32_30_MUL(wz1_a, a_a1) - Q32_30_MUL(wz2_a, a_a2);
    q32_t va_32 = Q32_30_MUL(in_a, a_b0) + Q32_30_MUL(wz2_a, a_b2);
    v_alpha = Q32_TO_Q16(va_32);
    wz2_a = wz1_a;
    wz1_a = in_a;

    q32_t in_b = u_32 - Q32_30_MUL(wz1_b, a_a1) - Q32_30_MUL(wz2_b, a_a2);
    q32_t vb_32 = Q32_30_MUL(in_b, b_b0) + Q32_30_MUL(wz1_b, b_b1) + Q32_30_MUL(wz2_b, b_b2);
    v_beta = Q32_TO_Q16(vb_32);
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
    if (!coeff_valid || fabsf(omega - last_omega) > 0.01f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    q32_t l_wz1_a = wz1_a;
    q32_t l_wz2_a = wz2_a;
    q32_t l_wz1_b = wz1_b;
    q32_t l_wz2_b = wz2_b;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        q32_t u = Q16_TO_Q32(buffer[i] - offset);
        q32_t in_a = u - Q32_30_MUL(l_wz1_a, a_a1) - Q32_30_MUL(l_wz2_a, a_a2);
        l_wz2_a = l_wz1_a;
        l_wz1_a = in_a;

        q32_t in_b = u - Q32_30_MUL(l_wz1_b, a_a1) - Q32_30_MUL(l_wz2_b, a_a2);
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

    v_alpha = Q32_TO_Q16(Q32_30_MUL(wz1_a, a_b0) + Q32_30_MUL(wz2_a, a_b2));
    v_beta = Q32_TO_Q16(Q32_30_MUL(wz1_b, b_b0) + Q32_30_MUL(wz1_b, b_b1) + Q32_30_MUL(wz2_b, b_b2));
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
// ===== Adaptive PLL ===========
// ==============================

AdaptivePLL::AdaptivePLL(float nf, float kp_, float ki_, float lr)
    : FrequencyAdaptivePLL(nf, kp_, ki_), learn_rate(lr) { init(); }

void AdaptivePLL::init() {
    FrequencyAdaptivePLL::init();
    integral_state = 0.0f;
    i_term = 0.0f;
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

    // PI terms
    float p_term = 0.0f;
    if (fabsf(raw_p_err) > PHASE_DEADBAND) {
        p_term = (kp * p_scale) * raw_p_err;
        integral_state += (raw_p_err * ts);
        i_term = ki * integral_state;
    }

    const float I_MAX = 10.0f;
    if (i_term > I_MAX) {
        i_term = I_MAX;
        integral_state = i_term / (ki + 1e-9f);
    } else if (i_term < -I_MAX) {
        i_term = -I_MAX;
        integral_state = i_term / (ki + 1e-9f);
    }

    float control = p_term + i_term;
    freq = nominal_freq + control;
    omega = TWO_PI_F * freq;

    phase_hist[hist_idx] = raw_p_err;
    control_hist[hist_idx] = control;
    hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    last_control_action = control;
}
