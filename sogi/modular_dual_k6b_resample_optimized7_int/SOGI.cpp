#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

static constexpr float TWO_PI_F = 6.2831853071795864769f;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;
static constexpr float MAG_EPS = 1e-6f;

// Q16.16 * Q2.30 -> Q18.46 -> Q16.16 (shift 30)
// Using 64-bit for intermediate products to prevent overflow and maintain precision
#define FIX_MUL(a, b) ((q16_t)(((int64_t)(a) * (b)) >> Q30_SHIFT))

// Fast inverse square root (still in float, then to fixed point if needed)
static inline float fast_rsqrt(float number)
{
    if (number <= 0.0f) return 1.0f / sqrtf(MAG_EPS);
    float x = number;
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);
    float y;
    memcpy(&y, &i, sizeof(y));
    const float xhalf = 0.5f * x;
    y = y * (1.5f - xhalf * y * y);
    y = y * (1.5f - xhalf * y * y);
    return y;
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

    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(q16_t u, float omega, float ts)
{
    if (!coeff_valid) {
        updateCoefficients(omega, ts);
    }

    // Alpha filter
    // in_a = u - a_a1 * wz1_a - a_a2 * wz2_a;
    q16_t in_a = u - FIX_MUL(wz1_a, a_a1) - FIX_MUL(wz2_a, a_a2);
    // v_alpha = a_b0 * in_a + a_b2 * wz2_a;
    v_alpha = FIX_MUL(in_a, a_b0) + FIX_MUL(wz2_a, a_b2);
    wz2_a = wz1_a;
    wz1_a = in_a;

    // Beta filter
    // in_b = u - a_a1 * wz1_b - a_a2 * wz2_b;
    q16_t in_b = u - FIX_MUL(wz1_b, a_a1) - FIX_MUL(wz2_b, a_a2);
    // v_beta = b_b0 * in_b + b_b1 * wz1_b + b_b2 * wz2_b;
    v_beta = FIX_MUL(in_b, b_b0) + FIX_MUL(wz1_b, b_b1) + FIX_MUL(wz2_b, b_b2);
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
    if (!coeff_valid) updateCoefficients(omega, ts);

    q16_t l_wz1_a = wz1_a;
    q16_t l_wz2_a = wz2_a;
    q16_t l_wz1_b = wz1_b;
    q16_t l_wz2_b = wz2_b;
    q16_t l_v_alpha = v_alpha;
    q16_t l_v_beta  = v_beta;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        q16_t u = buffer[i] - offset;
        q16_t in_a = u - FIX_MUL(l_wz1_a, a_a1) - FIX_MUL(l_wz2_a, a_a2);
        l_v_alpha = FIX_MUL(in_a, a_b0) + FIX_MUL(l_wz2_a, a_b2);
        l_wz2_a = l_wz1_a;
        l_wz1_a = in_a;

        q16_t in_b = u - FIX_MUL(l_wz1_b, a_a1) - FIX_MUL(l_wz2_b, a_a2);
        l_v_beta = FIX_MUL(in_b, b_b0) + FIX_MUL(l_wz1_b, b_b1) + FIX_MUL(l_wz2_b, b_b2);
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
    v_alpha = l_v_alpha; v_beta = l_v_beta;
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
    integral_q32 = 0;
}

void IRAM_ATTR FrequencyAdaptivePLL::update(q16_t v_alpha, q16_t v_beta, float ts)
{
    // Fast inverse square root on float magnitude
    float va_f = Q16_TO_FLOAT(v_alpha);
    float vb_f = Q16_TO_FLOAT(v_beta);
    float mag_sq = va_f * va_f + vb_f * vb_f + MAG_EPS;
    float inv_mag = fast_rsqrt(mag_sq);

    float raw_p_err = vb_f * inv_mag;

    // Integer integrator (simple for now)
    // ki is small, ts is small
    // delta_int = ki * raw_p_err * ts
    int64_t delta_i = (int64_t)(ki * raw_p_err * ts * ((double)(1LL << 32)));
    integral_q32 += delta_i;

    // Anti-windup
    const int64_t I_MAX = (int64_t)5 * (1LL << 32);
    if (integral_q32 > I_MAX) integral_q32 = I_MAX;
    else if (integral_q32 < -I_MAX) integral_q32 = -I_MAX;

    float integral_f = (float)integral_q32 / (float)(1LL << 32);
    float control = kp * raw_p_err + integral_f;

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
    float mag_sq = va_f*va_f + vb_f*vb_f + MAG_EPS;
    float inv_mag = fast_rsqrt(mag_sq);
    float raw_p_err = vb_f * inv_mag;

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

    const float I_MAX = 5.0f;
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
