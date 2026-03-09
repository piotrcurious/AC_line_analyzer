#include "SOGI.h"
#include <string.h>
#include <stdint.h>

#ifndef ARDUINO
#include <cmath>
#define fabsf std::abs
#define tanf std::tan
#define sqrtf std::sqrt
#define lrintf std::lrint
#endif

/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK — Phase 7 Implementation
 * =============================================================================
 */

static constexpr float OMEGA_MIN = 2.0f * (float)M_PI * 5.0f;
static constexpr float MAG_EPS = 1e-6f;

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

// ── P_sogi Implementation ───────────────────────────────────────────────────

SOGI::SOGI(float k_) : k(k_) { init(); }

void SOGI::init() {
    reset();
    coeff_valid = false;
    last_omega = 0.0f;
    last_ts = 0.0f;
}

void SOGI::reset() {
    v_alpha = 0.0f; v_beta = 0.0f;
    wz1 = wz2 = 0.0;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts) {
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;
    last_omega = omega;
    last_ts = ts;

    double wts = (double)omega * (double)ts;
    double wts2 = wts * wts;
    double k_wts = (double)k * wts;
    double det = 1.0 / (4.0 + 2.0 * k_wts + wts2);

    a_b0 = (float)(2.0 * k_wts * det);
    a_b2 = -a_b0;
    a_a1 = (float)(2.0 * (wts2 - 4.0) * det);
    a_a2 = (float)((4.0 - 2.0 * k_wts + wts2) * det);

    double b_b0_d = (double)k * wts2 * det;
    b_b0 = (float)b_b0_d;
    b_b1 = (float)(2.0 * b_b0_d);
    b_b2 = (float)b_b0_d;

    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(float input, float omega, float ts) {
    if (!coeff_valid || fabsf(omega - last_omega) > 0.05f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    double u = (double)input;

    // Direct-Form II shared denominator.
    double in = u - (double)a_a1 * wz1 - (double)a_a2 * wz2;

    // H_alpha (bandpass) and H_beta (lowpass) numerators.
    v_alpha = (float)((double)a_b0 * in + (double)a_b2 * wz2);
    v_beta  = (float)((double)b_b0 * in + (double)b_b1 * wz1 + (double)b_b2 * wz2);

    wz2 = wz1;
    wz1 = in;
}

void IRAM_ATTR SOGI::processWindow(
    const float* __restrict buffer,
    int bufLen,
    int startIdx,
    int count,
    float omega,
    float ts,
    float offset)
{
    if (count <= 0) return;
    if (!coeff_valid || fabsf(omega - last_omega) > 0.05f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    double l_wz1 = wz1, l_wz2 = wz2;
    float l_v_alpha = v_alpha;
    float l_v_beta  = v_beta;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        double u = (double)buffer[i] - (double)offset;

        double in = u - (double)a_a1 * l_wz1 - (double)a_a2 * l_wz2;
        l_v_alpha = (float)((double)a_b0 * in + (double)a_b2 * l_wz2);
        l_v_beta  = (float)((double)b_b0 * in + (double)b_b1 * l_wz1 + (double)b_b2 * l_wz2);

        l_wz2 = l_wz1;
        l_wz1 = in;
    };

    if (endIdx <= bufLen) {
        for (int i = startIdx; i < endIdx; ++i) process(i);
    } else {
        for (int i = startIdx; i < bufLen;    ++i) process(i);
        int remaining = endIdx - bufLen;
        for (int i = 0;        i < remaining; ++i) process(i);
    }

    wz1 = l_wz1;  wz2 = l_wz2;
    v_alpha = l_v_alpha;
    v_beta  = l_v_beta;
}

// ── Φ_pll_base Implementation ───────────────────────────────────────────────

FrequencyAdaptivePLL::FrequencyAdaptivePLL(float nominal_freq_, float kp_, float ki_)
    : nominal_freq(nominal_freq_), kp(kp_), ki(ki_) {
    init();
}

void FrequencyAdaptivePLL::init() {
    freq  = nominal_freq;
    omega = 2.0f * (float)M_PI * freq;
    integral = 0.0;
    integral_err_c = 0.0;
}

void IRAM_ATTR FrequencyAdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    double mag_sq = (double)v_alpha * v_alpha + (double)v_beta * v_beta + (double)MAG_EPS;
    float inv_mag = (float)(1.0 / sqrt(mag_sq));
    float raw_p_err = (float)((double)v_beta * inv_mag);

    // Kahan summation for integration
    double y = (double)((double)ki * (double)raw_p_err * (double)ts) - integral_err_c;
    double t = integral + y;
    integral_err_c = (t - integral) - y;
    integral = t;

    const double I_MAX = 5.0;
    if      (integral >  I_MAX) integral =  I_MAX;
    else if (integral < -I_MAX) integral = -I_MAX;

    float control = kp * raw_p_err + (float)integral;
    float f_new = nominal_freq + control;
    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    if      (f_new > f_max) f_new = f_max;
    else if (f_new < f_min) f_new = f_min;

    freq  = f_new;
    omega = 2.0f * (float)M_PI * freq;
}

// ── Φ_pll_adaptive Implementation ───────────────────────────────────────────

AdaptivePLL::AdaptivePLL(float nominal_freq_, float kp_, float ki_, float learn_rate_)
    : FrequencyAdaptivePLL(nominal_freq_, kp_, ki_), learn_rate(learn_rate_) {
    init();
}

void AdaptivePLL::init() {
    FrequencyAdaptivePLL::init();
    i_term         = 0.0f;
    last_control_action = 0.0f;
    gain_est       = 0.1f;
    hist_idx = 0;
    p_scale = 1.0f;
    learn_scale = 1.0f;
    for (int i = 0; i < SOGI_HIST_LEN; i++) {
        control_hist[i] = 0.0f;
        phase_hist[i]   = 0.0f;
    }
}

void IRAM_ATTR AdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    double mag_sq = (double)v_alpha * v_alpha + (double)v_beta * v_beta + (double)MAG_EPS;
    float inv_mag = (float)(1.0 / sqrt(mag_sq));
    float raw_p_err = (float)((double)v_beta * inv_mag);

    uint8_t prev_idx = (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);
    float prev_phase   = phase_hist[prev_idx];
    float prev_control = control_hist[prev_idx];

    {
        float dy        = raw_p_err - prev_phase;
        float denom     = prev_control * prev_control + 1e-6f;
        float err_gain  = dy - gain_est * prev_control;
        float eff_learn = learn_rate * learn_scale;
        gain_est += eff_learn * (prev_control * err_gain) / denom;

        if (gain_est < 0.01f) gain_est = 0.01f;
        if (gain_est > 5.0f)  gain_est = 5.0f;
    }

    float p_term = 0.0f;
    if (fabsf(raw_p_err) > PHASE_DEADBAND) {
        p_term = (kp * p_scale) * raw_p_err;

        double y = ((double)raw_p_err * (double)ts) - integral_err_c;
        double t = integral + y;
        integral_err_c = (t - integral) - y;
        integral = t;

        i_term = ki * (float)integral;
    }

    const float I_MAX = 5.0f;
    if (i_term > I_MAX) {
        i_term = I_MAX;
        if (fabsf(ki) > 1e-9f) {
            integral = (double)(i_term / ki);
        } else {
            integral = 0.0;
        }
        integral_err_c = 0.0;
    } else if (i_term < -I_MAX) {
        i_term = -I_MAX;
        if (fabsf(ki) > 1e-9f) {
            integral = (double)(i_term / ki);
        } else {
            integral = 0.0;
        }
        integral_err_c = 0.0;
    }

    float control = p_term + i_term;
    float f_new = nominal_freq + control;
    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    if      (f_new > f_max) f_new = f_max;
    else if (f_new < f_min) f_new = f_min;

    freq  = f_new;
    omega = 2.0f * (float)M_PI * freq;

    phase_hist[hist_idx]   = raw_p_err;
    control_hist[hist_idx] = control;
    hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);

    last_control_action = control;
}
