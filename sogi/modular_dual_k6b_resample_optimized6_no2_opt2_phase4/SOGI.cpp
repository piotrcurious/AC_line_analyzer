#include "SOGI.h"
#include <string.h>
#include <stdint.h>

#ifndef ARDUINO
#include <cmath>
#define fabsf std::abs
#define tanf std::tan
#define sqrtf std::sqrt
#define atan2f std::atan2
#define lrintf std::lrint
#endif

static constexpr float TWO_PI_F = 6.2831853071795864769f;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;
static constexpr float MAG_EPS = 1e-6f;

// ── SOGI Implementation ──────────────────────────────────────────────────────

SOGI::SOGI(float k_) : k(k_) { init(); }

void SOGI::init() {
    reset();
    coeff_valid = false;
    last_omega = 0.0f;
    last_ts = 0.0f;
}

void SOGI::reset() {
    v_alpha = 0.0f; v_beta  = 0.0f;
    v_alpha_prev = 0.0f; v_beta_prev = 0.0f;
    wz1 = wz2 = 0.0f;
    w_rot = 0.0f;
    has_prev_phi = false;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts) {
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;
    last_omega = omega; last_ts = ts;
    float wts  = 2.0f * tanf(omega * ts * 0.5f);
    float wts2 = wts * wts; float k_wts = k * wts; float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);
    a_b0 = 2.0f * k_wts * det; a_b2 = -a_b0; a_a1 = 2.0f * (wts2 - 4.0f) * det; a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;
    b_b0 = k * wts2 * det; b_b1 = 2.0f * b_b0; b_b2 = b_b0;
    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(float input, float omega, float ts) {
    if (!coeff_valid || fabsf(omega - last_omega) > 1.0f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }
    v_alpha_prev = v_alpha; v_beta_prev  = v_beta;
    float w0 = input - a_a1 * wz1 - a_a2 * wz2;
    v_alpha = a_b0 * w0 + a_b2 * wz2;
    v_beta = b_b0 * w0 + b_b1 * wz1 + b_b2 * wz2;
    wz2 = wz1; wz1 = w0;

    float mag_sq = v_alpha * v_alpha + v_beta * v_beta;
    if (mag_sq > 100.0f) {
        float phi_now = atan2f(v_alpha, -v_beta);
        if (!has_prev_phi) { w_rot = omega; has_prev_phi = true; }
        else {
            float phi_pre = atan2f(v_alpha_prev, -v_beta_prev);
            float d_phi = phi_now - phi_pre;
            while (d_phi > (float)M_PI) d_phi -= 2.0f * (float)M_PI;
            while (d_phi < -(float)M_PI) d_phi += 2.0f * (float)M_PI;
            w_rot = d_phi / (ts + 1e-12f);
        }
    } else { w_rot = omega; has_prev_phi = false; }
}

float SOGI::getFllError(float input) const {
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta + MAG_EPS;
    // Reversed sign: positive means input is FASTER than tuned omega
    return (v_alpha - input) * v_beta / mag_sq;
}

// ── AdaptiveFLL Implementation ───────────────────────────────────────────────

AdaptiveFLL::AdaptiveFLL(float nominal_freq_, float gamma_, float learn_rate_)
    : nominal_freq(nominal_freq_), gamma(gamma_), learn_rate(learn_rate_) {
    init();
}

void AdaptiveFLL::init() {
    freq = nominal_freq; omega = TWO_PI_F * freq; integral_err_c = 0.0f; gain_est = 0.1f;
    p_scale_factor = 1.0f; learn_scale_factor = 1.0f; hist_idx = 0;
    for(int i=0; i<HIST_LEN; i++) phase_hist[i] = 0.0f;
}

void AdaptiveFLL::setDistortionDamping(float p_scale, float learn_scale) {
    if (p_scale < 0.0f) p_scale = 0.0f; if (p_scale > 1.0f) p_scale = 1.0f;
    if (learn_scale < 0.0f) learn_scale = 0.0f; if (learn_scale > 1.0f) learn_scale = 1.0f;
    p_scale_factor = p_scale; learn_scale_factor = learn_scale;
}

void AdaptiveFLL::update(float fll_err, float rot_err, float ts) {
    // Both inputs are dimensionless frequency deviations (positive = input is faster)
    float diff = fll_err - rot_err;
    float agree = 1.0f / (1.0f + 25.0f * diff * diff);
    float fused_err = fll_err * agree + rot_err * (1.0f - agree) * 0.1f;

    uint8_t prev_idx = (hist_idx + HIST_LEN - 1) & (HIST_LEN - 1);
    float prev_phase = phase_hist[prev_idx];
    float dy = fused_err - prev_phase;
    float control = (gamma * p_scale_factor) * prev_phase * ts;
    float denom = control * control + 1e-9f;
    float err_gain = dy - gain_est * control;
    gain_est += (learn_rate * learn_scale_factor) * (control * err_gain) / denom;
    if (gain_est < 0.01f) gain_est = 0.01f; if (gain_est > 5.0f) gain_est = 5.0f;

    // SOGI-FLL Law: ω̇ = γ · ε
    float d_omega = (gamma * p_scale_factor) * fused_err * ts;

    // Kahan summation frequency integrator
    float y = d_omega - integral_err_c;
    float t = omega + y;
    integral_err_c = (t - omega) - y;
    omega = t;

    float f_max = nominal_freq * 1.5f; float f_min = nominal_freq * 0.6f;
    freq = omega / TWO_PI_F;
    if (freq > f_max) { freq = f_max; omega = TWO_PI_F * freq; }
    else if (freq < f_min) { freq = f_min; omega = TWO_PI_F * freq; }

    phase_hist[hist_idx] = fused_err; hist_idx = (hist_idx + 1) & (HIST_LEN - 1);
}
