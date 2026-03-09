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
 * UNIFIED TRANSFORM FRAMEWORK — Operator Implementations
 * =============================================================================
 */

static constexpr float OMEGA_MIN = 2.0f * (float)M_PI * 5.0f;

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
    w1 = w2 = 0.0;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts) {
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;
    last_omega = omega; last_ts = ts;

    // Tustin Discretization with shared denominator
    float wts = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    // Alpha (Bandpass) Numerator
    a_b0 = 2.0f * k_wts * det;
    a_a1 = 2.0f * (wts2 - 4.0f) * det;
    a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

    // Beta (Lowpass/Quadrature) Numerator
    b_b0 = (k * wts2) * det;
    b_b1 = 2.0f * b_b0;
    b_b2 = b_b0;

    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(float input, float omega, float ts) {
    if (!coeff_valid || fabsf(omega - last_omega) > 0.05f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    double u = (double)input;
    // Unified Direct Form II Implementation
    double w0 = u - (double)a_a1 * w1 - (double)a_a2 * w2;

    // v_alpha (Odd symmetry) = a_b0 * (w0 - w2)
    v_alpha = (float)( (double)a_b0 * (w0 - w2) );

    // v_beta (Even symmetry) = b_b0 * (w0 + w2) + b_b1 * w1
    v_beta = (float)( (double)b_b0 * (w0 + w2) + (double)b_b1 * w1 );

    w2 = w1;
    w1 = w0;
}

// ── Φ_fll Implementation ───────────────────────────────────────────────────

AdaptiveFLL::AdaptiveFLL(float nominal_freq_, float gamma_, float learn_rate_)
    : nominal_freq(nominal_freq_), gamma(gamma_), learn_rate(learn_rate_) {
    init();
}

void AdaptiveFLL::init() {
    freq_val = nominal_freq;
    integral = nominal_freq;
    integral_c = 0.0;
    gain_est = -0.0045f; // Initial sensitivity estimate

    prev_error = 0.0f;
    prev_control = 0.0f;

    avg_sum = 0.0; avg_idx = 0; current_spc = 128;
    for (int i = 0; i < FLL_AVG_MAX; i++) avg_buf[i] = 0.0f;
}

void IRAM_ATTR AdaptiveFLL::update(float u, float v_alpha, float v_beta, float ts, uint32_t spc) {
    // 1. Dynamic N Handling (Resizing the averaging window)
    if (spc > 0 && spc <= FLL_AVG_MAX && spc != current_spc) {
        double current_avg = (current_spc > 0) ? avg_sum / (double)current_spc : 0.0;
        current_spc = spc;
        avg_sum = current_avg * (double)current_spc;
        for (uint32_t i = 0; i < FLL_AVG_MAX; i++) avg_buf[i] = (float)current_avg;
        avg_idx = 0;
    }

    // 2. Normalized FLL Discriminator
    // e = -(u - v_alpha) * v_beta / (|v|^2 + 1)
    // Reframed as projection error cross-product.
    // If omega < omega_grid, v_alpha lags u -> (u - v_alpha) has positive projection on v_beta.
    // In our coordinate system, v_beta lags v_alpha by 90 deg.
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta;
    float raw_err = -(u - v_alpha) * v_beta / (mag_sq + 1.0f);

    // 3. Cycle Averaging (2w ripple rejection)
    avg_sum -= (double)avg_buf[avg_idx];
    avg_buf[avg_idx] = raw_err;
    avg_sum += (double)raw_err;
    avg_idx = (avg_idx + 1) % current_spc;
    float smooth_err = (float)(avg_sum / (double)current_spc);

    // 4. Online Gain Identification (NLMS)
    // Model: d_smooth_err = gain_est * d_control
    float dy = smooth_err - prev_error;
    float du = prev_control;

    // Leaky sensitivity target
    float omega_now = (float)(freq_val * 2.0 * M_PI);
    float gain_nom = -1.0f / (0.7071f * (omega_now + 1.0f));
    gain_est = (0.9995f) * gain_est + 0.0005f * gain_nom;

    if (fabsf(du) > 1e-9f) {
        float prediction_error = dy - gain_est * du;
        gain_est += learn_rate * (du * prediction_error) / (du * du + 1e-9f);
    }

    // Stability Clamps
    if (gain_est > -0.0001f) gain_est = -0.0001f;
    if (gain_est < -0.1f)    gain_est = -0.1f;

    // 5. Frequency Integration (Kahan Summation)
    // Control signal: d_freq = (gamma / |gain_est|) * smooth_err
    // We divide by |gain_est| to normalize the loop gain regardless of signal magnitude.
    float d_freq = (gamma / (fabsf(gain_est) + 1e-6f)) * smooth_err;
    float control_action = d_freq * ts;

    double y = (double)control_action - integral_c;
    double t = integral + y;
    integral_c = (t - integral) - y;
    integral = t;

    // Frequency Clamps (±50%)
    if (integral > (double)nominal_freq * 1.5) integral = (double)nominal_freq * 1.5;
    else if (integral < (double)nominal_freq * 0.6) integral = (double)nominal_freq * 0.6;

    freq_val = integral;

    // Save history for NLMS
    prev_error = smooth_err;
    prev_control = control_action;
}

// ── UnifiedSOGIAnalyzer Implementation ──────────────────────────────────────

UnifiedSOGIAnalyzer::UnifiedSOGIAnalyzer(float nominal_freq, float k, float gamma)
    : sogi_fund(k), sogi_h3(k), sogi_h5(k), sogi_h7(k), sogi_h9(k), sogi_h11(k),
      fll(nominal_freq, gamma) {
    init();
}

void UnifiedSOGIAnalyzer::init() {
    v_dc = 1650.0f;
    dc_acc = 1650.0;
    sogi_fund.init(); sogi_h3.init(); sogi_h5.init();
    sogi_h7.init(); sogi_h9.init(); sogi_h11.init();
    fll.init();
}

void IRAM_ATTR UnifiedSOGIAnalyzer::process(float u_raw, float ts, uint32_t spc) {
    // 1. T_dc: Recursive Average Estimator (2 rad/s LPF)
    double alpha_dc = (double)ts * 2.0;
    dc_acc += alpha_dc * ((double)u_raw - dc_acc);
    v_dc = (float)dc_acc;
    float u = u_raw - v_dc;

    // 2. P_dec: Recursive Harmonic Decoupling
    // Sum of all other active projections
    float v_others = sogi_h3.v_alpha + sogi_h5.v_alpha + sogi_h7.v_alpha + sogi_h9.v_alpha + sogi_h11.v_alpha;

    // Fundamental Pass
    sogi_fund.step(u - v_others, fll.getOmega(), ts);

    // 3. Φ_fll: Frequency Estimation
    fll.update(u, sogi_fund.v_alpha, sogi_fund.v_beta, ts, spc);

    // 4. Harmonic Projections
    float w1 = fll.getOmega();
    float u_fund = sogi_fund.v_alpha;

    sogi_h3.step(u - (v_others - sogi_h3.v_alpha + u_fund), 3.0f * w1, ts);
    sogi_h5.step(u - (v_others - sogi_h5.v_alpha + u_fund), 5.0f * w1, ts);
    sogi_h7.step(u - (v_others - sogi_h7.v_alpha + u_fund), 7.0f * w1, ts);
    sogi_h9.step(u - (v_others - sogi_h9.v_alpha + u_fund), 9.0f * w1, ts);
    sogi_h11.step(u - (v_others - sogi_h11.v_alpha + u_fund), 11.0f * w1, ts);
}

float UnifiedSOGIAnalyzer::getHarmonicAlpha(int h) const {
    switch(h) {
        case 1: return sogi_fund.v_alpha;
        case 3: return sogi_h3.v_alpha;
        case 5: return sogi_h5.v_alpha;
        case 7: return sogi_h7.v_alpha;
        case 9: return sogi_h9.v_alpha;
        case 11: return sogi_h11.v_alpha;
        default: return 0.0f;
    }
}
