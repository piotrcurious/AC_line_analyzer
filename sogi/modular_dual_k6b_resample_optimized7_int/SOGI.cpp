#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float TWO_PI_F = 2.0f * M_PI;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

static inline int64_t q40_30_mul(int64_t a, int32_t b) {
    int64_t a_hi = a >> 30;
    uint64_t a_lo = (uint64_t)a & 0x3FFFFFFF;
    return (a_hi * (int64_t)b) + ((int64_t)(a_lo * (int64_t)b) >> 30);
}

// ==============================
// ========  SOGI  ==============
// ==============================

SOGI::SOGI(float k_) : k(k_) { init(); }
void SOGI::init() { reset(); coeff_valid = false; last_omega = 0.0f; last_ts = 0.0f; }
void SOGI::reset() { v_alpha = v_beta = 0; wz1_a = wz2_a = wz1_b = wz2_b = 0; }

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
    last_omega = omega; last_ts = ts; coeff_valid = true;
}

void IRAM_ATTR SOGI::step(q16_t u, float omega, float ts)
{
    if (!coeff_valid || fabsf(omega - last_omega) > 0.1f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }
    q40_t u_40 = Q16_TO_Q40(u);
    q40_t in_a = u_40 - q40_30_mul(wz1_a, a_a1) - q40_30_mul(wz2_a, a_a2);
    q40_t va_40 = q40_30_mul(in_a, a_b0) + q40_30_mul(wz2_a, a_b2);
    v_alpha = Q40_TO_Q16(va_40);
    wz2_a = wz1_a; wz1_a = in_a;

    q40_t in_b = u_40 - q40_30_mul(wz1_b, a_a1) - q40_30_mul(wz2_b, a_a2);
    q40_t vb_40 = q40_30_mul(in_b, b_b0) + q40_30_mul(wz1_b, b_b1) + q40_30_mul(wz2_b, b_b2);
    v_beta = Q40_TO_Q16(vb_40);
    wz2_b = wz1_b; wz1_b = in_b;
}

// ==============================
// === TripleSOGIAnalyzer =======
// ==============================

TripleSOGIAnalyzer::TripleSOGIAnalyzer(float nf, float k)
    : sogi_nom(k), sogi_pll(k), sogi_3pll(k), nominal_freq(nf) { init(); }

void TripleSOGIAnalyzer::init() {
    sogi_nom.init(); sogi_pll.init(); sogi_3pll.init();
    grid_freq = nominal_freq; grid_phase = 0.0f;
    prev_nom_phase = 0.0f; buf_idx = 0; rate_sum = 0.0f;
    for (int i=0; i<BUF_LEN; i++) rate_buf[i] = 0.0f;
}

void IRAM_ATTR TripleSOGIAnalyzer::process(q16_t input, float ts) {
    // 1. Extract 3rd Harmonic
    sogi_3pll.step(input, 3.0f * TWO_PI_F * grid_freq, ts);

    // 2. Fundamental Cleanup
    q16_t clean_input = input - sogi_3pll.v_alpha;

    // 3. Fundamental SOGIs
    sogi_nom.step(clean_input, TWO_PI_F * nominal_freq, ts);
    sogi_pll.step(clean_input, TWO_PI_F * grid_freq, ts);

    // 4. Deterministic Frequency Solver
    float nom_alpha = Q16_TO_FLOAT(sogi_nom.v_alpha);
    float nom_beta = Q16_TO_FLOAT(sogi_nom.v_beta);
    float current_nom_phase = atan2f(nom_alpha, -nom_beta);

    float dp = current_nom_phase - prev_nom_phase;
    while (dp > M_PI) dp -= TWO_PI_F;
    while (dp < -M_PI) dp += TWO_PI_F;
    prev_nom_phase = current_nom_phase;

    float freq_inst = dp / (TWO_PI_F * ts);

    rate_sum -= rate_buf[buf_idx];
    rate_buf[buf_idx] = freq_inst;
    rate_sum += freq_inst;
    buf_idx = (buf_idx + 1) % BUF_LEN;

    grid_freq = rate_sum / BUF_LEN;

    // 5. Update Outputs
    v_alpha = sogi_pll.v_alpha;
    v_beta  = sogi_pll.v_beta;
    grid_phase = atan2f(Q16_TO_FLOAT(v_alpha), -Q16_TO_FLOAT(v_beta));
    v_mag = sqrtf(pow(Q16_TO_FLOAT(v_alpha), 2) + pow(Q16_TO_FLOAT(v_beta), 2));
    v_mag_3rd = sqrtf(pow(Q16_TO_FLOAT(sogi_3pll.v_alpha), 2) + pow(Q16_TO_FLOAT(sogi_3pll.v_beta), 2));
    h3_ratio = v_mag_3rd / (v_mag + 1e-9f);
}
