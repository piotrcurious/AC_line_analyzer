/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK — Operator Implementations
 * =============================================================================
 */

#include "SOGI.h"
#include <string.h>
#include <stdint.h>

#ifndef ARDUINO
#include <cmath>
#define fabsf std::abs
#define tanf std::tan
#define sqrtf std::sqrt
#endif

// Avoid collision with Arduino TWO_PI macro.
static constexpr float TWO_PI_F = 6.2831853071795864769f;

// ω_min: lower bound on the resonant frequency of P_sogi.
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

// MAG_EPS: regularisation addend in the magnitude computation.
static constexpr float MAG_EPS = 1e-6f;

// =============================================================================
//  P_sogi — Bandpass Projection Operator
// =============================================================================

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
    v_alpha = 0.0f;
    v_beta  = 0.0f;
    wz1 = wz2 = 0.0f;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts)
{
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;

    last_omega = omega;
    last_ts = ts;

    float wts  = 2.0f * tanf(omega * ts * 0.5f);
    float wts2 = wts * wts;
    float k_wts = k * wts;

    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    a_b0 =  2.0f * k_wts * det;
    a_b2 = -2.0f * k_wts * det;

    a_a1 =  2.0f * (wts2 - 4.0f) * det;
    a_a2 =  (4.0f - 2.0f * k_wts + wts2) * det;

    b_b0 = k * wts2 * det;
    b_b1 = 2.0f * b_b0;
    b_b2 = b_b0;

    coeff_valid = true;
}

void IRAM_ATTR SOGI::step(float input, float omega, float ts)
{
    if (!coeff_valid || fabsf(omega - last_omega) > 0.01f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    float u = input;
    float w0 = u - a_a1 * wz1 - a_a2 * wz2;
    v_alpha = a_b0 * w0 + a_b2 * wz2;
    v_beta = b_b0 * w0 + b_b1 * wz1 + b_b2 * wz2;

    wz2 = wz1;
    wz1 = w0;
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

    if (!coeff_valid || fabsf(omega - last_omega) > 0.01f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }

    float l_wz1 = wz1,  l_wz2 = wz2;
    float l_v_alpha = v_alpha;
    float l_v_beta  = v_beta;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        float u = buffer[i] - offset;
        float w0 = u - a_a1 * l_wz1 - a_a2 * l_wz2;
        l_v_alpha = a_b0 * w0 + a_b2 * l_wz2;
        l_v_beta  = b_b0 * w0 + b_b1 * l_wz1 + b_b2 * l_wz2;
        l_wz2 = l_wz1;
        l_wz1 = w0;
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

// =============================================================================
//  SOGIFLL — Frequency Locked Loop
// =============================================================================

SOGIFLL::SOGIFLL(float nominal_freq, float gamma)
    : nominal_freq(nominal_freq), gamma(gamma)
{
    init();
}

void SOGIFLL::init()
{
    freq = nominal_freq;
    omega = TWO_PI_F * freq;
    mag_smooth = 1.0f;
}

void IRAM_ATTR SOGIFLL::update(float u, float v_alpha, float v_beta, float ts)
{
    float error_v = u - v_alpha;
    float p_err = error_v * v_beta;
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta + MAG_EPS;

    omega -= (gamma * gamma_scale * p_err * ts) / mag_sq;

    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    freq = omega / TWO_PI_F;

    if (freq > f_max) {
        freq = f_max;
        omega = TWO_PI_F * freq;
    } else if (freq < f_min) {
        freq = f_min;
        omega = TWO_PI_F * freq;
    }

    mag_smooth = 0.99f * mag_smooth + 0.01f * sqrtf(mag_sq);
}
