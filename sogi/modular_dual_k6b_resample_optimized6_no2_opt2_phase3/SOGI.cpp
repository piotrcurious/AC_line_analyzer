#include "SOGI.h"
#include <string.h> // for memcpy
#include <stdint.h>

// Avoid collision with Arduino TWO_PI macro by using TWO_PI_F
static constexpr float TWO_PI_F = 6.2831853071795864769f;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;     // Prevent near-DC degeneracy
static constexpr float MAG_EPS = 1e-6f;                 // small epsilon for magnitude
static constexpr float INV_EPS = 1e-12f;

// Fast inverse square root (bit-level initial guess + NR iterations).
// Uses memcpy-based bit-cast to remain well-defined in C++.
static inline float fast_rsqrt(float number)
{
    // protect from extremely small/negative inputs
    if (number <= 0.0f) return 1.0f / sqrtf(MAG_EPS); // fallback (rare)

    float x = number;
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    // initial magic constant
    i = 0x5f3759df - (i >> 1);
    float y;
    memcpy(&y, &i, sizeof(y));

    // Two Newton-Raphson iterations for accuracy
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
    v_alpha = 0.0f;
    v_beta  = 0.0f;

    wz1_a = wz2_a = 0.0f;
    wz1_b = wz2_b = 0.0f;
}

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts)
{
    // clamp omega to avoid degeneracy when omega->0
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;

    float wts  = omega * ts;
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
    // If coefficients are not valid (first call or after parameter change) compute them.
    if (!coeff_valid) {
        updateCoefficients(omega, ts);
    }

    float u = input;

    float in_a = u - a_a1 * wz1_a - a_a2 * wz2_a;
    v_alpha = a_b0 * in_a + a_b2 * wz2_a;
    wz2_a = wz1_a;
    wz1_a = in_a;

    float in_b = u - a_a1 * wz1_b - a_a2 * wz2_b;
    v_beta = b_b0 * in_b + b_b1 * wz1_b + b_b2 * wz2_b;
    wz2_b = wz1_b;
    wz1_b = in_b;
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

    if (!coeff_valid) {
        updateCoefficients(omega, ts);
    }

    // Local copies for register promotion
    float l_wz1_a = wz1_a;
    float l_wz2_a = wz2_a;
    float l_wz1_b = wz1_b;
    float l_wz2_b = wz2_b;
    float l_v_alpha = v_alpha;
    float l_v_beta  = v_beta;

    int endIdx = startIdx + count;

    auto process = [&](int i)
    {
        float u = buffer[i] - offset;

        float in_a = u - a_a1 * l_wz1_a - a_a2 * l_wz2_a;
        l_v_alpha = a_b0 * in_a + a_b2 * l_wz2_a;
        l_wz2_a = l_wz1_a;
        l_wz1_a = in_a;

        float in_b = u - a_a1 * l_wz1_b - a_a2 * l_wz2_b;
        l_v_beta = b_b0 * in_b + b_b1 * l_wz1_b + b_b2 * l_wz2_b;
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

    // Write back states
    wz1_a = l_wz1_a;
    wz2_a = l_wz2_a;
    wz1_b = l_wz1_b;
    wz2_b = l_wz2_b;
    v_alpha = l_v_alpha;
    v_beta  = l_v_beta;
}

// ==============================
// ===== Frequency PLL ==========
// ==============================

FrequencyAdaptivePLL::FrequencyAdaptivePLL(
    float nominal_freq_,
    float kp_,
    float ki_)
    : nominal_freq(nominal_freq_), kp(kp_), ki(ki_)
{
    init();
}

void FrequencyAdaptivePLL::init()
{
    freq = nominal_freq;
    omega = TWO_PI_F * freq;
    mag_smooth = 1.0f;
    integral = 0.0f;
    integral_err_c = 0.0f;
}

void IRAM_ATTR FrequencyAdaptivePLL::update(
    float v_alpha,
    float v_beta,
    float ts)
{
    // magnitude smoothing using fast inverse sqrt (no sqrtf)
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta + MAG_EPS;
    float inv_mag = fast_rsqrt(mag_sq);

    float raw_p_err = v_beta * inv_mag;

    // Kahan summation for the integral, scaled by ts
    float y = (ki * raw_p_err * ts) - integral_err_c;
    float t = integral + y;
    integral_err_c = (t - integral) - y;
    integral = t;

    // Anti-windup
    const float I_MAX = 5.0f;
    if (integral > I_MAX) integral = I_MAX;
    else if (integral < -I_MAX) integral = -I_MAX;

    float control = kp * raw_p_err + integral;

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

static_assert((SOGI_HIST_LEN & (SOGI_HIST_LEN - 1)) == 0,
              "SOGI_HIST_LEN must be power of two");

AdaptivePLL::AdaptivePLL(
    float nominal_freq_,
    float kp_,
    float ki_,
    float learn_rate_)
    : FrequencyAdaptivePLL(nominal_freq_, kp_, ki_),
      learn_rate(learn_rate_)
{
    init();
}

void AdaptivePLL::init()
{
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

void IRAM_ATTR AdaptivePLL::update(
    float v_alpha,
    float v_beta,
    float ts)
{
    // --- basic SOGI phase error
    float mag_sq = v_alpha*v_alpha + v_beta*v_beta + MAG_EPS;
    float inv_mag = fast_rsqrt(mag_sq);
    float raw_p_err = v_beta * inv_mag;

    uint8_t prev_idx =
        (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);

    float prev_phase   = phase_hist[prev_idx];
    float prev_control = control_hist[prev_idx];

    // --- GAIN ADAPTATION (apply learn_scale here)
    {
        float dy = raw_p_err - prev_phase;
        float denom = prev_control * prev_control + 1e-6f;
        float err_gain = dy - gain_est * prev_control;
        // scale effective learning rate according to detector
        float eff_learn = learn_rate * learn_scale;
        gain_est += eff_learn * (prev_control * err_gain) / denom;

        // clamp learner
        if (gain_est < 0.01f) gain_est = 0.01f;
        if (gain_est > 5.0f)  gain_est = 5.0f;
    }

    // --- PROPORTIONAL / INTEGRAL
    float p_term = 0.0f;
    if (fabsf(raw_p_err) > PHASE_DEADBAND) {
        // apply p_scale to reduce how aggressively PLL reacts to phase error
        p_term = (kp * p_scale) * raw_p_err;

        // integrator (kept but not relied on if unused)
        float y = (raw_p_err * ts) - integral_err_c;
        float t = integral_state + y;
        integral_err_c = (t - integral_state) - y;
        integral_state = t;

        i_term = ki * integral_state;
    } else {
        // inside deadband: keep integrator untouched (or optionally small decay)
    }

    // --- integrator anti-windup (unchanged)
    const float I_MAX = 5.0f;
    if (i_term > I_MAX) {
        i_term = I_MAX;
        integral_state = i_term / ki;
        integral_err_c = 0.0f;
    } else if (i_term < -I_MAX) {
        i_term = -I_MAX;
        integral_state = i_term / ki;
        integral_err_c = 0.0f;
    }

    float control = p_term + i_term;

    // update frequency and omega as before
    freq = nominal_freq + control;
    omega = TWO_PI_F * freq;

    // update histories
    phase_hist[hist_idx] = raw_p_err;
    control_hist[hist_idx] = control;
    hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);

    last_control_action = control;
}
