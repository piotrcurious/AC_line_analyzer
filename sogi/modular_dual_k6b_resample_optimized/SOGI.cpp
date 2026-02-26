#include "SOGI.h"
#include <math.h>

SOGI::SOGI(float k) : k(k) {
    init();
}

void SOGI::init() {
    reset();
}

void SOGI::reset() {
    v_alpha = 0;
    v_beta = 0;
    wz1_a = wz2_a = 0;
    wz1_b = wz2_b = 0;
}

void IRAM_ATTR SOGI::step(float input, float omega, float ts) {
    float wts = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    float a_b0 = 2.0f * k_wts * det;
    float a_b2 = -2.0f * k_wts * det;
    float a_a1 = 2.0f * (wts2 - 4.0f) * det;
    float a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

    float b_b0 = k * wts2 * det;
    float b_b1 = 2.0f * b_b0;
    float b_b2 = b_b0;

    // SOGI Alpha path (Band Pass)
    float in_a = input - (a_a1 * wz1_a) - (a_a2 * wz2_a);
    v_alpha = (a_b0 * in_a) + (a_b2 * wz2_a);
    wz2_a = wz1_a;
    wz1_a = in_a;

    // SOGI Beta path (Quadrature Low Pass)
    float in_b = input - (a_a1 * wz1_b) - (a_a2 * wz2_b);
    v_beta = (b_b0 * in_b) + (b_b1 * wz1_b) + (b_b2 * wz2_b);
    wz2_b = wz1_b;
    wz1_b = in_b;
}

void IRAM_ATTR SOGI::processWindow(const float* buffer, int bufLen, int startIdx, int count, float omega, float ts, float offset) {
    if (count <= 0) return;

    float wts = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    float a_b0 = 2.0f * k_wts * det;
    float a_b2 = -2.0f * k_wts * det;
    float a_a1 = 2.0f * (wts2 - 4.0f) * det;
    float a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

    float b_b0 = k * wts2 * det;
    float b_b1 = 2.0f * b_b0;
    float b_b2 = b_b0;

    // Local copies of state to allow compiler to optimize them in registers
    float l_wz1_a = wz1_a;
    float l_wz2_a = wz2_a;
    float l_wz1_b = wz1_b;
    float l_wz2_b = wz2_b;
    float l_v_alpha = v_alpha;
    float l_v_beta = v_beta;

    // Avoid modulo in loop by splitting into two parts if necessary
    int endIdx = startIdx + count;
    if (endIdx <= bufLen) {
        for (int i = startIdx; i < endIdx; ++i) {
            float u = buffer[i] - offset;
            float in_a = u - (a_a1 * l_wz1_a) - (a_a2 * l_wz2_a);
            l_v_alpha = (a_b0 * in_a) + (a_b2 * l_wz2_a);
            l_wz2_a = l_wz1_a;
            l_wz1_a = in_a;

            float in_b = u - (a_a1 * l_wz1_b) - (a_a2 * l_wz2_b);
            l_v_beta = (b_b0 * in_b) + (b_b1 * l_wz1_b) + (b_b2 * l_wz2_b);
            l_wz2_b = l_wz1_b;
            l_wz1_b = in_b;
        }
    } else {
        // Part 1: from startIdx to bufLen
        for (int i = startIdx; i < bufLen; ++i) {
            float u = buffer[i] - offset;
            float in_a = u - (a_a1 * l_wz1_a) - (a_a2 * l_wz2_a);
            l_v_alpha = (a_b0 * in_a) + (a_b2 * l_wz2_a);
            l_wz2_a = l_wz1_a;
            l_wz1_a = in_a;

            float in_b = u - (a_a1 * l_wz1_b) - (a_a2 * l_wz2_b);
            l_v_beta = (b_b0 * in_b) + (b_b1 * l_wz1_b) + (b_b2 * l_wz2_b);
            l_wz2_b = l_wz1_b;
            l_wz1_b = in_b;
        }
        // Part 2: from 0 to remaining
        int remaining = endIdx - bufLen;
        for (int i = 0; i < remaining; ++i) {
            float u = buffer[i] - offset;
            float in_a = u - (a_a1 * l_wz1_a) - (a_a2 * l_wz2_a);
            l_v_alpha = (a_b0 * in_a) + (a_b2 * l_wz2_a);
            l_wz2_a = l_wz1_a;
            l_wz1_a = in_a;

            float in_b = u - (a_a1 * l_wz1_b) - (a_a2 * l_wz2_b);
            l_v_beta = (b_b0 * in_b) + (b_b1 * l_wz1_b) + (b_b2 * l_wz2_b);
            l_wz2_b = l_wz1_b;
            l_wz1_b = in_b;
        }
    }

    // Write back states
    wz1_a = l_wz1_a;
    wz2_a = l_wz2_a;
    wz1_b = l_wz1_b;
    wz2_b = l_wz2_b;
    v_alpha = l_v_alpha;
    v_beta = l_v_beta;
}


FrequencyAdaptivePLL::FrequencyAdaptivePLL(float nominal_freq, float kp, float ki)
    : nominal_freq(nominal_freq), kp(kp), ki(ki) {
    init();
}

void FrequencyAdaptivePLL::init() {
    freq = nominal_freq;
    omega = 2.0f * PI * freq;
    mag_smooth = 1.0f;
    integral = 0;
    integral_err_c = 0;
}

void IRAM_ATTR FrequencyAdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta;
    float mag_inst = sqrtf(mag_sq);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float raw_p_err = v_beta / mag_smooth; // for 90'

        // Kahan Summation for the Integral Term
        float y = (ki * raw_p_err) - integral_err_c;
        float t = integral + y;
        integral_err_c = (t - integral) - y;
        integral = t;

        // Anti-windup
        if (integral > 5.0f) integral = 5.0f;
        else if (integral < -5.0f) integral = -5.0f;

        float f_new = nominal_freq + (kp * raw_p_err) + integral;
        if (f_new > nominal_freq * 1.5f) f_new = nominal_freq * 1.5f;
        else if (f_new < nominal_freq * 0.6f) f_new = nominal_freq * 0.6f;

        freq = f_new;
        omega = 2.0f * PI * freq;
    }
}

AdaptivePLL::AdaptivePLL(float nominal_freq, float kp, float ki, float learn_rate)
    : FrequencyAdaptivePLL(nominal_freq, kp, ki), learn_rate(learn_rate) {
    init();
}

void AdaptivePLL::init() {
    FrequencyAdaptivePLL::init();
    integral_state = 0.0f;
    i_term = 0.0f;
    last_control_action = 0.0f;
    gain_est = 0.11f;
    hist_idx = 0;
    for (int i = 0; i < SOGI_HIST_LEN; i++) {
        control_hist[i] = 0.0f;
        phase_hist[i] = 0.0f;
    }
}

void IRAM_ATTR AdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    float mag_sq = v_alpha * v_alpha + v_beta * v_beta;
    float mag_inst = sqrtf(mag_sq);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float inv_mag = 1.0f / mag_smooth;
        float raw_p_err = v_beta * inv_mag;

        float predicted_effect = gain_est * last_control_action;
        float residual_err = raw_p_err - predicted_effect;

        uint8_t idx_prev = (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);
        float prev_phase = phase_hist[idx_prev];
        float prev_control = control_hist[idx_prev];
        float dy = raw_p_err - prev_phase;

        float denom = (prev_control * prev_control) + 1e-6f;
        float err_gain = (dy - gain_est * prev_control);
        gain_est += learn_rate * (prev_control * err_gain) / denom;

        if (gain_est > 5.0f) gain_est = 5.0f;
        else if (gain_est < -5.0f) gain_est = -5.0f;

        float p_term = 0;
        float abs_err = (raw_p_err < 0) ? -raw_p_err : raw_p_err;
        if (abs_err > PHASE_DEADBAND) {
            p_term = kp * raw_p_err;

            float y = residual_err - integral_err_c;
            float t = integral_state + y;
            integral_err_c = (t - integral_state) - y;
            integral_state = t;

            i_term = ki * integral_state;
        }

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

        float control_action = p_term + i_term;
        last_control_action = control_action;

        freq = nominal_freq + control_action;
        omega = 2.0f * PI * freq;

        phase_hist[hist_idx] = raw_p_err;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    }
}
