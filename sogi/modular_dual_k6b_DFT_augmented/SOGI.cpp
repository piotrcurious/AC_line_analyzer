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

void SOGI::step(float input, float omega, float ts) {
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

void SOGI::processWindow(const float* buffer, int bufLen, int startIdx, int count, float omega, float ts, float offset) {
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

    for (int i = 0; i < count; ++i) {
        int idx = (startIdx + i) % bufLen;
        float u = buffer[idx] - offset;

        // SOGI Alpha path (Band Pass)
        float in_a = u - (a_a1 * wz1_a) - (a_a2 * wz2_a);
        v_alpha = (a_b0 * in_a) + (a_b2 * wz2_a);
        wz2_a = wz1_a;
        wz1_a = in_a;

        // SOGI Beta path (Quadrature Low Pass)
        float in_b = u - (a_a1 * wz1_b) - (a_a2 * wz2_b);
        v_beta = (b_b0 * in_b) + (b_b1 * wz1_b) + (b_b2 * wz2_b);
        wz2_b = wz1_b;
        wz1_b = in_b;
    }
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

void FrequencyAdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float raw_p_err = v_beta / mag_smooth; // for 90'

        // Kahan Summation for the Integral Term
        float y = (ki * raw_p_err) - integral_err_c;
        float t = integral + y;
        integral_err_c = (t - integral) - y;
        integral = t;

        // Anti-windup
        integral = constrain(integral, -5.0f, 5.0f);

        float f_new = nominal_freq + (kp * raw_p_err) + integral;
        freq = constrain(f_new, nominal_freq * 0.8f, nominal_freq * 1.2f);
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
    filtered_discrepancy = 0.0f;
    dft_initialized = false;
}

void AdaptivePLL::update(float v_alpha, float v_beta, float ts) {
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float raw_p_err = v_beta / mag_smooth;

        float predicted_effect = gain_est * last_control_action;
        float residual_err = raw_p_err - predicted_effect;

        uint8_t idx_prev = (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);
        float prev_phase = phase_hist[idx_prev];
        float prev_control = control_hist[idx_prev];
        float dy = raw_p_err - prev_phase;

        float denom = (prev_control * prev_control) + 1e-6f;
        float err_gain = (dy - gain_est * prev_control);
        gain_est += learn_rate * (prev_control * err_gain) / denom;

        gain_est = constrain(gain_est, -5.0f, 5.0f);



       float p_term = 0;
      if (fabs(raw_p_err) > PHASE_DEADBAND) {
    // Only update when error is significant

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

        float f_new = nominal_freq + control_action;
        freq = f_new;
        //freq = constrain(f_new, nominal_freq * 0.5f, nominal_freq * 1.5f);
        omega = 2.0f * PI * freq;


        phase_hist[hist_idx] = raw_p_err;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    }
}

DFTAnalyzer::DFTAnalyzer() {
    fundamental = {0,0,0,0};
    h3 = {0,0,0,0};
    h5 = {0,0,0,0};
    h7 = {0,0,0,0};
    thd_approx = 0;
    res3 = res5 = res7 = 0;
    prev_res3 = prev_res5 = prev_res7 = 0;
    shape_stability = 1.0f;
}

static inline float wrap_pi(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

void DFTAnalyzer::analyze(const float* buffer, int bufLen, int startIdx, int count, float freq, float ts, float offset) {
    if (count <= 0) return;

    float omega = 2.0f * PI * freq;
    float sum_re1 = 0, sum_im1 = 0;
    float sum_re3 = 0, sum_im3 = 0;
    float sum_re5 = 0, sum_im5 = 0;
    float sum_re7 = 0, sum_im7 = 0;

    for (int i = 0; i < count; ++i) {
        int idx = (startIdx + i) % bufLen;
        float u = buffer[idx] - offset;
        float theta = omega * i * ts;

        sum_re1 += u * cosf(theta);
        sum_im1 += u * sinf(theta);
        sum_re3 += u * cosf(3.0f * theta);
        sum_im3 += u * sinf(3.0f * theta);
        sum_re5 += u * cosf(5.0f * theta);
        sum_im5 += u * sinf(5.0f * theta);
        sum_re7 += u * cosf(7.0f * theta);
        sum_im7 += u * sinf(7.0f * theta);
    }

    float inv_count_2 = 2.0f / (float)count;

    fundamental.real = sum_re1 * inv_count_2;
    fundamental.imag = sum_im1 * inv_count_2;
    fundamental.mag = sqrtf(fundamental.real * fundamental.real + fundamental.imag * fundamental.imag);
    fundamental.phase = atan2f(fundamental.real, fundamental.imag);

    h3.real = sum_re3 * inv_count_2;
    h3.imag = sum_im3 * inv_count_2;
    h3.mag = sqrtf(h3.real * h3.real + h3.imag * h3.imag);
    h3.phase = atan2f(h3.real, h3.imag);

    h5.real = sum_re5 * inv_count_2;
    h5.imag = sum_im5 * inv_count_2;
    h5.mag = sqrtf(h5.real * h5.real + h5.imag * h5.imag);
    h5.phase = atan2f(h5.real, h5.imag);

    h7.real = sum_re7 * inv_count_2;
    h7.imag = sum_im7 * inv_count_2;
    h7.mag = sqrtf(h7.real * h7.real + h7.imag * h7.imag);
    h7.phase = atan2f(h7.real, h7.imag);

    if (fundamental.mag > 10.0f) {
        thd_approx = sqrtf(h3.mag * h3.mag + h5.mag * h5.mag + h7.mag * h7.mag) / fundamental.mag;

        // Save previous for stability check
        prev_res3 = res3; prev_res5 = res5; prev_res7 = res7;

        // Calculate Phase Profile Residuals
        res3 = wrap_pi(h3.phase - 3.0f * fundamental.phase);
        res5 = wrap_pi(h5.phase - 5.0f * fundamental.phase);
        res7 = wrap_pi(h7.phase - 7.0f * fundamental.phase);

        // Shape stability: how much did the harmonic profile change?
        float d_res = fabsf(wrap_pi(res3 - prev_res3)) +
                      0.5f * fabsf(wrap_pi(res5 - prev_res5)) +
                      0.3f * fabsf(wrap_pi(res7 - prev_res7));

        // Convert to stability metric [0, 1]
        float current_stability = 1.0f / (1.0f + 10.0f * d_res);
        shape_stability = 0.8f * shape_stability + 0.2f * current_stability;
    } else {
        thd_approx = 0;
        res3 = res5 = res7 = 0;
        shape_stability = 1.0f;
    }
}

void AdaptivePLL::updateWithDFT(float v_alpha, float v_beta, const DFTAnalyzer& dft, float ts) {
    // 1. Update Magnitude
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        // 2. Extract Phase Errors
        float sogi_p_err = v_beta / mag_smooth;

        float window_duration = dft.fundamental.mag > 0 ? (float)1.0f / freq : 0.02f;

        // DFT measures absolute phase at window-center (if window is full period)
        // Project to window-end:
        float dft_fundamental_err = dft.fundamental.phase + (integral_state * PI * window_duration);
        dft_fundamental_err = wrap_pi(dft_fundamental_err);

        // 3. Harmonic-Aware Zero-Crossing Inference
        // We use the harmonic profile to calculate the offset between fundamental zero-crossing
        // and the composite signal's zero-crossing.
        // Formula: delta_zc = - sum(An * sin(alphan)) / sum(n * An * cos(alphan))
        // where alphan is the residual (phase_n - n * phase_1).

        float num = dft.h3.mag * sinf(dft.res3) + dft.h5.mag * sinf(dft.res5) + dft.h7.mag * sinf(dft.res7);
        float den = dft.fundamental.mag + 3.0f * dft.h3.mag * cosf(dft.res3) +
                    5.0f * dft.h5.mag * cosf(dft.res5) + 7.0f * dft.h7.mag * cosf(dft.res7);

        float delta_zc = (fabsf(den) > 1e-6f) ? -num / den : 0;

        // The 'overall' phase error that accounts for the zero-crossing shift:
        float overall_p_err = dft_fundamental_err + delta_zc;
        overall_p_err = wrap_pi(overall_p_err);

        // 4. Decision Logic based on Harmonic Profile
        // If the shape is stable, we trust the fundamental tracking.
        // If the shape is morphing, we dampen the update.
        float harmonic_trust = dft.shape_stability;
        if (dft.thd_approx < 0.02f) harmonic_trust = 1.0f; // clean signal is always stable

        // Discrepancy between SOGI and DFT-Fundamental
        float discrepancy = wrap_pi(sogi_p_err - dft_fundamental_err);
        filtered_discrepancy = 0.9f * filtered_discrepancy + 0.1f * discrepancy;

        // 5. Blended Error for PI controller
        // We use overall_p_err (which includes zero-crossing correction)
        // to keep the PLL aligned with the signal's "main body".
        float combined_err = (harmonic_trust * overall_p_err) + ((1.0f - harmonic_trust) * dft_fundamental_err);

        // 6. PI Controller
        float local_kp = 0.5f;
        float local_ki = 6.0f;

        float p_term = local_kp * combined_err;

        // Integral term tracks the actual grid frequency
        float dt_window = window_duration;
        float y = (local_ki * dft_fundamental_err * dt_window) - integral_err_c;
        float t = integral_state + y;
        integral_err_c = (t - integral_state) - y;
        integral_state = t;
        integral_state = constrain(integral_state, -10.0f, 10.0f);

        // 7. Apply Control Action
        float f_new = nominal_freq + p_term + integral_state;
        freq = constrain(f_new, nominal_freq - 15.0f, nominal_freq + 15.0f);
        omega = 2.0f * PI * freq;

        // History
        phase_hist[hist_idx] = combined_err;
        control_hist[hist_idx] = p_term + integral_state;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
        dft_initialized = true;
    }
}
