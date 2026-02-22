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
    prev_dft_phase = 0.0f;
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
    thd_approx = 0;
}

void DFTAnalyzer::analyze(const float* buffer, int bufLen, int startIdx, int count, float freq, float ts, float offset) {
    if (count <= 0) return;

    float omega = 2.0f * PI * freq;
    float sum_re1 = 0, sum_im1 = 0;
    float sum_re3 = 0, sum_im3 = 0;
    float sum_re5 = 0, sum_im5 = 0;

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
    }

    float inv_count_2 = 2.0f / (float)count;

    fundamental.real = sum_re1 * inv_count_2;
    fundamental.imag = sum_im1 * inv_count_2;
    fundamental.mag = sqrtf(fundamental.real * fundamental.real + fundamental.imag * fundamental.imag);
    fundamental.phase = atan2f(fundamental.imag, fundamental.real);

    h3.real = sum_re3 * inv_count_2;
    h3.imag = sum_im3 * inv_count_2;
    h3.mag = sqrtf(h3.real * h3.real + h3.imag * h3.imag);

    h5.real = sum_re5 * inv_count_2;
    h5.imag = sum_im5 * inv_count_2;
    h5.mag = sqrtf(h5.real * h5.real + h5.imag * h5.imag);

    if (fundamental.mag > 10.0f) { // threshold to avoid noise
        thd_approx = sqrtf(h3.mag * h3.mag + h5.mag * h5.mag) / fundamental.mag;
    } else {
        thd_approx = 0;
    }
}

void AdaptivePLL::updateWithDFT(float v_alpha, float v_beta, const DFTAnalyzer& dft, float ts) {
    // Current SOGI-based phase error
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float sogi_p_err = v_beta / mag_smooth;

        // DFT-based phase error
        float dft_p_err = 0.0f;
        if (dft_initialized) {
            float d_phi = dft.fundamental.phase - prev_dft_phase;
            if (d_phi > PI) d_phi -= 2.0f * PI;
            if (d_phi < -PI) d_phi += 2.0f * PI;

            // d_phi is the phase drift over one cycle.
            // Frequency error df = d_phi / (2*PI * T) = d_phi / (2*PI * count * ts)
            // But we can just use d_phi as a proportional error term.
            dft_p_err = d_phi;
        }
        prev_dft_phase = dft.fundamental.phase;
        dft_initialized = true;

        // Compare SOGI and DFT
        // SOGI error is instantaneous at the end of window.
        // DFT error (d_phi) is averaged over the window.

        float combined_err = sogi_p_err;

        // If THD is high, or if SOGI and DFT disagree significantly,
        // we suspect harmonic distortion is affecting SOGI.
        float discrepancy = fabsf(sogi_p_err - dft_p_err);

        float trust_factor = 1.0f;
        if (dft.thd_approx > 0.05f || discrepancy > 0.1f) {
            // Reduce trust in SOGI if distortion is high or discrepancy is large
            trust_factor = 1.0f / (1.0f + 5.0f * dft.thd_approx + 2.0f * discrepancy);
            if (trust_factor < 0.2f) trust_factor = 0.2f;

            // Blend with DFT error if we have it
            if (dft_initialized) {
                 combined_err = (trust_factor * sogi_p_err) + ((1.0f - trust_factor) * dft_p_err);
            } else {
                 combined_err = sogi_p_err * trust_factor;
            }
        }

        float raw_p_err = combined_err;

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
        omega = 2.0f * PI * freq;

        phase_hist[hist_idx] = raw_p_err;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    }
}
