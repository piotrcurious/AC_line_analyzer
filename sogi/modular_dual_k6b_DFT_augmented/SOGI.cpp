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

        float c1 = cosf(theta);
        float s1 = sinf(theta);

        sum_re1 += u * c1;
        sum_im1 += u * s1;

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
    // phase_err is atan2(real, imag) so that u = sin(theta + err)
    fundamental.phase_err = atan2f(fundamental.real, fundamental.imag);

    h3.mag = sqrtf(sum_re3 * sum_re3 + sum_im3 * sum_im3) * inv_count_2;
    h5.mag = sqrtf(sum_re5 * sum_re5 + sum_im5 * sum_im5) * inv_count_2;
    h7.mag = sqrtf(sum_re7 * sum_re7 + sum_im7 * sum_im7) * inv_count_2;

    if (fundamental.mag > 10.0f) { // threshold to avoid noise
        thd_approx = sqrtf(h3.mag * h3.mag + h5.mag * h5.mag + h7.mag * h7.mag) / fundamental.mag;
    } else {
        thd_approx = 0;
    }
}

void AdaptivePLL::updateWithDFT(float v_alpha, float v_beta, const DFTAnalyzer& dft, float ts) {
    // 1. Update Magnitude
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        // 2. Extract Phase Errors
        // SOGI phase error (approx sin(phi))
        float sogi_p_err = v_beta / mag_smooth;

        // DFT phase error (absolute phase relative to reference)
        // Correcting for window-center to window-end projection:
        // DFT measures the average phase over the window, which corresponds to the MIDDLE of the window.
        // SOGI measures the instantaneous phase at the END of the window.
        // If there is a frequency error df, the phase shifts by 2*PI*df*t.
        // The difference between middle and end is df * PI * T.
        float window_duration = dft.fundamental.mag > 0 ? (float)1.0f / freq : 0.02f;

        // We project the DFT phase to the end of the window.
        // To avoid circular dependency, we use the last known stable frequency offset (integral_state).
        float dft_p_err = dft.fundamental.phase_err + (integral_state * PI * window_duration);

        // Wrap DFT phase error to [-PI, PI]
        if (dft_p_err > PI) dft_p_err -= 2.0f * PI;
        if (dft_p_err < -PI) dft_p_err += 2.0f * PI;

        // 3. Harmonic Discrepancy Detection
        // A real frequency change will affect both DFT and SOGI similarly.
        // Harmonic distortion (especially odd harmonics like 3rd, 5th) affects SOGI
        // significantly because it's only a 2nd order filter, but DFT rejects them perfectly
        // (if the window is one period).
        float discrepancy = sogi_p_err - dft_p_err;
        filtered_discrepancy = 0.9f * filtered_discrepancy + 0.1f * discrepancy;

        // 4. Determine Trust and Combined Error
        float trust_in_dft = 0.0f;
        float distortion_score = dft.thd_approx + 0.5f * fabsf(filtered_discrepancy);

        if (distortion_score > 0.05f) {
            // Significant distortion -> trust DFT's harmonic rejection
            trust_in_dft = distortion_score * 10.0f;
            if (trust_in_dft > 0.95f) trust_in_dft = 0.95f;
        } else {
            // Clean signal -> use SOGI for faster response
            trust_in_dft = 0.1f;
        }

        float combined_err = (1.0f - trust_in_dft) * sogi_p_err + trust_in_dft * dft_p_err;

        // 5. Stable PI Controller
        // Windowed updates (50Hz rate) require lower gains than per-sample updates.
        float local_kp = 0.4f;
        float local_ki = 6.0f;

        // Proportional term
        float p_term = local_kp * combined_err;

        // Integral term (governs stable frequency)
        // Use Kahan summation or simple integration.
        // Note: ts here is the window duration if we call this once per cycle.
        // But the user calls it with 'ts' (per-sample). We should use window duration.
        float dt_window = window_duration;

        float y = (local_ki * dft_p_err * dt_window) - integral_err_c; // Use DFT for stable integration
        float t = integral_state + y;
        integral_err_c = (t - integral_state) - y;
        integral_state = t;

        // Anti-windup
        integral_state = constrain(integral_state, -10.0f, 10.0f);

        // 6. Update Frequency
        float control_action = p_term + integral_state;
        float f_new = nominal_freq + control_action;

        // Clamping to avoid crazy oscillations
        freq = constrain(f_new, nominal_freq - 10.0f, nominal_freq + 10.0f);
        omega = 2.0f * PI * freq;

        // 7. Update History (optional, for monitoring)
        phase_hist[hist_idx] = combined_err;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
        dft_initialized = true;
    }
}
