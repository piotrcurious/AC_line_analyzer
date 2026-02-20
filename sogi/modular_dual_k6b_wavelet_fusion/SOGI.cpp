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

    // EKF Init
    x_phase = 0;
    x_omega = 0;
    P[0][0] = 1.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 10.0f;

    Q[0][0] = 0.1f;    Q[0][1] = 0.0f;
    Q[1][0] = 0.0f;    Q[1][1] = 100.0f;
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

void AdaptivePLL::updateFused(float v_alpha, float v_beta, float wavelet_err, float abs_phase, float confidence, float ts) {
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);

    if (mag_smooth > 0.10f) {
        float sogi_p_err = v_beta / mag_smooth;

        // 1. EKF Prediction Step
        // x = [phase_error (rad), omega_offset (rad/s)]
        // ts is the time elapsed since last call (~1 cycle duration)
        float x_phase_pred = x_phase + x_omega * ts;
        float x_omega_pred = x_omega;

        // Covariance Prediction: P = F*P*F^T + Q
        // F = [[1, ts], [0, 1]]
        float P00_p = P[0][0] + ts * (P[1][0] + P[0][1] + ts * P[1][1]) + Q[0][0];
        float P01_p = P[0][1] + ts * P[1][1] + Q[0][1];
        float P10_p = P[1][0] + ts * P[1][1] + Q[1][0];
        float P11_p = P[1][1] + Q[1][1];

        // 2. EKF Measurement Step
        // z1: Phase Error measurement
        // We fuse SOGI and Wavelet Absolute Phase for z1
        float z1;
        float R_z1;

        if (confidence > 0.5f) {
            z1 = abs_phase;
            R_z1 = 0.001f; // Trust absolute anchor
        } else {
            z1 = sogi_p_err;
            R_z1 = 0.1f;
        }

        // z2: Frequency Offset measurement (rad/s)
        float z2 = wavelet_err / (ts + 1e-9f);
        float R_z2 = (confidence > 0.8f) ? 0.001f : 0.1f;

        // Innovation Gate for SOGI fallback
        if (confidence < 0.5f) {
            float disc = fabs(sogi_p_err - x_phase_pred);
            if (disc > 0.05f) R_z1 *= 100.0f;
        }

        // Kalman Gain Calculation
        // H = [[1, 0], [0, 1]]
        float S00 = P00_p + R_z1;
        float S01 = P01_p;
        float S10 = P10_p;
        float S11 = P11_p + R_z2;

        float det = S00 * S11 - S01 * S10;
        if (fabs(det) > 1e-12f) {
            float inv_det = 1.0f / det;
            float K00 = (P00_p * S11 - P01_p * S10) * inv_det;
            float K01 = (P01_p * S00 - P00_p * S01) * inv_det;
            float K10 = (P10_p * S11 - P11_p * S10) * inv_det;
            float K11 = (P11_p * S00 - P10_p * S01) * inv_det;

            // State Update
            x_phase = x_phase_pred + K00 * (z1 - x_phase_pred) + K01 * (z2 - x_omega_pred);
            x_omega = x_omega_pred + K10 * (z1 - x_phase_pred) + K11 * (z2 - x_omega_pred);

            // Covariance Update: P = (I - K*H) * P_pred
            P[0][0] = (1.0f - K00) * P00_p - K01 * P10_p;
            P[0][1] = (1.0f - K00) * P01_p - K01 * P11_p;
            P[1][0] = -K10 * P00_p + (1.0f - K11) * P10_p;
            P[1][1] = -K10 * P01_p + (1.0f - K11) * P11_p;
        }

        // 3. PLL Control Application
        // Frequency is directly derived from the observer's omega offset
        float frequency_term = x_omega / (2.0f * PI);

        // Small damping term to drive residual phase error to zero
        float damping_term = 0.1f * x_phase;

        float control_action = frequency_term + damping_term;
        last_control_action = control_action;

        freq = nominal_freq + control_action;
        omega = 2.0f * PI * freq;

        // Log for LMS (optional, but keep for consistency)
        phase_hist[hist_idx] = x_phase;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    }
}

void AdaptivePLL::shiftPhase(float delta_rad) {
    x_phase += delta_rad;
    // We don't change P or x_omega, just the phase anchor
}
