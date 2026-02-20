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

    // Augmented EKF Init
    x_theta = 0; // theta_signal (absolute relative to 50Hz)
    x_omega = 0; // omega_signal (absolute relative to 50Hz)
    x_beta = 0;

    pll_theta_offset = 0;

    memset(P, 0, sizeof(P));
    P[0][0] = 1.0f;
    P[1][1] = 10.0f;
    P[2][2] = 1.0f;

    memset(Q, 0, sizeof(Q));
    Q[0][0] = 0.001f;  // theta_sig random walk
    Q[1][1] = 0.01f;   // omega_sig random walk
    Q[2][2] = 0.0001f; // beta random walk
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

        // 1. Prediction Step
        // x = [theta_sig, omega_sig, beta]
        float theta_p = x_theta + x_omega * ts;
        float omega_p = x_omega;
        float beta_p  = x_beta;

        // P = F*P*F^T + Q
        float nP[3][3];
        nP[0][0] = P[0][0] + ts*(P[1][0] + P[0][1] + ts*P[1][1]) + Q[0][0];
        nP[0][1] = P[0][1] + ts*P[1][1] + Q[0][1];
        nP[0][2] = P[0][2] + ts*P[1][2] + Q[0][2];
        nP[1][0] = P[1][0] + ts*P[1][1] + Q[1][0];
        nP[1][1] = P[1][1] + Q[1][1];
        nP[1][2] = P[1][2] + Q[1][2];
        nP[2][0] = P[2][0] + ts*P[2][1] + Q[2][0];
        nP[2][1] = P[2][1] + Q[2][1];
        nP[2][2] = P[2][2] + Q[2][2];

        // 2. Update Step
        // Measurements: z1 = SOGI Error, z2 = Wavelet Abs, z3 = Wavelet Drift
        // h[0] = (theta_sig - theta_pll) + beta
        // h[1] = theta_sig
        // h[2] = omega_sig
        float z[3] = { sogi_p_err, abs_phase, wavelet_err / (ts + 1e-9f) };
        float h[3] = { (theta_p - pll_theta_offset) + beta_p, theta_p, omega_p };

        // Measurement Covariance R
        float R[3] = { 0.1f, 0.05f, 0.1f };

        // Discrepancy-based R inflation for SOGI
        float disc = fabs(z[0] - h[0]);
        if (disc > 0.05f) R[0] *= 100.0f;

        // Confidence-based R inflation for Wavelet
        if (confidence < 0.5f) { R[1] = 100.0f; R[2] = 10.0f; }

        for (int i = 0; i < 3; i++) {
            float H[3];
            if (i == 0)      { H[0]=1; H[1]=0; H[2]=1; } // z1
            else if (i == 1) { H[0]=1; H[1]=0; H[2]=0; } // z2
            else             { H[0]=0; H[1]=1; H[2]=0; } // z3

            float S = 0;
            for(int r=0; r<3; r++) for(int c=0; c<3; c++) S += H[r] * nP[r][c] * H[c];
            S += R[i];

            if (fabs(S) > 1e-12f) {
                float invS = 1.0f / S;
                float K[3];
                for(int r=0; r<3; r++) {
                    K[r] = 0;
                    for(int c=0; c<3; c++) K[r] += nP[r][c] * H[c];
                    K[r] *= invS;
                }

                float innov = z[i] - h[i];
                // Unwrap Innov for phase measurements
                if (i == 0 || i == 1) {
                    while (innov > PI) innov -= TWO_PI;
                    while (innov < -PI) innov += TWO_PI;
                }

                theta_p += K[0] * innov;
                omega_p += K[1] * innov;
                beta_p  += K[2] * innov;

                float tempP[3][3];
                for(int r=0; r<3; r++) {
                    for(int c=0; c<3; c++) {
                        tempP[r][c] = nP[r][c];
                        for(int k=0; k<3; k++) tempP[r][c] -= K[r] * H[k] * nP[k][c];
                    }
                }
                memcpy(nP, tempP, sizeof(nP));

                h[0] = (theta_p - pll_theta_offset) + beta_p;
                h[1] = theta_p;
                h[2] = omega_p;
            }
        }

        x_theta = theta_p;
        x_omega = omega_p;
        x_beta  = beta_p;
        for(int r=0; r<3; r++) for(int c=0; c<3; c++) P[r][c] = nP[r][c];

        // 3. Control Application
        float err_phase = x_theta - pll_theta_offset;
        while (err_phase > PI) err_phase -= TWO_PI;
        while (err_phase < -PI) err_phase += TWO_PI;

        float control_action = (x_omega / (2.0f * PI)) + (kp * err_phase);
        last_control_action = control_action;

        freq = nominal_freq + control_action;
        omega = 2.0f * PI * freq;

        phase_hist[hist_idx] = err_phase;
        control_hist[hist_idx] = control_action;
        hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);
    }
}

void AdaptivePLL::advanceTime(float ts) {
    pll_theta_offset += 2.0f * PI * (freq - nominal_freq) * ts;
    while (pll_theta_offset > PI) pll_theta_offset -= TWO_PI;
    while (pll_theta_offset < -PI) pll_theta_offset += TWO_PI;
}

float AdaptivePLL::getPhaseError() const {
    float err = x_theta - pll_theta_offset;
    while (err > PI) err -= TWO_PI;
    while (err < -PI) err += TWO_PI;
    return err;
}

void AdaptivePLL::shiftPhase(float delta_rad) {
    x_theta += delta_rad;
}
