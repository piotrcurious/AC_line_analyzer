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
    float in_a = input - (a_a1 * wz1_a) - (a_a2 * wz2_a);
    v_alpha = (a_b0 * in_a) + (a_b2 * wz2_a);
    wz2_a = wz1_a; wz1_a = in_a;
    float in_b = input - (a_a1 * wz1_b) - (a_a2 * wz2_b);
    v_beta = (b_b0 * in_b) + (b_b1 * wz1_b) + (b_b2 * wz2_b);
    wz2_b = wz1_b; wz1_b = in_b;
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
        float in_a = u - (a_a1 * wz1_a) - (a_a2 * wz2_a);
        v_alpha = (a_b0 * in_a) + (a_b2 * wz2_a);
        wz2_a = wz1_a; wz1_a = in_a;
        float in_b = u - (a_a1 * wz1_b) - (a_a2 * wz2_b);
        v_beta = (b_b0 * in_b) + (b_b1 * wz1_b) + (b_b2 * wz2_b);
        wz2_b = wz1_b; wz1_b = in_b;
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
        float raw_p_err = v_beta / mag_smooth;
        float y = (ki * raw_p_err) - integral_err_c;
        float t = integral + y;
        integral_err_c = (t - integral) - y;
        integral = t;
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
    last_control_action = 0.0f;
    gain_est = 0.11f;
    hist_idx = 0;
    for (int i = 0; i < SOGI_HIST_LEN; i++) { control_hist[i] = 0.0f; phase_hist[i] = 0.0f; }
    x_theta = 0; x_omega = 0; x_beta = 0; sampling_phi = 0;
    memset(P, 0, sizeof(P));
    P[0][0] = 0.1f; P[1][1] = 10.0f; P[2][2] = 0.1f;
    memset(Q, 0, sizeof(Q));
    Q[0][0] = 0.1f; Q[1][1] = 10.0f; Q[2][2] = 0.01f;
}

void AdaptivePLL::predict(float ts) {
    x_theta += x_omega * ts;
    while (x_theta > PI) x_theta -= TWO_PI;
    while (x_theta < -PI) x_theta += TWO_PI;
    float P11 = P[0][0], P12 = P[0][1], P13 = P[0][2];
    float P22 = P[1][1], P23 = P[1][2];
    float P33 = P[2][2];
    P[0][0] = P11 + 2.0f*P12*ts + P22*ts*ts + Q[0][0]*ts;
    P[0][1] = P12 + P22*ts;
    P[0][2] = P13 + P23*ts;
    P[1][0] = P[0][1]; P[1][1] = P22 + Q[1][1]*ts; P[1][2] = P23;
    P[2][0] = P[0][2]; P[2][1] = P[1][2]; P[2][2] = P33 + Q[2][2]*ts;
    sampling_phi += (omega - 2.0f * PI * nominal_freq) * ts;
    while (sampling_phi > PI) sampling_phi -= TWO_PI;
    while (sampling_phi < -PI) sampling_phi += TWO_PI;
}

void AdaptivePLL::updateSOGI(float v_alpha, float v_beta, float ts) {
    float mag_inst = sqrtf(v_alpha * v_alpha + v_beta * v_beta);
    mag_smooth = (MAG_ALPHA * mag_inst) + ((1.0f - MAG_ALPHA) * mag_smooth);
    if (mag_smooth > 0.10f) {
        float sogi_p_err = v_beta / mag_smooth;
        float z[1] = { sogi_p_err };
        float h[1] = { (x_theta - sampling_phi) + x_beta };
        float R[1] = { 0.05f };
        float H[1][3] = { {1.0f, 0.0f, 1.0f} };
        sequentialUpdate(z, h, R, H, 1);
        float err_phi = getPhaseError();
        float control_action_hz = (x_omega / TWO_PI) + kp * err_phi;
        freq = nominal_freq + control_action_hz;
        omega = freq * TWO_PI;
    }
}

void AdaptivePLL::updateWavelet(float abs_phase, float drift_rate, float confidence, float ts) {
    float z[2] = { abs_phase, drift_rate };
    float h[2] = { x_theta, x_omega };
    float R[2] = { 0.01f, 0.1f };
    float H[2][3] = { {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f} };
    if (confidence < 0.8f) { R[0] *= 10.0f; R[1] *= 10.0f; }
    sequentialUpdate(z, h, R, H, 2);
    float err_phi = getPhaseError();
    float control_action_hz = (x_omega / TWO_PI) + kp * err_phi;
    freq = nominal_freq + control_action_hz;
    omega = freq * TWO_PI;
}

void AdaptivePLL::sequentialUpdate(const float z[], const float h[], const float R[], const float H[][3], int num_measurements) {
    for (int i = 0; i < num_measurements; i++) {
        float S = 0;
        for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) S += H[i][r] * P[r][c] * H[i][c];
        S += R[i];
        if (fabs(S) > 1e-12f) {
            float invS = 1.0f / S;
            float K[3];
            for (int r = 0; r < 3; r++) { K[r] = 0; for (int c = 0; c < 3; c++) K[r] += P[r][c] * H[i][c]; K[r] *= invS; }
            float innov = z[i] - h[i];
            if (i == 0) { while (innov > PI) innov -= TWO_PI; while (innov < -PI) innov += TWO_PI; }
            x_theta += K[0] * innov; x_omega += K[1] * innov; x_beta += K[2] * innov;
            while (x_theta > PI) x_theta -= TWO_PI; while (x_theta < -PI) x_theta += TWO_PI;
            float KHP[3][3];
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    KHP[r][c] = 0;
                    for (int k = 0; k < 3; k++) KHP[r][c] += K[r] * H[i][k] * P[k][c];
                }
            }
            for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) P[r][c] -= KHP[r][c];
        }
    }
}

float AdaptivePLL::getPhaseError() const {
    float err = x_theta - sampling_phi;
    while (err > PI) err -= TWO_PI;
    while (err < -PI) err += TWO_PI;
    return err;
}

void AdaptivePLL::shiftPhase(float delta_rad) {
    x_theta += delta_rad;
    while (x_theta > PI) x_theta -= TWO_PI;
    while (x_theta < -PI) x_theta += TWO_PI;
}

void AdaptivePLL::setNominalFreq(float f) { nominal_freq = f; }
