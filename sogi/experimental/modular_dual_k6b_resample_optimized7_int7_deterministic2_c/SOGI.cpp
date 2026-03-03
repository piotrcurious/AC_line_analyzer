#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float TWO_PI_F = 2.0f * M_PI;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

static inline int64_t q40_30_mul(int64_t a, int32_t b) {
    int64_t a_hi = a >> 30;
    int64_t a_lo = a & 0x3FFFFFFF;
    return (a_hi * (int64_t)b) + ((a_lo * (int64_t)b) >> 30);
}

// ==============================
// ========  SOGI  ==============
// ==============================

SOGI::SOGI(float k_) : k(k_) { init(); }
void SOGI::init() { reset(); coeff_valid = false; last_omega = 0.0f; last_ts = 0.0f; }
void SOGI::reset() { v_alpha = v_beta = 0; wz1_a = wz2_a = wz1_b = wz2_b = 0; }

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts) {
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;
    float wts  = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float den = 4.0f + 2.0f * k_wts + wts2;
    float det = (den > 1e-12f) ? (1.0f / den) : 0.25f; 

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

void IRAM_ATTR SOGI::step(q16_t u, float omega, float ts) {
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
    prev_nom_phase = 0.0f; buf_idx = 0; 
    rate_sum = 0.0f;
    rate_err = 0.0f; 
    for (int i=0; i<BUF_LEN; i++) rate_buf[i] = 0.0f;
}

void IRAM_ATTR TripleSOGIAnalyzer::process(q16_t input, float ts) {
    sogi_3pll.step(input, 3.0f * TWO_PI_F * grid_freq, ts);
    q16_t clean_input = input - sogi_3pll.v_alpha;
    sogi_nom.step(clean_input, TWO_PI_F * nominal_freq, ts);
    sogi_pll.step(clean_input, TWO_PI_F * grid_freq, ts);
    float nom_alpha = Q16_TO_FLOAT(sogi_nom.v_alpha);
    float nom_beta = Q16_TO_FLOAT(sogi_nom.v_beta);
    float current_nom_phase = atan2f(nom_alpha, -nom_beta);
    float dp = current_nom_phase - prev_nom_phase;
    if (dp > M_PI)  dp -= TWO_PI_F;
    if (dp < -M_PI) dp += TWO_PI_F;
    prev_nom_phase = current_nom_phase;
    float freq_inst = dp / (TWO_PI_F * fmaxf(ts, 1e-9f));
    float y_sub = -rate_buf[buf_idx] - rate_err;
    float t_sub = rate_sum + y_sub;
    rate_err = (t_sub - rate_sum) - y_sub;
    rate_sum = t_sub;
    rate_buf[buf_idx] = freq_inst;
    float y_add = freq_inst - rate_err;
    float t_add = rate_sum + y_add;
    rate_err = (t_add - rate_sum) - y_add;
    rate_sum = t_add;
    buf_idx = (buf_idx + 1) % BUF_LEN;
    grid_freq = rate_sum / (float)BUF_LEN;
    v_alpha = sogi_pll.v_alpha;
    v_beta  = sogi_pll.v_beta;
    float va_f = Q16_TO_FLOAT(v_alpha);
    float vb_f = Q16_TO_FLOAT(v_beta);
    float v3a_f = Q16_TO_FLOAT(sogi_3pll.v_alpha);
    float v3b_f = Q16_TO_FLOAT(sogi_3pll.v_beta);
    grid_phase = atan2f(va_f, -vb_f);
    v_mag = sqrtf(va_f * va_f + vb_f * vb_f + 1e-12f);
    v_mag_3rd = sqrtf(v3a_f * v3a_f + v3b_f * v3b_f);
    h3_ratio = v_mag_3rd / (v_mag + 1e-6f);
}

// ==============================
// === SlidingGNAnalyzer ========
// ==============================

SlidingGNAnalyzer::SlidingGNAnalyzer(float nominal_freq_hz, int K, int N)
    : Nwin(N), idx_head(0), samples_ready(0), cur_time(0.0f),
      Kharm(K), P(1 + 2 * K + 1), lambda(1e-3f) {
    alloc_mem();
    for (int i = 0; i < P; i++) p[i] = 0.0f;
    p[0] = nominal_freq_hz;
    for (int k = 0; k < K; k++) { reC[k] = 0.0f; imC[k] = 0.0f; }
    offset = 0.0f;
    lambda = lambda_init;
}

SlidingGNAnalyzer::~SlidingGNAnalyzer() {
    free_mem();
}

void SlidingGNAnalyzer::alloc_mem() {
    tbuf = (float*)calloc(Nwin, sizeof(float));
    sbuf = (float*)calloc(Nwin, sizeof(float));
    p = (float*)calloc(P, sizeof(float));
    delta_p = (float*)calloc(P, sizeof(float));
    reC = (float*)calloc(Kharm, sizeof(float));
    imC = (float*)calloc(Kharm, sizeof(float));
    JtJ = (float*)calloc(P * P, sizeof(float));
    Jtr = (float*)calloc(P, sizeof(float));
    work = (float*)calloc(P * P, sizeof(float));

    JtJ_work = (float*)calloc(P * P, sizeof(float));
    Jtr_work = (float*)calloc(P, sizeof(float));
    p_work   = (float*)calloc(P, sizeof(float));
    A_solver = (float*)calloc(P * P, sizeof(float));
    B_solver = (float*)calloc(P, sizeof(float));
    Ji_local = (float*)calloc(P, sizeof(float));
}

void SlidingGNAnalyzer::free_mem() {
    if (tbuf) free(tbuf);
    if (sbuf) free(sbuf);
    if (p) free(p);
    if (delta_p) free(delta_p);
    if (reC) free(reC);
    if (imC) free(imC);
    if (JtJ) free(JtJ);
    if (Jtr) free(Jtr);
    if (work) free(work);

    if (JtJ_work) free(JtJ_work);
    if (Jtr_work) free(Jtr_work);
    if (p_work)   free(p_work);
    if (A_solver) free(A_solver);
    if (B_solver) free(B_solver);
    if (Ji_local) free(Ji_local);
}

void IRAM_ATTR SlidingGNAnalyzer::addSample(q16_t sample, float ts) {
    cur_time += ts;
    float s = Q16_TO_FLOAT(sample);
    tbuf[idx_head] = cur_time;
    sbuf[idx_head] = s;
    idx_head = (idx_head + 1) % Nwin;
    if (samples_ready < Nwin) samples_ready++;
}

bool SlidingGNAnalyzer::processSampleAndSolve(q16_t sample, float ts, int max_iters) {
    addSample(sample, ts);
    return solve(max_iters);
}

void SlidingGNAnalyzer::init_params_from_data(float init_f) {
    p[0] = init_f;
    if (samples_ready == 0) return;

    // Simple DFT-like projection to initialize amplitudes
    float re_acc[Kharm], im_acc[Kharm];
    for (int k = 0; k < Kharm; k++) { re_acc[k] = 0.0f; im_acc[k] = 0.0f; }
    float offs = 0.0f;

    for (int i = 0; i < samples_ready; i++) {
        int idx = (idx_head + (Nwin - samples_ready) + i) % Nwin;
        float t = tbuf[idx] - cur_time;
        float s = sbuf[idx];
        offs += s;
        for (int k = 1; k <= Kharm; k++) {
            float ang = TWO_PI_F * k * init_f * t;
            re_acc[k - 1] += s * cosf(ang);
            im_acc[k - 1] += s * sinf(ang);
        }
    }

    float denom = (float)samples_ready;
    if (denom <= 0.0f) denom = 1.0f;
    for (int k = 1; k <= Kharm; k++) {
        float rk = (2.0f / denom) * re_acc[k - 1];
        float ik = -(2.0f / denom) * im_acc[k - 1];
        p[1 + 2 * (k - 1)] = rk;
        p[1 + 2 * (k - 1) + 1] = ik;
        reC[k-1] = rk;
        imC[k-1] = ik;
    }
    offset = offs / denom;
    p[P - 1] = offset;
}

void SlidingGNAnalyzer::mat_zero(float *m, int rows, int cols) {
    int n = rows * cols;
    for (int i = 0; i < n; i++) m[i] = 0.0f;
}

void SlidingGNAnalyzer::vec_zero(float *v, int n) {
    for (int i = 0; i < n; i++) v[i] = 0.0f;
}

float SlidingGNAnalyzer::hypot2_vec(float *v, int n) {
    float s = 0.0f;
    for (int i = 0; i < n; i++) s += v[i] * v[i];
    return s;
}

// Internal cost function: sum of squared residuals
static float calculateCost(int nSamples, int Nwin, int idx_head, float* tbuf, float* sbuf, float* p, int Kharm, int P, float cur_time) {
    float f = p[0];
    float cost = 0.0f;
    for (int i = 0; i < nSamples; i++) {
        int b_idx = (idx_head + (Nwin - nSamples) + i) % Nwin;
        float t = tbuf[b_idx] - cur_time;
        float s = sbuf[b_idx];
        float m = p[P - 1];
        for (int k = 1; k <= Kharm; k++) {
            float ang = TWO_PI_F * k * f * t;
            m += p[1 + 2 * (k - 1)] * cosf(ang) - p[1 + 2 * (k - 1) + 1] * sinf(ang);
        }
        float r = s - m;
        cost += r * r;
    }
    return cost;
}

void SlidingGNAnalyzer::buildResidualsAndJacobian(float *JtJ_out, float *Jtr_out, float sigma) {
    mat_zero(JtJ_out, P, P);
    vec_zero(Jtr_out, P);
    float f = p[0];
    int nSamples = samples_ready;
    if (nSamples == 0) return;
    float t_ref = cur_time;
    float inv_var = 1.0f / (sigma * sigma);

    for (int i = 0; i < nSamples; i++) {
        int b_idx = (idx_head + (Nwin - nSamples) + i) % Nwin;
        float t = tbuf[b_idx] - t_ref;
        float s = sbuf[b_idx];
        float m = p[P - 1];

        for (int j=0; j<P; j++) Ji_local[j] = 0;

        float df = 0.0f;
        for (int k = 1; k <= Kharm; k++) {
            float ang = TWO_PI_F * k * f * t;
            float cos_a = cosf(ang);
            float sin_a = sinf(ang);
            float Rek = p[1 + 2 * (k - 1)];
            float Imk = p[1 + 2 * (k - 1) + 1];
            m += Rek * cos_a - Imk * sin_a;
            df += -(float)k * TWO_PI_F * t * (Rek * sin_a + Imk * cos_a);
            Ji_local[1 + 2 * (k - 1)] = cos_a;
            Ji_local[1 + 2 * (k - 1) + 1] = -sin_a;
        }
        Ji_local[0] = df;
        Ji_local[P - 1] = 1.0f;

        float r = s - m;
        for (int a = 0; a < P; a++) {
            float Ja = Ji_local[a] * inv_var;
            Jtr_out[a] += Ja * r;
            for (int b = a; b < P; b++) {
                JtJ_out[a * P + b] += Ja * Ji_local[b];
            }
        }
    }
    for (int a = 0; a < P; a++) {
        for (int b = 0; b < a; b++) {
            JtJ_out[a * P + b] = JtJ_out[b * P + a];
        }
    }
}

bool SlidingGNAnalyzer::solve_normal_equations_and_apply(float *JtJ_in, float *Jtr_in, float *out_delta) {
    int n = P;
    for (int i = 0; i < n; i++) {
        B_solver[i] = Jtr_in[i];
        for (int j = 0; j < n; j++) A_solver[i * n + j] = JtJ_in[i * n + j];
    }
    for (int col = 0; col < n; ++col) {
        int piv = col;
        float maxabs = fabsf(A_solver[col * n + col]);
        for (int r = col + 1; r < n; r++) {
            float v = fabsf(A_solver[r * n + col]);
            if (v > maxabs) { maxabs = v; piv = r; }
        }
        if (maxabs < 1e-20f) return false;
        if (piv != col) {
            for (int j = col; j < n; j++) std::swap(A_solver[col * n + j], A_solver[piv * n + j]);
            std::swap(B_solver[col], B_solver[piv]);
        }
        float diag = A_solver[col * n + col];
        for (int j = col + 1; j < n; j++) {
            float fac = A_solver[j * n + col] / diag;
            if (fac == 0.0f) continue;
            for (int k = col + 1; k < n; k++) A_solver[j * n + k] -= fac * A_solver[col * n + k];
            B_solver[j] -= fac * B_solver[col];
            A_solver[j * n + col] = 0.0f;
        }
    }
    for (int i = n - 1; i >= 0; --i) {
        float s = B_solver[i];
        for (int j = i + 1; j < n; j++) s -= A_solver[i * n + j] * out_delta[j];
        float diag = A_solver[i * n + i];
        if (fabsf(diag) < 1e-25f) return false;
        out_delta[i] = s / diag;
    }
    return true;
}

bool SlidingGNAnalyzer::solve(int max_iters, int max_harmonics) {
    if (samples_ready < 16) return false;
    int old_Kharm = Kharm;
    if (max_harmonics > 0 && max_harmonics < Kharm) Kharm = max_harmonics;

    // Check if amplitudes are zero - if so, we need to initialize
    bool all_zero = true;
    for (int k=0; k<Kharm; k++) {
        if (fabsf(p[1+2*k]) > 1e-6f || fabsf(p[1+2*k+1]) > 1e-6f) { all_zero = false; break; }
    }
    if (all_zero || p[0] < 1.0f) {
        init_params_from_data(grid_freq);
    } else {
        for (int k = 0; k < Kharm; k++) {
            reC[k] = p[1 + 2 * k];
            imC[k] = p[1 + 2 * k + 1];
        }
        offset = p[P - 1];
    }

    float sigma = quant_lsb / sqrtf(12.0f);
    if (sigma <= 0.0f) sigma = 1e-6f;

    bool converged = false;
    float current_cost = calculateCost(samples_ready, Nwin, idx_head, tbuf, sbuf, p, Kharm, P, cur_time);

    for (int iter = 0; iter < max_iters; ++iter) {
        buildResidualsAndJacobian(JtJ, Jtr, sigma);

        for (int i = 0; i < P; i++) {
            for (int j = 0; j < P; j++) JtJ_work[i * P + j] = JtJ[i * P + j];
            JtJ_work[i * P + i] = JtJ_work[i * P + i] * (1.0f + lambda) + lambda;
        }

        if (!solve_normal_equations_and_apply(JtJ_work, Jtr, delta_p)) {
            lambda *= lambda_scale_up;
            continue;
        }

        for (int i = 0; i < P; i++) { p_work[i] = p[i] + delta_p[i]; }

        float tentative_cost = calculateCost(samples_ready, Nwin, idx_head, tbuf, sbuf, p_work, Kharm, P, cur_time);

        if (tentative_cost < current_cost) {
            for (int i = 0; i < P; i++) p[i] = p_work[i];
            current_cost = tentative_cost;
            lambda = std::max(lambda * lambda_scale_down, 1e-12f);
            if (hypot2_vec(delta_p, P) < min_delta_norm) { converged = true; break; }
        } else {
            lambda *= lambda_scale_up;
            if (lambda > 1e12f) break;
        }
    }

    grid_freq = p[0];
    for (int k = 0; k < Kharm; k++) {
        reC[k] = p[1 + 2 * k];
        imC[k] = p[1 + 2 * k + 1];
    }
    offset = p[P - 1];
    if (Kharm >= 1) {
        grid_phase = atan2f(imC[0], reC[0]);
        while (grid_phase > M_PI) grid_phase -= TWO_PI_F;
        while (grid_phase <= -M_PI) grid_phase += TWO_PI_F;
    }
    Kharm = old_Kharm;
    return converged;
}
