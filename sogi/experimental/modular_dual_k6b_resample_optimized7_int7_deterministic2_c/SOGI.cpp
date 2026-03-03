#include "SOGI.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <algorithm> // For std::clamp if using C++17

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static constexpr float TWO_PI_F = 2.0f * M_PI;
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

// Improved multiplication: Ensure we don't lose the sign bit during the shift
static inline int64_t q40_30_mul(int64_t a, int32_t b) {
    int64_t a_hi = a >> 30;
    int64_t a_lo = a & 0x3FFFFFFF; // Mask for the fractional part
    return (a_hi * (int64_t)b) + ((a_lo * (int64_t)b) >> 30);
}

// ==============================
// ========  SOGI  ==============
// ==============================

SOGI::SOGI(float k_) : k(k_) { init(); }
void SOGI::init() { reset(); coeff_valid = false; last_omega = 0.0f; last_ts = 0.0f; }
void SOGI::reset() { v_alpha = v_beta = 0; wz1_a = wz2_a = wz1_b = wz2_b = 0; }

void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts)
{
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;
    float wts  = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    
    // Guard against division by zero if ts is 0
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

void IRAM_ATTR SOGI::step(q16_t u, float omega, float ts)
{
    if (!coeff_valid || fabsf(omega - last_omega) > 0.1f || fabsf(ts - last_ts) > 1e-9f) {
        updateCoefficients(omega, ts);
    }
    
    q40_t u_40 = Q16_TO_Q40(u);

    // Filter A (Bandpass - Alpha)
    // Internal states can grow; using 64-bit q40_t provides headroom, 
    // but we should ensure no wrapping occurs.
    q40_t in_a = u_40 - q40_30_mul(wz1_a, a_a1) - q40_30_mul(wz2_a, a_a2);
    q40_t va_40 = q40_30_mul(in_a, a_b0) + q40_30_mul(wz2_a, a_b2);
    
    v_alpha = Q40_TO_Q16(va_40);
    wz2_a = wz1_a; wz1_a = in_a;

    // Filter B (Lowpass - Beta)
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
    
    // Kahan/High-precision Init
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

    // Robust Phase Unwrapping
    float dp = current_nom_phase - prev_nom_phase;
    if (dp > M_PI)  dp -= TWO_PI_F;
    if (dp < -M_PI) dp += TWO_PI_F;
    prev_nom_phase = current_nom_phase;

    float freq_inst = dp / (TWO_PI_F * fmaxf(ts, 1e-9f));

    // --- Kahan Summation for rate_sum ---
    // Subtract old value with error compensation
    float y_sub = -rate_buf[buf_idx] - rate_err;
    float t_sub = rate_sum + y_sub;
    rate_err = (t_sub - rate_sum) - y_sub;
    rate_sum = t_sub;

    // Add new value with error compensation
    rate_buf[buf_idx] = freq_inst;
    float y_add = freq_inst - rate_err;
    float t_add = rate_sum + y_add;
    rate_err = (t_add - rate_sum) - y_add;
    rate_sum = t_add;

    buf_idx = (buf_idx + 1) % BUF_LEN;
    grid_freq = rate_sum / (float)BUF_LEN;

    // Output Updates
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

// sliding_gn_analyzer.h  (single-file)
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Replace these with your own q16 conversions if different
using q16_t = int32_t;
static inline float Q16_TO_FLOAT(q16_t x) { return ((float)x) / 32768.0f; }
static inline q16_t FLOAT_TO_Q16(float f) { return (q16_t)roundf(f * 32768.0f); }

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
static constexpr float TWO_PI_F = 2.0f * M_PI;

class SlidingGNAnalyzer {
public:
    // Constructor: nominal frequency (Hz), number of harmonics K, window length N
    SlidingGNAnalyzer(float nominal_freq_hz, int K, int N);

    // Add one sample (q16 fixed point) with sample time delta ts (seconds) since previous sample.
    // Note: ts may vary sample-to-sample; we accumulate absolute timestamps internally.
    void addSample(q16_t sample, float ts);

    // Run GN solver using the current window. Returns true if solver converged.
    bool solve(int max_iters = 6, int max_harmonics = -1);

    // Combined helper: add sample then run solver
    bool processSampleAndSolve(q16_t sample, float ts, int max_iters = 6);

    // Outputs after solve:
    float grid_freq = 50.0f;    // estimated fundamental frequency (Hz)
    float grid_phase = 0.0f;    // estimated fundamental phase (radians), convention: atan2(ImC1, ReC1)
    float offset = 0.0f;        // estimated DC offset
    // Access estimated harmonic phasors: for k=1..K
    // ReC[k-1], ImC[k-1] (real arrays)
    float *ReC() { return reC; }
    float *ImC() { return imC; }
    int K() const { return Kharm; }
    int N() const { return Nwin; }

    // Tuning knobs:
    float quant_lsb = 1.0f / 32768.0f; // LSB in signal units (adjust if different ADC/scaling)
    float lambda_init = 1e-3f;         // initial LM damping (increase for ill-conditioned)
    float lambda_scale_up = 10.0f;
    float lambda_scale_down = 0.1f;
    float min_delta_norm = 1e-6f;      // GN convergence tolerance (norm of parameter update)
    // Weighting: if you have an estimate of noise stddev sigma_noise, set quant_lsb accordingly.
    // Quantization sigma ≈ LSB / sqrt(12) by uniform quantization model.

private:
    // internal window buffer
    int Nwin;
    int idx_head;
    int samples_ready;
    float *tbuf;   // absolute timestamps in seconds
    float *sbuf;   // floating samples (converted from q16)
    float cur_time;

    // problem size
    int Kharm;

    // parameters: p = [ f, ReC1, ImC1, ReC2, ImC2, ..., ReCK, ImCK, offset ]
    int P;          // number of parameters
    float *p;       // parameter vector length P
    float *delta_p; // temporary update vector length P

    // output phasors backing store
    float *reC;
    float *imC;

    // GN workspace (small matrices/vectors)
    float *JtJ;     // P x P symmetric normal matrix (stored row-major)
    float *Jtr;     // P vector
    float *work;    // general workspace for linear solver

    // LM damping parameter
    float lambda;

    // helpers
    void alloc_mem();
    void free_mem();
    void init_params_from_data(float init_f);
    void buildResidualsAndJacobian(float *residual_out /*N*/, float *JtJ_out /*P*P*/, float *Jtr_out /*P*/, float sigma);
    bool solve_normal_equations_and_apply(float *JtJ_in, float *Jtr_in, float *out_delta); // uses Gauss pivot
    static void mat_zero(float *m, int rows, int cols);
    static void vec_zero(float *v, int n);
    static float hypot2_vec(float *v, int n);
};

// ---------------- Implementation ----------------

#include <algorithm> // for std::swap

SlidingGNAnalyzer::SlidingGNAnalyzer(float nominal_freq_hz, int K, int N)
    : Nwin(N), idx_head(0), samples_ready(0), cur_time(0.0f),
      Kharm(K), P(1 + 2*K + 1), lambda(1e-3f)
{
    alloc_mem();
    // initial parameter guess: frequency = nominal, phasors small
    for (int i=0;i<P;i++) p[i] = 0.0f;
    p[0] = nominal_freq_hz; // f
    // ReC/ImC default zero
    for (int k=0;k<K;k++) { reC[k] = 0.0f; imC[k] = 0.0f; }
    offset = 0.0f;
    lambda = lambda_init;
}

void SlidingGNAnalyzer::alloc_mem() {
    tbuf = (float*)malloc(sizeof(float)*Nwin);
    sbuf = (float*)malloc(sizeof(float)*Nwin);
    p = (float*)malloc(sizeof(float)*P);
    delta_p = (float*)malloc(sizeof(float)*P);
    reC = (float*)malloc(sizeof(float)*Kharm);
    imC = (float*)malloc(sizeof(float)*Kharm);
    JtJ = (float*)malloc(sizeof(float)*P*P);
    Jtr = (float*)malloc(sizeof(float)*P);
    work = (float*)malloc(sizeof(float)*P*P); // reuse as workspace
    // zero buffers
    for (int i=0;i<Nwin;i++) { tbuf[i]=0.0f; sbuf[i]=0.0f; }
    for (int i=0;i<P;i++) { p[i]=0.0f; delta_p[i]=0.0f; Jtr[i]=0.0f; }
    for (int i=0;i<P*P;i++) JtJ[i]=0.0f;
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
}

// add sample (q16) with delta time ts (seconds)
void SlidingGNAnalyzer::addSample(q16_t sample, float ts) {
    cur_time += ts;
    float s = Q16_TO_FLOAT(sample);
    // store at head (circular)
    tbuf[idx_head] = cur_time;
    sbuf[idx_head] = s;
    idx_head = (idx_head + 1) % Nwin;
    if (samples_ready < Nwin) samples_ready++;
}

// process combined
bool SlidingGNAnalyzer::processSampleAndSolve(q16_t sample, float ts, int max_iters) {
    addSample(sample, ts);
    return solve(max_iters);
}

// initialize parameters from data (first guess) - DFT-like projection at nominal f
void SlidingGNAnalyzer::init_params_from_data(float init_f) {
    // param p[0] = f
    p[0] = init_f;
    // compute times relative to newest sample (latest time = cur_time)
    if (samples_ready == 0) return;
    // zero phasor accumulators
    for (int k=0;k<Kharm;k++) { reC[k] = 0.0f; imC[k] = 0.0f; }
    float offs = 0.0f;
    for (int i=0;i<samples_ready;i++) {
        int idx = (idx_head + i) % Nwin; // careful: idx_head points to next write position; data older at idx_head
        // we want oldest->newest; but we just accumulate regardless
        float t = tbuf[idx] - cur_time; // negative or zero
        float s = sbuf[idx];
        offs += s;
        for (int k=1;k<=Kharm;k++) {
            float ang = TWO_PI_F * k * init_f * t;
            reC[k-1] += s * cosf(ang);
            imC[k-1] += s * sinf(ang); // note: we will map signs later to match model Re{C e^{j...}} => ReC*cos - ImC*sin
        }
    }
    // normalize by N (simple projection)
    float denom = (float)samples_ready;
    if (denom <= 0.0f) denom = 1.0f;
    for (int k=0;k<Kharm;k++) {
        // We measured s*exp(+j k omega t) -> want ReC and ImC such that m = ReC*cos - ImC*sin.
        // The projection above yields: re_proj = sum s*cos, im_proj = sum s*sin
        // So set ReC = (2/N)*re_proj, ImC = -(2/N)*im_proj to match sign convention used later.
        reC[k] = (2.0f/denom) * reC[k];
        imC[k] = -(2.0f/denom) * imC[k];
        // put into parameter vector
        p[1 + 2*k - 1] = reC[k];       // careful indexing: p[1 + 2*(k-1)]
        p[1 + 2*k]     = imC[k];
    }
    offset = offs / denom;
    p[P-1] = offset;
}

// zeros a matrix
void SlidingGNAnalyzer::mat_zero(float *m, int rows, int cols) {
    int n = rows*cols;
    for (int i=0;i<n;i++) m[i]=0.0f;
}
void SlidingGNAnalyzer::vec_zero(float *v, int n) { for (int i=0;i<n;i++) v[i]=0.0f; }
float SlidingGNAnalyzer::hypot2_vec(float *v, int n) {
    float s=0.0f; for (int i=0;i<n;i++) s += v[i]*v[i]; return s;
}

// Build residuals and normal matrix J^T J and J^T r
// We directly accumulate weighted J^T J and J^T r using quantization-normalization
void SlidingGNAnalyzer::buildResidualsAndJacobian(float * /*residual_out*/, float *JtJ_out, float *Jtr_out, float sigma) {
    // sigma : stddev of measurement noise (uniform quantization approx => LSB/sqrt(12)). We'll weight by 1/sigma.
    // We'll compute weighted normal equations: JtWJ and JtWr with W = I * (1/sigma^2)
    mat_zero(JtJ_out, P, P);
    vec_zero(Jtr_out, P);

    // For each sample i produce residual r_i and Jacobian row Ji (length P)
    // We compute model m_i = offset + sum_k ( ReCk * cos(k*2π*f*t) - ImCk * sin(k*2π*f*t) )
    // Param ordering: p[0]=f ; then p[1]=ReC1, p[2]=ImC1, p[3]=ReC2, p[4]=ImC2 ... p[P-1]=offset
    float f = p[0];
    // Note timestamps are absolute; we shift to latest reference cur_time to keep numeric range small.
    int nSamples = samples_ready;
    if (nSamples == 0) return;
    float t_ref = cur_time; // latest time
    float inv_var = 1.0f / (sigma*sigma);

    for (int i=0;i<nSamples;i++) {
        // map buffer index oldest->newest: oldest at idx_head, newest at idx_head-1
        int buf_idx = (idx_head + i) % Nwin;
        float t = tbuf[buf_idx] - t_ref; // negative or zero, seconds
        float s = sbuf[buf_idx];
        // compute model m
        float m = p[P-1]; // offset
        // we'll also build partials into local array Ji[P]
        float Ji_local[64]; // support up to P<=64; change if you need bigger
        if (P > 64) { /* defensive: should not happen in our usage */ }

        // derivative w.r.t f:
        // ∂m/∂f = - Σ_k k * 2π * t * ( ReCk * sin(k*2π*f*t) + ImCk * cos(k*2π*f*t) )
        float df = 0.0f;
        for (int k=1;k<=Kharm;k++) {
            float ang = TWO_PI_F * k * f * t;
            float cos_a = cosf(ang);
            float sin_a = sinf(ang);
            float Rek = p[1 + 2*(k-1)];
            float Imk = p[1 + 2*(k-1) + 1];
            m += Rek * cos_a - Imk * sin_a;
            df += - (float)k * TWO_PI_F * t * ( Rek * sin_a + Imk * cos_a );
        }
        // fill Ji_local
        Ji_local[0] = df; // d/d f
        // then ReCk and ImCk
        for (int k=1;k<=Kharm;k++) {
            float ang = TWO_PI_F * k * f * t;
            float cos_a = cosf(ang);
            float sin_a = sinf(ang);
            // ∂m/∂ReCk = cos(k*2π*f*t)
            // ∂m/∂ImCk = -sin(k*2π*f*t)
            Ji_local[1 + 2*(k-1)    ] = cos_a;
            Ji_local[1 + 2*(k-1) + 1] = -sin_a;
        }
        Ji_local[P-1] = 1.0f; // offset

        // residual r = s - m
        float r = s - m;

        // Weighted accumulation of normal equations
        // JtJ += inv_var * Ji^T * Ji
        // Jtr += inv_var * Ji^T * r
        for (int a=0;a<P;a++) {
            float Ja = Ji_local[a] * inv_var;
            // Jtr[a] += Ja * r
            Jtr_out[a] += Ja * r;
            for (int b=a;b<P;b++) {
                JtJ_out[a*P + b] += Ja * Ji_local[b];
            }
        }
    }
    // Fill symmetric lower triangle
    for (int a=0;a<P;a++) {
        for (int b=0;b<a;b++) {
            JtJ_out[a*P + b] = JtJ_out[b*P + a];
        }
    }
}

// Solve normal equations (JtJ x = Jtr) with LM damping applied by caller (JtJ_in already damped).
// We perform Gauss elimination with partial pivoting; returns true if solved successfully.
bool SlidingGNAnalyzer::solve_normal_equations_and_apply(float *JtJ_in, float *Jtr_in, float *out_delta) {
    // Copy JtJ_in into work matrix; make augmented with Jtr_in
    int n = P;
    // create augmented matrix A (n x (n+1)) in work
    float *A = work; // reuse P*P allocated; but we need n*(n+1) floats: we allocated P*P so we'll do in-place solve by constructing smaller arrays.
    // Because work size == P*P, we'll do elimination using JtJ_in copy in-place (since JtJ_in and Jtr_in may be reused later).
    // Create a local copy of JtJ_in into A (n x n) and extend with Jtr_in into a separate vector b
    // allocate small local arrays on stack for moderate P
    float B[64]; // n up to 64; increase if needed
    if (n > 64) return false;
    float A_local[64*64];
    for (int i=0;i<n;i++) {
        B[i] = Jtr_in[i];
        for (int j=0;j<n;j++) {
            A_local[i*n + j] = JtJ_in[i*n + j];
        }
    }
    // Gauss elimination with partial pivot
    for (int col=0; col<n; ++col) {
        // find pivot
        int piv = col;
        float maxabs = fabsf(A_local[col*n + col]);
        for (int r=col+1;r<n;r++) {
            float v = fabsf(A_local[r*n + col]);
            if (v > maxabs) { maxabs = v; piv = r; }
        }
        if (maxabs < 1e-20f) {
            // singular or ill-conditioned
            return false;
        }
        // swap rows col and piv if needed
        if (piv != col) {
            for (int j=col;j<n;j++) std::swap(A_local[col*n + j], A_local[piv*n + j]);
            std::swap(B[col], B[piv]);
        }
        // normalize and eliminate
        float diag = A_local[col*n + col];
        for (int j=col+1;j<n;j++) {
            float fac = A_local[j*n + col] / diag;
            if (fac == 0.0f) continue;
            // row j -= fac * row col
            for (int k=col+1;k<n;k++) {
                A_local[j*n + k] -= fac * A_local[col*n + k];
            }
            B[j] -= fac * B[col];
            A_local[j*n + col] = 0.0f;
        }
    }
    // back-substitute
    for (int i=n-1;i>=0;--i) {
        float s = B[i];
        for (int j=i+1;j<n;j++) s -= A_local[i*n + j] * out_delta[j];
        float diag = A_local[i*n + i];
        if (fabsf(diag) < 1e-25f) return false;
        out_delta[i] = s / diag;
    }
    return true;
}

bool SlidingGNAnalyzer::solve(int max_iters, int max_harmonics) {
    if (samples_ready < 4) return false; // too few samples
    if (max_harmonics > 0 && max_harmonics < Kharm) Kharm = max_harmonics; // shrink if requested

    // Initial parameter guess: warm start from previous p. If first time, do DFT-style init
    // Detect if p is zeroed: check frequency near zero
    if (p[0] <= 0.0f) {
        init_params_from_data(grid_freq);
        // push reC/imC into p
        for (int k=0;k<Kharm;k++) {
            p[1 + 2*k]     = reC[k];
            p[1 + 2*k + 1] = imC[k];
        }
        p[P-1] = offset;
    } else {
        // ensure reC/imC reflect current p
        for (int k=0;k<Kharm;k++) {
            reC[k] = p[1 + 2*k];
            imC[k] = p[1 + 2*k + 1];
        }
        offset = p[P-1];
    }

    // measurement sigma (quantization noise model)
    float sigma = quant_lsb / sqrtf(12.0f); // uniform quantization stddev
    if (sigma <= 0.0f) sigma = 1e-6f;

    // Main GN loop
    bool converged = false;
    float prev_err_norm = 1e30f;
    for (int iter=0; iter<max_iters; ++iter) {
        // build JtJ and Jtr (weighted)
        mat_zero(JtJ, P, P);
        vec_zero(Jtr, P);
        buildResidualsAndJacobian(nullptr, JtJ, Jtr, sigma);

        // compute residual norm for monitoring: ||Jtr|| as surrogate
        float err_norm = 0.0f;
        for (int i=0;i<P;i++) err_norm += Jtr[i]*Jtr[i];
        // adapt lambda based on success in previous iteration
        // Apply LM damping: add lambda * diag(JtJ) to JtJ
        // We create a damped copy in work matrix for solving (but our solve function copies input).
        float JtJ_damped[64*64];
        if (P > 64) return false;
        for (int i=0;i<P;i++) {
            for (int j=0;j<P;j++) {
                JtJ_damped[i*P + j] = JtJ[i*P + j];
            }
        }
        for (int i=0;i<P;i++) {
            JtJ_damped[i*P + i] *= (1.0f + lambda);
        }

        // Solve for delta_p: (JtJ_damped) * delta = Jtr
        for (int i=0;i<P;i++) delta_p[i] = 0.0f;
        bool ok = solve_normal_equations_and_apply(JtJ_damped, Jtr, delta_p);
        if (!ok) {
            // increase damping and retry
            lambda *= lambda_scale_up;
            continue;
        }

        // apply tentative update and compute new residual norm (quick check: approximate)
        // Apply update: p_new = p + delta_p
        float p_saved[64];
        for (int i=0;i<P;i++) { p_saved[i] = p[i]; p[i] += delta_p[i]; }

        // compute new weighted residual norm quickly by rebuilding JtJ/Jtr and using Jtr^T * delta (approx)
        mat_zero(JtJ, P, P); vec_zero(Jtr, P);
        buildResidualsAndJacobian(nullptr, JtJ, Jtr, sigma);
        float new_err_norm = 0.0f;
        for (int i=0;i<P;i++) new_err_norm += Jtr[i]*Jtr[i];

        if (new_err_norm < err_norm) {
            // accept step, reduce lambda
            lambda = fmaxf(lambda * lambda_scale_down, 1e-12f);
            // commit p already updated
        } else {
            // reject step: restore p, increase lambda and retry
            for (int i=0;i<P;i++) p[i] = p_saved[i];
            lambda *= lambda_scale_up;
            // if lambda grows too large, break
            if (lambda > 1e12f) break;
            continue;
        }

        // check convergence by delta norm
        float delta_norm = hypot2_vec(delta_p, P);
        if (delta_norm < min_delta_norm) { converged = true; break; }
        prev_err_norm = err_norm;
    }

    // copy parameter vector to outputs
    grid_freq = p[0];
    for (int k=0;k<Kharm;k++) {
        reC[k] = p[1 + 2*k];
        imC[k] = p[1 + 2*k + 1];
    }
    offset = p[P-1];

    // compute fundamental phase (atan2(ImC1, ReC1)) and normalize to [-pi,pi]
    if (Kharm >= 1) {
        grid_phase = atan2f(imC[0], reC[0]);
        if (grid_phase > M_PI) grid_phase -= TWO_PI_F;
        if (grid_phase <= -M_PI) grid_phase += TWO_PI_F;
    } else {
        grid_phase = 0.0f;
    }
    // output phasors are available via ReC()/ImC()

    return converged;
}
