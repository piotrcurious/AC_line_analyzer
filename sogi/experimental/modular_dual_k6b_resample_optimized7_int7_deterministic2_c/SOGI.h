#ifndef SOGI_H
#define SOGI_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdint.h>
#include <cmath>
#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
#endif

// Fixed-point definitions
typedef int32_t q16_t;
#define Q16_SHIFT 16
#define Q16_ONE (1 << Q16_SHIFT)
#define FLOAT_TO_Q16(x) ((q16_t)((x) * Q16_ONE))
#define Q16_TO_FLOAT(x) ((float)(x) / Q16_ONE)

// High-precision states (Q40)
typedef int64_t q40_t;
#define Q40_SHIFT 40
#define Q40_ONE (1LL << Q40_SHIFT)
#define Q16_TO_Q40(x) (((int64_t)(x)) << (Q40_SHIFT - Q16_SHIFT))
#define Q40_TO_Q16(x) ((int32_t)((x) >> (Q40_SHIFT - Q16_SHIFT)))

// Coefficients (Q30)
typedef int32_t q30_t;
#define Q30_SHIFT 30
#define Q30_ONE (1 << Q30_SHIFT)
#define FLOAT_TO_Q30(x) ((q30_t)((x) * Q30_ONE))

class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();
    void IRAM_ATTR step(q16_t input, float omega, float ts);

    q16_t v_alpha;
    q16_t v_beta;
    float k;

private:
    q40_t wz1_a, wz2_a;
    q40_t wz1_b, wz2_b;
    q30_t a_b0, a_b2, a_a1, a_a2, b_b0, b_b1, b_b2;
    bool coeff_valid;
    float last_omega, last_ts;
    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

class TripleSOGIAnalyzer {
public:
    TripleSOGIAnalyzer(float nominal_f = 50.0f, float k = 0.7071f);
    void init();
    void IRAM_ATTR process(q16_t input, float ts);

    // Outputs
    float grid_freq;
    float grid_phase;
    float v_mag;
    float v_mag_3rd;
    float h3_ratio;
 

    // Fixed-point fundamental for display
    q16_t v_alpha, v_beta;

private:
    SOGI sogi_nom;  // Fixed at nominal frequency (e.g. 50Hz)
    SOGI sogi_pll;  // Adaptive to estimated grid frequency
    SOGI sogi_3pll; // Adaptive to 3 * estimated grid frequency

    float nominal_freq;
    float prev_nom_phase;

    // Sliding window for deterministic frequency solver
    static constexpr int BUF_LEN = 256; // Increased for stability across frame sizes
    float rate_buf[BUF_LEN];
    int buf_idx;
    float rate_sum;
       float rate_err;
};

class SlidingGNAnalyzer {
public:
    // Constructor: nominal frequency (Hz), number of harmonics K, window length N
    SlidingGNAnalyzer(float nominal_freq_hz, int K, int N);
    ~SlidingGNAnalyzer();

    // Add one sample (q16 fixed point) with sample time delta ts (seconds) since previous sample.
    void IRAM_ATTR addSample(q16_t sample, float ts);

    // Run GN solver using the current window. Returns true if solver converged.
    bool solve(int max_iters = 6, int max_harmonics = -1);

    // Combined helper: add sample then run solver
    bool processSampleAndSolve(q16_t sample, float ts, int max_iters = 6);

    // Outputs after solve:
    float grid_freq = 50.0f;    // estimated fundamental frequency (Hz)
    float grid_phase = 0.0f;    // estimated fundamental phase (radians)
    float offset = 0.0f;        // estimated DC offset

    float *ReC() { return reC; }
    float *ImC() { return imC; }
    int K() const { return Kharm; }
    int N() const { return Nwin; }

    // Tuning knobs:
    float quant_lsb = 1.0f / 65536.0f; // Adjusted for Q16
    float lambda_init = 1e-3f;
    float lambda_scale_up = 10.0f;
    float lambda_scale_down = 0.1f;
    float min_delta_norm = 1e-6f;

private:
    int Nwin;
    int idx_head;
    int samples_ready;
    float *tbuf;   // absolute timestamps in seconds
    float *sbuf;   // floating samples
    float cur_time;

    int Kharm;
    int P;          // number of parameters: [f, ReC1, ImC1, ..., ReCK, ImCK, offset]
    float *p;       // parameter vector
    float *delta_p; // update vector

    float *reC;
    float *imC;

    float *JtJ;     // P x P normal matrix
    float *Jtr;     // P vector
    float *work;    // workspace

    // Member workspace to avoid stack overflow on ESP32
    float *JtJ_work;
    float *Jtr_work;
    float *p_work;
    float *A_solver;
    float *B_solver;
    float *Ji_local;

    float lambda;

    void alloc_mem();
    void free_mem();
    void init_params_from_data(float init_f);
    void buildResidualsAndJacobian(float *JtJ_out, float *Jtr_out, float sigma);
    bool solve_normal_equations_and_apply(float *JtJ_in, float *Jtr_in, float *out_delta);

    static void mat_zero(float *m, int rows, int cols);
    static void vec_zero(float *v, int n);
    static float hypot2_vec(float *v, int n);
};

#endif // SOGI_H
