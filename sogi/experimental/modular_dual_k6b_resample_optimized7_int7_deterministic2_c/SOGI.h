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

#endif // SOGI_H
