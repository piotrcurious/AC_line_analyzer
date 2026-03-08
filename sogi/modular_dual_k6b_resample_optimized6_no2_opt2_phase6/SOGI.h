#ifndef SOGI_H
#define SOGI_H

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <cmath>
#include <cstring>
#include <cstdint>
#define IRAM_ATTR
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#endif

/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK (UTF) — Phase 6 (Reframed & Optimized)
 * =============================================================================
 *
 * This framework expresses signal processing as a composition of operators:
 *   T_resamp : Hardware-to-Virtual Grid Mapping (Adaptive N in [100, 256])
 *   T_dc     : DC Offset Removal (Recursive Average Estimator)
 *   P_sogi   : Orthogonal Projection onto Quadrature Space {sin, cos}
 *   P_dec    : Recursive Harmonic Decoupling Operator (Fundamental + Odd Harmonics)
 *   Φ_fll    : Frequency Tracking with Online Loop-Gain Identification (NLMS)
 *
 * Numerical Stability Improvements:
 *   - SOGI: Unified state (Direct Form II) with 'double' precision delay registers.
 *   - FLL: Kahan summation for frequency integration to eliminate truncation drift.
 *   - FLL: Cycle-averaging buffer to eliminate 2w ripple in the frequency estimate.
 *   - T_resamp: Timeline tracked in 'double' to minimize virtual grid jitter.
 *
 * Quantization Noise Mitigation:
 *   - Adaptive N selection to maintain a ~6400Hz virtual sampling rate.
 *   - Resampler uses box-filter integration with sub-sample boundary accounting.
 */

// ── P_sogi: SOGI Projection Operator ────────────────────────────────────────
// Projects input onto [v_alpha, v_beta] subspace.
// Reframed: Unified state variables to minimize numerical divergence between components.
class SOGI {
public:
    SOGI(float k = 0.7071f);
    void init();
    void reset();

    // Advance projection by one virtual step
    // input: DC-removed signal
    // omega: target resonance (rad/s)
    // ts: virtual time step (s)
    void IRAM_ATTR step(float input, float omega, float ts);

    float v_alpha; // In-phase component (Bandpass)
    float v_beta;  // Quadrature component (Lowpass/Integrator)
    float k;       // Bandwidth parameter (1/Q)

private:
    // Unified state variables (Direct Form II)
    // Using double to prevent resonant magnitude drift over long runs.
    double w1, w2;

    // Cached coefficients
    float a_b0, a_a1, a_a2;
    float b_b0, b_b1, b_b2;

    bool coeff_valid;
    float last_omega;
    float last_ts;

    void IRAM_ATTR updateCoefficients(float omega, float ts);
};

// ── Φ_fll: Adaptive Frequency-Locked Loop ───────────────────────────────────
// Type-1 Frequency Estimator with Online Gain Identification.
#define FLL_AVG_MAX  256

class AdaptiveFLL {
public:
    AdaptiveFLL(float nominal_freq, float gamma, float learn_rate = 0.1f);
    void init();

    // Updates frequency estimate.
    // Normalized discriminator: e = (v_alpha - u) * v_beta / (|v|^2 + 1)
    // Sign corrected for gamma > 0 and v_beta lagging v_alpha.
    void IRAM_ATTR update(float u_raw_minus_dc, float v_alpha, float v_beta, float ts, uint32_t spc);

    float getFreq() const { return (float)freq_val; }
    float getOmega() const { return (float)(freq_val * 2.0 * M_PI); }
    float getGainEst() const { return gain_est; }

private:
    double freq_val;    // Frequency in Hz (double precision)
    float  gain_est;    // Online identification of loop sensitivity (NLMS)

    double integral;    // FLL state
    double integral_c;  // Kahan compensator

    // Cycle-averaging for 2w ripple rejection
    float    avg_buf[FLL_AVG_MAX];
    double   avg_sum;
    uint32_t avg_idx;
    uint32_t current_spc;

    // NLMS history
    float prev_error;
    float prev_control;

    float nominal_freq;
    float gamma;
    float learn_rate;
};

// ── UnifiedSOGIAnalyzer: Composition {T_dc, P_dec(P_sogi), Φ_fll} ─────────────
class UnifiedSOGIAnalyzer {
public:
    UnifiedSOGIAnalyzer(float nominal_freq = 50.0f, float k = 0.7071f, float gamma = 2500.0f);

    void init();

    // Process one virtual sample from resampler
    void IRAM_ATTR process(float u_raw, float ts, uint32_t spc);

    float getFreq() const { return fll.getFreq(); }
    float getVAlpha() const { return sogi_fund.v_alpha; }
    float getVBeta() const { return sogi_fund.v_beta; }
    float getDC() const { return v_dc; }
    float getGainEst() const { return fll.getGainEst(); }
    float getHarmonicAlpha(int h) const;

private:
    SOGI sogi_fund;
    SOGI sogi_h3, sogi_h5, sogi_h7, sogi_h9, sogi_h11;
    AdaptiveFLL fll;

    double dc_acc;
    float  v_dc;
};

#endif // SOGI_H
