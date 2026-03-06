/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK — Operator Implementations
 * =============================================================================
 *
 * This file implements three operators from the signal processing chain:
 *
 *   P_sogi          — Bandpass projection onto the quadrature subspace {α, β}
 *   Φ_pll_base      — Type-1 phase-frequency estimator (FrequencyAdaptivePLL)
 *   Φ_pll_adaptive  — Type-1 estimator with online loop-gain identification (AdaptivePLL)
 *
 * ── P_sogi: Second-Order Generalised Integrator ──────────────────────────────
 *
 *   Continuous-time transfer functions (resonant bandpass pair):
 *
 *     H_α(s) = k·ω·s        / (s² + k·ω·s + ω²)   — bandpass  (in-phase)
 *     H_β(s) = k·ω²         / (s² + k·ω·s + ω²)   — lowpass   (quadrature)
 *
 *   Both share the same denominator (same poles), so they form a complementary
 *   pair: H_α + H_β/ω ≡ 1 (all-pass identity). Together they project x(t)
 *   onto the 2D subspace spanned by {sin(ωt), cos(ωt)}.
 *
 *   Discrete-time realisation: Tustin (bilinear) transform s = (2/Ts)·(z−1)/(z+1).
 *   This maps the jω axis conformally onto the unit circle, preserving stability
 *   and giving exact frequency-domain matching at ω when pre-warped (not done
 *   here — see SHORTCOMING below).
 *
 *   The common denominator after Tustin substitution, normalised:
 *     D(z) = 1 + a₁z⁻¹ + a₂z⁻²
 *   where:
 *     det = 1 / (4 + 2·K·wts + wts²),   wts = ω·Ts
 *     a₁  = 2·(wts² − 4) · det
 *     a₂  = (4 − 2·K·wts + wts²) · det
 *
 *   H_α numerator (bandpass, odd symmetry): b₀ = 2·K·wts·det, b₁ = 0, b₂ = −b₀
 *   H_β numerator (lowpass, even symmetry): b₀ = K·wts²·det,  b₁ = 2·b₀, b₂ = b₀
 *
 *   The two IIR filters share identical denominators but different numerators.
 *   They are implemented as two parallel Direct-Form II sections.
 *
 *   SHORTCOMING: Tustin without frequency pre-warping introduces a small
 *   frequency-axis compression. At ω·Ts << 1 (which holds here: at 50 Hz and
 *   128 samples/cycle, ω·Ts ≈ 0.049 rad) the error is < 0.1% and is
 *   negligible. For coarse virtual grids (low SAMPLES_PER_CYCLE) this should
 *   be reconsidered.
 *
 * ── Φ_pll_base: Frequency-Adaptive PLL ──────────────────────────────────────
 *
 *   The phase discriminator computes:
 *     ε = v_β / |{α, β}|  ≈  sin(Δφ)  ≈  Δφ   (for small Δφ)
 *
 *   This is the normalised cross-product discriminator. Normalisation by
 *   magnitude makes the loop gain independent of signal amplitude.
 *
 *   The loop filter is a PI controller:
 *     u(k) = Kp·ε(k) + Ki·∑ε·Ts
 *
 *   Control output maps to frequency:
 *     f̂ = f_nominal + u
 *     ω̂ = 2π·f̂
 *
 *   This is a Type-1 loop (one integrator in the forward path). It tracks
 *   a constant frequency offset with zero steady-state error (when Ki > 0).
 *
 *   SHORTCOMING: the discriminator uses v_β alone, which is the small-angle
 *   approximation to sin(Δφ). For large phase errors (|Δφ| > ~30°) the
 *   nonlinear sin(·) characteristic means the loop gain drops, slowing
 *   acquisition. A full four-quadrant discriminator using atan2(v_β, v_α)
 *   would give linear response over [−π, π] at the cost of a division.
 *
 * ── Φ_pll_adaptive: Online Loop-Gain Identification ─────────────────────────
 *
 *   Extends Φ_pll_base with a recursive least-squares (RLS)-like estimator
 *   for the effective loop gain. The model is:
 *
 *     Δε(k) ≈ gain_est · u(k−1)
 *
 *   where Δε = ε(k) − ε(k−1) is the change in phase error and u(k−1) is
 *   the previous control action. This is a single-parameter ARX model of
 *   the closed-loop step response.
 *
 *   The update rule is normalised LMS (NLMS):
 *     gain_est += η · u(k−1) · (Δε − gain_est·u(k−1)) / (u(k−1)² + ε)
 *
 *   where η = learn_rate × learn_scale (the distortion gate weight from
 *   the harmonic detector in the main loop).
 *
 *   SHORTCOMING: only the immediately preceding sample is used for the
 *   system identification (prev_idx = hist_idx − 1). The circular history
 *   ring (SOGI_HIST_LEN entries) is over-provisioned — it stores the full
 *   history but the update only reads one step back. A multi-lag correlation
 *   estimator (true RLS over the window) would give better gain tracking
 *   under transient conditions.
 *
 *   BUG: in the anti-windup back-calculation, `integral_state = i_term / ki`.
 *   If ki == 0.0f this is a divide-by-zero. Guard with `if (ki != 0.0f)`.
 *
 * =============================================================================
 */

#include "SOGI.h"
#include <string.h>
#include <stdint.h>

// Avoid collision with Arduino TWO_PI macro.
static constexpr float TWO_PI_F = 6.2831853071795864769f;

// ω_min: lower bound on the resonant frequency of P_sogi.
// Prevents the resonator denominator from collapsing toward DC (ω → 0),
// which would make det → ∞ and destabilise the filter coefficients.
static constexpr float OMEGA_MIN = TWO_PI_F * 5.0f;

// MAG_EPS: regularisation addend in the magnitude computation.
// Prevents division by zero in the phase discriminator when |{α,β}| ≈ 0
// (e.g. during startup before P_sogi has converged).
static constexpr float MAG_EPS = 1e-6f;

// INV_EPS: unused — leftover from an earlier version. Can be removed.
static constexpr float INV_EPS = 1e-12f;

// =============================================================================
//  Magnitude normalisation: fast approximation to 1/√x
//
//  This computes the normalisation factor for the phase discriminator:
//    inv_mag = 1 / |{v_α, v_β}|  =  1 / √(v_α² + v_β²)
//
//  Algorithm: Quake III bit-manipulation initial guess + 2× Newton-Raphson.
//  f(y) = 1/y² − x = 0  →  y_{n+1} = y_n·(1.5 − 0.5·x·y_n²)
//
//  Two NR iterations give ~4 ULP accuracy (acceptable for a phase discriminator
//  whose error tolerance is set by the loop bandwidth, not floating-point precision).
//
//  The memcpy-based bit-cast is the standard C++ UB-safe replacement for the
//  original pointer-punning trick.
//
//  SHORTCOMING: for inputs very close to MAG_EPS the initial guess is poor and
//  two NR iterations may not converge. The guard `if (number <= 0) return ...`
//  catches the degenerate case but the transition region [0, MAG_EPS] is only
//  approximately handled.
// =============================================================================
static inline float fast_rsqrt(float number)
{
    if (number <= 0.0f) return 1.0f / sqrtf(MAG_EPS);

    float x = number;
    uint32_t i;
    memcpy(&i, &x, sizeof(i));
    i = 0x5f3759df - (i >> 1);          // initial approximation via bit-field manipulation
    float y;
    memcpy(&y, &i, sizeof(y));

    const float xhalf = 0.5f * x;
    y = y * (1.5f - xhalf * y * y);    // NR iteration 1
    y = y * (1.5f - xhalf * y * y);    // NR iteration 2

    return y;
}

// =============================================================================
//  P_sogi — Bandpass Projection Operator
// =============================================================================

SOGI::SOGI(float k_) : k(k_) {
    init();
}

// Reset the operator to the zero-input, zero-state condition.
// All internal state (delay registers) are zeroed; coefficients are invalidated.
void SOGI::init() {
    reset();
    coeff_valid = false;
}

// Zero the IIR delay registers and projection outputs.
// Called on startup and can be called to force re-acquisition after a
// large input discontinuity (e.g. signal loss).
void SOGI::reset() {
    v_alpha = 0.0f;    // in-phase projection output (H_α)
    v_beta  = 0.0f;    // quadrature projection output (H_β)

    // Direct-Form II delay registers for H_α section
    wz1_a = wz2_a = 0.0f;
    // Direct-Form II delay registers for H_β section
    wz1_b = wz2_b = 0.0f;
}

// Compute and cache the Tustin-discretised filter coefficients for the
// current (ω, Ts) operating point.
//
// Coefficients are parameterised by the dimensionless product wts = ω·Ts.
// They are shared between the H_α and H_β sections (common denominator).
//
// The bilinear transform derivation (D = common denominator scalar):
//   D   = 4 + 2·K·wts + wts²    (denominator of both transfer functions)
//   det = 1/D
//
//   H_α: B(z) = 2·K·wts·(1 − z⁻²)   →  a_b0 = 2·K·wts·det,  a_b2 = −a_b0
//   H_β: B(z) = K·wts²·(1 + z⁻¹)²   →  b_b0 = K·wts²·det,   b_b1 = 2·b_b0, b_b2 = b_b0
//
//   Shared denominator: A(z) = 1 + a_a1·z⁻¹ + a_a2·z⁻²
//     a_a1 = 2·(wts² − 4)·det
//     a_a2 = (4 − 2·K·wts + wts²)·det
//
// SHORTCOMING: coeff_valid is set to true once and never cleared when ω
// is updated externally (e.g. by Φ_pll changing pll.omega). The step()
// function only recomputes if !coeff_valid, so after the first call the
// filter runs on stale coefficients until the caller explicitly invalidates
// them. Consider checking whether omega has changed by more than a threshold
// and recomputing automatically.
void IRAM_ATTR SOGI::updateCoefficients(float omega, float ts)
{
    if (omega < OMEGA_MIN) omega = OMEGA_MIN;

    float wts  = omega * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;

    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    // H_α numerator (bandpass, anti-symmetric)
    a_b0 =  2.0f * k_wts * det;
    a_b2 = -2.0f * k_wts * det;   // = −a_b0

    // Shared denominator
    a_a1 =  2.0f * (wts2 - 4.0f) * det;
    a_a2 =  (4.0f - 2.0f * k_wts + wts2) * det;

    // H_β numerator (lowpass, symmetric)
    b_b0 = k * wts2 * det;
    b_b1 = 2.0f * b_b0;
    b_b2 = b_b0;

    coeff_valid = true;
}

// Advance P_sogi by one virtual time step.
// Input: a single DC-removed signal sample.
// Output: updated v_alpha (H_α projection) and v_beta (H_β projection).
//
// Each section is implemented as Direct-Form II:
//   w(k) = u(k) − a₁·w(k−1) − a₂·w(k−2)   [common state variable]
//   y_α  = b₀·w(k) + b₂·w(k−2)             [bandpass output]
//   y_β  = b₀·w(k) + b₁·w(k−1) + b₂·w(k−2) [quadrature output]
//
// Note: the two sections compute their state variable w independently
// (wz1_a/wz2_a for H_α, wz1_b/wz2_b for H_β) even though they share
// identical denominator coefficients. This is redundant — they should
// converge to the same w(k) sequence in steady state. A single shared
// state variable would halve the delay-register storage and arithmetic.
// SHORTCOMING: dual state computation is correct but wasteful.
void IRAM_ATTR SOGI::step(float input, float omega, float ts)
{
    if (!coeff_valid) {
        updateCoefficients(omega, ts);
    }

    float u = input;

    // H_α section: bandpass projection
    float in_a = u - a_a1 * wz1_a - a_a2 * wz2_a;
    v_alpha = a_b0 * in_a + a_b2 * wz2_a;
    wz2_a = wz1_a;
    wz1_a = in_a;

    // H_β section: quadrature (lowpass-like) projection
    float in_b = u - a_a1 * wz1_b - a_a2 * wz2_b;
    v_beta = b_b0 * in_b + b_b1 * wz1_b + b_b2 * wz2_b;
    wz2_b = wz1_b;
    wz1_b = in_b;
}

// Batch application of P_sogi over a circular buffer window.
// Used when an entire cycle's worth of samples must be re-projected
// (e.g. after frequency update requires coefficient refresh).
//
// The circular buffer wrap is handled by splitting into at most two
// linear runs: [startIdx, bufLen) and [0, remaining).
// This avoids modular index arithmetic inside the inner loop.
//
// State variables are promoted to local registers for the duration
// of the loop to assist the compiler's register allocator.
// They are written back to the struct after the loop completes.
//
// SHORTCOMING: coefficients are not recomputed inside the loop.
// If omega changes mid-window (e.g. PLL updates ω between window calls),
// the projection is computed with the coefficients from the start of
// the window. For slowly varying ω this is acceptable; for fast
// acquisition transients it introduces a short period of mismatch.
void IRAM_ATTR SOGI::processWindow(
    const float* __restrict buffer,
    int bufLen,
    int startIdx,
    int count,
    float omega,
    float ts,
    float offset)
{
    if (count <= 0) return;

    if (!coeff_valid) {
        updateCoefficients(omega, ts);
    }

    // Register-promote all state to help the compiler avoid repeated
    // struct-member loads inside the inner loop.
    float l_wz1_a = wz1_a,  l_wz2_a = wz2_a;
    float l_wz1_b = wz1_b,  l_wz2_b = wz2_b;
    float l_v_alpha = v_alpha;
    float l_v_beta  = v_beta;

    int endIdx = startIdx + count;

    // Inner step lambda (same equations as SOGI::step, operating on locals).
    auto process = [&](int i)
    {
        float u = buffer[i] - offset;   // subtract DC estimate before projecting

        float in_a = u - a_a1 * l_wz1_a - a_a2 * l_wz2_a;
        l_v_alpha = a_b0 * in_a + a_b2 * l_wz2_a;
        l_wz2_a = l_wz1_a;
        l_wz1_a = in_a;

        float in_b = u - a_a1 * l_wz1_b - a_a2 * l_wz2_b;
        l_v_beta = b_b0 * in_b + b_b1 * l_wz1_b + b_b2 * l_wz2_b;
        l_wz2_b = l_wz1_b;
        l_wz1_b = in_b;
    };

    // Circular buffer traversal: split into one or two linear runs.
    if (endIdx <= bufLen) {
        for (int i = startIdx; i < endIdx; ++i) process(i);
    } else {
        for (int i = startIdx; i < bufLen;    ++i) process(i);
        int remaining = endIdx - bufLen;
        for (int i = 0;        i < remaining; ++i) process(i);
    }

    // Write register-promoted state back to struct.
    wz1_a = l_wz1_a;  wz2_a = l_wz2_a;
    wz1_b = l_wz1_b;  wz2_b = l_wz2_b;
    v_alpha = l_v_alpha;
    v_beta  = l_v_beta;
}

// =============================================================================
//  Φ_pll_base — Type-1 Phase-Frequency Estimator
// =============================================================================

FrequencyAdaptivePLL::FrequencyAdaptivePLL(
    float nominal_freq_,
    float kp_,
    float ki_)
    : nominal_freq(nominal_freq_), kp(kp_), ki(ki_)
{
    init();
}

void FrequencyAdaptivePLL::init()
{
    freq  = nominal_freq;
    omega = TWO_PI_F * freq;
    mag_smooth = 1.0f;
    integral = 0.0f;
    integral_err_c = 0.0f;   // Kahan compensator register
}

// Advance Φ_pll_base by one virtual time step.
//
// Phase discriminator:
//   ε = v_β / |{v_α, v_β}|
//
//   This is the normalised projection of the quadrature component onto the
//   unit circle. Near lock (Δφ ≈ 0), v_α ≈ A and v_β ≈ A·sin(Δφ), so
//   ε ≈ sin(Δφ) ≈ Δφ. The normalisation by magnitude removes amplitude
//   dependence from the loop gain.
//
//   The fast_rsqrt computes 1/|{v_α, v_β}| without a sqrtf call.
//   MAG_EPS is added to the magnitude squared before inversion to avoid
//   division by zero during startup.
//
// Loop filter (PI):
//   u = Kp·ε + Ki·∫ε·dt
//
//   The integral is accumulated with Kahan compensated summation to reduce
//   floating-point truncation error over long runs. This matters when
//   Ki·ε·Ts is many orders of magnitude smaller than the running integral.
//
// Frequency update:
//   f̂ = clamp(f_nominal + u,  f_nominal×0.6,  f_nominal×1.5)
//
//   The clamp prevents the loop from pulling ω to zero or diverging.
//   Range ±50% around nominal handles most grid frequency deviations.
//
// SHORTCOMING: the anti-windup is applied after the frequency clamp, not
// before. If f_new is clamped, the integral continues to accumulate beyond
// the useful range, causing integrator windup. The integral should be
// conditionally frozen when the frequency clamp is active (back-calculation
// anti-windup).
void IRAM_ATTR FrequencyAdaptivePLL::update(
    float v_alpha,
    float v_beta,
    float ts)
{
    // Normalised phase discriminator output
    float mag_sq    = v_alpha * v_alpha + v_beta * v_beta + MAG_EPS;
    float inv_mag   = fast_rsqrt(mag_sq);
    float raw_p_err = v_beta * inv_mag;   // ε ≈ sin(Δφ) ≈ Δφ

    // Kahan-compensated integral accumulation: ∫ε·dt ≈ Σ ε(k)·Ts
    float y = (ki * raw_p_err * ts) - integral_err_c;
    float t = integral + y;
    integral_err_c = (t - integral) - y;
    integral = t;

    // Hard clamp on integral term (basic anti-windup).
    // SHORTCOMING: does not freeze integral when frequency is saturated.
    const float I_MAX = 5.0f;
    if      (integral >  I_MAX) integral =  I_MAX;
    else if (integral < -I_MAX) integral = -I_MAX;

    float control = kp * raw_p_err + integral;

    // Map control output to frequency estimate, clamp to ±50% of nominal.
    float f_new = nominal_freq + control;
    float f_max = nominal_freq * 1.5f;
    float f_min = nominal_freq * 0.6f;
    if      (f_new > f_max) f_new = f_max;
    else if (f_new < f_min) f_new = f_min;

    freq  = f_new;
    omega = TWO_PI_F * freq;
}

// =============================================================================
//  Φ_pll_adaptive — Type-1 Estimator with Online Loop-Gain Identification
// =============================================================================

// SOGI_HIST_LEN must be a power of two so that the modular index wrap
// `(idx + 1) & (SOGI_HIST_LEN - 1)` compiles to a single AND instruction.
static_assert((SOGI_HIST_LEN & (SOGI_HIST_LEN - 1)) == 0,
              "SOGI_HIST_LEN must be a power of two");

AdaptivePLL::AdaptivePLL(
    float nominal_freq_,
    float kp_,
    float ki_,
    float learn_rate_)
    : FrequencyAdaptivePLL(nominal_freq_, kp_, ki_),
      learn_rate(learn_rate_)
{
    init();
}

void AdaptivePLL::init()
{
    FrequencyAdaptivePLL::init();

    integral_state = 0.0f;
    i_term         = 0.0f;
    last_control_action = 0.0f;
    gain_est       = 0.1f;    // initial loop-gain estimate (arbitrary; will adapt)

    hist_idx = 0;
    for (int i = 0; i < SOGI_HIST_LEN; i++) {
        control_hist[i] = 0.0f;
        phase_hist[i]   = 0.0f;
    }

    // p_scale and learn_scale are set externally via setDistortionDamping()
    // from the harmonic detector in the main loop. They are not initialised
    // here — the caller must set them before the first update() call.
    // SHORTCOMING: no safe default initialisation; if setDistortionDamping()
    // is never called, p_scale and learn_scale are uninitialised (undefined
    // behaviour in C++). Add default values in the constructor or here.
}

// Advance Φ_pll_adaptive by one virtual time step.
//
// The adaptive layer adds two mechanisms on top of Φ_pll_base:
//
//  1. Online loop-gain identification (NLMS update):
//       model:       Δε(k) = gain_est · u(k−1)
//       error:       e_g   = Δε(k) − gain_est · u(k−1)
//       NLMS update: gain_est += η · u(k−1) · e_g / (u(k−1)² + ε)
//     where η = learn_rate × learn_scale (gate from harmonic detector).
//
//  2. Phase deadband (PHASE_DEADBAND): proportional and integral terms are
//     frozen when |ε| < PHASE_DEADBAND, reducing jitter when the PLL is
//     well-locked. The integrator state is held (not decayed), which means
//     a steady-state frequency offset accumulated before entering the deadband
//     is preserved — this is correct behaviour for a locked loop.
//
//  External coupling:
//     p_scale:     gain multiplier on Kp, set by setDistortionDamping().
//                  Reduces proportional gain when 3rd-harmonic distortion is detected.
//     learn_scale: gain multiplier on the NLMS learning rate, set by
//                  setDistortionDamping(). Freezes gain adaptation under distortion.
//
// BUG (anti-windup back-calculation):
//   `integral_state = i_term / ki` is executed when i_term is clamped.
//   If ki == 0.0f, this is a divide-by-zero. Guard with:
//     if (fabsf(ki) > 1e-9f) integral_state = i_term / ki;
//     else integral_state = 0.0f;
//
// SHORTCOMING (history ring under-utilised):
//   The ring buffers control_hist[] and phase_hist[] have SOGI_HIST_LEN
//   entries but the NLMS update reads only one step back (prev_idx = hist_idx−1).
//   The ring serves only as a 1-sample delay. Full use of the history would
//   enable multi-lag correlation for better gain tracking, but is not implemented.
void IRAM_ATTR AdaptivePLL::update(
    float v_alpha,
    float v_beta,
    float ts)
{
    // ── Phase discriminator (same as Φ_pll_base) ─────────────────────────────
    float mag_sq    = v_alpha * v_alpha + v_beta * v_beta + MAG_EPS;
    float inv_mag   = fast_rsqrt(mag_sq);
    float raw_p_err = v_beta * inv_mag;   // ε ≈ Δφ

    // ── Online loop-gain identification (NLMS on 1-sample delay) ─────────────
    // Read the immediately preceding history entry.
    uint8_t prev_idx = (hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1);
    float prev_phase   = phase_hist[prev_idx];
    float prev_control = control_hist[prev_idx];

    {
        float dy        = raw_p_err - prev_phase;            // observed Δε
        float denom     = prev_control * prev_control + 1e-6f; // normalisation (u²+ε)
        float err_gain  = dy - gain_est * prev_control;      // prediction error
        float eff_learn = learn_rate * learn_scale;           // gate-weighted step size
        gain_est += eff_learn * (prev_control * err_gain) / denom;  // NLMS update

        // Clamp gain estimate to a physically plausible range.
        if (gain_est < 0.01f) gain_est = 0.01f;
        if (gain_est > 5.0f)  gain_est = 5.0f;
    }

    // ── PI loop filter with phase deadband ────────────────────────────────────
    float p_term = 0.0f;
    if (fabsf(raw_p_err) > PHASE_DEADBAND) {
        // Proportional term: Kp is scaled by p_scale (harmonic distortion gate).
        p_term = (kp * p_scale) * raw_p_err;

        // Kahan-compensated integral accumulation.
        float y = (raw_p_err * ts) - integral_err_c;
        float t = integral_state + y;
        integral_err_c = (t - integral_state) - y;
        integral_state = t;

        i_term = ki * integral_state;
    }
    // Outside deadband: p_term = 0, i_term unchanged (hold last value).

    // ── Integrator anti-windup (back-calculation) ─────────────────────────────
    // BUG: ki == 0 causes divide-by-zero here. See header comment.
    const float I_MAX = 5.0f;
    if (i_term > I_MAX) {
        i_term = I_MAX;
        integral_state = i_term / ki;   // BUG: divide-by-zero if ki == 0
        integral_err_c = 0.0f;
    } else if (i_term < -I_MAX) {
        i_term = -I_MAX;
        integral_state = i_term / ki;   // BUG: divide-by-zero if ki == 0
        integral_err_c = 0.0f;
    }

    // ── Frequency update ──────────────────────────────────────────────────────
    // SHORTCOMING: no frequency clamp here (unlike Φ_pll_base). The adaptive
    // PLL can in principle drive freq outside the ±50% band if gain_est or
    // p_scale are large. Add the same clamp as in FrequencyAdaptivePLL::update.
    float control = p_term + i_term;
    freq  = nominal_freq + control;
    omega = TWO_PI_F * freq;

    // ── History update ────────────────────────────────────────────────────────
    phase_hist[hist_idx]   = raw_p_err;
    control_hist[hist_idx] = control;
    hist_idx = (hist_idx + 1) & (SOGI_HIST_LEN - 1);

    last_control_action = control;
}
