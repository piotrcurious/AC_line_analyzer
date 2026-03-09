/* =============================================================================
 * UNIFIED TRANSFORM FRAMEWORK — AC Signal Tracking System
 * =============================================================================
 *
 * SYSTEM OVERVIEW
 * ───────────────
 * This system is a composition of linear and nonlinear operators that together
 * track the instantaneous frequency, phase, and harmonic content of an AC signal.
 *
 * The full operator chain is:
 *
 *   x(t) ──[T_adc]──► x[n_hw] ──[T_resamp]──► x[n_v] ──[P_sogi]──► {α,β}
 *                                                                        │
 *                                                          [P_sogi3]──► {α3,β3}
 *                                                                        │
 *                                                           ║            │
 *                                                    [Φ_pll]◄────────────┘
 *                                                           ║
 *                                                          {ω, φ}
 *
 *  T_adc    : Sampling operator. Maps continuous signal x(t) → x[n_hw].
 *             Hardware-timed, non-uniform timestamps. Two interleaved channels.
 *             ADC quantisation is an additive stochastic transform Q(x) = x + ε
 *             where ε is bounded by ½ LSB. Modelled here as white noise (ignored).
 *
 *  T_resamp : Decimating projection onto a uniform virtual grid.
 *             Maps irregular hardware timestamps → evenly-spaced virtual samples.
 *             Implemented as a box-filter accumulator (nearest-neighbour window).
 *             The window is Haar-wavelet-like: rectangular, length = decimation ratio.
 *             BUG: window length varies per virtual sample (integer rounding of
 *             elapsed/total_pairs can assign 0 hardware samples to a virtual slot).
 *             A true Haar projection would guarantee equal support per output sample.
 *
 *  P_sogi   : Bandpass projection operator onto the 2D quadrature subspace {sin(ωt), cos(ωt)}.
 *             Implemented as a Second-Order Generalised Integrator (SOGI).
 *             SOGI is a resonant IIR whose steady-state output is the orthogonal
 *             pair (α = in-phase, β = quadrature) of the input at frequency ω.
 *             Mathematically: P_sogi(x, ω) = ⟨x, e^{jωt}⟩ decomposed into ℝ² .
 *             The gain-bandwidth is set by SOGI_K (= 1/Q of the resonator).
 *
 *  P_sogi3  : Same operator as P_sogi but tuned to 3ω.
 *             Extracts the 3rd harmonic component of the input.
 *             Ratio |P_sogi3| / |P_sogi| = THD proxy for 3rd harmonic.
 *
 *  Φ_pll    : Phase-frequency estimator. A nonlinear feedback operator.
 *             Input: quadrature pair {α, β} from P_sogi.
 *             Output: instantaneous frequency estimate ω̂ and phase φ̂.
 *             The error signal is the cross-product (α·cos(φ̂) − β·sin(φ̂)),
 *             which is the first-order linearisation of the phase error sin(Δφ) ≈ Δφ.
 *             This linearisation is valid when the PLL is locked (|Δφ| << π).
 *             Outside lock range the system is nonlinear; acquisition is not
 *             guaranteed for large initial frequency offsets.
 *
 * PHASE LIFTING (unwrap)
 * ──────────────────────
 *             atan2 returns phase on S¹ (the circle, range [−π, π]).
 *             To track continuous phase we lift from S¹ to its covering space ℝ
 *             by accumulating the winding number. This is the standard covering-map
 *             construction: φ_unwrapped = φ_wrapped + 2π·k, k ∈ ℤ.
 *             BUG: the winding counter (phase_track.phase_offset) is never reset
 *             between cycles, so it grows without bound over long runs.
 *             Harmless for float32 up to ~10^7 radians (~30 hours at 50 Hz) but
 *             should be periodically normalised.
 *
 * HARMONIC DAMPING (distortion gate)
 * ────────────────────────────────────
 *             When |P_sogi3| / |P_sogi| > threshold, the PLL input is attenuated.
 *             This is a gain-scheduled nonlinearity: the loop gain is modulated
 *             by the harmonic content of the input.
 *             SHORTCOMING: learn_att is binary (0 or 1). A smooth function of
 *             the ratio (like damp_factor itself) would give better transient
 *             behaviour near the threshold.
 *
 * DC REMOVAL
 * ──────────
 *             v_dc_offset is a single-pole IIR estimate of the signal mean,
 *             computed over the virtual sample buffer once per cycle.
 *             Time constant ≈ 50 cycles @ 50 Hz ≈ 1 second.
 *             BUG: the DC estimate is applied to the SOGI input (x − v_dc) but
 *             is computed from the raw (not DC-removed) buffer. Converges
 *             correctly but has a slow transient on startup.
 *
 * KNOWN SHORTCOMINGS (labelled inline with BUG: or SHORTCOMING:)
 * ────────────────────────────────────────────────────────────────
 *  1. PLL_KI = 0 → purely proportional loop → steady-state frequency error
 *     proportional to the true frequency deviation. Add integrator for
 *     zero-steady-state error.
 *  2. HARMONIC_SMOOTH_ALPHA = 0.99 → very fast EMA (99% weight on new sample).
 *     Comment says "small → slower smoothing" which is correct for this formula,
 *     but 0.99 gives almost no smoothing. Likely intended to be 0.01 or similar.
 *  3. logTask reads harmonic_mag1_smooth / harmonic_mag3_smooth without
 *     synchronisation → data race on multi-core ESP32.
 *  4. CONV_FRAME_SIZE = 16, comment "no idea why" → this is the ESP32-IDF DMA
 *     descriptor granularity constraint (must be multiple of 4 bytes, max
 *     determined by SOC_ADC_DIGI_DATA_BYTES_PER_CONV * n_channels).
 *  5. Dead code block (lines 376-389 in original) duplicates active code above
 *     it. Removed below; keep only one implementation.
 * =============================================================================
 */

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_heap_caps.h"

// ── Hardware pin / ADC channel mapping ──────────────────────────────────────
// T_adc operator configuration: two interleaved channels on ADC1.
#define ADC_PIN_V   36   // ADC1_CH0 — voltage channel
#define ADC_PIN_I   39   // ADC1_CH3 — current channel
#define V_CHANNEL   0
#define I_CHANNEL   3

// ── Operator parameters ─────────────────────────────────────────────────────
// NOMINAL_FREQ: initial ω̂ seed = 2π × 50 rad/s
#define NOMINAL_FREQ       50.0f

// SOGI_K: bandwidth parameter of the bandpass projection operator P_sogi.
//   K = 1/Q. Larger K → wider passband → faster response, less harmonic rejection.
//   K = 1/√2 ≈ 0.7071 gives a Butterworth-like (maximally flat) bandpass profile.
#define SOGI_K             0.7071f

// PLL loop filter gains.
//   KP sets the proportional gain of Φ_pll.
//   SHORTCOMING: KI = 0 means this is a Type-0 loop (no integrator).
//   A Type-1 loop (KI > 0) would track a constant frequency offset with zero
//   steady-state error. Current implementation will have residual freq error
//   proportional to the difference between NOMINAL_FREQ and the true frequency.
#define PLL_KP             2.55f
#define PLL_KI             0.000000f   // BUG: integrator disabled — see SHORTCOMING 1

// Virtual grid resolution: number of uniform samples per AC cycle.
//   This defines the output rate of T_resamp.
#define SAMPLES_PER_CYCLE  128

// ── 3rd-harmonic projection operator (P_sogi3) parameters ───────────────────
// Same bandwidth parameter as P_sogi — could be tuned independently.
#define SOGI3_K                    SOGI_K

// Threshold on the spectral ratio |P_sogi3| / |P_sogi|.
//   Above this value the harmonic distortion gate activates.
#define SOGI3_HARMONIC_THRESHOLD   0.1f

// Minimum gain of the distortion gate (applied to the PLL input).
//   1.0 = gate fully open (no damping), 0.0 = gate fully closed (PLL frozen).
#define SOGI3_DAMP_MIN             0.01f

// EMA (exponential moving average) coefficient for smoothing |P_sogi| and |P_sogi3|.
//   Formula: smooth = (1 − α) × smooth + α × new_value
//   BUG: α = 0.99 gives 99% weight to the newest sample → almost no smoothing.
//   A typical smoothing coefficient would be α = 0.01..0.05 for slow tracking.
//   Verify intent: if fast tracking is desired this is correct; if smoothing
//   is desired, flip to 0.01.
#define HARMONIC_SMOOTH_ALPHA      0.99f

// ── T_adc operator: hardware sampling rate ──────────────────────────────────
// Total hardware samples/s across both interleaved channels.
// Effective per-channel rate = ADC_OVERSAMPLE_RATE / 2 = 125 kHz.
#define ADC_OVERSAMPLE_RATE   250000

// DMA frame size in bytes.
// NOTE (not a bug — just undocumented): the ESP32-IDF requires conv_frame_size
// to be a multiple of SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4 bytes on ESP32).
// Maximum usable value is constrained by max_store_buf_size / n_active_patterns.
// 16 bytes = 4 ADC result structs = 2 V/I pairs per DMA interrupt.
#define CONV_FRAME_SIZE       16

// ── Virtual sample ring buffer ───────────────────────────────────────────────
// Holds the output of T_resamp. Size must cover at least one full cycle.
#define FRAME_BUFFER_SIZE     1024

// ── Serial output decimation ─────────────────────────────────────────────────
#define SERIAL_EVERY_N_CYCLES   2

// ── Global operator instances ────────────────────────────────────────────────
SOGIVisualizer vis;
SOGI           sogi_v(SOGI_K);          // P_sogi:  fundamental projection
SOGI           sogi_v3(SOGI3_K);        // P_sogi3: 3rd-harmonic projection
AdaptivePLL    pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);  // Φ_pll

// ── T_adc output ring: timestamped DMA frames ───────────────────────────────
// Each frame is a raw snapshot from the ADC DMA engine with a hardware
// cycle-counter timestamp (ESP32 CCOUNT register, wraps at 2^32 cycles).
struct TimestampedFrame {
    uint32_t end_timestamp;          // CCOUNT at end of DMA conversion
    uint8_t  raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
};

volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx    = 0;
volatile uint32_t frame_read_idx     = 0;
volatile uint32_t isr_callback_count = 0;
volatile uint32_t frames_dropped     = 0;

adc_continuous_handle_t adc_handle = NULL;

// ── Hardware timing parameters ───────────────────────────────────────────────
// These scale the CCOUNT integer timestamps to physical seconds.
uint32_t cpu_freq_hz           = 0;
float    inv_cpu_freq          = 0.0f;   // = 1 / cpu_freq_hz  [s/cycle]
uint32_t cycles_per_adc_sample = 0;
uint32_t single_cycle_cycles   = 0;     // CCOUNT ticks per one AC mains cycle
uint32_t ticks_per_sample      = 0;     // CCOUNT ticks per one virtual grid step

// ── ADC quantisation inverse: maps 12-bit integer → millivolts ──────────────
// Q^{-1}: undoes the quantisation operator applied by T_adc.
// Range: 0..4095 → 0..3300 mV.
static const float RAW_TO_MV = 3300.0f / 4095.0f;

// ── T_resamp output: virtual sample ring ────────────────────────────────────
// v_buf / i_buf hold the output of T_resamp on the uniform virtual grid.
// Indexed modulo SAMPLES_PER_CYCLE as a circular buffer.
float    v_buf[SAMPLES_PER_CYCLE];
float    i_buf[SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;               // monotonically increasing virtual sample counter
uint32_t next_sample_time = 0;     // CCOUNT target for next virtual grid emission

// T_resamp accumulator state: box-filter window between virtual grid points.
struct ResamplerState {
    uint32_t last_frame_end_ts = 0;  // CCOUNT of previous DMA frame end
    float    acc_v = 0.0f;           // voltage accumulator for current window
    float    acc_i = 0.0f;           // current accumulator for current window
    int      acc_count = 0;          // number of hardware samples in window
    bool     initialized = false;    // false until first valid DMA frame processed
} resamp;

// ── Cycle boundary counter (used to gate the per-cycle processing in loop()) ─
uint32_t last_cycle_boundary_samples = 0;

// ── DC component estimate (subtracted before P_sogi to remove offset) ────────
// Tracked as a slow single-pole IIR over the virtual sample buffer.
// SHORTCOMING: initialised at mid-rail (1650 mV). On a real signal that is
// DC-biased differently the initial transient will be large.
float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;

// ── Phase covering-map state (S¹ → ℝ lifting) ───────────────────────────────
// atan2 returns phase on S¹ ≅ [−π, π). To make the phase observable as a
// monotonically increasing quantity we lift to the universal cover ℝ by
// tracking the integer winding number (phase_track.phase_offset accumulates ±2π).
// BUG: phase_offset grows without bound. Normalise periodically to avoid
// float32 precision loss at large accumulated values (~10^7 rad after ~30 h).
struct PhaseTrack {
    float prev_phase   = 0.0f;
    float phase_offset = 0.0f;   // accumulated 2π winding correction
    bool  initialized  = false;
} phase_track;

// ── Resampler quality counters ───────────────────────────────────────────────
uint32_t interp_ok_count   = 0;   // virtual samples emitted with valid window data
uint32_t interp_fail_count = 0;   // virtual samples emitted with empty window (zero-order hold)

// ── Harmonic ratio smoothing state for the distortion gate ──────────────────
// EMA of |P_sogi| and |P_sogi3|. Initialised to a small non-zero value to
// prevent division by zero on startup.
float harmonic_mag1_smooth = 1e-6f;
float harmonic_mag3_smooth = 1e-6f;

// ── FreeRTOS task handles ────────────────────────────────────────────────────
TaskHandle_t visTaskHandle = NULL;
TaskHandle_t logTaskHandle = NULL;
QueueHandle_t visQueue = NULL;
QueueHandle_t logQueue = NULL;

// Snapshot passed to the visualiser task: one complete virtual cycle snapshot.
struct VisSnapshot {
    float *v_copy;
    float *i_copy;
    int    aligned_start;   // index into circular buffer where the cycle starts
    float  pll_freq;
    float  pll_mag;
    float  vdc, idc;
};

// Snapshot passed to the serial log task.
// BUG: harmonic ratio is read directly from shared globals in logTask without
// a mutex, creating a data race on the dual-core ESP32.
struct SerialMsg {
    float pll_freq;
    float core_us;
    uint32_t isr_callback_count;
    uint32_t frames_dropped;
    uint32_t interp_ok;
    uint32_t interp_total;
    float vdc, idc;
};

// ── Hardware cycle counter (CCOUNT register) ─────────────────────────────────
// This is the integer time axis for all timestamp arithmetic.
// Wraps at 2^32 cycles. All differences use uint32_t subtraction which is
// wrap-safe by construction (modular arithmetic on ℤ/2^32ℤ).
static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a"(ccount));
    return ccount;
}

// Apply Q^{-1}: integer ADC code → millivolts.
static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw) {
    return (float)raw * RAW_TO_MV;
}

// Recompute the virtual grid step (ticks_per_sample) from the current frequency
// estimate. This keeps the uniform virtual grid locked to the tracked frequency.
//   single_cycle_cycles = floor(f_cpu / f_ac) = CCOUNT ticks per AC period
//   ticks_per_sample    = single_cycle_cycles / SAMPLES_PER_CYCLE
// SHORTCOMING: integer division discards the fractional part. The resulting
// grid has a periodic timing error of up to 1 CCOUNT tick per virtual sample.
// For f_cpu=240MHz, f_ac=50Hz this is 240e6/(50*128)=37500 ticks/sample, so
// fractional error is <3e-5 — negligible in practice but accumulates over many
// cycles if not corrected by the PLL.
static inline void IRAM_ATTR updateTimingParameters(float frequency) {
    float fc            = (frequency < 40.0f) ? 40.0f : (frequency > 90.0f ? 90.0f : frequency);
    single_cycle_cycles = (uint32_t)lrintf((float)cpu_freq_hz / fc);
    uint32_t ticks = (single_cycle_cycles / SAMPLES_PER_CYCLE);
    if (ticks == 0) ticks = 1;
    ticks_per_sample = ticks;
}

// Signed 32-bit difference on the modular CCOUNT time axis.
// Correct as long as the true difference is < 2^31 cycles (~8.9 s at 240 MHz).
static inline int32_t IRAM_ATTR signed_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b);
}

// =============================================================================
//  ISR: T_adc output → frame ring
//  Runs in interrupt context. Only copies data and records timestamp.
//  No floating-point, no blocking.
//
//  SHORTCOMING: when the ring is full the oldest frame is silently dropped
//  (frames_dropped counter incremented). This is a hard-RT trade-off but
//  means the resampler may see a gap in the hardware timestamp sequence,
//  producing a spurious large elapsed value and therefore incorrect virtual
//  sample spacing for one DMA frame.
// =============================================================================
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                              const adc_continuous_evt_data_t *edata,
                                              void *user_data) {
    uint32_t ts = get_cycle_count();
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_RELAXED);
    uint32_t next_wr = (wr + 1) % FRAME_BUFFER_SIZE;

    if (next_wr == rd) {
        // Ring full: evict oldest frame to make space (destructive overwrite).
        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
        ++frames_dropped;
    }

    uint32_t sz = edata->size;
    if (sz > CONV_FRAME_SIZE) sz = CONV_FRAME_SIZE;
    memcpy((void *)frame_buffer[wr].raw_data, edata->conv_frame_buffer, sz);

    frame_buffer[wr].end_timestamp = ts;
    frame_buffer[wr].data_size     = (uint16_t)sz;

    __atomic_store_n(&frame_write_idx, next_wr, __ATOMIC_RELEASE);
    ++isr_callback_count;
    return false;
}

// =============================================================================
//  ADC hardware initialisation (T_adc operator setup)
// =============================================================================
bool initADCContinuous() {
    adc_continuous_handle_cfg_t cfg = {
        .max_store_buf_size = 4096,
        .conv_frame_size    = CONV_FRAME_SIZE,
    };
    if (adc_continuous_new_handle(&cfg, &adc_handle) != ESP_OK) return false;

    adc_digi_pattern_config_t pat[2];
    pat[0] = { .atten = ADC_ATTEN_DB_12, .channel = V_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };
    pat[1] = { .atten = ADC_ATTEN_DB_12, .channel = I_CHANNEL, .unit = ADC_UNIT_1, .bit_width = ADC_BITWIDTH_12 };

    adc_continuous_config_t dig = {
        .pattern_num    = 2,
        .adc_pattern    = pat,
        .sample_freq_hz = ADC_OVERSAMPLE_RATE,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    if (adc_continuous_config(adc_handle, &dig) != ESP_OK) return false;

    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback };
    adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL);
    adc_continuous_start(adc_handle);
    return true;
}

// =============================================================================
//  Visualiser task — receives one VisSnapshot per virtual cycle
// =============================================================================
void visTask(void *pv) {
    VisSnapshot snap;
    for (;;) {
        if (xQueueReceive(visQueue, &snap, portMAX_DELAY) == pdTRUE) {
            vis.update(snap.v_copy, snap.i_copy, SAMPLES_PER_CYCLE,
                       snap.aligned_start, SAMPLES_PER_CYCLE,
                       snap.pll_freq, snap.pll_mag,
                       snap.vdc, snap.idc);
            heap_caps_free(snap.v_copy);
            heap_caps_free(snap.i_copy);
        }
    }
}

// =============================================================================
//  Serial log task
//  BUG: harmonic_mag3_smooth and harmonic_mag1_smooth are read here without
//  synchronisation. On the dual-core ESP32 this is a true data race. Fix by
//  including the ratio in SerialMsg (computed under the same lock as the rest
//  of the message) instead of reading globals from another core.
// =============================================================================
void logTask(void *pv) {
    SerialMsg msg;
    for (;;) {
        if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdTRUE) {
            Serial.printf(
                "F:%.4fHz  Core:%.1fus  ISR:%lu  Drop:%lu  Interp:%lu/%lu  Vdc:%.1f Idc:%.1f  H3ratio:%.3f\n",
                msg.pll_freq,
                msg.core_us,
                (unsigned long)msg.isr_callback_count,
                (unsigned long)msg.frames_dropped,
                (unsigned long)msg.interp_ok,
                (unsigned long)msg.interp_total,
                msg.vdc, msg.idc,
                // BUG: unsynchronised cross-core read of shared globals:
                (double)(harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f))
            );
        }
    }
}

// =============================================================================
//  processIncomingDMA()
//  Implements T_resamp + P_sogi + P_sogi3 + harmonic gate + Φ_pll update.
//
//  For each DMA frame:
//    1. Reconstruct per-sample timestamps by linear interpolation within
//       the frame using measured CCOUNT elapsed time (integer arithmetic).
//    2. Accumulate hardware samples into the box-filter window (T_resamp).
//    3. When the virtual grid clock fires (ts >= next_sample_time):
//       a. Emit the window average as one virtual sample.
//       b. Advance P_sogi and P_sogi3 by one virtual time step.
//       c. Compute harmonic ratio, update distortion gate gain.
//       d. Feed gate-weighted {α, β} into Φ_pll.
// =============================================================================
void IRAM_ATTR processIncomingDMA() {
    uint32_t rd = __atomic_load_n(&frame_read_idx, __ATOMIC_ACQUIRE);
    uint32_t wr = __atomic_load_n(&frame_write_idx, __ATOMIC_ACQUIRE);

    if (rd == wr) return;

    while (rd != wr) {
        volatile TimestampedFrame *f_ptr = &frame_buffer[rd];
        uint32_t frame_end_ts = f_ptr->end_timestamp;
        uint16_t sz = f_ptr->data_size;

        const adc_digi_output_data_t *p = (const adc_digi_output_data_t *)f_ptr->raw_data;
        int n = sz / sizeof(adc_digi_output_data_t);
        if (n <= 0) {
            rd = (rd + 1) % FRAME_BUFFER_SIZE;
            continue;
        }

        int total_pairs = n / 2;

        // ----- Timestamp reconstruction -----
        // Map the DMA frame onto the integer time axis using the measured
        // CCOUNT difference between successive frame endpoints.
        // elapsed = number of CCOUNT ticks covering all pairs in this frame.
        // Each pair is assigned ts = frame_start + (pair+1)/total_pairs * elapsed.
        // This is a uniform interpolation (linear operator on ℤ).
        uint32_t elapsed = 0;
        if (resamp.initialized) {
            elapsed = (uint32_t)(frame_end_ts - resamp.last_frame_end_ts);
            // SHORTCOMING: if a frame was dropped (frames_dropped incremented),
            // frame_end_ts - last_frame_end_ts spans 2 or more DMA periods.
            // The interpolation below still distributes the pairs uniformly
            // within this larger window, which is correct if the ADC ran
            // continuously, but the virtual grid may emit multiple samples
            // in one go, draining the accumulator prematurely.
            if (total_pairs == 0) total_pairs = 1;
        } else {
            // Fallback for the very first frame: use nominal ADC timing.
            elapsed = (uint32_t)cycles_per_adc_sample * (uint32_t)total_pairs;
        }
        uint32_t frame_start_ts = resamp.last_frame_end_ts;

        const int max_emit_per_frame = SAMPLES_PER_CYCLE * 2;
        int emitted_in_frame = 0;

        for (int pair = 0; pair < total_pairs; ++pair) {
            // Demultiplex interleaved V/I channels.
            int i = pair * 2;
            uint16_t rv = 0, ri = 0;
            if (p[i].type1.channel == V_CHANNEL) rv = p[i].type1.data;
            else if (p[i].type1.channel == I_CHANNEL) ri = p[i].type1.data;
            if (i+1 < n) {
                if (p[i+1].type1.channel == V_CHANNEL) rv = p[i+1].type1.data;
                else if (p[i+1].type1.channel == I_CHANNEL) ri = p[i+1].type1.data;
            }

            // Apply Q^{-1}: integer code → millivolts
            float fv = adcRawToMillivolts(rv);
            float fi = adcRawToMillivolts(ri);

            // Integer-interpolated timestamp for this pair (wrap-safe uint64 multiply).
            uint32_t ts = frame_start_ts + (uint32_t)(
                ((uint64_t)(pair + 1) * (uint64_t)elapsed) / (uint64_t)total_pairs
            );

            if (!resamp.initialized) {
                // Seed the timestamp state on the first valid pair.
                resamp.last_frame_end_ts = frame_end_ts;
                resamp.initialized = true;
                if (next_sample_time == 0) next_sample_time = frame_end_ts;
                resamp.acc_v += fv;
                resamp.acc_i += fi;
                resamp.acc_count++;
                continue;
            }

            // Accumulate into box-filter window (T_resamp numerator).
            resamp.acc_v += fv;
            resamp.acc_i += fi;
            resamp.acc_count++;

            // ── Virtual grid emission ─────────────────────────────────────────
            // Fire whenever this pair's timestamp reaches or passes the next
            // virtual grid point. Multiple emissions can occur if the ADC
            // ran faster than expected (e.g. after a dropped frame).
            while ( signed_time_diff(ts, next_sample_time) >= 0 ) {

                if (resamp.acc_count > 0) {
                    // ── T_resamp: box-filter average → virtual sample ─────────
                    float inv_count = 1.0f / (float)resamp.acc_count;
                    float v_val = resamp.acc_v * inv_count;
                    float i_val = resamp.acc_i * inv_count;

                    v_buf[buf_wr % SAMPLES_PER_CYCLE] = v_val;
                    i_buf[buf_wr % SAMPLES_PER_CYCLE] = i_val;

                    // Virtual time step in seconds (used as Δt by all operators below).
                    // SHORTCOMING: this is the nominal step, not the measured step.
                    // If ticks_per_sample was just updated by updateTimingParameters,
                    // there is a one-sample discontinuity in the virtual Δt seen by
                    // P_sogi and Φ_pll. For small frequency changes this is negligible.
                    float ts_virtual = (float)ticks_per_sample * inv_cpu_freq;

                    // ── P_sogi: advance the fundamental bandpass projection ───
                    // Input: DC-removed voltage sample.
                    // Output: sogi_v.v_alpha (in-phase), sogi_v.v_beta (quadrature)
                    // at the fundamental frequency ω̂.
                    sogi_v.step(v_val - v_dc_offset, pll.omega, ts_virtual);

                    // ── P_sogi3: advance the 3rd-harmonic projection ──────────
                    // Input: same DC-removed signal.
                    // Output: sogi_v3.v_alpha, sogi_v3.v_beta at 3·ω̂.
                    sogi_v3.step(v_val - v_dc_offset, 3.0f * pll.omega, ts_virtual);

                    // ── Harmonic magnitude: Euclidean norm in projection space ─
                    // |P_sogi(x)|  = √(α² + β²) at ω̂
                    // |P_sogi3(x)| = √(α3² + β3²) at 3ω̂
                    float mag1 = sqrtf(sogi_v.v_alpha  * sogi_v.v_alpha  + sogi_v.v_beta  * sogi_v.v_beta);
                    float mag3 = sqrtf(sogi_v3.v_alpha * sogi_v3.v_alpha + sogi_v3.v_beta * sogi_v3.v_beta);

                    // ── EMA smoothing of projection norms ─────────────────────
                    // BUG: HARMONIC_SMOOTH_ALPHA = 0.99 gives very fast tracking
                    // (99% weight on new sample). If the intent is a slow smoothing
                    // filter, this coefficient should be ~0.01. Verify.
                    harmonic_mag1_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + HARMONIC_SMOOTH_ALPHA * mag1;
                    harmonic_mag3_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + HARMONIC_SMOOTH_ALPHA * mag3;

                    // ── Distortion gate: gain-scheduled attenuation ───────────
                    // ratio = |P_sogi3| / |P_sogi| ≈ 3rd harmonic fraction (THD proxy)
                    float ratio = harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f);

                    // damp_factor: maps ratio ∈ [threshold, threshold×4] → gain ∈ [1, DAMP_MIN]
                    // Linear interpolation (describing-function approximation of
                    // the nonlinear gate).
                    float damp_factor = 1.0f;
                    if (ratio > SOGI3_HARMONIC_THRESHOLD) {
                        float overshoot = (ratio - SOGI3_HARMONIC_THRESHOLD) / (3.0f * SOGI3_HARMONIC_THRESHOLD);
                        if (overshoot < 0.0f) overshoot = 0.0f;
                        if (overshoot > 1.0f) overshoot = 1.0f;
                        damp_factor = 1.0f - overshoot * (1.0f - SOGI3_DAMP_MIN);
                    }

                    // learn_att: binary gate on the PLL integrator (if KI were non-zero).
                    // SHORTCOMING: binary switching causes a step discontinuity in
                    // the loop gain exactly at the threshold. A smooth function of
                    // ratio (e.g. the same damp_factor) would avoid the transient.
                    float learn_att = (ratio > SOGI3_HARMONIC_THRESHOLD) ? 0.0f : 1.0f;

                    pll.setDistortionDamping(damp_factor, learn_att);

                    // ── Φ_pll: feed gate-weighted quadrature pair into the loop ─
                    // The gate multiplies {α, β} by damp_factor before the phase
                    // discriminator, reducing the effective loop gain under distortion.
                    float alpha_in = sogi_v.v_alpha * damp_factor;
                    float beta_in  = sogi_v.v_beta  * damp_factor;

                    pll.update(alpha_in, beta_in, ts_virtual);

                    // Advance virtual write pointer and grid clock.
                    buf_wr++;
                    next_sample_time += ticks_per_sample;
                    interp_ok_count++;

                } else {
                    // Empty window: no hardware samples arrived before this virtual
                    // grid point. Advance the grid clock without emitting a sample.
                    // The previous SOGI/PLL state is held (zero-order hold).
                    // BUG: interp_fail_count is incremented in this branch but
                    // the virtual sample slot in v_buf / i_buf is not updated,
                    // leaving stale data from the previous cycle at that index.
                    // Should write v_dc_offset / i_dc_offset as a neutral fill.
                    next_sample_time += ticks_per_sample;
                    emitted_in_frame++;
                    if (emitted_in_frame > max_emit_per_frame) break;
                    // NOTE: interp_fail_count is not incremented here in the
                    // original code — the fail path falls through to the same
                    // emitted_in_frame increment below, so the counter is shared
                    // but only interp_ok_count is explicitly updated. The ratio
                    // interp_ok / interp_total reflects this.
                    continue;
                }

                // ── Emission safety limiter ───────────────────────────────────
                // Prevents a runaway emission loop if timestamps are inconsistent
                // (e.g. after a dropped frame gives a large elapsed value).
                resamp.acc_v = 0.0f;
                resamp.acc_i = 0.0f;
                resamp.acc_count = 0;

                emitted_in_frame++;
                if (emitted_in_frame > max_emit_per_frame) break;

            } // while virtual grid fires
            if (emitted_in_frame > max_emit_per_frame) break;
        } // for each hardware pair

        resamp.last_frame_end_ts = frame_end_ts;

        rd = (rd + 1) % FRAME_BUFFER_SIZE;
        __atomic_store_n(&frame_read_idx, rd, __ATOMIC_RELEASE);
    }
}

// =============================================================================
//  setup()
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(100);
    cpu_freq_hz  = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    // Effective per-channel hardware rate = total rate / 2 (two interleaved channels).
    cycles_per_adc_sample = cpu_freq_hz / (ADC_OVERSAMPLE_RATE / 2);

    updateTimingParameters(NOMINAL_FREQ);

    // Initialise virtual sample ring to mid-rail (neutral DC value).
    for (int i = 0; i < SAMPLES_PER_CYCLE; ++i) {
        v_buf[i] = v_dc_offset;
        i_buf[i] = i_dc_offset;
    }

    // Initialise EMA smoothers to small non-zero value to prevent divide-by-zero.
    harmonic_mag1_smooth = 1e-6f;
    harmonic_mag3_smooth = 1e-6f;

    vis.begin();
    initADCContinuous();

    visQueue = xQueueCreate(4, sizeof(VisSnapshot));
    logQueue = xQueueCreate(8, sizeof(SerialMsg));
    // Visualiser task on core 0, signal processing in loop() on core 1.
    xTaskCreatePinnedToCore(visTask, "visTask", 8192, NULL, tskIDLE_PRIORITY + 2, &visTaskHandle, 0);
    xTaskCreatePinnedToCore(logTask, "logTask", 4096, NULL, tskIDLE_PRIORITY + 1, &logTaskHandle, 0);

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

// =============================================================================
//  loop()
//  Runs on core 1. Processes one complete virtual cycle per call:
//    1. Drain DMA ring via processIncomingDMA() (T_resamp + operators).
//    2. Gate on SAMPLES_PER_CYCLE new virtual samples having been emitted.
//    3. Estimate DC component (single-pole IIR over the virtual buffer).
//    4. Extrapolate the P_sogi phase estimate to the present moment (first-order
//       Taylor expansion: φ(t_now) ≈ φ(t_last) + ω̂ · Δt).
//    5. Lift the wrapped phase from S¹ to ℝ (winding number accumulation).
//    6. Compute the buffer-aligned start index for cycle-coherent display.
//    7. Dispatch snapshot to visualiser and logger tasks.
// =============================================================================
void IRAM_ATTR loop() {
    static uint32_t serial_ctr  = 0;

    processIncomingDMA();

    // Gate: wait until a full virtual cycle's worth of new samples has been emitted.
    if (buf_wr - last_cycle_boundary_samples < SAMPLES_PER_CYCLE) return;

    uint32_t now = get_cycle_count();
    // lag = time elapsed since the last virtual sample was emitted.
    uint32_t last_virtual_ts = next_sample_time - ticks_per_sample;
    int32_t  lag_ticks = signed_time_diff(now, last_virtual_ts);
    float    lag_sec   = (float)lag_ticks * inv_cpu_freq;

    // ── DC component estimation ───────────────────────────────────────────────
    // Sum the current virtual cycle buffer and update the slow IIR tracker.
    // Time constant = 1/(0.02 × 50 Hz) ≈ 1 second. Sufficient to reject mains
    // AC but responds slowly to amplifier offset drift.
    float v_sum = 0.0f, i_sum = 0.0f;
    for (int k = 0; k < SAMPLES_PER_CYCLE; ++k) {
        v_sum += v_buf[k];
        i_sum += i_buf[k];
    }
    float inv_n = 1.0f / SAMPLES_PER_CYCLE;
    v_dc_offset = 0.98f * v_dc_offset + 0.02f * (v_sum * inv_n);
    i_dc_offset = 0.98f * i_dc_offset + 0.02f * (i_sum * inv_n);

    // Nominal virtual sample period in seconds (used as Δt below).
    float ts_virtual = (float)ticks_per_sample * inv_cpu_freq;
    uint32_t proc_start = get_cycle_count();

    // ── Phase extrapolation to present moment ─────────────────────────────────
    // The last P_sogi update was at time t_last = next_sample_time − ticks_per_sample.
    // The present moment is now = get_cycle_count().
    // First-order Taylor: φ(t_now) = φ(t_last) + ω̂ · (t_now − t_last)
    // This is valid for small lag_sec (< 1 period). For larger lags the
    // nonlinearity of the wrapped phase requires a full SOGI step, not a
    // linear approximation.
    //
    // NOTE on atan2 argument order: atan2(v_alpha, -v_beta) gives the phase
    // of the signal (not the internal SOGI oscillator phase). The sign on
    // v_beta follows from the SOGI state convention: v_alpha leads v_beta by
    // π/2 at steady state, so atan2(α, −β) = atan2(sin φ, cos φ) = φ.
    float phase_corr = pll.omega * lag_sec;
    float phase = atan2f(sogi_v.v_alpha, -sogi_v.v_beta) + phase_corr;
    if (phase < 0.0f) phase += 2.0f * (float)PI;
    else if (phase > 2.0f * (float)PI) phase -= 2.0f * (float)PI;

    // ── S¹ → ℝ covering map (phase unwrap) ───────────────────────────────────
    // Detect a jump of magnitude > π between successive wrapped phases and
    // correct by adding ±2π to the accumulated offset.
    // BUG: phase_offset grows without bound (see header comment).
    if (phase_track.initialized) {
        float delta = phase - phase_track.prev_phase;
        if      (delta < -(float)PI) phase_track.phase_offset += 2.0f * (float)PI;
        else if (delta >  (float)PI) phase_track.phase_offset -= 2.0f * (float)PI;
    } else {
        phase_track.initialized = true;
    }
    phase_track.prev_phase = phase;

    float unwrapped  = phase + phase_track.phase_offset;
    // Normalise back to [0, 2π) for the cycle-alignment computation.
    float phase_norm = fmodf(unwrapped, 2.0f * (float)PI);
    if (phase_norm < 0.0f) phase_norm += 2.0f * (float)PI;

    // ── Cycle-aligned buffer start index ─────────────────────────────────────
    // Compute how many virtual samples back the last zero-crossing was, then
    // back-index into the circular buffer to find the cycle start.
    // samples_per_cycle_f = 1 / (f_pll × Δt_virtual) = virtual samples per period.
    // samples_back        = fractional sample count corresponding to current phase.
    float samples_per_cycle_f = 1.0f / (pll.freq * ts_virtual);
    float samples_back        = (phase_norm / (2.0f * (float)PI)) * samples_per_cycle_f;
    int   curr_head_idx       = (int)(buf_wr % SAMPLES_PER_CYCLE);
    int   aligned_start       = (int)((curr_head_idx
                                       + SAMPLES_PER_CYCLE
                                       - (int)(samples_back + 0.5f))
                                      % SAMPLES_PER_CYCLE);

    last_cycle_boundary_samples = buf_wr;
    // Update virtual grid step for next cycle using the latest frequency estimate.
    updateTimingParameters(pll.freq);

    uint32_t proc_end = get_cycle_count();
    float    core_us  = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;

    // ── Dispatch visualiser snapshot ──────────────────────────────────────────
    // Allocate and copy the circular buffer contents for the visualiser task.
    // The allocation is in the main heap; if it fails the frame is silently skipped.
    VisSnapshot snap;
    snap.v_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    snap.i_copy = (float*)heap_caps_malloc(sizeof(float) * SAMPLES_PER_CYCLE, MALLOC_CAP_DEFAULT);
    if (snap.v_copy && snap.i_copy) {
        memcpy(snap.v_copy, v_buf, sizeof(v_buf));
        memcpy(snap.i_copy, i_buf, sizeof(i_buf));
        snap.aligned_start = aligned_start;
        snap.pll_freq = pll.freq;
        snap.pll_mag  = pll.mag_smooth;
        snap.vdc = v_dc_offset;
        snap.idc = i_dc_offset;

        if (xQueueSend(visQueue, &snap, 0) != pdTRUE) {
            heap_caps_free(snap.v_copy);
            heap_caps_free(snap.i_copy);
        }
    } else {
        if (snap.v_copy) heap_caps_free(snap.v_copy);
        if (snap.i_copy) heap_caps_free(snap.i_copy);
    }

    // ── Dispatch serial log snapshot ─────────────────────────────────────────
    if (++serial_ctr >= SERIAL_EVERY_N_CYCLES) {
        serial_ctr = 0;
        SerialMsg sm;
        sm.pll_freq = pll.freq;
        sm.core_us  = core_us;
        sm.isr_callback_count = isr_callback_count;
        sm.frames_dropped     = frames_dropped;
        sm.interp_ok          = interp_ok_count;
        sm.interp_total       = interp_ok_count + interp_fail_count;
        sm.vdc = v_dc_offset;
        sm.idc = i_dc_offset;
        // Reset per-cycle counters after snapshot.
        interp_ok_count = 0;
        interp_fail_count = 0;
        xQueueSend(logQueue, &sm, 0);
    }

    yield();
}
