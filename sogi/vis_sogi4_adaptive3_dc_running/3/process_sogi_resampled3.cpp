// SOGIResampler.cpp
// Production-ready resampler + SOGI processing for an embedded system.
//
// Assumptions:
//  - BUF_N, ts_buf[], samp_buf[] are provided externally and populated in circular order.
//  - timestamps are monotonic in buffer order (circularly) and adjacent gaps << 2^31.
//  - samples are not written from an ISR while processing (no concurrency protections required).
//  - 'sogi' is a modifiable struct provided by the caller (stateful filter).
//
// Compile: C++11 or later.

#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>
#include <cassert>

// --- User-visible constants / declarations (adapt as needed) ---
#ifndef BUF_N
#define BUF_N 1024
#endif

extern uint32_t ts_buf[BUF_N];
extern float    samp_buf[BUF_N];

// Example SOGI structure — adapt to your existing definition.
struct SOGIState {
    uint32_t ticks_per_sample; // integer ticks between ideal samples
    float    inv_cpu_freq;     // 1 / CPU frequency (seconds per tick)
    float    omega;            // angular frequency (rad/s)
    float    v_alpha;          // SOGI alpha output
    float    v_beta;           // SOGI beta output
    // filter delay states
    float    wz1_a;
    float    wz2_a;
    float    wz1_b;
    float    wz2_b;
};

#ifndef SOGI_K
#define SOGI_K 1.41421356f
#endif

// Return type for interpolation
struct InterpResult {
    float value;
    float confidence; // 0.0 -> no trust, 1.0 -> full trust
};

// --- Helper utilities ---
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// --- Resampler class ---
class SOGIResampler {
public:
    SOGIResampler(SOGIState &sogi_ref, uint32_t *ts_buf_ptr, float *samp_buf_ptr,
                  size_t buf_size)
    : sogi(sogi_ref), ts(ts_buf_ptr), samp(samp_buf_ptr), N(buf_size),
      cached_ticks_per_sample(0), cached_omega(0.0f), cached_inv_cpu_freq(0.0f),
      scan_offset(0), prev_start_idx(-1)
    {
        assert(ts != nullptr && samp != nullptr && N > 1);
    }

    // Interpolate a value at target_time. start_idx is the buffer index of the earliest sample.
    // count is number of contiguous samples available starting at start_idx (circular wrap allowed).
    // If bracket was not found, confidence==0 and value is nearest sample.
    InterpResult get_interpolated_sample(uint32_t target_time, int start_idx, int count) {
        if (count <= 0) return {0.0f, 0.0f};
        // normalize start_idx
        start_idx = mod_index(start_idx);

        // Reset scan offset on new batch (start_idx changed)
        if (start_idx != prev_start_idx) {
            scan_offset = 0;
            prev_start_idx = start_idx;
        }
        if (scan_offset >= count) scan_offset = 0;

        int idx_prev = (start_idx + scan_offset) % (int)N;
        int idx_next = (idx_prev + 1) % (int)N;
        int scan_limit = count;

        bool bracket_found = false;
        uint32_t t_prev = 0, t_next = 0;

        // Forward scan for [t_prev, t_next] that includes target_time
        while (scan_limit > 0) {
            t_prev = ts[idx_prev];
            t_next = ts[idx_next];

            // unsigned diffs handle wrap (assumes real gaps << 2^31)
            uint32_t dt_meas_u = t_next - t_prev;
            uint32_t dt_targ_u = target_time - t_prev;

            if (dt_targ_u <= dt_meas_u) {
                bracket_found = true;
                break;
            }

            // advance
            idx_prev = idx_next;
            idx_next = (idx_next + 1) % (int)N;
            scan_offset++;
            scan_limit--;
        }

        if (!bracket_found) {
            // return nearest known sample with zero confidence
            float nearest = samp[idx_prev];
            return { nearest, 0.0f };
        }

        // Read sample values
        float y_prev = samp[idx_prev];
        float y_next = samp[idx_next];

        uint32_t dt_meas_u = t_next - t_prev;
        uint32_t dt_targ_u = target_time - t_prev;

        if (dt_meas_u == 0) {
            // duplicate timestamps — cannot interpolate reliably
            return { y_prev, 0.0f };
        }

        float ratio = (float)dt_targ_u / (float)dt_meas_u;
        float val = y_prev + ratio * (y_next - y_prev);

        // Confidence: compare measured gap vs ideal ticks_per_sample
        // If measured gap <= 1.5 * ideal -> full confidence
        // Otherwise decays as (1.5 / gap_ratio), clamped [0,1].
        float gap_ratio = (float)dt_meas_u / (float)sogi.ticks_per_sample;
        float conf = (gap_ratio <= 1.5f) ? 1.0f : 1.5f / gap_ratio;
        conf = clampf(conf, 0.0f, 1.0f);

        return { val, conf };
    }

    // Main processing loop: generate 'count' ideal-grid steps starting at ts[start_idx].
    // Mirrors original semantics; updates sogi in-place.
    void process_sogi_resampled(int start_idx, int count) {
        if (count <= 1) return;

        // Snapshot SOGI params for coefficient caching
        uint32_t cur_ticks = sogi.ticks_per_sample;
        float cur_inv_cpu = sogi.inv_cpu_freq;
        float cur_omega = sogi.omega;

        // Recompute coefficients only when parameters changed
        if (cur_ticks != cached_ticks_per_sample ||
            cur_inv_cpu != cached_inv_cpu_freq ||
            cur_omega != cached_omega) {

            compute_coeffs(cur_ticks, cur_inv_cpu, cur_omega);
            cached_ticks_per_sample = cur_ticks;
            cached_inv_cpu_freq = cur_inv_cpu;
            cached_omega = cur_omega;
        }

        // Start virtual time at first timestamp
        uint32_t virtual_time = ts[mod_index(start_idx)];

        for (int i = 0; i < count; ++i) {
            // 1. Interpolate
            InterpResult meas = get_interpolated_sample(virtual_time, start_idx, count);

            // 2. Weighted input (Kalman-lite)
            float u_weighted = (meas.value * meas.confidence) +
                               (sogi.v_alpha * (1.0f - meas.confidence));

            // 3. SOGI update (DF-II like)
            // Alpha
            float in_a = u_weighted - (a_a1 * sogi.wz1_a) - (a_a2 * sogi.wz2_a);
            sogi.v_alpha = (a_b0 * in_a) + (a_b2 * sogi.wz2_a);
            sogi.wz2_a = sogi.wz1_a;
            sogi.wz1_a = in_a;

            // Beta
            float in_b = u_weighted - (a_a1 * sogi.wz1_b) - (a_a2 * sogi.wz2_b);
            sogi.v_beta = (b_b0 * in_b) + (b_b1 * sogi.wz1_b) + (b_b2 * sogi.wz2_b);
            sogi.wz2_b = sogi.wz1_b;
            sogi.wz1_b = in_b;

            // 4. Advance virtual time by ideal ticks
            virtual_time += sogi.ticks_per_sample;
        }
    }

private:
    // compute and cache coefficients for the fixed time-base SOGI
    void compute_coeffs(uint32_t ticks_per_sample, float inv_cpu_freq, float omega) {
        // ts = ticks_per_sample * inv_cpu_freq (seconds per sample)
        float ts_sec = (float)ticks_per_sample * inv_cpu_freq;
        float os = omega;
        float k = SOGI_K;

        float wts = os * ts_sec;
        float wts2 = wts * wts;
        float k_wts = k * wts;
        float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

        a_b0 = 2.0f * k_wts * det;
        a_b2 = -2.0f * k_wts * det;
        a_a1 = 2.0f * (wts2 - 4.0f) * det;
        a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

        b_b0 = k * wts2 * det;
        b_b1 = 2.0f * b_b0;
        b_b2 = b_b0;
    }

    // helper: ensure index is within [0,N)
    inline int mod_index(int idx) const {
        int m = idx % (int)N;
        return (m < 0) ? (m + (int)N) : m;
    }

private:
    SOGIState &sogi;
    uint32_t *ts;
    float    *samp;
    size_t    N;

    // cached params for coefficients
    uint32_t cached_ticks_per_sample;
    float    cached_inv_cpu_freq;
    float    cached_omega;

    // filter coefficients (a_*, b_*)
    float a_b0 = 0.0f;
    float a_b2 = 0.0f;
    float a_a1 = 0.0f;
    float a_a2 = 0.0f;
    float b_b0 = 0.0f;
    float b_b1 = 0.0f;
    float b_b2 = 0.0f;

    // per-instance scanning state (keeps O(N) scanning across a batch)
    int scan_offset;
    int prev_start_idx;
};
