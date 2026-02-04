#include "mock_arduino.h"
#include <vector>
#include <iostream>
#include <iomanip>

MockESP ESP;
MockSerial Serial;

// --- Definitions from the project ---
#define SAMPLES_PER_CYCLE 512
#define NOMINAL_FREQ 50.0f

struct SOGI_PLL {
    float v_alpha, v_beta;
    float theta;
    float freq;
    float omega;
    float integral;
    uint32_t ticks_per_sample;
    float u_prev;
    float filtered_err;
    float mag_smooth;
    float inv_cpu_freq;
    float wz1_a, wz2_a;
    float wz1_b, wz2_b;
};

struct InterpResult {
    float value;
    float confidence;
};

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

class SOGIResampler {
public:
    SOGIResampler(struct SOGI_PLL &sogi_ref, uint32_t *ts_buf_ptr, float *samp_buf_ptr,
                  size_t buf_size)
    : sogi(sogi_ref), ts(ts_buf_ptr), samp(samp_buf_ptr), N(buf_size),
      cached_ticks_per_sample(0), cached_omega(0.0f), cached_inv_cpu_freq(0.0f),
      scan_offset(0), prev_start_idx(-1)
    {}

    InterpResult get_interpolated_sample(uint32_t target_time, int start_idx, int count) {
        if (count <= 0) return {0.0f, 0.0f};
        start_idx = mod_index(start_idx);

        if (start_idx != prev_start_idx) {
            scan_offset = 0;
            prev_start_idx = start_idx;
        }
        if (scan_offset >= (int)count) scan_offset = 0;

        int idx_prev = (start_idx + scan_offset) % (int)N;
        int idx_next = (idx_prev + 1) % (int)N;
        int scan_limit = count;

        bool bracket_found = false;
        uint32_t t_prev = 0, t_next = 0;

        while (scan_limit > 0) {
            t_prev = ts[idx_prev];
            t_next = ts[idx_next];
            uint32_t dt_meas_u = t_next - t_prev;
            uint32_t dt_targ_u = target_time - t_prev;
            if (dt_targ_u <= dt_meas_u) {
                bracket_found = true;
                break;
            }
            idx_prev = idx_next;
            idx_next = (idx_next + 1) % (int)N;
            scan_offset++;
            scan_limit--;
        }

        if (!bracket_found) {
            return { samp[idx_prev], 0.0f };
        }

        float y_prev = samp[idx_prev];
        float y_next = samp[idx_next];
        uint32_t dt_meas_u = t_next - t_prev;
        uint32_t dt_targ_u = target_time - t_prev;

        if (dt_meas_u == 0) return { y_prev, 0.0f };

        float ratio = (float)dt_targ_u / (float)dt_meas_u;
        float val = y_prev + ratio * (y_next - y_prev);

        float gap_ratio = (float)dt_meas_u / (float)sogi.ticks_per_sample;
        float conf = (gap_ratio <= 1.5f) ? 1.0f : 1.5f / gap_ratio;
        if (conf < 0.0f) conf = 0.0f;
        if (conf > 1.0f) conf = 1.0f;

        return { val, conf };
    }

    void process_sogi_resampled(int start_idx, int count) {
        if (count <= 1) return;
        uint32_t cur_ticks = sogi.ticks_per_sample;
        float cur_inv_cpu = sogi.inv_cpu_freq;
        float cur_omega = sogi.omega;

        if (cur_ticks != cached_ticks_per_sample ||
            cur_inv_cpu != cached_inv_cpu_freq ||
            cur_omega != cached_omega) {
            compute_coeffs(cur_ticks, cur_inv_cpu, cur_omega);
            cached_ticks_per_sample = cur_ticks;
            cached_inv_cpu_freq = cur_inv_cpu;
            cached_omega = cur_omega;
        }

        uint32_t virtual_time = ts[mod_index(start_idx)];
        for (int i = 0; i < count; ++i) {
            InterpResult meas = get_interpolated_sample(virtual_time, start_idx, count);
            float u_weighted = (meas.value * meas.confidence) +
                               (sogi.v_alpha * (1.0f - meas.confidence));

            float in_a = u_weighted - (a_a1 * sogi.wz1_a) - (a_a2 * sogi.wz2_a);
            sogi.v_alpha = (a_b0 * in_a) + (a_b2 * sogi.wz2_a);
            sogi.wz2_a = sogi.wz1_a;
            sogi.wz1_a = in_a;

            float in_b = u_weighted - (a_a1 * sogi.wz1_b) - (a_a2 * sogi.wz2_b);
            sogi.v_beta = (b_b0 * in_b) + (b_b1 * sogi.wz1_b) + (b_b2 * sogi.wz2_b);
            sogi.wz2_b = sogi.wz1_b;
            sogi.wz1_b = in_b;

            virtual_time += sogi.ticks_per_sample;
        }
    }

private:
    void compute_coeffs(uint32_t ticks_per_sample, float inv_cpu_freq, float omega) {
        float ts_sec = (float)ticks_per_sample * inv_cpu_freq;
        float os = omega;
        float k = 0.7071f;

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

    inline int mod_index(int idx) const {
        int m = idx % (int)N;
        return (m < 0) ? (m + (int)N) : m;
    }

    struct SOGI_PLL &sogi;
    uint32_t *ts;
    float    *samp;
    size_t    N;
    uint32_t cached_ticks_per_sample;
    float    cached_inv_cpu_freq;
    float    cached_omega;
    float a_b0, a_b2, a_a1, a_a2, b_b0, b_b1, b_b2;
    int scan_offset;
    int prev_start_idx;
};

// --- Test Case ---
void test_interpolation() {
    std::cout << "--- Test: Interpolation ---" << std::endl;
    SOGI_PLL sogi = {0};
    sogi.ticks_per_sample = 4800;
    sogi.inv_cpu_freq = 1.0f/240000000.0f;
    sogi.omega = 2.0f*PI*50.0f;

    const int N = 100;
    uint32_t ts[N];
    float samp[N];
    for (int i=0; i<N; i++) {
        ts[i] = 1000 + i * 4800; // perfect spacing
        samp[i] = sinf(2.0f * PI * 50.0f * (float)ts[i] * sogi.inv_cpu_freq);
    }

    SOGIResampler resampler(sogi, ts, samp, N);

    // Exact hit
    InterpResult res = resampler.get_interpolated_sample(1000 + 10 * 4800, 0, N);
    std::cout << "Exact hit: val=" << std::fixed << std::setprecision(6) << res.value
              << ", conf=" << res.confidence << " (expected " << samp[10] << ", 1.0)" << std::endl;

    // Midpoint
    res = resampler.get_interpolated_sample(1000 + 10 * 4800 + 2400, 0, N);
    float expected = (samp[10] + samp[11]) * 0.5f;
    std::cout << "Midpoint: val=" << res.value << ", conf=" << res.confidence
              << " (expected " << expected << ", 1.0)" << std::endl;

    // Gap (jitter)
    uint32_t original_ts20 = ts[20];
    ts[20] = 1000 + 19 * 4800 + 10000; // Artificial delay
    res = resampler.get_interpolated_sample(1000 + 19 * 4800 + 4800, 0, N);
    std::cout << "Gap/Jitter: val=" << res.value << ", conf=" << res.confidence
              << " (expected confidence < 1.0 due to gap ratio 10000/4800 > 1.5)" << std::endl;
    ts[20] = original_ts20; // restore
}

void test_sogi_processing() {
    std::cout << "\n--- Test: SOGI Resampled Processing ---" << std::endl;
    SOGI_PLL sogi = {0};
    sogi.ticks_per_sample = 9375; // 240MHz / (512 * 50Hz)
    sogi.inv_cpu_freq = 1.0f/240000000.0f;
    sogi.omega = 2.0f*PI*50.0f;
    sogi.mag_smooth = 1.0f;

    const int N = 1024;
    uint32_t ts[N];
    float samp[N];
    for (int i=0; i<N; i++) {
        ts[i] = i * 9375;
        // 50Hz sine wave, 1.0V peak
        samp[i] = 1.0f * sinf(2.0f * PI * 50.0f * (float)ts[i] * sogi.inv_cpu_freq);
    }

    SOGIResampler resampler(sogi, ts, samp, N);

    // Process 2 cycles
    for (int j=0; j<2; j++) {
        resampler.process_sogi_resampled(j*512, 512);
        std::cout << "After Cycle " << j+1 << ": v_alpha=" << sogi.v_alpha
                  << ", v_beta=" << sogi.v_beta
                  << ", mag=" << sqrtf(sogi.v_alpha*sogi.v_alpha + sogi.v_beta*sogi.v_beta) << std::endl;
    }
}

int main() {
    test_interpolation();
    test_sogi_processing();
    return 0;
}
