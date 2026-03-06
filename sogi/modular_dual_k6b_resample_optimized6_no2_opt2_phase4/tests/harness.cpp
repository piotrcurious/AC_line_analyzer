#include "../SOGI.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    struct {
        unsigned int data: 12;
        unsigned int channel: 4;
    } type1;
} adc_digi_output_data_t;

#define V_CHANNEL 0
#define I_CHANNEL 3
#define CONV_FRAME_SIZE 16
#define MAX_SAMPLES_PER_CYCLE 256
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f
#define FLL_GAMMA 2500.0f
#define ADC_OVERSAMPLE_RATE 250000
#define TARGET_VIRTUAL_RATE 6400.0f

uint32_t samples_per_cycle = 128;
uint32_t cpu_freq_hz = 240000000;
float inv_cpu_freq = 1.0f / 240000000.0f;
uint32_t cycles_per_adc_sample = 240000000 / (ADC_OVERSAMPLE_RATE / 2);
uint32_t ticks_per_sample_int = 0;
double ticks_per_sample_frac = 0;
uint32_t next_sample_int = 0;
double next_sample_frac = 0;
float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;
bool dc_bootstrap_done = false;

float v_buf[MAX_SAMPLES_PER_CYCLE];
float i_buf[MAX_SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;
uint32_t last_cycle_boundary_samples = 0;

SOGI sogi_v1(SOGI_K);
SOGI sogi_v3(SOGI_K);
SOGI sogi_v5(SOGI_K);
AdaptiveFLL fll(NOMINAL_FREQ, FLL_GAMMA, 0.1f);

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    double   acc_v = 0.0;
    double   acc_i = 0.0;
    double   acc_weight = 0.0;
    uint32_t prev_hw_ts = 0;
    float    prev_hw_frac = 0.0f;
    bool     initialized = false;
} resamp;

static inline float adcRawToMillivolts(uint16_t raw) {
    return (float)raw * (3300.0f / 4095.0f);
}

static inline void updateTimingParameters(float frequency) {
    double tps_d = (double)cpu_freq_hz / (double)frequency / (double)samples_per_cycle;
    ticks_per_sample_int = (uint32_t)tps_d;
    ticks_per_sample_frac = tps_d - (double)ticks_per_sample_int;
}

static inline int32_t signed_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b);
}

#define __atomic_load_n(ptr, memorder) (*ptr)
#define __atomic_store_n(ptr, val, memorder) (*ptr = val)
#define __atomic_fetch_add(ptr, val, memorder) (*ptr += val)
#define __ATOMIC_ACQUIRE 0
#define __ATOMIC_RELEASE 0
#define __ATOMIC_RELAXED 0

void processFrame(const std::vector<adc_digi_output_data_t>& frame_data, uint32_t frame_end_ts) {
    int n = frame_data.size();
    if (n <= 0) return;
    int total_pairs = n / 2;
    uint32_t elapsed = (resamp.initialized) ? (uint32_t)(frame_end_ts - resamp.last_frame_end_ts) : (uint32_t)cycles_per_adc_sample * (uint32_t)total_pairs;
    uint32_t frame_start_ts = resamp.last_frame_end_ts;
    uint32_t spc = samples_per_cycle;

    for (int pair = 0; pair < total_pairs; ++pair) {
        int i = pair * 2;
        uint16_t rv = 0, ri = 0;
        if (frame_data[i].type1.channel == V_CHANNEL) rv = frame_data[i].type1.data; else ri = frame_data[i].type1.data;
        if (i+1 < n) { if (frame_data[i+1].type1.channel == V_CHANNEL) rv = frame_data[i+1].type1.data; else ri = frame_data[i+1].type1.data; }
        float fv = adcRawToMillivolts(rv); float fi = adcRawToMillivolts(ri);
        uint32_t ts = frame_start_ts + (uint32_t)(((uint64_t)(pair + 1) * (uint64_t)elapsed) / (uint64_t)total_pairs);

        if (!resamp.initialized) {
            resamp.last_frame_end_ts = frame_end_ts; resamp.prev_hw_ts = ts; resamp.prev_hw_frac = 0.0f;
            resamp.initialized = true; next_sample_int = ts; next_sample_frac = 0;
            continue;
        }

        while (signed_time_diff(ts, next_sample_int) >= 0) {
            double d_win = (double)signed_time_diff(next_sample_int, resamp.prev_hw_ts) + next_sample_frac - (double)resamp.prev_hw_frac;
            if (d_win < 0.0) d_win = 0.0;
            resamp.acc_v += (double)fv * d_win; resamp.acc_i += (double)fi * d_win; resamp.acc_weight += d_win;

            if (resamp.acc_weight > 0.0) {
                float v_val = (float)(resamp.acc_v / resamp.acc_weight);
                float i_val = (float)(resamp.acc_i / resamp.acc_weight);
                v_buf[buf_wr % spc] = v_val; i_buf[buf_wr % spc] = i_val;

                float u = v_val - v_dc_offset;
                float u1 = u - sogi_v3.v_alpha - sogi_v5.v_alpha;
                float u3 = u - sogi_v1.v_alpha - sogi_v5.v_alpha;
                float u5 = u - sogi_v1.v_alpha - sogi_v3.v_alpha;
                float ts_v = (float)((double)ticks_per_sample_int + ticks_per_sample_frac) * inv_cpu_freq;

                sogi_v1.step(u1, fll.omega, ts_v);
                sogi_v3.step(u3, 3.0f * fll.omega, ts_v);
                sogi_v5.step(u5, 5.0f * fll.omega, ts_v);

                float mag1 = sqrt(sogi_v1.v_alpha*sogi_v1.v_alpha + sogi_v1.v_beta*sogi_v1.v_beta + 1e-3);
                float mag3 = sqrt(sogi_v3.v_alpha*sogi_v3.v_alpha + sogi_v3.v_beta*sogi_v3.v_beta + 1e-3);
                float mag5 = sqrt(sogi_v5.v_alpha*sogi_v5.v_alpha + sogi_v5.v_beta*sogi_v5.v_beta + 1e-3);
                float ratio = (mag3 + mag5) / (mag1 + 1e-3);

                float damp = 1.0f;
                if (ratio > 0.1f) damp = 1.0f - (ratio - 0.1f) * 2.5f;
                if (damp < 0.01f) damp = 0.01f;

                float fll_err = sogi_v1.getFllError(u1);
                float rot_err = (sogi_v1.getRotationRate() - fll.omega) / (fll.omega + 1.0f);

                fll.setDistortionDamping(damp, (ratio > 0.2f ? 0.0f : 1.0f));
                fll.update(fll_err, rot_err, ts_v);

                buf_wr++;
                std::cout << std::fixed << std::setprecision(6) << "V_SAMPLE," << v_val << "," << fll.freq << "," << mag1 << "," << ratio << "," << fll.gain_est << std::endl;
            }

            resamp.acc_v = 0.0; resamp.acc_i = 0.0; resamp.acc_weight = 0.0;
            resamp.prev_hw_ts = next_sample_int; resamp.prev_hw_frac = (float)next_sample_frac;
            next_sample_frac += ticks_per_sample_frac;
            next_sample_int  += ticks_per_sample_int + (uint32_t)next_sample_frac;
            next_sample_frac -= (uint32_t)next_sample_frac;
        }
        double d_rem = (double)signed_time_diff(ts, resamp.prev_hw_ts) - (double)resamp.prev_hw_frac;
        if (d_rem < 0.0) d_rem = 0.0;
        resamp.acc_v += (double)fv * d_rem; resamp.acc_i += (double)fi * d_rem; resamp.acc_weight += d_rem;
        resamp.prev_hw_ts = ts; resamp.prev_hw_frac = 0.0f;
    }
    resamp.last_frame_end_ts = frame_end_ts;
}

int main() {
    updateTimingParameters(NOMINAL_FREQ);
    uint32_t current_ts = 1000000;
    resamp.last_frame_end_ts = current_ts;
    float v_in, i_in;
    std::vector<adc_digi_output_data_t> frame;
    buf_wr = 0;
    while (std::cin >> v_in >> i_in) {
        adc_digi_output_data_t dv, di;
        dv.type1.channel = V_CHANNEL; dv.type1.data = (uint16_t)(v_in * (4095.0f / 3300.0f));
        di.type1.channel = I_CHANNEL; di.type1.data = (uint16_t)(i_in * (4095.0f / 3300.0f));
        frame.push_back(dv); frame.push_back(di);
        if (frame.size() >= (CONV_FRAME_SIZE / sizeof(uint16_t))) {
            current_ts += cycles_per_adc_sample * (frame.size() / 2);
            processFrame(frame, current_ts);
            frame.clear();
            uint32_t spc = samples_per_cycle;
            if (buf_wr - last_cycle_boundary_samples >= spc) {
                float v_sum = 0; for (uint32_t k = 0; k < spc; k++) v_sum += v_buf[k];
                float v_avg = v_sum / spc;
                if (!dc_bootstrap_done) { v_dc_offset = v_avg; dc_bootstrap_done = true; }
                else { v_dc_offset = 0.95f * v_dc_offset + 0.05f * v_avg; }
                last_cycle_boundary_samples = buf_wr;
                uint32_t target_spc = (uint32_t)lrintf(TARGET_VIRTUAL_RATE / fll.freq);
                if (target_spc < 100) target_spc = 100; if (target_spc > MAX_SAMPLES_PER_CYCLE) target_spc = MAX_SAMPLES_PER_CYCLE;
                samples_per_cycle = target_spc;
                updateTimingParameters(fll.freq);
                std::cout << "CYCLE_END," << fll.freq << "," << v_dc_offset << "," << samples_per_cycle << std::endl;
            }
        }
    }
    return 0;
}
