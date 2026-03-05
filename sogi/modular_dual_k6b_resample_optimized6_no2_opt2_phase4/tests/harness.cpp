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

// Mocking ADC data structure
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
#define FLL_GAMMA 200000.0f
#define ADC_OVERSAMPLE_RATE 250000
#define HARMONIC_SMOOTH_ALPHA 0.05f
#define SOGI3_HARMONIC_THRESHOLD 0.1f
#define SOGI3_DAMP_MIN 0.01f

// Global state
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
float harmonic_mag1_smooth = 1e-6f;
float harmonic_mag3_smooth = 1e-6f;
uint32_t interp_ok_count = 0;
uint32_t interp_fail_count = 0;

float v_buf[MAX_SAMPLES_PER_CYCLE];
float i_buf[MAX_SAMPLES_PER_CYCLE];
uint32_t buf_wr = 0;
uint32_t last_cycle_boundary_samples = 0;

SOGI sogi_v(SOGI_K);
SOGI sogi_v3(SOGI_K);
SOGIFLL pll(NOMINAL_FREQ, FLL_GAMMA);

struct ResamplerState {
    uint32_t last_frame_end_ts = 0;
    float    acc_v = 0.0f;
    float    acc_i = 0.0f;
    float    acc_weight = 0.0f;
    uint32_t prev_hw_ts = 0;
    float    prev_hw_frac = 0.0f;
    bool     initialized = false;
} resamp;

struct PhaseTrack {
    float   prev_phase  = 0.0f;
    int32_t winding     = 0;
    bool    initialized = false;
} phase_track;

static inline float adcRawToMillivolts(uint16_t raw) {
    return (float)raw * (3300.0f / 4095.0f);
}

static inline void updateTimingParameters(float frequency) {
    float fc = (frequency < 40.0f) ? 40.0f : (frequency > 90.0f ? 90.0f : frequency);
    double cycle_duration_ticks = (double)cpu_freq_hz / (double)fc;
    double tps_d = cycle_duration_ticks / (double)samples_per_cycle;
    ticks_per_sample_int = (uint32_t)tps_d;
    ticks_per_sample_frac = tps_d - (double)ticks_per_sample_int;
}

static inline int32_t signed_time_diff(uint32_t a, uint32_t b) {
    return (int32_t)(a - b);
}

// Mocking atomic operations for host
#define __atomic_load_n(ptr, memorder) (*ptr)
#define __atomic_store_n(ptr, val, memorder) (*ptr = val)
#define __atomic_fetch_add(ptr, val, memorder) (*ptr += val)
#define __atomic_exchange_n(ptr, val, memorder) ({ uint32_t old = *ptr; *ptr = val; old; })
#define __ATOMIC_ACQUIRE 0
#define __ATOMIC_RELEASE 0
#define __ATOMIC_RELAXED 0
#define __ATOMIC_ACQ_REL 0

void processFrame(const std::vector<adc_digi_output_data_t>& frame_data, uint32_t frame_end_ts) {
    int n = frame_data.size();
    if (n <= 0) return;

    int total_pairs = n / 2;
    uint32_t elapsed = 0;
    if (resamp.initialized) {
        elapsed = (uint32_t)(frame_end_ts - resamp.last_frame_end_ts);
        if (total_pairs == 0) total_pairs = 1;
    } else {
        elapsed = (uint32_t)cycles_per_adc_sample * (uint32_t)total_pairs;
    }
    uint32_t frame_start_ts = resamp.last_frame_end_ts;

    uint32_t spc = samples_per_cycle;

    for (int pair = 0; pair < total_pairs; ++pair) {
        int i = pair * 2;
        uint16_t rv = 0, ri = 0;
        if (frame_data[i].type1.channel == V_CHANNEL) rv = frame_data[i].type1.data;
        else if (frame_data[i].type1.channel == I_CHANNEL) ri = frame_data[i].type1.data;
        if (i + 1 < n) {
            if (frame_data[i+1].type1.channel == V_CHANNEL) rv = frame_data[i+1].type1.data;
            else if (frame_data[i+1].type1.channel == I_CHANNEL) ri = frame_data[i+1].type1.data;
        }

        float fv = adcRawToMillivolts(rv);
        float fi = adcRawToMillivolts(ri);

        uint32_t ts = frame_start_ts + (uint32_t)(((uint64_t)(pair + 1) * (uint64_t)elapsed) / (uint64_t)total_pairs);

        if (!resamp.initialized) {
            resamp.last_frame_end_ts = frame_end_ts;
            resamp.prev_hw_ts = ts;
            resamp.prev_hw_frac = 0.0f;
            resamp.initialized = true;
            next_sample_int = ts;
            next_sample_frac = 0;
            continue;
        }

        while (signed_time_diff(ts, next_sample_int) >= 0) {
            float d_win = (float)signed_time_diff(next_sample_int, resamp.prev_hw_ts) + (float)next_sample_frac - resamp.prev_hw_frac;
            if (d_win < 0.0f) d_win = 0.0f;

            resamp.acc_v += fv * d_win;
            resamp.acc_i += fi * d_win;
            resamp.acc_weight += d_win;

            float v_val, i_val;
            float ts_virtual = (float)(ticks_per_sample_int + ticks_per_sample_frac) * inv_cpu_freq;

            if (resamp.acc_weight > 0.0f) {
                v_val = resamp.acc_v / resamp.acc_weight;
                i_val = resamp.acc_i / resamp.acc_weight;
                interp_ok_count++;
            } else {
                v_val = v_dc_offset;
                i_val = i_dc_offset;
                interp_fail_count++;
            }

            v_buf[buf_wr % spc] = v_val;
            i_buf[buf_wr % spc] = i_val;

            sogi_v.step(v_val - v_dc_offset, pll.omega, ts_virtual);
            sogi_v3.step(v_val - v_dc_offset, 3.0f * pll.omega, ts_virtual);

            float mag1 = sqrt(sogi_v.v_alpha * sogi_v.v_alpha + sogi_v.v_beta * sogi_v.v_beta);
            float mag3 = sqrt(sogi_v3.v_alpha * sogi_v3.v_alpha + sogi_v3.v_beta * sogi_v3.v_beta);
            harmonic_mag1_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag1_smooth + HARMONIC_SMOOTH_ALPHA * mag1;
            harmonic_mag3_smooth = (1.0f - HARMONIC_SMOOTH_ALPHA) * harmonic_mag3_smooth + HARMONIC_SMOOTH_ALPHA * mag3;

            float ratio = harmonic_mag3_smooth / (harmonic_mag1_smooth + 1e-9f);

            pll.update(v_val - v_dc_offset, sogi_v.v_alpha, sogi_v.v_beta, ts_virtual);

            buf_wr++;
            std::cout << std::fixed << std::setprecision(6)
                      << "V_SAMPLE," << v_val << "," << pll.freq << "," << mag1 << "," << ratio << "," << 0.0 << std::endl;

            resamp.acc_v = 0; resamp.acc_i = 0; resamp.acc_weight = 0;
            resamp.prev_hw_ts = next_sample_int;
            resamp.prev_hw_frac = (float)next_sample_frac;

            next_sample_frac += ticks_per_sample_frac;
            next_sample_int  += ticks_per_sample_int + (uint32_t)next_sample_frac;
            next_sample_frac -= (uint32_t)next_sample_frac;
        }

        float d_rem = (float)signed_time_diff(ts, resamp.prev_hw_ts) - resamp.prev_hw_frac;
        if (d_rem < 0.0f) d_rem = 0.0f;
        resamp.acc_v += fv * d_rem;
        resamp.acc_i += fi * d_rem;
        resamp.acc_weight += d_rem;
        resamp.prev_hw_ts = ts;
        resamp.prev_hw_frac = 0.0f;
    }
    resamp.last_frame_end_ts = frame_end_ts;
}

int main() {
    updateTimingParameters(NOMINAL_FREQ);
    pll.setDistortionDamping(1.0f, 1.0f);

    uint32_t current_ts = 1000000;
    resamp.last_frame_end_ts = current_ts;

    // Simulation loop: read raw V/I values from stdin and group them into frames
    float v_in, i_in;
    std::vector<adc_digi_output_data_t> frame;
    buf_wr = 0; // Reset virtual sample counter
    while (std::cin >> v_in >> i_in) {
        adc_digi_output_data_t dv, di;
        dv.type1.channel = V_CHANNEL;
        dv.type1.data = (uint16_t)(v_in * (4095.0f / 3300.0f));
        di.type1.channel = I_CHANNEL;
        di.type1.data = (uint16_t)(i_in * (4095.0f / 3300.0f));

        frame.push_back(dv);
        frame.push_back(di);

        if (frame.size() >= (CONV_FRAME_SIZE / sizeof(uint16_t))) {
            current_ts += cycles_per_adc_sample * (frame.size() / 2);
            processFrame(frame, current_ts);
            frame.clear();

            // Periodic loop tasks (DC offset estimation, etc.)
            uint32_t spc = samples_per_cycle;
            if (buf_wr - last_cycle_boundary_samples >= spc) {
                float v_sum = 0;
                for (uint32_t k = 0; k < spc; k++) v_sum += v_buf[k];
                float v_avg = v_sum / spc;
                if (!dc_bootstrap_done) { v_dc_offset = v_avg; dc_bootstrap_done = true; }
                else { v_dc_offset = 0.98f * v_dc_offset + 0.02f * v_avg; }

                last_cycle_boundary_samples = buf_wr;

                // Variable sampling rate logic
                uint32_t target_spc = (uint32_t)round(6400.0f / pll.freq);
                if (target_spc < 100) target_spc = 100;
                if (target_spc > MAX_SAMPLES_PER_CYCLE) target_spc = MAX_SAMPLES_PER_CYCLE;
                samples_per_cycle = target_spc;
                updateTimingParameters(pll.freq);

                std::cout << "CYCLE_END," << pll.freq << "," << v_dc_offset << "," << samples_per_cycle << std::endl;
            }
        }
    }

    return 0;
}
