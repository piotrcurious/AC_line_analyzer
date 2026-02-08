/*
 * ESP32 SOGI-PLL - MSOGI deterministic control (two SOGIs adapt to sogi.freq)
 * - Minimal filtering (only deterministic averaging over the processed window)
 * - Amplitude normalization to avoid PD collapse
 * - Short integrator freeze after control action to avoid control-induced limit-cycles
 * - SOGI resonant frequencies now follow the PLL estimate with a deterministic rate limit
 */

#include <Arduino.h>
#include <math.h>
#include <string.h>

#include "SOGIvisualizer.h"
SOGIVisualizer vis;

#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071067811865476f

#define PLL_KP 1.2f
#define PLL_KI 0.0f        // keep 0 if you prefer pure P; set small if you want I
#define SAMPLES_PER_CYCLE 512 // power of two preferred
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.0002f

// integrator freeze after applying a frequency change bigger than this (cycles)
#define INTEGRATOR_FREEZE_CYCLES 2
#define FREQ_UPDATE_MIN_DELTA_HZ 0.0005f

// --- NEW: maximum SOGI resonant frequency change per processed cycle (Hz)
// This is a deterministic step limit (not a smoothing filter). Tune to balance stability vs adaptation speed.
#define MAX_WR_DELTA_HZ_PER_CYCLE 0.5f

static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

struct SogiBlock {
    // fixed resonant frequency for this block (rad/s)
    float wr;
    // states for discrete implementation (wz1/wz2 correspond to in history)
    float wz1, wz2;
    // outputs
    float vd, vq;
    // temporary accumulator for window averaging
    float sum_vd, sum_vq, sum_amp;
    uint32_t samples_accum;
};

struct SOGI_PLL {
    // resampler/PLL state
    float freq;      // Hz (control variable)
    float omega;     // rad/s (2PI*freq)
    float integral;  // integrator
    uint32_t ticks_per_sample; // CPU ticks between samples
    float inv_cpu_freq; // 1 / cpu_hz
    float u_prev;     // previous frequency control increment (Hz)
    int integrator_freeze; // cycles left to freeze integrator
} sogi;

SogiBlock msogi[2]; // msogi[0] tuned low initially, msogi[1] tuned high initially

float dc_offset_sampling = 2048.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;
uint32_t single_cycle_cycles = 0;

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float samp_buf[BUF_N];
uint32_t ts_buf[BUF_N];
int buf_idx = 0;

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    float cpu_hz = (float)ESP.getCpuFreqMHz() * 1000000.0f;
    sogi.inv_cpu_freq = 1.0f / cpu_hz;

    // ticks per sample so that SAMPLES_PER_CYCLE samples fit into one nominal period
    uint32_t new_ticks = (uint32_t)max(1.0f, (cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE)));
    sogi.ticks_per_sample = new_ticks;
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);
}

void initSOGI(float f_start) {
    memset(&sogi, 0, sizeof(sogi));
    sogi.freq = f_start;
    sogi.omega = 2.0f * PI * f_start;
    sogi.ticks_per_sample = 0;
    updateTimingParameters(f_start);

    // initialize MSOGI blocks: start them spanning the band for initial robustness
    msogi[0].wr = 2.0f * PI * 45.0f;
    msogi[1].wr = 2.0f * PI * 55.0f;
    for (int i=0;i<2;i++) {
        msogi[i].wz1 = msogi[i].wz2 = 0.0f;
        msogi[i].vd = msogi[i].vq = 0.0f;
        msogi[i].sum_vd = msogi[i].sum_vq = msogi[i].sum_amp = 0.0f;
        msogi[i].samples_accum = 0;
    }
}

// Discrete SOGI per-sample update (coefficients recomputed from Ts and block's wr)
void sogi_block_step(SogiBlock &b, float u, float Ts) {
    float kr = SOGI_K;
    float w = b.wr;            // block current resonant rad/s (may be adaptive)
    float wTs = w * Ts;
    float wTs2 = wTs * wTs;
    float k_wTs = kr * wTs;
    float det = 1.0f / (4.0f + 2.0f * k_wTs + wTs2);

    float a_b0 = 2.0f * k_wTs * det;
    float a_b2 = -2.0f * k_wTs * det;
    float a_a1 = 2.0f * (wTs2 - 4.0f) * det;
    float a_a2 = (4.0f - 2.0f * k_wTs + wTs2) * det;

    float b_b0 = kr * wTs2 * det;
    float b_b1 = 2.0f * b_b0;
    float b_b2 = b_b0;

    // update "alpha" SOGI (v_d)
    float in_a = u - (a_a1 * b.wz1) - (a_a2 * b.wz2);
    float vd = (a_b0 * in_a) + (a_b2 * b.wz2);
    // shift states
    b.wz2 = b.wz1;
    b.wz1 = in_a;

    // quadrature path
    float in_b = u - (a_a1 * b.wz1) - (a_a2 * b.wz2);
    float vq = (b_b0 * in_b) + (b_b1 * b.wz1) + (b_b2 * b.wz2);

    b.vd = vd;
    b.vq = vq;

    // accumulate for deterministic window average
    b.sum_vd += vd;
    b.sum_vq += vq;
    float amp = sqrtf(vd*vd + vq*vq) + 1e-9f;
    b.sum_amp += amp;
    b.samples_accum++;
}

void process_sogi_window(int start_idx, int count) {
    if (count <= 0) return;
    // reset accumulators
    for (int i = 0; i < 2; ++i) {
        msogi[i].sum_vd = msogi[i].sum_vq = msogi[i].sum_amp = 0.0f;
        msogi[i].samples_accum = 0;
    }

    // Ts derived from ticks_per_sample
    float Ts = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;

    // process each sample deterministically
    for (int i = 0; i < count; ++i) {
        int idx = (start_idx + i) % BUF_N;
        float u = samp_buf[idx];
        // update each SOGI block; coefficients use the block's current wr
        sogi_block_step(msogi[0], u, Ts);
        sogi_block_step(msogi[1], u, Ts);
    }
}

void do_strobe_computation() {
    // compute averaged normalized quadrature per block (deterministic)
    float err = 0.0f;
    float weights[2] = {0.5f, 0.5f}; // equal weights; tune if desired

    for (int i = 0; i < 2; ++i) {
        if (msogi[i].samples_accum == 0) continue;
        float avg_vq = msogi[i].sum_vq / (float)msogi[i].samples_accum;
        float avg_amp = msogi[i].sum_amp / (float)msogi[i].samples_accum;
        float norm_q = avg_vq / fmaxf(avg_amp, 1e-9f);
        err += weights[i] * norm_q;
    }

    float raw_p_err = err;

    // Integrator with deterministic freeze
    if (sogi.integrator_freeze <= 0) {
        sogi.integral += PLL_KI * raw_p_err;
        sogi.integral = constrain(sogi.integral, -10.0f, 10.0f);
    } else {
        sogi.integrator_freeze--;
    }

    float control = (PLL_KP * raw_p_err) + sogi.integral;
    if (fabsf(control) < 1e-6f) control = 0.0f;

    float f_new = NOMINAL_FREQ + control;
    f_new = constrain(f_new, 42.0f, 58.0f);

    float delta_hz = fabsf(f_new - sogi.freq);

    if (delta_hz > 0.0f) {
        sogi.u_prev = f_new - sogi.freq;
        sogi.freq = f_new;
        sogi.omega = 2.0f * PI * sogi.freq;
        updateTimingParameters(sogi.freq);

        if (delta_hz >= FREQ_UPDATE_MIN_DELTA_HZ) {
            sogi.integrator_freeze = INTEGRATOR_FREEZE_CYCLES;
        }
    }

    // --- NEW: make SOGI resonant frequencies follow the PLL estimate, with deterministic rate limit ---
    // target rad/s for adaptive SOGI blocks:
    float target_w = 2.0f * PI * sogi.freq;
    float max_dw = 2.0f * PI * MAX_WR_DELTA_HZ_PER_CYCLE; // rad/s per processed cycle

    for (int i = 0; i < 2; ++i) {
        float dw = target_w - msogi[i].wr;
        if (fabsf(dw) > max_dw) {
            // step deterministically toward target but do not exceed max_dw
            msogi[i].wr += copysignf(max_dw, dw);
        } else {
            msogi[i].wr = target_w;
        }
    }

    // PD/metrics (no smoothing) can be exposed to vis or logs as needed
}

// rest of setup/loop remains the same as previous version...
// (copy setup() and loop() from previous full example; they are unchanged
// except they will call process_sogi_window() and do_strobe_computation() as before)
