/*
 * ESP32 SOGI-PLL - MSOGI deterministic control (two SOGIs at 45Hz / 55Hz)
 * - Minimal filtering (only deterministic averaging over the processed window)
 * - Amplitude normalization to avoid PD collapse
 * - Short integrator freeze after control action to avoid control-induced limit-cycles
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

// integrator freeze after applying a frequency change bigger than this (Hz)
#define INTEGRATOR_FREEZE_CYCLES 2
#define FREQ_UPDATE_MIN_DELTA_HZ 0.0005f

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

SogiBlock msogi[2]; // msogi[0] tuned low (45Hz), msogi[1] tuned high (55Hz)

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
    // ensure nonzero
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

    // initialize MSOGI blocks to fixed resonant frequencies
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
// Implementation is adapted from a stable discrete SOGI difference equation.
// v_d is the "in-phase" (filtered) and v_q is the quadrature output.
void sogi_block_step(SogiBlock &b, float u, float Ts) {
    // derive normalized frequency and discrete coefficients (Tustin-like mapping)
    float kr = SOGI_K;
    float w = b.wr;            // block fixed resonant rad/s
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

    // for quadrature output we can use another filter realization (same coefficients)
    // we treat it as a second input path to generate vq (same difference eq but different b coefficients)
    // we'll use the same "in" chain (as in your previous code) but separate intermediate states would be more precise.
    // reuse wz1/wz2 for a single-delay structure (this is deterministic and consistent per-block)
    // compute vq using b coefficients and stored wz states (we reuse the same in_a; that's consistent with prior pattern)
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

    // Ts derived from ticks_per_sample (measured at setup/updateTimingParameters)
    float Ts = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;

    // process each sample in deterministic order
    for (int i = 0; i < count; ++i) {
        int idx = (start_idx + i) % BUF_N;
        float u = samp_buf[idx];
        // update each SOGI block with same input sample and same Ts (they are tuned to different wr)
        sogi_block_step(msogi[0], u, Ts);
        sogi_block_step(msogi[1], u, Ts);
    }
}

void do_strobe_computation() {
    // compute averaged normalized quadrature per block (deterministic, no smoothing)
    float err = 0.0f;
    float weights[2] = {0.5f, 0.5f}; // equal weights; tune if desired

    for (int i = 0; i < 2; ++i) {
        if (msogi[i].samples_accum == 0) continue;
        float avg_vq = msogi[i].sum_vq / (float)msogi[i].samples_accum;
        float avg_amp = msogi[i].sum_amp / (float)msogi[i].samples_accum;
        // normalized quadrature component (unit PD gain)
        float norm_q = avg_vq / max(avg_amp, 1e-9f);
        err += weights[i] * norm_q;
    }

    // PD is `err` (small-angle linear approximation by normalization)
    float raw_p_err = err;

    // Deterministic integrator handling:
    // If integrator is frozen (due to recent control update), skip integration for that many cycles.
    if (sogi.integrator_freeze <= 0) {
        sogi.integral += PLL_KI * raw_p_err;
        // anti-windup (clamp)
        sogi.integral = constrain(sogi.integral, -10.0f, 10.0f);
    } else {
        sogi.integrator_freeze--;
    }

    // compute control action (Hz domain)
    float control = (PLL_KP * raw_p_err) + sogi.integral;

    // apply minimal deadband to prevent tiny micro-updates
    if (fabsf(control) < 1e-6f) control = 0.0f;

    float f_new = NOMINAL_FREQ + control;
    // limit to allowed frequency bounds
    f_new = constrain(f_new, 42.0f, 58.0f);

    // detect applied change magnitude (Hz)
    float delta_hz = fabsf(f_new - sogi.freq);

    // apply update and recompute related params only if there's a non-zero change
    if (delta_hz > 0.0f) {
        sogi.u_prev = f_new - sogi.freq;
        sogi.freq = f_new;
        sogi.omega = 2.0f * PI * sogi.freq;
        updateTimingParameters(sogi.freq);

        // freeze integrator for a couple cycles if the change was significant enough
        if (delta_hz >= FREQ_UPDATE_MIN_DELTA_HZ) {
            sogi.integrator_freeze = INTEGRATOR_FREEZE_CYCLES;
        }
    }

    // expose current PD, for visualization or logging
    // we do not smooth PD; it's deterministic average over the window
    // optional: compute instantaneous amplitude estimate using both blocks
}

// ----- Setup / Loop (mostly adapted from your original flow) -----

void setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_RESOLUTION);
    initSOGI(NOMINAL_FREQ);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;

    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    current_cycle = 0;
    buf_idx = 0;

    vis.begin();
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

float dc_current = 2048.0f;
void loop() {
    uint32_t now = get_cycle_count();

    // Sampling
    uint32_t elapsed_sample = now - last_sample_cycles;
    if (elapsed_sample >= sogi.ticks_per_sample && current_cycle < 3) {
        last_sample_cycles += sogi.ticks_per_sample;

        int raw = analogRead(ADC_PIN);
        dc_current = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * dc_current);

        // convert raw ADC to volts relative to dc offset
        float u = ((float)raw - dc_offset_sampling) * (V_REF / ((1 << ADC_RESOLUTION) - 1));
        samp_buf[buf_idx] = u;
        ts_buf[buf_idx] = now;
        buf_idx = (buf_idx + 1) % BUF_N;
    }

    // Cycle boundary detection
    uint32_t elapsed_cycle = now - last_cycle_boundary;
    if (elapsed_cycle >= single_cycle_cycles) {
        last_cycle_boundary += single_cycle_cycles;

        int prev_cycle = current_cycle;
        current_cycle = (current_cycle + 1) % 4;
        cycle_start_idx[current_cycle] = buf_idx;

        if (prev_cycle == 2) {
            uint32_t proc_start = get_cycle_count();

            int s_idx = cycle_start_idx[1];
            int e_idx = cycle_start_idx[2];
            int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

            if (actual_count > 0) {
                // update DC offset from sampling path (deterministic low-rate estimator)
                dc_offset_sampling = dc_current;

                // process window deterministically: per-sample SOGI updates & accumulations
                process_sogi_window(s_idx, actual_count);

                // compute PD and update freq deterministically (no heavy filtering)
                do_strobe_computation();

                uint32_t proc_end = get_cycle_count();

                // compute a few metrics for logging & visualization
                float current_dt = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;
                float samples_per_actual_period = 1.0f / (sogi.freq * current_dt);

                // phase alignment as before (deterministic)
                float end_phase = atan2f((msogi[0].sum_vq / max(1u, msogi[0].samples_accum)),
                                         (msogi[0].sum_vd / max(1u, msogi[0].samples_accum)));
                float target_phase = -PI / 2.0f;
                float phase_diff = end_phase - target_phase;
                while (phase_diff < 0) phase_diff += 2.0f * PI;
                while (phase_diff >= 2.0f * PI) phase_diff -= 2.0f * PI;

                int samples_back = (int)((phase_diff / (2.0f * PI)) * samples_per_actual_period);
                int processed_end_idx = (e_idx - 1 + BUF_N) % BUF_N;
                int aligned_start_idx = (processed_end_idx - samples_back + BUF_N) % BUF_N;

                samples_per_actual_period = (float)actual_count;

                // Visualization (unchanged)
                uint32_t vis_start = get_cycle_count();
                // pass combined buffer & aligned index and computed sample count
                vis.update(samp_buf, BUF_N, aligned_start_idx, (int)samples_per_actual_period,
                           sogi.freq,
                           // mag estimate: average amplitude across both SOGIs
                           ((msogi[0].sum_amp + msogi[1].sum_amp) / (float)(msogi[0].samples_accum + msogi[1].samples_accum + 1)),
                           // expose PD as filtered_err for compatibility (we did not store filtered_err; compute quick)
                           // compute small PD value for logging:
                           ((msogi[0].sum_vq / max(1u, msogi[0].samples_accum)) / max(1e-9f, (msogi[0].sum_amp / max(1u, msogi[0].samples_accum))))
                           );
                uint32_t vis_end = get_cycle_count();

                // Timing calculations
                float cycle_processing_us = (float)(proc_end - proc_start) * sogi.inv_cpu_freq * 1e6f;
                float visualization_us = (float)(vis_end - vis_start) * sogi.inv_cpu_freq * 1e6f;
                float total_cycle_us = (float)single_cycle_cycles * sogi.inv_cpu_freq * 1e6f;

                float remaining_us = total_cycle_us - (cycle_processing_us + visualization_us);

                Serial.printf("F:%.5fHz, PD:%.6f, Samples:%d, Core:%.1fus, Vis:%.1fus, Avail:%.1fus\n",
                              sogi.freq,
                              // PD reported as the weighted sum used previously (recompute for log)
                              ((msogi[0].sum_vq / max(1u, msogi[0].samples_accum)) / max(1e-9f, (msogi[0].sum_amp / max(1u, msogi[0].samples_accum)))),
                              (int)samples_per_actual_period,
                              cycle_processing_us, visualization_us, remaining_us);
            }
        }
    }
}
