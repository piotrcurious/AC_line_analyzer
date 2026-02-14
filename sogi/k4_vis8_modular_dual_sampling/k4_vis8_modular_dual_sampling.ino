/*
 * ESP32 Modular SOGI-PLL with Dual Sampling
 * Refactored for modularity and multiple SOGI instances.
 */

#include <Arduino.h>
#include <math.h>
#include "SOGI.h"
#include "SOGIvisualizer.h"

#define ADC_PIN_V 36
#define ADC_PIN_I 39
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f

#define PLL_KP 1.3f
#define PLL_KI 0.001f
#define SAMPLES_PER_CYCLE 256 // target samples per cycle
#define ADC_RESOLUTION 12
#define V_REF 3.3f

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
SOGI sogi_5th(SOGI_K); // Example of a 5th harmonic SOGI
FrequencyAdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;  // Accumulated 2π wraps
    bool initialized = false;
} phase_track;

float v_dc_offset = 2048.0f;
float i_dc_offset = 2048.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;
uint32_t single_cycle_cycles = 0;
uint32_t ticks_per_sample = 0;
float inv_cpu_freq = 0;

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float v_samp_buf[BUF_N];
float i_samp_buf[BUF_N];
int buf_idx = 0;

static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    float cpu_hz = (float)ESP.getCpuFreqMHz() * 1000000.0f;
    inv_cpu_freq = 1.0f / cpu_hz;

    uint32_t old_ticks = ticks_per_sample;
    ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);

    uint32_t now = get_cycle_count();
    if (old_ticks > 0) {
        uint32_t elapsed = now - last_sample_cycles;
        float progress = (float)elapsed / (float)old_ticks;
        last_sample_cycles = now - (uint32_t)(progress * (float)ticks_per_sample);
    }
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_RESOLUTION);

    updateTimingParameters(NOMINAL_FREQ);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;

    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    current_cycle = 0;
    buf_idx = 0;

    vis.begin();

    // Ensure SPI pins have enough drive
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

void loop() {
    uint32_t now = get_cycle_count();

    // --- Synchronous Sampling (Voltage and Current) ---
    uint32_t elapsed_sample = now - last_sample_cycles;
    if (elapsed_sample >= ticks_per_sample && current_cycle < 3) {
        last_sample_cycles += ticks_per_sample;

        int raw_v = analogRead(ADC_PIN_V);
        int raw_i = analogRead(ADC_PIN_I);

        v_samp_buf[buf_idx] = ((float)raw_v) * (V_REF / 4095.0f);
        i_samp_buf[buf_idx] = ((float)raw_i) * (V_REF / 4095.0f);

        buf_idx = (buf_idx + 1) % BUF_N;
    }

    now = get_cycle_count();
    // --- Cycle Boundary Detection ---
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
                // --- DC Offset Estimation ---
                float v_sum = 0, i_sum = 0;
                for (int i = 0; i < actual_count; ++i) {
                    int idx = (s_idx + i) % BUF_N;
                    v_sum += v_samp_buf[idx];
                    i_sum += i_samp_buf[idx];
                }
                float v_win_dc = v_sum / (float)actual_count;
                float i_win_dc = i_sum / (float)actual_count;
                v_dc_offset = (0.2f * v_win_dc) + (0.8f * v_dc_offset);
                i_dc_offset = (0.2f * i_win_dc) + (0.8f * i_dc_offset);

                float ts = (float)ticks_per_sample * inv_cpu_freq;

                // --- Modular SOGI Processing ---
                // 1. Process Fundamental for PLL
                sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);

                // 2. Update PLL based on fundamental voltage
                pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

                // 3. Optional: Process harmonics or other signals
                // f.e. 5th harmonic rejection/monitoring
                sogi_5th.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, 5.0f * pll.omega, ts, v_dc_offset);

                // --- Phase Calculation and Alignment for Visualization ---
                // We use a temporary SOGI to calculate phase from the start of the window without affecting main states
                SOGI phase_sogi(SOGI_K);
                phase_sogi.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);

                float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
                if (phase < 0) phase += 2.0f * PI;

                // Phase Unwrapping
                if (phase_track.initialized) {
                    float phase_delta = phase - phase_track.prev_phase;
                    if (phase_delta < -PI) phase_track.phase_offset += 2.0f * PI;
                    else if (phase_delta > PI) phase_track.phase_offset -= 2.0f * PI;
                } else {
                    phase_track.initialized = true;
                }
                phase_track.prev_phase = phase;
                float unwrapped_phase = phase + phase_track.phase_offset;
                float phase_for_alignment = fmodf(unwrapped_phase, 2.0f * PI);
                if (phase_for_alignment < 0) phase_for_alignment += 2.0f * PI;

                float samples_per_cycle = 1.0f / (pll.freq * ts);
                float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle;
                int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                int vis_count = (int)(samples_per_cycle + 0.5f);

                updateTimingParameters(pll.freq);

                uint32_t proc_end = get_cycle_count();
                uint32_t vis_start = get_cycle_count();

                // Update Visualizer with both Voltage and Current buffers
                vis.update(v_samp_buf, i_samp_buf, BUF_N, aligned_start_idx, vis_count, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);

                uint32_t vis_end = get_cycle_count();

                float core_us = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;
                float vis_us = (float)(vis_end - vis_start) * inv_cpu_freq * 1e6f;
                float total_us = (float)single_cycle_cycles * inv_cpu_freq * 1e6f;

                Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.3f, Idc:%.3f, Core:%.1fus, Vis:%.1fus, Avail:%.1fus\n",
                              pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us, vis_us, total_us - (core_us + vis_us));
            }
        }
    }
}
