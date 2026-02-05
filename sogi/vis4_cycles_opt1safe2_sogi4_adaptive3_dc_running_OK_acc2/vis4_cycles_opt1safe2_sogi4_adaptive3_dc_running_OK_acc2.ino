/*
 * ESP32 SOGI-PLL - Frequency Adaptive Orchestration with Timing Accounting
 * Ensures SAMPLES_PER_CYCLE remains constant relative to the period
 * and monitors CPU headroom for the processing cycle.
 */

#include <Arduino.h>
#include <math.h>

#include "SOGIvisualizer.h"

SOGIVisualizer vis;

#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f

#define PLL_KP 1.2f
#define PLL_KI 0.00f
#define SAMPLES_PER_CYCLE 512 // must be power of two 
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.0002f

static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

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
    float wz1_a, wz2_a; // States for alpha filter
    float wz1_b, wz2_b; // States for beta filter
} sogi;

float dc_offset_sampling = 2048.0f;  
float dc_offset_processing = 2048.0f; 
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

    uint32_t old_ticks = sogi.ticks_per_sample;
    sogi.ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);

    uint32_t now = get_cycle_count();
    if (old_ticks > 0) {
        uint32_t elapsed = now - last_sample_cycles;
        float progress = (float)elapsed / (float)old_ticks;
        last_sample_cycles = now - (uint32_t)(progress * (float)sogi.ticks_per_sample);
    }
}

void initSOGI(float f_start) {
    memset(&sogi, 0, sizeof(sogi));
    sogi.freq = f_start;
    sogi.omega = 2.0f * PI * f_start;
    sogi.mag_smooth = 1.0f;
    sogi.ticks_per_sample = 0;
    updateTimingParameters(f_start);
}

void process_sogi_window(int start_idx, int count) {
    if (count <= 0) return;

    float ts = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;
    float os = sogi.omega;
    float k = SOGI_K;

    float wts = os * ts;
    float wts2 = wts * wts;
    float k_wts = k * wts;
    float det = 1.0f / (4.0f + 2.0f * k_wts + wts2);

    float a_b0 = 2.0f * k_wts * det;
    float a_b2 = -2.0f * k_wts * det;
    float a_a1 = 2.0f * (wts2 - 4.0f) * det;
    float a_a2 = (4.0f - 2.0f * k_wts + wts2) * det;

    float b_b0 = k * wts2 * det;
    float b_b1 = 2.0f * b_b0;
    float b_b2 = b_b0;

    for (int i = 0; i < count; ++i) {
        int idx = (start_idx + i) % BUF_N;
        float u = samp_buf[idx];

        float in_a = u - (a_a1 * sogi.wz1_a) - (a_a2 * sogi.wz2_a);
        sogi.v_alpha = (a_b0 * in_a) + (a_b2 * sogi.wz2_a); 
        sogi.wz2_a = sogi.wz1_a;
        sogi.wz1_a = in_a;

        float in_b = u - (a_a1 * sogi.wz1_b) - (a_a2 * sogi.wz2_b);
        sogi.v_beta = (b_b0 * in_b) + (b_b1 * sogi.wz1_b) + (b_b2 * sogi.wz2_b);
        sogi.wz2_b = sogi.wz1_b;
        sogi.wz1_b = in_b;
    }
}

void do_strobe_computation() {
    float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);
    sogi.mag_smooth = (0.2f * mag_inst) + (0.8f * sogi.mag_smooth);
    
    if (sogi.mag_smooth > 0.10f) {
        float raw_p_err = sogi.v_beta / sogi.mag_smooth;
        sogi.filtered_err = (1.0f * raw_p_err) + (0.0f * sogi.filtered_err);

        sogi.integral += PLL_KI * sogi.filtered_err;
        sogi.integral = constrain(sogi.integral, -10.0f, 10.0f);

        float f_new = NOMINAL_FREQ + (PLL_KP * sogi.filtered_err) + sogi.integral;
        sogi.freq = constrain(f_new, 42.0f, 58.0f);
        sogi.omega = 2.0f * PI * sogi.freq;

        updateTimingParameters(sogi.freq);
    }
}

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
}

float dc_current = 2048; 
void loop() {
    uint32_t now = get_cycle_count();

    // --- Sampling (Cycles 0, 1, 2) ---
    uint32_t elapsed_sample = now - last_sample_cycles;
    if (elapsed_sample >= sogi.ticks_per_sample && current_cycle < 3) { 
        last_sample_cycles += sogi.ticks_per_sample;

        int raw = analogRead(ADC_PIN);
        dc_current = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * dc_current);
        
        float u = ((float)raw - dc_offset_sampling) * (V_REF / 4095.0f);
        samp_buf[buf_idx] = u;
        ts_buf[buf_idx] = now;
        buf_idx = (buf_idx + 1) % BUF_N;
    }

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
                dc_offset_sampling = dc_current;
                
                process_sogi_window(s_idx, actual_count);
                do_strobe_computation();
                
                // End of core processing
                uint32_t proc_end = get_cycle_count();

                float current_dt = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;
                float samples_per_actual_period = 1.0f / (sogi.freq * current_dt);

                float end_phase = atan2f(sogi.v_beta, sogi.v_alpha);
                float target_phase = -PI / 2.0f; 
                float phase_diff = end_phase - target_phase;
                
                while (phase_diff < 0) phase_diff += 2.0f * PI;
                while (phase_diff >= 2.0f * PI) phase_diff -= 2.0f * PI;

                int samples_back = (int)((phase_diff / (2.0f * PI)) * samples_per_actual_period);
                int processed_end_idx = (e_idx - 1 + BUF_N) % BUF_N;
                int aligned_start_idx = (processed_end_idx - samples_back + BUF_N) % BUF_N;

                samples_per_actual_period = (float)actual_count;
                
                // Visualization timing
                uint32_t vis_start = get_cycle_count();
                vis.update(samp_buf, BUF_N, aligned_start_idx, (int)samples_per_actual_period, 
                           sogi.freq, sogi.mag_smooth, sogi.filtered_err);
                uint32_t vis_end = get_cycle_count();
                
                // Timing calculations
                float cycle_processing_us = (float)(proc_end - proc_start) * sogi.inv_cpu_freq * 1e6f;
                float visualization_us = (float)(vis_end - vis_start) * sogi.inv_cpu_freq * 1e6f;
                float total_cycle_us = (float)single_cycle_cycles * sogi.inv_cpu_freq * 1e6f;
                
                // Available headroom: Total Cycle - (Core Processing + Visualization)
                float remaining_us = total_cycle_us - (cycle_processing_us + visualization_us);

                Serial.printf("F:%.5fHz, Mag:%.3f, Err:%.3f, ActSamples:%d, Core:%.1fus, Vis:%.1fus, Avail:%.1fus\n", 
                              sogi.freq, sogi.mag_smooth, sogi.filtered_err, (int)samples_per_actual_period,
                              cycle_processing_us, visualization_us, remaining_us);           
            }
        }
    }
}
