// broken cycle boundary

/*
 * ESP32 Modular Adaptive SOGI-PLL with Dual Sampling (Based on k6b)
 * Refactored for modularity and dual channel monitoring.
 */

#include <Arduino.h>
#include <math.h>
#include "SOGI.h"
#include "SOGIvisualizer.h"
#include "analog.h"

#define ADC_PIN_V 36
#define ADC_PIN_I 39
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f

#define PLL_KP 1.25f
//#define PLL_KI 0.000001f
#define PLL_KI 0.000000f

#define SAMPLES_PER_CYCLE 220 // approx 10khz to avoid quantization error
          // max ~220 to prevent overrrun 
          
#define ADC_RESOLUTION 12
#define V_REF 3.3f

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;  // Accumulated 2π wraps
    bool initialized = false;
} phase_track;

// Bresenham scheduling state (32-bit to match ccount)
uint32_t base_ticks_per_sample = 0;
uint32_t ticks_remainder = 0;
uint32_t bresenham_acc = 0;
uint32_t sample_slot_count = 0; // how many scheduled slots we've advanced inside current electrical cycle [0..SAMPLES_PER_CYCLE-1]

float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;
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

void updateTimingParameters_old(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    float cpu_hz = (float)ESP.getCpuFreqMHz() * 1000000.0f;
    inv_cpu_freq = 1.0f / cpu_hz;

    uint32_t old_ticks = ticks_per_sample;
    ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    base_ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);

    uint32_t now = get_cycle_count();
    if (old_ticks > 0) {
        uint32_t elapsed = now - last_sample_cycles;
        float progress = (float)elapsed / (float)old_ticks;
        last_sample_cycles = now - (uint32_t)(progress * (float)ticks_per_sample);
    }
}

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    uint32_t cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_hz;

    // single_cycle_cycles = CPU ticks in one electrical cycle (rounded nearest)
    single_cycle_cycles = (uint32_t) lrintf((float)cpu_hz / f_clamped);

    // Bresenham distribution parameters:
    base_ticks_per_sample = single_cycle_cycles / (uint32_t)SAMPLES_PER_CYCLE; // integer floor
    ticks_remainder = single_cycle_cycles % (uint32_t)SAMPLES_PER_CYCLE;       // remainder to distribute
    bresenham_acc = 0; // restart remainder distribution at boundary

    // Keep a safe initial ticks_per_sample (we use base here; loop will update it after first sample)
    ticks_per_sample = base_ticks_per_sample;

    // keep compatibility with code that expects ticks_per_sample etc.
    // single_cycle_cycles and ticks_per_sample are now self-consistent via Bresenham stepping.
}

void setup() {
    Serial.begin(115200);
    //analogReadResolution(ADC_RESOLUTION);

    updateTimingParameters(NOMINAL_FREQ);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;

    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    current_cycle = 0;
    buf_idx = 0;

    vis.begin();
    analogSystemInit();
    analogPinInit(ADC_PIN_V, ADC_ATTEN_DB_12); // Initialize GPIO 34
    analogPinInit(ADC_PIN_I, ADC_ATTEN_DB_12); // Initialize GPIO 34
    
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);

}

// Add these globals or static variables to your project if not already present
uint32_t max_jitter_ticks = 0;
uint32_t overrun_count = 0;

void loop() {
    // --- Synchronous Sampling (single-slot-per-loop, Bresenham distributed) ---
    uint32_t now = get_cycle_count();
    
    // Check if we have reached or passed the scheduled time for the next sample
    // Using signed subtraction handles wrap-around of the cycle counter correctly
    if ((int32_t)(now - last_sample_cycles) >= 0) {
        
        // --- DEBUG ACCOUNTING ---
        uint32_t latency = now - last_sample_cycles;
        if (current_cycle <3 && latency > max_jitter_ticks) {
            max_jitter_ticks = latency;
        }

        // Overrun check: If we are more than 1.5x the expected period late, 
        // it means we likely missed the window entirely or the loop is too slow.
        if (current_cycle <3 && latency > (ticks_per_sample + (ticks_per_sample >> 1))) {
            overrun_count++;
            // OPTIONAL: Hard-sync last_sample_cycles to 'now' to stop the bleeding
            last_sample_cycles = now; 
        }
        // -------------------------

        // Decide whether to actually sample into buffer (preserve gating)
        // Cycles 0, 1, 2 are sampled; Cycle 3 is for overhead/serial
        bool do_sample = (current_cycle < 3);

        if (do_sample) {
            v_samp_buf[buf_idx] = (float)analogReadMillivolts(ADC_PIN_V);
            i_samp_buf[buf_idx] = (float)analogReadMillivolts(ADC_PIN_I);
            buf_idx = (buf_idx + 1) % BUF_N;
        }

        // Increment scheduled time by the CURRENT ticks_per_sample
        // then calculate the NEXT ticks_per_sample using Bresenham
        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        // Bresenham: compute integer step for the NEXT scheduled slot
        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) {
            bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE;
            step += 1;
        }
        ticks_per_sample = step;

        // --- TIME-BASED cycle boundary detection ---
        // Check if the scheduled sample time has crossed the electrical cycle boundary
        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;

            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            if (prev_cycle == 2) {
                // --- Deferred heavy processing ---
                uint32_t proc_start = get_cycle_count();
                
                // Reset Bresenham and slot counters for the next 3-cycle acquisition block
                sample_slot_count = 0;
                bresenham_acc = 0;
                
                int s_idx = cycle_start_idx[1];
                int e_idx = cycle_start_idx[2];
                int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

                if (actual_count > 0) {
                    // DC Estimation
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

                    // Use the theoretical TS for SOGI/PLL math
                    float ts = (float)ticks_per_sample * inv_cpu_freq;

                    // Process Window with Adaptive PLL
                    sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);
                    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

                    // Phase Alignment for Visualization
                    SOGI phase_sogi(SOGI_K);
                    phase_sogi.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);

                    float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
                    if (phase < 0) phase += 2.0f * PI;

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

                    float samples_per_cycle_f = 1.0f / (pll.freq * ts);
                    float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle_f;
                    int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                    int vis_count = (int)(samples_per_cycle_f + 0.5f);

                    updateTimingParameters(pll.freq);

                    if (vis_count > actual_count) vis_count = actual_count;
                    if (vis_count > 0) {
                        vis.update(v_samp_buf, i_samp_buf, BUF_N, aligned_start_idx, vis_count, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);
                    }

                    uint32_t proc_end = get_cycle_count();
                    float core_us = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;
                    float max_jitter_us = (float)max_jitter_ticks * inv_cpu_freq * 1e6f;

                    //Serial.printf("F:%.4fHz, Core:%.1fus, MaxJit:%.1fus, Overrun:%u, Vdc:%.1f\n",
                    //              pll.freq, core_us, max_jitter_us, overrun_count, v_dc_offset);

                      Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.1f, Idc:%.1f, Core:%.1fus, GainEst:%.4f\n",
                              pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us, pll.gain_est);
            
                    yield();// yield to prevent stuff running into sampling 
                    yield();
                    yield();
                    yield();
                    max_jitter_ticks = 0; 
                }
            } 
        } 
    } 
}
