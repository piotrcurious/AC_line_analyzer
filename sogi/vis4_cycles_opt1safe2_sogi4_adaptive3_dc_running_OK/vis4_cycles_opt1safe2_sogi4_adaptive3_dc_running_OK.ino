/*
 * ESP32 SOGI-PLL - Frequency Adaptive Orchestration
 * Ensures SAMPLES_PER_CYCLE remains constant relative to the period
 * by updating dt (ticks_per_sample) when frequency shifts.
 */

#include <Arduino.h>
#include <math.h>

#include "SOGIvisualizer.h"

SOGIVisualizer vis;

// Return type for interpolation
struct InterpResult {
    float value;
    float confidence; // 0.0 -> no trust, 1.0 -> full trust
};

#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
//#define SOGI_K (1.4142f)
#define SOGI_K 0.7071

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
};

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
        float k = 0.7071f; // Use the same K as in project

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

struct SOGI_PLL sogi;

float dc_offset_sampling = 2048.0f;  // Used during sampling (read-only)
float dc_offset_processing = 2048.0f;  // Updated during processing
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;
uint32_t single_cycle_cycles = 0;

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float samp_buf[BUF_N];
uint32_t ts_buf[BUF_N];
int buf_idx = 0;

SOGIResampler resampler(sogi, ts_buf, samp_buf, BUF_N);

void updateTimingParameters(float frequency) {
    // 1. Constrain and cache CPU speed
    const float f_clamped = constrain(frequency, 40.0f, 60.0f);
    const float cpu_hz = (float)ESP.getCpuFreqMHz() * 1e6f;

    // 2. Pre-calculate terms to save cycles later
    sogi.inv_cpu_freq = 1.0f / cpu_hz;
    const uint32_t old_ticks = sogi.ticks_per_sample;

    // Use float calculation for consistency with SAMPLES_PER_CYCLE
    sogi.ticks_per_sample = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    single_cycle_cycles = (uint32_t)(cpu_hz / f_clamped);

    // 3. Phase Continuity Adjustment
    const uint32_t now = get_cycle_count();
    if (old_ticks > 0 && last_sample_cycles != 0) {
        uint32_t elapsed = now - last_sample_cycles;

        // Ensure we don't divide by zero or handle weird edge cases
        float progress = (float)elapsed / (float)old_ticks;

        // Clamp progress to 1.0 to prevent jumping into the future
        if (progress > 1.0f) progress = 1.0f;

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
    resampler.process_sogi_resampled(start_idx, count);
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

    // --- 1. Robust Sampling Hook ---
    // We use a 'while' to catch up if a heavy calculation (like SOGI)
    // caused us to miss a sample window.
    while ((now - last_sample_cycles) >= sogi.ticks_per_sample) {
        // Increment by fixed ticks to maintain long-term phase-lock
        last_sample_cycles += sogi.ticks_per_sample;

        if (current_cycle < 3) {
            int raw = analogRead(ADC_PIN);
            float raw_f = (float)raw;

            // Update DC tracker (EMA Filter)
            dc_current = (DC_ALPHA * raw_f) + ((1.0f - DC_ALPHA) * dc_current);

            // Subtract the offset captured at the start of this cycle-set
            float u = (raw_f - dc_offset_sampling) * (V_REF / 4095.0f);

            // Store in circular buffer
            samp_buf[buf_idx] = u;
            ts_buf[buf_idx] = now;
            buf_idx = (buf_idx + 1) % BUF_N;
        }
        
        // Safety: prevent infinite loop if ticks_per_sample is somehow 0
        if (sogi.ticks_per_sample == 0) break;
    }

    // --- Cycle Boundary Detection ---
    uint32_t elapsed_cycle = now - last_cycle_boundary;
    if (elapsed_cycle >= single_cycle_cycles) {
        last_cycle_boundary += single_cycle_cycles;

        int prev_cycle = current_cycle;
        current_cycle = (current_cycle + 1) % 4;
        cycle_start_idx[current_cycle] = buf_idx;

        if (prev_cycle == 2) {
            // Process the data from Cycle 1 to Cycle 2
            int s_idx = cycle_start_idx[1];
            int e_idx = cycle_start_idx[2];
            int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;
            
            if (actual_count > 0) {
                // Update sampling DC offset for next cycle
                dc_offset_sampling = dc_current;
                
                process_sogi_window(s_idx, actual_count);
                do_strobe_computation();
                
                // --- Phase Alignment for Visualizer ---
                // atan2f returns [-PI, PI]. We want to align to the negative peak.
                float end_phase = atan2f(sogi.v_beta, sogi.v_alpha);
                const float target_phase = -1.570796f; // -PI / 2

                float phase_diff = end_phase - target_phase;
                
                // Normalized wrap-around [0, 2PI]
                if (phase_diff < 0) phase_diff += 6.283185f;
                if (phase_diff >= 6.283185f) phase_diff -= 6.283185f;

                // Calculate how many samples back the trigger point occurred
                // We use actual_count/2PI ratio to adjust for freq drift
                float samples_per_rad = (float)actual_count / 6.283185f;
                int samples_back = (int)(phase_diff * samples_per_rad);

                int processed_end_idx = (e_idx - 1 + BUF_N) % BUF_N;
                int aligned_start_idx = (processed_end_idx - samples_back + BUF_N) % BUF_N;

                // Update Visualizer
                vis.update(samp_buf, BUF_N, aligned_start_idx, actual_count,
                           sogi.freq, sogi.mag_smooth, sogi.filtered_err);
                
                Serial.printf("F:%.4fHz | Mag:%.3f | Offset:%.1f | ActSamples:%d\n",
                              sogi.freq, sogi.mag_smooth, dc_offset_sampling, actual_count);
            }
        }
    }
}
