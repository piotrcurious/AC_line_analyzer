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

#define PLL_KP 1.1f
#define PLL_KI 0.000001f
#define SAMPLES_PER_CYCLE 200 // approx 10khz to avoid quantization error
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.0001f

static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

// history length for adaptive estimator (power-of-two or small number)
#define SOGI_HIST_LEN 8

struct SOGI_PLL {
    float v_alpha, v_beta;
    float theta;
    float freq;
    float omega;
    float integral = 0.0;
    //float integral_err_c = 0.0;
    uint32_t ticks_per_sample; 
    float u_prev;
    float filtered_err;
    float mag_smooth;
    float inv_cpu_freq;
    float wz1_a, wz2_a; // States for alpha filter
    float wz1_b, wz2_b; // States for beta filter
    float last_correction;   // The phase adjustment applied in the previous step
    float prev_applied_phase;
    float phase_est;         // Integrated phase to track internal state
    float freq_estimate;

    float integral_state;        // plain integral state (unscaled)
float integral_err_c;        // Kahan correction for integrator
float i_term;                // current integral contribution (PLL_KI * integral_state)
float last_control_action;   // last applied control (for diagnostics)
float gain_est;              // adaptive estimated gain: phase_change ≈ gain_est * control
float gain_learn_rate;       // adaptation speed
float control_hist[SOGI_HIST_LEN];     // circular buffer of past controls
float phase_hist[SOGI_HIST_LEN];       // circular buffer of past measured phase errors
uint8_t hist_idx;            // insertion index into circular buffers
 
} sogi;

// Add to your globals (near where sogi is defined)
struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;  // Accumulated 2π wraps
    bool initialized = false;
} phase_track;

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

/*
    float period_sec = 1.0f / frequency;
    // Update the master boundary for the next cycle
    single_cycle_cycles = (uint32_t)(period_sec * cpu_hz);
*/    
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

    sogi.integral_state = 0.0f;
sogi.integral_err_c = 0.0f;
sogi.i_term = 0.0f;
sogi.last_control_action = 0.0f;
sogi.gain_est = 0.11f;            // start conservative (0)
sogi.gain_learn_rate = 0.1001f;    // example; tune down if unstable
sogi.hist_idx = 0;
for (int i=0;i<SOGI_HIST_LEN;i++){ sogi.control_hist[i]=0.0f; sogi.phase_hist[i]=0.0f; }
}

void process_sogi_window(int start_idx, int count) {
    if (count <= 0) return;

    // --- Pass 1: Calculate Window-Specific DC Offset ---
    // Since this is called on a phase-aligned "clean cycle," 
    // the average of these samples is the most accurate DC estimate.
    float sum = 0;
    for (int i = 0; i < count; ++i) {
        int idx = (start_idx + i) % BUF_N;
        sum += samp_buf[idx];
    }
    float current_window_dc = sum / (float)count;

    // Optional: Smooth the transition of the global DC offset 
    // to prevent SOGI state kicks if the DC shifts significantly.
    // If you want instant tracking, just use: dc_offset_sampling = current_window_dc;
    dc_offset_sampling = (0.2f * current_window_dc) + (0.8f * dc_offset_sampling);

    // --- Pre-compute SOGI Coefficients ---
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

    // --- Pass 2: SOGI Filtering with DC Correction ---
    for (int i = 0; i < count; ++i) {
        int idx = (start_idx + i) % BUF_N;
        
        // Use the smoothed window-based DC offset
        float u = samp_buf[idx] - dc_offset_sampling;

        // SOGI Alpha path (Band Pass)
        float in_a = u - (a_a1 * sogi.wz1_a) - (a_a2 * sogi.wz2_a);
        sogi.v_alpha = (a_b0 * in_a) + (a_b2 * sogi.wz2_a); 
        sogi.wz2_a = sogi.wz1_a;
        sogi.wz1_a = in_a;

        // SOGI Beta path (Quadrature Low Pass)
        float in_b = u - (a_a1 * sogi.wz1_b) - (a_a2 * sogi.wz2_b);
        sogi.v_beta = (b_b0 * in_b) + (b_b1 * sogi.wz1_b) + (b_b2 * sogi.wz2_b);
        sogi.wz2_b = sogi.wz1_b;
        sogi.wz1_b = in_b;
    }
}

void do_strobe_computation() {
    float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);

    // Slight smoothing of magnitude to prevent division by zero/noise spikes
    sogi.mag_smooth = (1.0f * mag_inst) + (0.0f * sogi.mag_smooth);

    if (sogi.mag_smooth > 0.10f) {
        // 1) Raw phase error (normalized) — you used v_beta for 90°
        float raw_p_err = sogi.v_beta / sogi.mag_smooth; // same as before

        // --- 2) Predictive cancellation using adaptive single-gain model ---
        // The estimator models phase_change ≈ gain_est * control_action.
        // We compute predicted effect of last control on current measured error,
        // then form residual_error = raw_p_err - predicted_effect.
        float predicted_effect = sogi.gain_est * sogi.last_control_action;
        float residual_err = raw_p_err - predicted_effect;

        // --- 3) Update adaptive gain model using history (simple LMS update)
        // We use the difference in successive measured errors (dy) versus the previous control
        // to learn how control influences the change in error.
        // dy = current_raw - previous_raw
        uint8_t idx_prev = (sogi.hist_idx + SOGI_HIST_LEN - 1) & (SOGI_HIST_LEN - 1); // power-of-two wrap
        float prev_phase = sogi.phase_hist[idx_prev];
        float prev_control = sogi.control_hist[idx_prev];

        // update history (we'll push current raw_p_err and predicted control later)
        float dy = raw_p_err - prev_phase;

        // Simple normalized LMS-like update (robust to varying u)
        float denom = (prev_control * prev_control) + 1e-6f; // avoid div-by-zero
        float err_gain = (dy - sogi.gain_est * prev_control); // prediction error
        sogi.gain_est += sogi.gain_learn_rate * (prev_control * err_gain) / denom;

        // Optionally clamp gain_est to safe bounds (tune to your plant)
        if (sogi.gain_est > 2.0f) sogi.gain_est = 2.0f;
        else if (sogi.gain_est < -2.0f) sogi.gain_est = -2.0f;

        // --- 4) Controller separation: P-term uses *residual* or raw?
        // Keep P snappy: use residual_err for P (so it responds to the part not explained by applied control)
//        float p_term = PLL_KP * residual_err;
        float p_term = PLL_KP * raw_p_err;

        // 5) Kahan summation for the Integral on the residual_err (not raw)
        // We integrate the residual error so the integrator doesn't integrate the commanded correction itself.
        float y = residual_err - sogi.integral_err_c;  // note: integrate raw residual, integrate actual error, not scaled
        float t = sogi.integral_state + y;
        sogi.integral_err_c = (t - sogi.integral_state) - y;
        sogi.integral_state = t;

        // Now scale the integral_state by KI to form i_term (this separates state from gain)
        sogi.i_term = PLL_KI * sogi.integral_state;

        // Anti-windup: constrain i_term and reflect on integral_state if needed
        const float I_MAX = 5.0f; // tune: maximum i_term magnitude (same units as your previous integral cap)
        if (sogi.i_term > I_MAX) {
            sogi.i_term = I_MAX;
            sogi.integral_state = sogi.i_term / PLL_KI;
            sogi.integral_err_c = 0.0f; // reset Kahan correction to avoid buildup mismatches
        } else if (sogi.i_term < -I_MAX) {
            sogi.i_term = -I_MAX;
            sogi.integral_state = sogi.i_term / PLL_KI;
            sogi.integral_err_c = 0.0f;
        }

        // --- 6) Compose control action and apply constraints ---
        float control_action = p_term + sogi.i_term;

        // Save last control action for next-step prediction
        sogi.last_control_action = control_action;

        // Frequency update: use the separated control_action
        float f_new = NOMINAL_FREQ + control_action;
        sogi.freq = constrain(f_new, 42.0f, 58.0f);
        sogi.omega = 2.0f * PI * sogi.freq;

        //updateTimingParameters(sogi.freq); // as you had

        // --- 7) Push new measurements into history circular buffers ---
        // Store raw_p_err as the observable phase metric (before cancellation) so estimator sees actual measured signal
        sogi.phase_hist[sogi.hist_idx] = raw_p_err;
        sogi.control_hist[sogi.hist_idx] = control_action;
        sogi.hist_idx = (sogi.hist_idx + 1) & (SOGI_HIST_LEN - 1); // power-of-two wrap

        // Done: No extra low-pass filtering here — residual_err is computed deterministically
    }
}

void do_strobe_computation_old() {
    float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);
    
    // Slight smoothing of magnitude to prevent division by zero/noise spikes
    sogi.mag_smooth = (1.0f * mag_inst) + (0.0f * sogi.mag_smooth);
    
    if (sogi.mag_smooth > 0.10f) {
        // 1. Calculate the raw phase error (normalized)
        //float raw_p_err = sogi.v_alpha / sogi.mag_smooth; // for 0'
        //if (sogi.v_beta > 0) raw_p_err = -raw_p_err; // for 0'
        
        float raw_p_err = sogi.v_beta / sogi.mag_smooth; // for 90'
        
        // 2. Kahan Summation for the Integral Term
        // This preserves small error updates that would be lost when added to a large 'sogi.integral'
        float y = (PLL_KI * raw_p_err) - sogi.integral_err_c; 
        float t = sogi.integral + y;                         
        sogi.integral_err_c = (t - sogi.integral) - y;       
        sogi.integral = t;

        // Anti-windup
        sogi.integral = constrain(sogi.integral, -5.0f, 5.0f);

        // 3. Frequency Update
        // We use the raw_p_err for the Proportional part to keep response snappy
        float f_new = NOMINAL_FREQ + (PLL_KP * raw_p_err) + sogi.integral;
        
        sogi.freq = constrain(f_new, 42.0f, 58.0f);
        sogi.omega = 2.0f * PI * sogi.freq;

        //updateTimingParameters(sogi.freq);
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
        gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
}

float dc_current = 2048; 
void loop() {
    uint32_t now = get_cycle_count();

    // --- Sampling (Cycles 0, 1, 2) ---
    uint32_t elapsed_sample = now - last_sample_cycles;
    if (elapsed_sample >= sogi.ticks_per_sample && current_cycle < 3) { 
        last_sample_cycles += sogi.ticks_per_sample;
        ts_buf[buf_idx] = get_cycle_count();
        int raw = analogRead(ADC_PIN);
        //dc_current = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * dc_current);    
        float u = ((float)raw) ;//* (V_REF / 4095.0f);
        samp_buf[buf_idx] = u;
        
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
        //dc_offset_sampling = dc_current;
        float current_dt = (float)sogi.ticks_per_sample * sogi.inv_cpu_freq;
        float samples_per_actual_period = 1.0f / (sogi.freq * current_dt);

        // Process ACTUAL samples only
        process_sogi_window(s_idx, actual_count);
        do_strobe_computation();

        // For phase calculation ONLY, save state and process one clean cycle
        float saved_alpha = sogi.v_alpha;
        float saved_beta = sogi.v_beta;
        float saved_wz1_a = sogi.wz1_a, saved_wz2_a = sogi.wz2_a;
        float saved_wz1_b = sogi.wz1_b, saved_wz2_b = sogi.wz2_b;

        // Reset to known state for phase measurement
        sogi.v_alpha = 0; sogi.v_beta = 0;
        sogi.wz1_a = 0; sogi.wz2_a = 0;
        sogi.wz1_b = 0; sogi.wz2_b = 0;

        // Process exactly one cycle for clean phase
        float samples_per_cycle = 1.0f / (sogi.freq * current_dt);
        int one_cycle_count = (int)(samples_per_cycle + 0.5f);
        // CRITICAL: Don't process more samples than actually collected!
// During frequency transitions, one_cycle_count (based on new freq) 
// might exceed actual_count (collected at old freq)
int safe_process_count = min(one_cycle_count, actual_count);

//process_sogi_window(s_idx, safe_process_count);
process_sogi_window(s_idx, actual_count);


//        process_sogi_window(s_idx, one_cycle_count);

        // Calculate phase from clean state (YOUR ORIGINAL FORMULA)
        float phase = atan2f(sogi.v_alpha, -sogi.v_beta);
        if (phase < 0) phase += 2.0f * PI;

        // Restore the real SOGI state
        sogi.v_alpha = saved_alpha;
        sogi.v_beta = saved_beta;
        sogi.wz1_a = saved_wz1_a; sogi.wz2_a = saved_wz2_a;
        sogi.wz1_b = saved_wz1_b; sogi.wz2_b = saved_wz2_b;

// PHASE UNWRAPPING - detect and track 2π wraps
if (phase_track.initialized) {
    float phase_delta = phase - phase_track.prev_phase;
    
    // Detect wrap: if phase jumps by more than π, a wrap occurred
    if (phase_delta < -PI) {
        // Wrapped forward: 6.28 → 0.01
        phase_track.phase_offset += 2.0f * PI;
    } else if (phase_delta > PI) {
        // Wrapped backward: 0.01 → 6.28 (shouldn't happen but handle it)
        phase_track.phase_offset -= 2.0f * PI;
    }
} else {
    phase_track.initialized = true;
}

phase_track.prev_phase = phase;

// Unwrap for continuity tracking
float unwrapped_phase = phase + phase_track.phase_offset;

// But use modulo for alignment - we only want phase WITHIN this cycle
float phase_for_alignment = fmodf(unwrapped_phase, 2.0f * PI);
if (phase_for_alignment < 0) phase_for_alignment += 2.0f * PI;

// YOUR ORIGINAL CALCULATION with the wrapped-back phase
float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle;

//int aligned_start_idx = (s_idx + one_cycle_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;

        int vis_count = (int)(samples_per_cycle + 0.5f);

        updateTimingParameters(sogi.freq);
        
        uint32_t proc_end = get_cycle_count();
        uint32_t vis_start = get_cycle_count();


        vis.update(samp_buf, BUF_N, aligned_start_idx, vis_count, sogi.freq, sogi.mag_smooth, sogi.filtered_err, dc_offset_sampling);

//### Summary of changes:
//* **Context Awareness:** The visualizer now knows the `dc_offset` that the processor is using.
//* **Relative Plotting:** The zero-line is no longer just the middle of the screen; it is the visual representation of the DC bias point.
//* **Scaling:** The min/max tracking now correctly wraps around the raw signal (e.g., oscillating between 1.5V and 1.8V), and the grid line stays at the calculated 1.65V (or whatever the DC is).

        uint32_t vis_end = get_cycle_count();

        float cycle_processing_us = (float)(proc_end - proc_start) * sogi.inv_cpu_freq * 1e6f;
        float visualization_us = (float)(vis_end - vis_start) * sogi.inv_cpu_freq * 1e6f;
        float total_cycle_us = (float)single_cycle_cycles * sogi.inv_cpu_freq * 1e6f;
        float remaining_us = total_cycle_us - (cycle_processing_us + visualization_us);

        Serial.printf("F:%.5fHz, Mag:%.3f, Err:%.3f, ActSamples:%d, Core:%.1fus, Vis:%.1fus, Avail:%.1fus\n", 
                      sogi.freq, sogi.mag_smooth, sogi.filtered_err, (int)samples_per_actual_period,
                      cycle_processing_us, visualization_us, remaining_us);           
    }
}
    }
}
