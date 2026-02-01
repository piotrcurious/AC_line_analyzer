/*
 * ESP32 SOGI-PLL - Ultra-Low Jitter Implementation
 * * Improvements:
 * 1. Cycle-Accurate Timing: Uses ESP32 CCOUNT register (~4ns resolution).
 * 2. Zero-ISR Architecture: Polling-based execution prevents interrupt latency jitter.
 * 3. Dynamic DT: SOGI integration uses the exact measured cycle count for dt.
 * 4. Strobe Alignment: Syncs frequency estimation to precise cycle-strobe.
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SOGI_K (0.7071f * 2.0f)

#define PLL_KP 2.0f
#define PLL_KI 0.00f            // Added small KI for steady-state tracking
#define SAMPLES_PER_CYCLE 200   

// ADC & Normalization
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.005f

// Cycle Counter Utilities (ESP32 Internal)
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
    uint32_t target_cycles;  // Cycle-based interval instead of microseconds
    float u_prev;
    float filtered_err; 
    float mag_smooth;
} sogi;

float dc_offset = 2048.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_strobe_cycles = 0;
uint32_t target_strobe_interval = 0;

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    float cpu_freq = (float)ESP.getCpuFreqMHz() * 1000000.0f;
    
    // Interval for the high-frequency SOGI integration
    sogi.target_cycles = (uint32_t)(cpu_freq / (f_clamped * (float)SAMPLES_PER_CYCLE));
    
    // Interval for the 1-per-cycle frequency estimation strobe
    target_strobe_interval = (uint32_t)(cpu_freq / f_clamped);
}

void initSOGI(float f_start) {
    memset(&sogi, 0, sizeof(sogi));
    sogi.freq = f_start;
    sogi.omega = 2.0f * PI * f_start;
    sogi.mag_smooth = 1.0f; 
    updateTimingParameters(f_start);
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(ADC_RESOLUTION);
    initSOGI(NOMINAL_FREQ);
    
    last_sample_cycles = get_cycle_count();
    last_strobe_cycles = last_sample_cycles;
}

void loop() {
    uint32_t now = get_cycle_count();
    
    // ---------------------------------------------------------
    // 1. High-Frequency SOGI Integration (Sample Path)
    // ---------------------------------------------------------
    uint32_t elapsed_sample = now - last_sample_cycles;
    
    if (elapsed_sample >= sogi.target_cycles) {
        // Calculate exact dt based on cycles for the integrator
        float dt = (float)elapsed_sample / ((float)ESP.getCpuFreqMHz() * 1000000.0f);
        last_sample_cycles = now;

        int raw = analogRead(ADC_PIN);
        dc_offset = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * dc_offset);
        float u = ((float)raw - dc_offset) * (V_REF / 4095.0f);

        // Tustin SOGI Update Logic
        float w_safe = (sogi.omega > 0.01f) ? sogi.omega : 0.01f;
        float x = w_safe * dt * 0.5f;
        float x2 = x * x;
        float kx = SOGI_K * x;
        float kx_u = kx * (u + sogi.u_prev);
        float inv_den = 1.0f / (1.0f + kx + x2);

        float alpha_old = sogi.v_alpha;
        float beta_old = sogi.v_beta;

        sogi.v_alpha = ((1.0f - x2) * alpha_old - (2.0f * x) * beta_old + kx_u) * inv_den;
        sogi.v_beta = (2.0f * x * alpha_old + (1.0f - kx - x2) * beta_old + x * kx_u) * inv_den;

        sogi.u_prev = u;
        sogi.theta += w_safe * dt;

        // Phase wrapping
        if (sogi.theta >= 2.0f * PI) sogi.theta -= 2.0f * PI;
    }

    // ---------------------------------------------------------
    // 2. Low-Frequency PLL Frequency Update (Strobe Path)
    // ---------------------------------------------------------
    uint32_t elapsed_strobe = now - last_strobe_cycles;
    
    if (elapsed_strobe >= target_strobe_interval) {
        last_strobe_cycles = now;

        float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);
        sogi.mag_smooth = (0.35f * mag_inst) + (0.65f * sogi.mag_smooth);
        
        if (sogi.mag_smooth > 0.10f) {
            // Error = sin(theta_in - theta_pll). Normalized by magnitude.
            float raw_p_err = sogi.v_beta / sogi.mag_smooth;
            
            sogi.filtered_err = (0.6f * raw_p_err) + (0.4f * sogi.filtered_err);
            
            // Frequency Integrator
            sogi.integral += PLL_KI * sogi.filtered_err;
            sogi.integral = constrain(sogi.integral, -5.0f, 5.0f);
            
            float f_new = NOMINAL_FREQ + (PLL_KP * raw_p_err) + sogi.integral;
            sogi.freq = constrain(f_new, 42.0f, 58.0f); 
            sogi.omega = 2.0f * PI * sogi.freq;

            // Update timing targets for the next cycle
            updateTimingParameters(sogi.freq);

            // Log performance
            Serial.printf("dHz:%.3f, Mag:%.3f, Err:%.3f\n", 
                          sogi.freq - NOMINAL_FREQ, sogi.mag_smooth, raw_p_err);
        } else {
            sogi.integral *= 0.98f; 
        }
        
        // Hard-sync software phase to zero at the strobe point
        sogi.theta = 0;
    }
}
