/*
 * ESP32 SOGI-PLL - Jitter-Compensated Implementation
 * * Improvements for Sampling Jitter:
 * 1. Sliding Window Timing: last_sample_cycles increments by target_cycles, 
 * not 'now', preserving the phase of the sampling clock across iterations.
 * 2. Instantaneous DT: SOGI integration uses the exact cycle delta for the current 
 * step, making the math robust to OS-induced execution delays.
 * 3. Pre-calculated Constants: Moved reciprocal divisions out of the hot loop.
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SOGI_K (1.4142f)         // 0.7071 * 2

#define PLL_KP 2.0f
#define PLL_KI 0.00f             
#define SAMPLES_PER_CYCLE 200   

// ADC & Normalization
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.005f

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
    uint32_t target_cycles;  
    float u_prev;
    float filtered_err; 
    float mag_smooth;
    float inv_cpu_freq; // Pre-calculated for speed
} sogi;

float dc_offset = 2048.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_strobe_cycles = 0;
uint32_t target_strobe_interval = 0;

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    float cpu_hz = (float)ESP.getCpuFreqMHz() * 1000000.0f;
    
    sogi.inv_cpu_freq = 1.0f / cpu_hz;
    sogi.target_cycles = (uint32_t)(cpu_hz / (f_clamped * (float)SAMPLES_PER_CYCLE));
    target_strobe_interval = (uint32_t)(cpu_hz / f_clamped);
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
    
    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_strobe_cycles = start_c;
}

void loop() {
    uint32_t now = get_cycle_count();
    
    // ---------------------------------------------------------
    // 1. Jitter-Compensated SOGI Integration
    // ---------------------------------------------------------
    uint32_t elapsed_sample = now - last_sample_cycles;
    
    if (elapsed_sample >= sogi.target_cycles) {
        /* * JITTER FIX: We use the actual 'elapsed_sample' for dt calculation 
         * so the integrator math is correct for this specific (possibly delayed) step.
         * Then we increment 'last_sample_cycles' by the target, NOT 'now'. 
         * This ensures that any "overshoot" is carried over to the next interval,
         * maintaining a perfect long-term average sampling rate.
         */
        float dt = (float)elapsed_sample * sogi.inv_cpu_freq;
        last_sample_cycles += sogi.target_cycles; 

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
    // 2. Frequency Update (Strobe Path)
    // ---------------------------------------------------------
    uint32_t elapsed_strobe = now - last_strobe_cycles;
    
    if (elapsed_strobe >= target_strobe_interval) {
        // Carry over the strobe jitter as well
        last_strobe_cycles += target_strobe_interval;

        float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);
        sogi.mag_smooth = (0.2f * mag_inst) + (0.8f * sogi.mag_smooth); // Slower mag filter
        
        if (sogi.mag_smooth > 0.10f) {
            // Use Orthogonal Component for Phase Error
            float raw_p_err = sogi.v_beta / sogi.mag_smooth;
            sogi.filtered_err = (0.5f * raw_p_err) + (0.5f * sogi.filtered_err);
            
            sogi.integral += PLL_KI * sogi.filtered_err;
            sogi.integral = constrain(sogi.integral, -10.0f, 10.0f);
            
            float f_new = NOMINAL_FREQ + (PLL_KP * sogi.filtered_err) + sogi.integral;
            sogi.freq = constrain(f_new, 42.0f, 58.0f); 
            sogi.omega = 2.0f * PI * sogi.freq;

            updateTimingParameters(sogi.freq);

            // Using Serial.printf sparingly to avoid blocking
            //static int log_div = 0;
            //if (++log_div >= 10) {
                Serial.printf("F:%.5fHz, Mag:%.3f, Err:%.3f\n", sogi.freq-NOMINAL_FREQ, sogi.mag_smooth, sogi.filtered_err);
                //log_div = 0;
            //}
        }
        
        // Reset phase at strobe to keep software oscillator locked to hardware signal
        sogi.theta = 0;
    }
}
