/*
 * ESP32 SOGI-PLL - High Bandwidth Tuning
 * 1. ISR: Master Clock marking the "Golden" PLL period.
 * 2. Raw Proportional Path: Instantaneous response to frequency shifts.
 * 3. High-Gain Integrator: Rapidly kills error during 1Hz/sec ramps.
 * 4. Tustin SOGI: Maintains orthogonality at 128 samples/cycle.
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f          // Ideal Butterworth damping
#define PLL_KP 2.00f            // Doubled: Snappy response to phase steps
#define PLL_KI 15.0f            // Significantly increased: Fast tracking of ramps
#define SAMPLES_PER_CYCLE 128   

// ADC & Normalization
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.05f          // Faster DC tracking for startup

// Strobe Sync Variables
volatile bool strobe_triggered = false;
hw_timer_t *strobe_timer = NULL;

struct SOGI_PLL {
  float v_alpha, v_beta;
  float theta;      
  float freq;       
  float omega;      
  float integral;   
  float target_dt;  
  float u_prev;
  float filtered_err; 
  float mag_smooth;
} sogi;

float dc_offset = 2048.0f;

void IRAM_ATTR onStrobeTimer() {
  strobe_triggered = true;
}

void updateStrobeInterval(float frequency) {
  float f_clamped = constrain(frequency, 40.0f, 60.0f);
  uint32_t interval_us = (uint32_t)(1000000.0f / f_clamped);
  timerAlarm(strobe_timer, interval_us, true, 0);
  sogi.target_dt = 1.0f / (f_clamped * (float)SAMPLES_PER_CYCLE);
}

void initSOGI(float f_start) {
  memset(&sogi, 0, sizeof(sogi));
  sogi.freq = f_start;
  sogi.omega = 2.0f * PI * f_start;
  sogi.target_dt = 1.0f / (f_start * (float)SAMPLES_PER_CYCLE);
  sogi.mag_smooth = 1.0f; 
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(ADC_RESOLUTION);
  initSOGI(NOMINAL_FREQ);

  strobe_timer = timerBegin(1000000); 
  timerAttachInterrupt(strobe_timer, &onStrobeTimer);
  updateStrobeInterval(NOMINAL_FREQ);
}

void loop() {
  static uint64_t last_sample_t = 0;
  uint64_t now_t = esp_timer_get_time();

  float dt = (float)(now_t - last_sample_t) / 1000000.0f;

  if (dt >= sogi.target_dt) {
    last_sample_t = now_t;

    int raw = analogRead(ADC_PIN);
    dc_offset = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * dc_offset);
    float u = ((float)raw - dc_offset) * (V_REF / 4095.0f);

    // Tustin SOGI Update
    float w = sogi.omega;
    float k = SOGI_K;
    float x = w * dt / 2.0f;
    
    float den = 1.0f + k*x + x*x;
    float alpha_old = sogi.v_alpha;
    float beta_old  = sogi.v_beta;

    sogi.v_alpha = ( (1.0f - x*x)*alpha_old - 2.0f*x*beta_old + k*x*(u + sogi.u_prev) ) / den;
    sogi.v_beta  = ( 2.0f*x*alpha_old + (1.0f - k*x - x*x)*beta_old + k*x*x*(u + sogi.u_prev) ) / den;

    sogi.u_prev = u;
    
    sogi.theta += sogi.omega * dt;
    if (sogi.theta >= 2.0f * PI) sogi.theta -= 2.0f * PI;
  }

  if (strobe_triggered) {
    strobe_triggered = false;

    float mag_inst = sqrtf(sogi.v_alpha * sogi.v_alpha + sogi.v_beta * sogi.v_beta);
    // Faster magnitude tracking for responsive loop gains
    sogi.mag_smooth = (0.35f * mag_inst) + (0.65f * sogi.mag_smooth);
    
    if (sogi.mag_smooth > 0.10f) {
      // Phase Error (V_beta normalized)
      float raw_p_err = sogi.v_beta / sogi.mag_smooth;
      
      // Integrator Path Filtering: Light smoothing to prevent ADC jitter from building up
      sogi.filtered_err = (0.6f * raw_p_err) + (0.4f * sogi.filtered_err);
      
      // Update Integral (Frequency learning)
      // High KI ensures the integrator doesn't "drag" behind the test signal
      sogi.integral += PLL_KI * sogi.filtered_err * (1.0f / NOMINAL_FREQ);
      sogi.integral = constrain(sogi.integral, -10.0f, 10.0f);
      
      // Frequency Control Law:
      // Proportional term uses raw_p_err for instant "kick"
      float f_new = NOMINAL_FREQ + (PLL_KP * raw_p_err) + sogi.integral;
      
      sogi.freq = constrain(f_new, 42.0f, 58.0f); 
      sogi.omega = 2.0f * PI * sogi.freq;

      Serial.printf("Hz:%.3f, Mag:%.3f, P_Err:%.3f, Int:%.3f\n", 
                    sogi.freq-NOMINAL_FREQ, sogi.mag_smooth, raw_p_err, sogi.integral);
    } else {
      sogi.integral *= 0.95f; // Faster reset when signal is gone
    }
    
    // Reset software phase to trigger point
    sogi.theta = 0;
    updateStrobeInterval(sogi.freq);
  }
}
