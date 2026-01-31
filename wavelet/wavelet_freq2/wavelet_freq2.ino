#include <Arduino.h>
#include <hal/cpu_hal.h>
#include <math.h>
#include "phase_estimator.h"

#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define SAMPLES_PER_CYCLE 128
#define CYCLES_TO_CAPTURE 3
#define STROBE_DIV 4.0f
#define BUF_SZ (SAMPLES_PER_CYCLE * CYCLES_TO_CAPTURE)

struct Sample { uint16_t v; int32_t jit; };
struct Sys {
  uint32_t cpu_hz;
  bool frame_ready;
  Sample buf[BUF_SZ];

  int32_t strobe_offset_cycles;
  bool capturing;
  uint16_t idx;
  uint32_t next_strobe_tick;
  uint32_t next_sample_tick;
  uint32_t cycles_per_sample;
  uint32_t cycles_per_strobe;

  float grid_f;               // current freq used for conversions
  int32_t phase_pending;      // cycles to add to next strobe (can be negative)
} sys;

// Phase estimator instance
PhaseEstimator phase_est;
uint32_t frame_counter = 0;

inline int32_t clamp_i32(int32_t x,int32_t lo,int32_t hi){ return x<lo?lo:(x>hi?hi:x); }

void apply_phase_rad(float rad){
  if (rad == 0.0f) return;
  double cycles_per_grid = (double)sys.cpu_hz / (double)sys.grid_f; // cycles per grid period
  double cyc = (rad / (2.0 * M_PI)) * cycles_per_grid; // desired shift in cycles
  // clamp to +/- one grid period to avoid runaway
  int32_t maxc = (int32_t)(cycles_per_grid);
  sys.phase_pending = clamp_i32((int32_t)lrint(cyc), -maxc, maxc);
}

void apply_phase_deg(float deg){ apply_phase_rad(deg * (M_PI/180.0f)); }

void tune_pll(float f){
  if (f <= 0) return;
  sys.grid_f = f;
  sys.cycles_per_sample = (uint32_t)((double)sys.cpu_hz / (f * SAMPLES_PER_CYCLE));
  sys.cycles_per_strobe = (uint32_t)((double)sys.cpu_hz * STROBE_DIV / f);

  // Update phase estimator with new interval for accurate tracking
  float interval_s = (float)sys.cycles_per_strobe / sys.cpu_hz;
  phase_est.set_frequency_params(NOMINAL_FREQ, interval_s, SAMPLES_PER_CYCLE);
}

const char* state_to_string(PhaseEstState state) {
  switch(state) {
    case PE_INITIALIZING: return "INIT";
    case PE_STABLE: return "STABLE";
    case PE_NONLINEAR_DRIFT: return "NONLINEAR";
    case PE_READY: return "READY";
    default: return "UNKNOWN";
  }
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  memset(&sys,0,sizeof(sys));
  sys.cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000u;
  sys.grid_f = NOMINAL_FREQ;
  tune_pll(sys.grid_f);
  sys.next_strobe_tick = cpu_hal_get_cycle_count() + sys.cpu_hz; // start in 1s
  
  // Initialize phase estimator with custom config
  PhaseEstConfig pe_config;
  pe_config.history_depth = 16; // 16 buffers as requested
  pe_config.correction_threshold_rad = 0.3f;  // ~17 degrees
  pe_config.nonlinear_threshold_rad = 0.1f;   // ~5.7 degrees
  pe_config.stable_tolerance_rad = 0.02f;     // ~1.1 degrees
  
  if (!phase_est.begin(&pe_config)) {
    Serial.println("ERROR: Failed to initialize phase estimator!");
  } else {
    Serial.println("Phase estimator initialized successfully");
    
    // Set frequency estimation parameters
    // Buffer interval = STROBE_DIV / frequency (time for 3 cycles)
    float buffer_interval_s = STROBE_DIV / NOMINAL_FREQ;
    phase_est.set_frequency_params(NOMINAL_FREQ, buffer_interval_s, SAMPLES_PER_CYCLE);
    
    Serial.printf("Frequency estimation configured: %.3f Hz nominal, %.4f s/buffer\n",
                 NOMINAL_FREQ, buffer_interval_s);
  }
}

inline bool poll_pll(){
  uint32_t now = cpu_hal_get_cycle_count();
  if (!sys.capturing){
    if ((int32_t)(now - sys.next_strobe_tick) >= 0){
      sys.capturing = true;
      sys.idx = 0;
      sys.strobe_offset_cycles = (int32_t)(now - sys.next_strobe_tick);
      sys.next_sample_tick = now;
      // schedule next strobe: base interval + any pending phase shift (one-shot)
      sys.next_strobe_tick = sys.next_strobe_tick + sys.cycles_per_strobe + sys.phase_pending;
      sys.phase_pending = 0;
    }
  }
  if (sys.capturing){
    if ((int32_t)(now - sys.next_sample_tick) >= 0){
      sys.buf[sys.idx].v = analogRead(ADC_PIN);
      sys.buf[sys.idx].jit = (int32_t)(cpu_hal_get_cycle_count() - sys.next_sample_tick);
      sys.next_sample_tick += sys.cycles_per_sample;
      if (++sys.idx >= BUF_SZ){
        sys.capturing = false;
        sys.frame_ready = true;
        return true;
      }
    }
  }
  return false;
}

void loop(){
  if (poll_pll()){
    frame_counter++;
    
    double dt_strobe_s = (double)sys.strobe_offset_cycles / sys.cpu_hz;

    // Extract raw ADC values for phase estimator
    uint16_t raw_buffer[BUF_SZ];
    for (uint16_t i = 0; i < BUF_SZ; i++) {
      raw_buffer[i] = sys.buf[i].v;
    }
    
    // Add frame to phase estimator history
    phase_est.add_frame(raw_buffer, BUF_SZ);
    
    // === PHASE & FREQUENCY ESTIMATION ===
    PhaseEstResult pe_result;
    FrequencyEstResult freq_result;
    
    bool pe_valid = phase_est.estimate_phase(pe_result);
    bool freq_valid = phase_est.estimate_frequency(freq_result);
    
    if (!pe_valid) {
      sys.frame_ready = false;
      return; // Not ready yet
    }
    
    // === CONTROL STRATEGY ===
    // Balanced gains for stability and convergence
    float phase_gain = 0.0f;
    float freq_gain = 0.0f;
    
    switch(pe_result.state) {
      case PE_STABLE:
        phase_gain = 0.5f;
        freq_gain = 0.15f;
        break;
        
      case PE_NONLINEAR_DRIFT:
        phase_gain = 0.3f;
        freq_gain = 0.25f;
        break;
        
      case PE_INITIALIZING:
      case PE_READY:
        phase_gain = 0.4f;
        freq_gain = 0.10f;
        break;
        
      default:
        phase_gain = 0.0f;
        freq_gain = 0.0f;
        break;
    }
    
    // === APPLY FREQUENCY CORRECTION ===
    // Use pll_correction_hz for error relative to current PLL frequency
    if (freq_valid && freq_gain > 0.0f && fabs(freq_result.pll_correction_hz) > 0.001f) {
      float freq_corr = freq_result.pll_correction_hz * freq_gain;
      float new_freq = sys.grid_f + freq_corr;
      
      // Clamp to reasonable range (±10 Hz from nominal for rapid changes)
      new_freq = fmaxf(NOMINAL_FREQ - 10.0f, fminf(NOMINAL_FREQ + 10.0f, new_freq));
      
      if (new_freq != sys.grid_f) {
        tune_pll(new_freq);
        
        if (frame_counter % 5 == 0 || fabs(freq_corr) > 0.02f) {
          Serial.printf("Freq: %.4f Hz | Corr: %+.4f Hz | Err: %+.4f Hz | Conf: %.2f\n",
                       new_freq, freq_corr, freq_result.frequency_error_hz, freq_result.confidence);
        }
      }
    }
    
    // === APPLY PHASE CORRECTION ===
    if (phase_gain > 0.0f && fabs(pe_result.linear_drift_rate) > 1e-6f) {
      float phase_corr = -pe_result.linear_drift_rate * phase_gain;
      
      if (fabs(phase_corr) > 1e-6f) {
        apply_phase_rad(phase_corr);
        phase_est.notify_correction_applied(phase_corr);
        
        if (frame_counter % 10 == 0 || fabs(phase_corr) > 0.05f) {
          Serial.printf("Phase corr: %.4f rad (%.2f deg)\n",
                       phase_corr, phase_corr * 180.0f / M_PI);
        }
      }
    }
    
    // === CORRECTION TRACKING FEEDBACK ===
    if (pe_result.correction_applied && pe_result.correction_effective) {
      Serial.printf("⚡ Correction measured: %.4f rad (%.2f deg)\n",
                   pe_result.correction_magnitude,
                   pe_result.correction_magnitude * 180.0f / M_PI);
    }
    
    // === STATUS REPORTING ===
    // Detailed report every 10 frames
    if (frame_counter % 10 == 0) {
      Serial.printf("\n=== Frame %u | State: %s ===\n", 
                   frame_counter, state_to_string(pe_result.state));
      
      if (freq_valid) {
        Serial.printf("PLL: %.4f Hz | Grid Est: %.4f Hz | Error: %+.4f Hz\n",
                     sys.grid_f, freq_result.frequency_hz, freq_result.frequency_error_hz);
      } else {
        Serial.printf("PLL: %.4f Hz | Freq estimate not ready\n", sys.grid_f);
      }
      
      Serial.printf("Phase drift: %.4f rad/buf (%.2f deg/buf) | Var: %.6f\n",
                   pe_result.linear_drift_rate,
                   pe_result.linear_drift_rate * 180.0f / M_PI,
                   pe_result.drift_variance);
      
      // Print phase trend
      Serial.print("Phase trend: ");
      uint8_t start = (pe_result.valid_samples > 8) ? (pe_result.valid_samples - 8) : 0;
      for (uint8_t i = start; i < pe_result.valid_samples; i++) {
        Serial.printf("%.3f ", pe_result.phase_trend[i]);
      }
      Serial.println();
    }
    
    // Brief status every frame
    if (frame_counter % 5 == 0) {
      Serial.printf("[%u] %.4f Hz | Drift: %.3f rad/buf | %s\n",
                   frame_counter, sys.grid_f, pe_result.linear_drift_rate,
                   state_to_string(pe_result.state));
    }

    sys.frame_ready = false;
  }
}
