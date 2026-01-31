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
}

const char* state_to_string(PhaseEstState state) {
  switch(state) {
    case PE_INITIALIZING: return "INIT";
    case PE_STABLE: return "STABLE";
    case PE_CORRECTION_DETECTED: return "CORRECTION";
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
    double dt_sample0_s = (double)sys.buf[0].jit / sys.cpu_hz;

    // Extract raw ADC values for phase estimator
    uint16_t raw_buffer[BUF_SZ];
    for (uint16_t i = 0; i < BUF_SZ; i++) {
      raw_buffer[i] = sys.buf[i].v;
    }
    
    // Add frame to phase estimator history
    phase_est.add_frame(raw_buffer, BUF_SZ);
    
    // Perform phase estimation if ready
    PhaseEstResult pe_result;
    float phase_err_rad = 0.0f;
    
    if (phase_est.estimate_phase(pe_result)) {
      // Phase estimation successful
      
      // CONTROL LOGIC based on state machine
      switch(pe_result.state) {
        case PE_STABLE:
          // Stable linear drift - apply correction based on drift rate
          // Use the linear drift rate to predict and compensate
          phase_err_rad = -pe_result.linear_drift_rate;
          break;
          
        case PE_CORRECTION_DETECTED:
          // Phase correction was detected - DO NOT apply counter-correction
          // This would fight against the correction that was just applied
          phase_err_rad = 0.0f;
          Serial.printf("! CORRECTION DETECTED: %.3f rad (%.1f deg)\n",
                       pe_result.correction_magnitude,
                       pe_result.correction_magnitude * 180.0f / M_PI);
          break;
          
        case PE_NONLINEAR_DRIFT:
          // Non-linear changes (signal variations) - be conservative
          // Only correct if drift is significant
          if (fabs(pe_result.recent_phase_shift) > 0.05f) {
            phase_err_rad = -pe_result.recent_phase_shift * 0.5f; // Gentle correction
          }
          break;
          
        case PE_INITIALIZING:
        case PE_READY:
        default:
          // Not enough data or generic ready state
          phase_err_rad = 0.0f;
          break;
      }
      
      // Print detailed phase estimation info every 10 frames
      if (frame_counter % 10 == 0) {
        Serial.printf("\n=== Frame %u | State: %s ===\n", 
                     frame_counter, state_to_string(pe_result.state));
        Serial.printf("Drift Rate: %.4f rad/buf (%.2f deg/buf)\n",
                     pe_result.linear_drift_rate,
                     pe_result.linear_drift_rate * 180.0f / M_PI);
        Serial.printf("Recent Shift: %.4f rad (%.2f deg)\n",
                     pe_result.recent_phase_shift,
                     pe_result.recent_phase_shift * 180.0f / M_PI);
        Serial.printf("Drift Variance: %.6f\n", pe_result.drift_variance);
        Serial.printf("Correction: %s", pe_result.correction_detected ? "YES" : "NO");
        if (pe_result.correction_detected) {
          Serial.printf(" (%.3f rad / %.1f deg)", 
                       pe_result.correction_magnitude,
                       pe_result.correction_magnitude * 180.0f / M_PI);
        }
        Serial.println();
        
        // Print phase trend (last 8 entries for readability)
        Serial.print("Phase Trend [rad]: ");
        uint8_t start = (pe_result.valid_samples > 8) ? 
                        (pe_result.valid_samples - 8) : 0;
        for (uint8_t i = start; i < pe_result.valid_samples; i++) {
          Serial.printf("%.3f ", pe_result.phase_trend[i]);
        }
        Serial.println();
      }
    }
    
    // --- PLACEHOLDER: user EKF/frequency estimator
    float est_f = sys.grid_f; // keep current as placeholder
    
    // Apply controls
    tune_pll(est_f);
    if (fabs(phase_err_rad) > 1e-6f) {
      apply_phase_rad(phase_err_rad);
      Serial.printf("Applied phase correction: %.4f rad (%.2f deg)\n",
                   phase_err_rad, phase_err_rad * 180.0f / M_PI);
    }

    // Basic status every frame
    Serial.printf("F:%.3fHz | StrobeJit:%d cyc (%.2fus) | Samp0Jit:%d cyc | State:%s\n",
      est_f, sys.strobe_offset_cycles, dt_strobe_s*1e6, sys.buf[0].jit,
      state_to_string(phase_est.get_state()));

    sys.frame_ready = false;
  }
}
