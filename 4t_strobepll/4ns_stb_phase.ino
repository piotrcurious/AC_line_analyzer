#include <Arduino.h>
#include <hal/cpu_hal.h>
#include <math.h>

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

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  memset(&sys,0,sizeof(sys));
  sys.cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000u;
  sys.grid_f = NOMINAL_FREQ;
  tune_pll(sys.grid_f);
  sys.next_strobe_tick = cpu_hal_get_cycle_count() + sys.cpu_hz; // start in 1s
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
    double dt_strobe_s = (double)sys.strobe_offset_cycles / sys.cpu_hz;
    double dt_sample0_s = (double)sys.buf[0].jit / sys.cpu_hz;

    // --- PLACEHOLDER: user EKF/phase detector should compute:
    //   float est_f = ...;
    //   float phase_err_rad = ...; // positive means strobe lags grid (need advance)
    float est_f = sys.grid_f; // keep current as placeholder
    float phase_err_rad = 0.0f; // placeholder: replace with algorithm result

    // Apply controls
    tune_pll(est_f);
    if (fabs(phase_err_rad) > 1e-6f) apply_phase_rad(phase_err_rad);

    Serial.printf("F:%.3fHz | StrobeJit:%d cyc (%.2fus) | Samp0Jit:%d cyc\n",
      est_f, sys.strobe_offset_cycles, dt_strobe_s*1e6, sys.buf[0].jit);

    sys.frame_ready = false;
  }
}
