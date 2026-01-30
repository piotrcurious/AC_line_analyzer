/*
 * ESP32 Heterodyne PLL-EKF - Correct Architecture
 * 
 * KEY INSIGHT: This is a HETERODYNE system
 * - Grid oscillates at ω_grid (unknown)
 * - Strobe oscillates at ω_strobe (controlled by us)
 * - We observe the BEAT between them
 * 
 * STATE SPACE:
 * x = [ω_grid, ω̇_grid, ω_strobe_error]
 * 
 * where:
 * - ω_grid = true grid frequency (rad/s)
 * - ω̇_grid = grid frequency rate of change
 * - ω_strobe_error = (ω_strobe_actual - ω_grid/4)
 *   This captures the PLL lock error
 * 
 * MEASUREMENTS (all differential/derived):
 * 1. Zero-crossing interval → gives grid period
 * 2. Cycles per frame → gives frequency ratio
 * 3. Phase drift per frame → gives beat frequency
 * 
 * CONTROL:
 * ω_strobe_commanded = ω_grid/4 - K·ω_strobe_error
 */

#include <Arduino.h>
#include <math.h>

// Configuration
#define ADC_PIN 36
#define NOMINAL_FREQ 50.0f
#define NOMINAL_OMEGA (2.0f * PI * NOMINAL_FREQ)
#define SOGI_K 1.414f

#define SAMPLES_PER_CYCLE 128
#define CYCLES_TO_CAPTURE 3
#define BUFFER_SIZE (SAMPLES_PER_CYCLE * CYCLES_TO_CAPTURE)

// ADC
#define ADC_RESOLUTION 12
#define V_REF 3.3f
#define DC_ALPHA 0.02f

// EKF Tuning
/*
#define Q_OMEGA 1e-7f           // Grid frequency process noise
#define Q_OMEGA_DOT 1e-10f      // Grid frequency rate noise
#define Q_STROBE_ERROR 1e-6f    // Strobe error accumulation
#define R_PERIOD 1e-9f          // Period measurement noise
#define R_CYCLE_COUNT 0.01f     // Cycle count measurement noise
#define R_PHASE_DRIFT 1e-5f     // Phase drift measurement noise
*/
#define Q_OMEGA 0.1f           // Grid frequency process noise
#define Q_OMEGA_DOT 0.0001f      // Grid frequency rate noise
#define Q_STROBE_ERROR 0.01f    // Strobe error accumulation
#define R_PERIOD 0.0001f          // Period measurement noise
#define R_CYCLE_COUNT 0.001f     // Cycle count measurement noise
#define R_PHASE_DRIFT 0.001f     // Phase drift measurement noise



// PLL Control
#define PLL_LOCK_GAIN 0.8f      // Lock loop gain
#define STROBE_DIVISOR 4        // Target: ω_strobe = ω_grid/4

// Limits
#define OMEGA_MIN (2.0f * PI * 45.0f)
#define OMEGA_MAX (2.0f * PI * 55.0f)

// Strobe
volatile bool strobe_triggered = false;
volatile bool buffer_ready = false;
hw_timer_t *strobe_timer = NULL;
volatile uint64_t last_strobe_us = 0;

struct Sample {
  float value;
  uint64_t timestamp_us;
};

struct SOGI_Result {
  float v_alpha;
  float v_beta;
  float magnitude;
  float phase;
  bool valid;
};

struct EKF_State {
  float omega_grid;          // True grid frequency (rad/s)
  float omega_dot_grid;      // Grid frequency rate (rad/s²)
  float omega_strobe_error;  // Strobe PLL error (rad/s)
  float P[3][3];             // Covariance
};

struct ZeroCrossing {
  uint64_t time_us;
  float slope;
  int index;  // Sample index
  bool valid;
};

struct HeterodynePLL {
  Sample buffer[BUFFER_SIZE];
  int buffer_idx;
  bool capturing;
  
  float dc_offset;
  
  uint64_t capture_start_us;
  uint64_t capture_end_us;
  uint64_t last_process_us;
  
  // Previous frame data for differential measurements
  ZeroCrossing prev_zc[4];
  int prev_zc_count;
  float prev_phase_estimate;
  uint64_t prev_frame_start_us;
  
  EKF_State ekf;
  
  // PLL control
  float omega_strobe_commanded;
  float strobe_interval_us;
  
  // Statistics
  int frame_count;
  float avg_period_us;
} pll;

void IRAM_ATTR onStrobeTimer() {
  last_strobe_us = esp_timer_get_time();
  strobe_triggered = true;
  
  if (!pll.capturing) {
    pll.capturing = true;
    pll.buffer_idx = 0;
    pll.capture_start_us = last_strobe_us;
  }
}

void initPLL() {
  memset(&pll, 0, sizeof(pll));
  pll.dc_offset = 2048.0f;
  
  // Initialize EKF
  pll.ekf.omega_grid = NOMINAL_OMEGA;
  pll.ekf.omega_dot_grid = 0.0f;
  pll.ekf.omega_strobe_error = 0.0f;  // Assume initially locked
  
  // Initial uncertainty
  pll.ekf.P[0][0] = 10.0f;       // ω uncertainty
  pll.ekf.P[1][1] = 1.0f;        // ω̇ uncertainty
  pll.ekf.P[2][2] = 1.0f;        // Strobe error uncertainty
  // Off-diagonals zero
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i != j) pll.ekf.P[i][j] = 0.0f;
    }
  }
  
  // Initialize strobe
  pll.omega_strobe_commanded = NOMINAL_OMEGA / STROBE_DIVISOR;
  pll.strobe_interval_us = 1000000.0f / (pll.omega_strobe_commanded / (2.0f * PI));
  
  pll.prev_zc_count = 0;
  pll.frame_count = 0;
}

/*
 * Find zero-crossings
 */
int find_zero_crossings(Sample *buf, int start, int count,
                        ZeroCrossing *zc, int max_zc) {
  int num_found = 0;
  
  for (int i = start; i < start + count - 1 && num_found < max_zc; i++) {
    float v0 = buf[i].value;
    float v1 = buf[i + 1].value;
    
    if (v0 <= 0.0f && v1 > 0.0f) {
      float frac = -v0 / (v1 - v0);
      frac = constrain(frac, 0.0f, 1.0f);
      
      uint64_t t0 = buf[i].timestamp_us;
      uint64_t t1 = buf[i + 1].timestamp_us;
      
      zc[num_found].time_us = t0 + (uint64_t)(frac * (float)(t1 - t0));
      zc[num_found].slope = (v1 - v0) * 1000000.0f / (float)(t1 - t0);
      zc[num_found].index = i;
      zc[num_found].valid = (zc[num_found].slope > 0.005f);
      
      num_found++;
    }
  }
  
  return num_found;
}

/*
 * EKF Predict
 */

void ekf_predict(EKF_State *ekf, float dt) {
  // 1. State Prediction (Simplified)
  // New omega = old_omega + (omega_dot * dt)
  ekf->omega_grid += ekf->omega_dot_grid * dt;
  // omega_dot and strobe_error are modeled as constants (Random Walk)
  
  // 2. Covariance Prediction: P = F*P*F' + Q
  // Given F = [1, dt, 0; 0, 1, 0; 0, 0, 1]
  // Pre-calculating sparse multiplication saves significant CPU cycles
  float P00 = ekf->P[0][0], P01 = ekf->P[0][1], P10 = ekf->P[1][0], P11 = ekf->P[1][1];

  // Temporary P calculation for the 2x2 block affected by F
  ekf->P[0][0] = P00 + dt * (P10 + P01 + dt * P11) + (Q_OMEGA * dt);
  ekf->P[0][1] = P01 + dt * P11;
  ekf->P[1][0] = P10 + dt * P11;
  ekf->P[1][1] = P11 + (Q_OMEGA_DOT * dt);
  
  // The strobe error state is decoupled in F, so only add noise
  ekf->P[2][2] = ekf->P[2][2] + (Q_STROBE_ERROR * dt);

  // 3. Symmetry Enforcement (Optional but recommended)
  float avg = (ekf->P[0][1] + ekf->P[1][0]) * 0.5f;
  ekf->P[0][1] = ekf->P[1][0] = avg;
}
 
void ekf_predict_old(EKF_State *ekf, float dt) {
  // State transition
  float omega_new = ekf->omega_grid + ekf->omega_dot_grid * dt;
  float omega_dot_new = ekf->omega_dot_grid;
  float error_new = ekf->omega_strobe_error;  // Persistent bias
  
  ekf->omega_grid = omega_new;
  ekf->omega_dot_grid = omega_dot_new;
  ekf->omega_strobe_error = error_new;
  
  // Jacobian F (state transition is linear here)
  float F[3][3] = {
    {1.0f, dt, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
  };
  
  // Process noise Q
  float Q[3][3] = {
    {Q_OMEGA * dt, 0.0f, 0.0f},
    {0.0f, Q_OMEGA_DOT * dt, 0.0f},
    {0.0f, 0.0f, Q_STROBE_ERROR * dt}
  };
  
  // P = F*P*F' + Q
  float P_old[3][3];
  memcpy(P_old, ekf->P, sizeof(P_old));
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0.0f;
      for (int k = 0; k < 3; k++) {
        for (int l = 0; l < 3; l++) {
          sum += F[i][k] * P_old[k][l] * F[j][l];
        }
      }
      ekf->P[i][j] = sum + Q[i][j];
    }
  }
}

/*
 * EKF Update with Period Measurement
 * 
 * Measurement: T_measured (time between zero-crossings)
 * Expected: T = 2π / ω_grid
 * 
 * This is a DIRECT measurement of grid frequency
 */
// -----------------------------------------------------------------------------
// EKF update using measured grid period (zero-cross → zero-cross)
// Proper ω̇ observability, non-inverted, jitter-safe
// -----------------------------------------------------------------------------
void ekf_update_period(EKF_State *ekf, float T_measured_us)
{
    double T = T_measured_us * 1e-6f;

    // sanity
    if (T < 0.015f || T > 0.025f)
        return;

    // expected integrated phase over one cycle
    double h =
        ekf->omega_grid * T +
        0.5f * ekf->omega_dot_grid * T * T;

    // innovation: should equal 2π
    double innov = (2.0f * PI) - h;

    // measurement Jacobian
    double H[3] = {
        T,
        0.5f * T * T,
        0.0f
    };

    // timestamp jitter → phase noise
#define TIMESTAMP_JITTER_US 4.0d
    double sigma_t = TIMESTAMP_JITTER_US * 1e-6d;
    double sigma_phi = ekf->omega_grid * sigma_t;

    double R = sigma_phi * sigma_phi;

    // === standard EKF update ===

    double S = R;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            S += H[i] * ekf->P[i][j] * H[j];

    double K[3];
    for (int i = 0; i < 3; i++) {
        K[i] = 0.0f;
        for (int j = 0; j < 3; j++)
            K[i] += ekf->P[i][j] * H[j];
        K[i] /= S;
    }

    ekf->omega_grid         += K[0] * innov;
    ekf->omega_dot_grid     += K[1] * innov;
    ekf->omega_strobe_error += K[2] * innov;

    // Joseph update (keeps symmetry + PSD)
    double I_KH[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];

    double Pn[3][3] = {0};
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                Pn[i][j] += I_KH[i][k] * ekf->P[k][j];

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            ekf->P[i][j] = Pn[i][j] + K[i] * R * K[j];
}


/*
 * EKF Update with Cycle Count Measurement
 * 
 * Measurement: Number of cycles captured in frame
 * Expected: (ω_grid/2π) * frame_duration
 * 
 * This measures the frequency ratio between grid and strobe
 */
void ekf_update_cycle_count(EKF_State *ekf, int cycles_counted, float frame_duration_s) {
  // Expected number of cycles
  float cycles_expected = (ekf->omega_grid / (2.0f * PI)) * frame_duration_s;
  
  // Innovation
  float innov = (float)cycles_counted - cycles_expected;
  
  // Only update if count is reasonable
  if (fabs(innov) > 2.0f) return;  // Too far off, ignore
  
  // Measurement Jacobian H = d(cycles)/dx
  // cycles = (ω/2π) * T
  // dcycles/dω = T/2π
  float H[3] = {frame_duration_s / (2.0f * PI), 0.0f, 0.0f};
  
  // Measurement noise (cycle counting is noisy due to windowing)
  float R = R_CYCLE_COUNT;
  
  // Innovation covariance
  float S = 0.0f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      S += H[i] * ekf->P[i][j] * H[j];
    }
  }
  S += R;
  
  // Kalman gain
  float K[3];
  for (int i = 0; i < 3; i++) {
    K[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      K[i] += ekf->P[i][j] * H[j];
    }
    K[i] /= S;
  }
  
  // State update
  ekf->omega_grid += K[0] * innov;
  ekf->omega_dot_grid += K[1] * innov;
  ekf->omega_strobe_error += K[2] * innov;
  
  // Covariance update
  float I_KH[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
    }
  }
  
  float P_temp[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P_temp[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        P_temp[i][j] += I_KH[i][k] * ekf->P[k][j];
      }
    }
  }
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ekf->P[i][j] = P_temp[i][j] + K[i] * R * K[j];
    }
  }
}

/*
 * EKF Update with Phase Drift Measurement (inter-frame)
 * 
 * Measurement: Phase change between frames
 * This is a DIFFERENTIAL measurement
 * 
 * Expected: Δφ = (ω_grid - ω_strobe*4) * Δt
 * This directly measures the beat frequency (strobe error)
 */
void ekf_update_phase_drift(EKF_State *ekf, float phase_drift_rad, float dt_frames) {
  // Expected phase drift
  float omega_strobe_actual = pll.omega_strobe_commanded + ekf->omega_strobe_error;
  float omega_beat = ekf->omega_grid - omega_strobe_actual * STROBE_DIVISOR;
  float phase_drift_expected = omega_beat * dt_frames;
  
  // Wrap both to [-π, π]
  while (phase_drift_rad > PI) phase_drift_rad -= 2.0f * PI;
  while (phase_drift_rad < -PI) phase_drift_rad += 2.0f * PI;
  while (phase_drift_expected > PI) phase_drift_expected -= 2.0f * PI;
  while (phase_drift_expected < -PI) phase_drift_expected += 2.0f * PI;
  
  // Innovation
  float innov = phase_drift_rad - phase_drift_expected;
  while (innov > PI) innov -= 2.0f * PI;
  while (innov < -PI) innov += 2.0f * PI;
  
  // Measurement Jacobian
  // Δφ = (ω_grid - (ω_strobe_cmd + ω_strobe_err)*4) * dt
  // dΔφ/dω_grid = dt
  // dΔφ/dω_strobe_err = -4*dt
  float H[3] = {dt_frames, 0.0f, -STROBE_DIVISOR * dt_frames};
  
  float R = R_PHASE_DRIFT;
  
  // Innovation covariance
  float S = 0.0f;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      S += H[i] * ekf->P[i][j] * H[j];
    }
  }
  S += R;
  
  // Kalman gain
  float K[3];
  for (int i = 0; i < 3; i++) {
    K[i] = 0.0f;
    for (int j = 0; j < 3; j++) {
      K[i] += ekf->P[i][j] * H[j];
    }
    K[i] /= S;
  }
  
  // State update
  ekf->omega_grid += K[0] * innov;
  ekf->omega_dot_grid += K[1] * innov;
  ekf->omega_strobe_error += K[2] * innov;
  
  // Covariance update
  float I_KH[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - K[i] * H[j];
    }
  }
  
  float P_temp[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      P_temp[i][j] = 0.0f;
      for (int k = 0; k < 3; k++) {
        P_temp[i][j] += I_KH[i][k] * ekf->P[k][j];
      }
    }
  }
  
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      ekf->P[i][j] = P_temp[i][j] + K[i] * R * K[j];
    }
  }
}

/*
 * Apply state constraints and prevent covariance collapse
 */
void ekf_constrain(EKF_State *ekf) {
  ekf->omega_grid = constrain(ekf->omega_grid, OMEGA_MIN, OMEGA_MAX);
  ekf->omega_dot_grid = constrain(ekf->omega_dot_grid, -100.0f, 100.0f);
  ekf->omega_strobe_error = constrain(ekf->omega_strobe_error, -20.0f, 20.0f);
  
  // Prevent covariance collapse
  for (int i = 0; i < 3; i++) {
    ekf->P[i][i] = fmaxf(ekf->P[i][i], 1e-10f);
  }
}

/*
 * Update PLL control based on EKF estimate
 */
void update_pll_control() {
  // Target strobe frequency
  float omega_strobe_target = pll.ekf.omega_grid / STROBE_DIVISOR;
  
  // Feedback correction for strobe error
  float correction = PLL_LOCK_GAIN * pll.ekf.omega_strobe_error;
  
  // Commanded frequency
  pll.omega_strobe_commanded = omega_strobe_target - correction;
  
  // Constrain
  pll.omega_strobe_commanded = constrain(pll.omega_strobe_commanded,
                                         2.0f * PI * 10.0f,
                                         2.0f * PI * 15.0f);
  
  // Update timer interval
  float strobe_freq_hz = pll.omega_strobe_commanded / (2.0f * PI);
  pll.strobe_interval_us = 1000000.0f / strobe_freq_hz;
  
  timerAlarm(strobe_timer, (uint32_t)pll.strobe_interval_us, true, 0);
}

/*
 * Run SOGI for magnitude estimation (diagnostic only)
 */
SOGI_Result run_sogi_cycle2(Sample *buf, int cycle2_start, int count) {
  SOGI_Result result = {0};
  
  if (cycle2_start + count > BUFFER_SIZE) {
    result.valid = false;
    return result;
  }
  
  float v_alpha = 0.0f;
  float v_beta = 0.0f;
  float u_prev = 0.0f;
  
  float w = pll.ekf.omega_grid;
  float k = SOGI_K;
  
  for (int i = 0; i < count; i++) {
    float u = buf[cycle2_start + i].value;
    
    float dt;
    if (i > 0) {
      dt = (float)(buf[cycle2_start + i].timestamp_us - 
                   buf[cycle2_start + i - 1].timestamp_us) / 1000000.0f;
    } else if (cycle2_start > 0) {
      dt = (float)(buf[cycle2_start].timestamp_us - 
                   buf[cycle2_start - 1].timestamp_us) / 1000000.0f;
    } else {
      dt = 1.0f / (NOMINAL_FREQ * SAMPLES_PER_CYCLE);
    }
    
    float x = w * dt / 2.0f;
    float den = 1.0f + k*x + x*x;
    
    float alpha_old = v_alpha;
    float beta_old = v_beta;
    
    v_alpha = ((1.0f - x*x)*alpha_old - 2.0f*x*beta_old + k*x*(u + u_prev)) / den;
    v_beta = (2.0f*x*alpha_old + (1.0f - k*x - x*x)*beta_old + k*x*x*(u + u_prev)) / den;
    
    u_prev = u;
  }
  
  result.v_alpha = v_alpha;
  result.v_beta = v_beta;
  result.magnitude = sqrtf(v_alpha*v_alpha + v_beta*v_beta);
  result.phase = atan2f(v_beta, v_alpha);
  result.valid = result.magnitude > 0.05f;
  
  return result;
}

void process_buffer() {
  uint64_t process_start = esp_timer_get_time();
  
  pll.capture_end_us = pll.buffer[pll.buffer_idx - 1].timestamp_us;
  float frame_duration = (float)(pll.capture_end_us - pll.capture_start_us) / 1000000.0f;
  
  // Find zero-crossings
  ZeroCrossing zc[6];
  int zc_count = find_zero_crossings(pll.buffer, 0, BUFFER_SIZE, zc, 6);
  
  if (zc_count < 2) {
    Serial.println("Insufficient zero-crossings");
    buffer_ready = false;
    return;
  }
  
  // EKF Predict
  uint64_t now = esp_timer_get_time();
  float dt_predict = (float)(now - pll.last_process_us) / 1000000.0f;
  if (dt_predict > 0.001f && dt_predict < 2.0f) {
    ekf_predict(&pll.ekf, dt_predict);
  }
  pll.last_process_us = now;
  
  // Measurement 1: Zero-crossing intervals (period measurement)
  for (int i = 0; i < zc_count - 1; i++) {
    float period_us = (float)(zc[i+1].time_us - zc[i].time_us);
    ekf_update_period(&pll.ekf, period_us);
  }
  
  // Measurement 2: Cycle count
  ekf_update_cycle_count(&pll.ekf, zc_count, frame_duration);
  
  // Measurement 3: Phase drift (if we have previous frame)
  if (pll.prev_zc_count > 0 && pll.frame_count > 0) {
    // Compute phase drift using SOGI
    SOGI_Result sogi_curr = run_sogi_cycle2(pll.buffer, SAMPLES_PER_CYCLE, SAMPLES_PER_CYCLE);
    
    if (sogi_curr.valid) {
      float phase_drift = sogi_curr.phase - pll.prev_phase_estimate;
      float dt_frames = (float)(pll.capture_start_us - pll.prev_frame_start_us) / 1000000.0f;
      
      ekf_update_phase_drift(&pll.ekf, phase_drift, dt_frames);
      
      pll.prev_phase_estimate = sogi_curr.phase;
    }
  } else {
    // Initialize
    SOGI_Result sogi_init = run_sogi_cycle2(pll.buffer, SAMPLES_PER_CYCLE, SAMPLES_PER_CYCLE);
    if (sogi_init.valid) {
      pll.prev_phase_estimate = sogi_init.phase;
    }
  }
  
  // Apply constraints
  ekf_constrain(&pll.ekf);
  
  // Update PLL control
  update_pll_control();
  
  // Save current frame data for next iteration
  memcpy(pll.prev_zc, zc, sizeof(ZeroCrossing) * zc_count);
  pll.prev_zc_count = zc_count;
  pll.prev_frame_start_us = pll.capture_start_us;
  
  // Compute statistics
  if (zc_count > 1) {
    float total_period = (float)(zc[zc_count-1].time_us - zc[0].time_us);
    pll.avg_period_us = total_period / (zc_count - 1);
  }
  
  float sample_rate = (float)(BUFFER_SIZE - 1) / frame_duration;
  
  SOGI_Result sogi_diag = run_sogi_cycle2(pll.buffer, SAMPLES_PER_CYCLE, SAMPLES_PER_CYCLE);
  
  uint64_t process_end = esp_timer_get_time();
  float process_time_ms = (float)(process_end - process_start) / 1000.0f;
  
  float freq_hz = pll.ekf.omega_grid / (2.0f * PI);
  float freq_error_mhz = (freq_hz - NOMINAL_FREQ) * 1000.0f;
  float freq_rate = pll.ekf.omega_dot_grid / (2.0f * PI);
  float strobe_hz = pll.omega_strobe_commanded / (2.0f * PI);
  float strobe_error_hz = pll.ekf.omega_strobe_error / (2.0f * PI);
  
  pll.frame_count++;
  
  Serial.printf("F:%+.2f mHz, dF/dt:%+.3f Hz/s, Strobe:%.2f Hz, Δω_s:%+.3f Hz, "
                "ZC:%d, Mag:%.2f V, SR:%.2f Hz, Proc:%.2f ms, σω²:%.2e\n",
                freq_error_mhz, freq_rate, strobe_hz, strobe_error_hz,
                zc_count, sogi_diag.magnitude, sample_rate/1000.0, process_time_ms,
                pll.ekf.P[0][0]);
  
  buffer_ready = false;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n=== ESP32 Heterodyne PLL-EKF ===");
  Serial.println("State: [ω_grid, ω̇_grid, ω_strobe_error]");
  Serial.println("================================\n");
  
  analogReadResolution(ADC_RESOLUTION);
  
  initPLL();
  
  strobe_timer = timerBegin(1000000);
  timerAttachInterrupt(strobe_timer, &onStrobeTimer);
  timerAlarm(strobe_timer, (uint32_t)pll.strobe_interval_us, true, 0);
  
  Serial.printf("Initial strobe: %.2f Hz\n\n", 
                pll.omega_strobe_commanded / (2.0f * PI));
}

void loop() {
  static uint64_t last_sample_t = 0;
  uint64_t now_t = esp_timer_get_time();
  
  float target_dt = 1.0f / (NOMINAL_FREQ * SAMPLES_PER_CYCLE);
  float dt = (float)(now_t - last_sample_t) / 1000000.0f;
  
  if (dt >= target_dt * 0.9f) {
    last_sample_t = now_t;
    
    int raw = analogRead(ADC_PIN);
    pll.dc_offset = (DC_ALPHA * (float)raw) + ((1.0f - DC_ALPHA) * pll.dc_offset);
    float u = ((float)raw - pll.dc_offset) * (V_REF / 4095.0f);
    
    if (pll.capturing && pll.buffer_idx < BUFFER_SIZE) {
      pll.buffer[pll.buffer_idx].value = u;
      pll.buffer[pll.buffer_idx].timestamp_us = now_t;
      pll.buffer_idx++;
      
      if (pll.buffer_idx >= BUFFER_SIZE) {
        pll.capturing = false;
        buffer_ready = true;
      }
    }
  }
  
  if (buffer_ready) {
    process_buffer();
  }
}
