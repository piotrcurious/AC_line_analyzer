/* ESP32 SOGI-PLL + DFT (Goertzel) Harmonic Subtraction Hybrid Target: ESP32 Arduino Core 3.3.1

Strategy:

Sample ADC at a timer-driven rate (adjusted using SOGI-PLL frequency)

Per-sample SOGI generates v_alpha, v_beta -> phase via atan2 and PLL frequency

Periodic windowed Goertzel (DFT) computes fundamental and harmonics, THD

If THD high, build harmonic model (amplitude+phase) and subtract harmonics from raw samples

Adjust sampling interval so a window contains an integer number of cycles (reduces bin quantization)


Notes / Caveats:

This example uses adc1_get_raw for faster ADC reads. Calibrate ADC attenuation / scaling for your front-end.

For production and high sample rates, consider I2S ADC DMA rather than adc1_get_raw or analogRead.

Windowing used: rectangular. If you use window functions (Hann), account for amplitude correction when reconstructing harmonics.


Author: ChatGPT (assistant) */

#include <Arduino.h> #include "driver/adc.h" #include <math.h>

// ----------------------- User-configurable parameters -------------------- const adc1_channel_t ADC_CHANNEL = ADC1_CHANNEL_0; // GPIO36 const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_11;     // full range

// nominal mains frequency volatile float f_nominal = 50.0f; // Hz

// Desired samples per cycle (nominal target). Choose >= 50 for good DFT performance. const int SAMPLES_PER_CYCLE_TARGET = 200; // can be tuned // How many cycles per DFT window (prefer integer, e.g., 5, 10) const int CYCLES_PER_WINDOW = 5;

// PLL / SOGI tuning const float sogi_k_default = 1.414f; // bandwidth factor (1..2) const float pll_kp = 200.0f;  // PI proportional gain for PLL (tuned for sample rate) const float pll_ki = 50.0f;   // PI integral gain

// Goertzel / DFT update rate const int WINDOW_HOP = 200; // hop samples between DFT evaluations (can overlap) const int MAX_HARMONICS = 6; // compute up to 6th harmonic (2..6) const float THD_HIGH = 0.08f; // Enter harmonic subtraction when THD > 8% const float THD_LOW  = 0.03f; // Exit when THD < 3%

// Minimum and maximum sample rate limits (to avoid impossible timer intervals) const float FS_MIN = 2000.0f; // Hz const float FS_MAX = 50000.0f; // Hz

// ----------------------- Runtime state ---------------------------------- hw_timer_t *sampleTimer = NULL;   // hardware timer portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; volatile bool sampleFlag = false; // ISR sets when new sample ready

// circular buffer for samples (raw) const int MAX_WINDOW = 8192; // maximum samples we can hold volatile int16_t sampleBuf[MAX_WINDOW]; volatile int bufWrite = 0; volatile int bufCount = 0;

// SOGI-PLL state (discrete-time Tustin implemented) struct SOGIState { float fs = 10000.0f;   // current sampling frequency float omega = 2.0f * M_PI * 50.0f; // rad/s (expected) float k = sogi_k_default; // bandwidth factor

// discrete state float x1 = 0.0f; // v_alpha float x2 = 0.0f; // internal derivative-like state

// PLL float phi = 0.0f;   // instantaneous phase estimate float omega_est = 2.0f * M_PI * 50.0f; // rad/s estimated float omega_err_int = 0.0f; // integrator

// synthesized harmonics model (updated from DFT) bool harmonic_model_ready = false; int harmonics_K = 0; float harm_amp[MAX_HARMONICS+1]; // index 2..K float harm_phase[MAX_HARMONICS+1]; };

SOGIState sogi;

// DFT/Goertzel state int samples_per_cycle = SAMPLES_PER_CYCLE_TARGET; volatile int samples_per_window = SAMPLES_PER_CYCLE_TARGET * CYCLES_PER_WINDOW; volatile int hop = WINDOW_HOP;

// buffers for Goertzel window float goertzel_window[MAX_HARMONICS+2]; // scratch

// For frequency fusion / smoothing float fused_freq = 50.0f; // Hz

// ----------------------- Utility functions ------------------------------- static inline int adcReadRaw() { // faster low-level ADC read (blocking). return adc1_get_raw(ADC_CHANNEL); }

// Simple atomic-safe push to circular buffer void pushSample(int16_t s) { portENTER_CRITICAL(&timerMux); sampleBuf[bufWrite] = s; bufWrite++; if (bufWrite >= MAX_WINDOW) bufWrite = 0; if (bufCount < MAX_WINDOW) bufCount++; portEXIT_CRITICAL(&timerMux); }

// read N samples from buffer into float array (latest N samples, newest last) int readWindow(float *out, int N) { if (N > MAX_WINDOW) return 0; portENTER_CRITICAL(&timerMux); int available = bufCount; if (available < N) { portEXIT_CRITICAL(&timerMux); return 0; // not enough data } int idx = bufWrite - N; if (idx < 0) idx += MAX_WINDOW; for (int i = 0; i < N; ++i) { out[i] = (float)sampleBuf[idx]; idx++; if (idx >= MAX_WINDOW) idx = 0; } portEXIT_CRITICAL(&timerMux); return N; }

// ----------------------- SOGI discrete update ---------------------------- // Discrete SOGI using simple bilinear-like discretization (Tustin style) // We'll implement a normalized SOGI that outputs v_alpha (in-phase) and v_beta (quadrature approx)

void sogi_init(float fs) { sogi.fs = fs; sogi.omega = 2.0f * M_PI * f_nominal; sogi.omega_est = sogi.omega; for (int k=0;k<=MAX_HARMONICS;k++){ sogi.harm_amp[k]=0; sogi.harm_phase[k]=0; } }

// Call per sample with cleaned input (after harmonic subtraction) void sogi_update_sample(float x_in) { // basic SOGI implementation (discrete second-order integrator pair) // continuous canonical form: // x1' = x2 // x2' = -w0^2 x1 + k*w0 (vin - x1) // we'll discretize with forward Euler using sample interval Ts = 1/fs (simple and stable if fs >> w0)

float Ts = 1.0f / sogi.fs; float w0 = sogi.omega_est; // use estimated omega for filter center float k = sogi.k;

// simple integration step float x1 = sogi.x1; float x2 = sogi.x2;

float x1_dot = x2; float x2_dot = -w0w0x1 + kw0(x_in - x1);

x1 += x1_dot * Ts; x2 += x2_dot * Ts;

sogi.x1 = x1; sogi.x2 = x2;

// v_alpha = x1, v_beta = x2 / w0 float v_alpha = x1; float v_beta  = x2 / max(1e-9f, w0);

// phase detector float phi = atan2f(v_beta, v_alpha);

// PLL PI controller to drive omega_est // phase error relative to internal oscillator phase float phase_err = phi; // because our VCO's internal reference is embedded in omega_est integration below // Integrator sogi.omega_err_int += pll_ki * phase_err * Ts; // proportional float pterm = pll_kp * phase_err; // update omega_est sogi.omega_est = (2.0f * M_PI * f_nominal) + pterm + sogi.omega_err_int;

// for convenience, update phi estimate by integrating omega_est sogi.phi += sogi.omega_est * Ts; // normalize phi if (sogi.phi > M_PI) sogi.phi -= 2.0fM_PI; if (sogi.phi < -M_PI) sogi.phi += 2.0fM_PI; }

// ----------------------- Harmonic synthesis/subtraction ------------------ // Evaluate reconstructed harmonics at current sample index (time t) // Note: the DFT provides amplitude and phase referenced to window center; we must apply delay compensation

float synth_harmonics_at_time(int sample_index_from_window_end, float fs, SOGIState &s) { // sample_index_from_window_end: 0 is newest sample (now), positive values go back in time // If model phases are referenced to the window center delay, we have already stored phases corrected for delay. if (!s.harmonic_model_ready) return 0.0f; float t = 0.0f; // time offset relative to "now" // We'll just evaluate cos(2pikf_estt + phi_k) // but we don't have absolute timestamp, so we use sample index -> t = -n / fs for past samples float acc = 0.0f; for (int k = 2; k <= s.harmonics_K; ++k) { float A = s.harm_amp[k]; float phi = s.harm_phase[k]; float fk = (k * sogi.omega_est) / (2.0f*M_PI); // use current omega_est for frequency // compute phase at now (we assume s.harm_phase already aligned to now) // If we stored phases aligned to window center, they should have been shifted when model saved. acc += A * cosf(phi); } return acc; }

// For simplicity in this example the model phases are stored already "aligned" to current time when built. // A production implementation must compute exact phase shift: phi_now = phi_measured + 2pif*(delay_samples)/fs

// ----------------------- Goertzel (single bin) -------------------------- // Compute complex DFT bin using Goertzel for frequency f_k for N samples

struct GoertzelResult { float re; float im; float mag; float phase; };

GoertzelResult goertzel_compute(float x, int N, float fs, float target_f) { GoertzelResult r = {0,0,0,0}; float k = (float)N * target_f / fs; float omega = 2.0f * M_PI * k / N; float coeff = 2.0f * cosf(omega); float s_prev = 0.0f; float s_prev2 = 0.0f; for (int n = 0; n < N; ++n) { float s = x[n] + coeff * s_prev - s_prev2; s_prev2 = s_prev; s_prev = s; } float re = s_prev - s_prev2 * cosf(omega); float im = s_prev2 * sinf(omega); r.re = re; r.im = im; r.mag = sqrtf(rere + im*im); r.phase = atan2f(im, re); return r; }

// ----------------------- DFT & fusion task --------------------------------

void process_window_and_update_model() { int N = samples_per_window; if (N <= 0 || N > MAX_WINDOW) return; static float data[MAX_WINDOW]; int got = readWindow(data, N); if (got != N) return; // not enough data yet

// Optionally apply DC removal (high-pass simple) float mean = 0; for (int i=0;i<N;i++) mean += data[i]; mean /= N; for (int i=0;i<N;i++) data[i] -= mean;

// Compute fundamental using Goertzel at estimated freq float f_est = fused_freq; // Hz (use fused or sogi estimate) GoertzelResult fund = goertzel_compute(data, N, sogi.fs, f_est); float P1 = fund.mag * fund.mag;

// compute harmonics float PH = 0.0f; GoertzelResult harmRes[MAX_HARMONICS+1]; for (int k=2;k<=MAX_HARMONICS;k++) { float fk = k * f_est; if (fk > sogi.fs/2.0f) { harmRes[k].mag = 0; harmRes[k].phase=0; continue; } harmRes[k] = goertzel_compute(data, N, sogi.fs, fk); PH += (harmRes[k].mag * harmRes[k].mag); } float thd = (P1 <= 0) ? 0.0f : sqrtf(PH)/sqrtf(P1);

// compute DFT-derived frequency using phase difference method static float prevFundPhase = 0.0f; static unsigned long prevFundTimestamp = 0; float phi_d = fund.phase; unsigned long now_ms = millis(); float dt_sec = (prevFundTimestamp==0) ? ( (float)N / sogi.fs ) : ( (now_ms - prevFundTimestamp) / 1000.0f ); float df = 0.0f; if (prevFundTimestamp != 0) { // unwrap float dphi = phi_d - prevFundPhase; while (dphi > M_PI) dphi -= 2.0fM_PI; while (dphi < -M_PI) dphi += 2.0fM_PI; df = dphi / (2.0f * M_PI * dt_sec); } float f_d = f_est + df; // approximate prevFundPhase = phi_d; prevFundTimestamp = now_ms;

// Align DFT phase to "now" by compensating for group delay (~N/2 samples) float delay_samples = (float)N/2.0f; float phi_aligned = phi_d + 2.0f * M_PI * f_d * (delay_samples / sogi.fs); while (phi_aligned > M_PI) phi_aligned -= 2.0fM_PI; while (phi_aligned < -M_PI) phi_aligned += 2.0fM_PI;

// decide fusion weights float w_d = 0.0f; if (thd > THD_HIGH) w_d = 0.8f; else if (thd < THD_LOW) w_d = 0.0f; else w_d = (thd - THD_LOW) / (THD_HIGH - THD_LOW); float w_s = 1.0f - w_d;

// update fused frequency (simple weighted) float sogi_freq = sogi.omega_est / (2.0f*M_PI); fused_freq = w_s * sogi_freq + w_d * f_d; if (fused_freq < 1.0f) fused_freq = f_nominal;

// If THD is high, build harmonic model and subtract if (thd > THD_HIGH) { sogi.harmonic_model_ready = true; sogi.harmonics_K = MAX_HARMONICS; for (int k=2;k<=MAX_HARMONICS;k++) { sogi.harm_amp[k] = harmRes[k].mag / (float)N; // rough scaling; tune if using window // align harmonic phase to now (similar delay compensation) float phi_k = harmRes[k].phase + 2.0fM_PI * (kf_d) * (delay_samples / sogi.fs); while (phi_k > M_PI) phi_k -= 2.0fM_PI; while (phi_k < -M_PI) phi_k += 2.0fM_PI; sogi.harm_phase[k] = phi_k; } } else if (thd < THD_LOW) { sogi.harmonic_model_ready = false; sogi.harmonics_K = 0; }

// adjust sampling to make window contain integer number of cycles // target samples_per_cycle = round(samples_per_cycle * (fused_freq / f_nominal)) int new_samples_per_cycle = max(4, (int)round((float)SAMPLES_PER_CYCLE_TARGET * (f_nominal / fused_freq))); int new_samples_per_window = new_samples_per_cycle * CYCLES_PER_WINDOW; // constrain float new_fs = fused_freq * new_samples_per_cycle; if (new_fs < FS_MIN) new_fs = FS_MIN; if (new_fs > FS_MAX) new_fs = FS_MAX;

// apply changes if (abs(new_samples_per_window - samples_per_window) != 0) { samples_per_window = new_samples_per_window; samples_per_cycle = new_samples_per_cycle; // update hop to maintain at least some overlap hop = max(8, samples_per_window / 4); // update timer frequency (safe to call) // compute new timer period in microseconds float new_period_us = 1e6f / new_fs; // set via timerChangeAlarm? we'll set flag and update on main thread to avoid ISR reconfig // We'll store in global and call updateSamplingInterval() from loop portENTER_CRITICAL(&timerMux); sogi.fs = new_fs; portEXIT_CRITICAL(&timerMux); }

// debug print (low-rate) static unsigned long lastPrint = 0; if (millis() - lastPrint > 500) { lastPrint = millis(); Serial.print("thd="); Serial.print(thd,4); Serial.print(" fundMag="); Serial.print(fund.mag); Serial.print(" fused_f="); Serial.print(fused_freq,4); Serial.print(" samples_per_window="); Serial.println(samples_per_window); } }

// ----------------------- Sampling timer ISR -----------------------------

// We'll implement sampling ISR to read ADC and push into buffer; main loop performs SOGI update and DFT periodically volatile unsigned long sampleIndex = 0;

void IRAM_ATTR onTimer() { int raw = adcReadRaw(); // optional scaling: center around 0 int16_t s = (int16_t)(raw - 2048); pushSample(s); sampleFlag = true; sampleIndex++; }

// ----------------------- Setup / Loop ----------------------------------

void setupTimerForRate(float fs) { // Use hw timer 0, prescaler 80 -> 1 microsecond ticks on 80MHz APB if (sampleTimer) { timerEnd(sampleTimer); sampleTimer = NULL; } sampleTimer = timerBegin(0, 80, true); timerAttachInterrupt(sampleTimer, &onTimer, true); unsigned long alarm_us = (unsigned long)max(1.0f, 1e6f / fs); timerAlarmWrite(sampleTimer, alarm_us, true); timerAlarmEnable(sampleTimer); }

void setup() { Serial.begin(115200); delay(1000); Serial.println("SOGI-DFT Harmonic Subtraction - ESP32 Example");

// ADC init adc1_config_width(ADC_WIDTH_BIT_12); adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

// initial sampling frequency float initial_fs = fused_freq * SAMPLES_PER_CYCLE_TARGET; if (initial_fs < FS_MIN) initial_fs = FS_MIN; if (initial_fs > FS_MAX) initial_fs = FS_MAX; sogi_init(initial_fs); setupTimerForRate(initial_fs); }

unsigned long lastDFTProcessIndex = 0;

void loop() { // If there's a new sample, update SOGI per-sample. We read freshest sample from buffer if (sampleFlag) { portENTER_CRITICAL(&timerMux); sampleFlag = false; // read newest sample (last written) int idx = bufWrite - 1; if (idx < 0) idx += MAX_WINDOW; int16_t raw = sampleBuf[idx]; float x = (float)raw; portEXIT_CRITICAL(&timerMux);

// subtract harmonics if model ready
float clean = x;
if (sogi.harmonic_model_ready) {
  // for this simple example we only subtract static amplitude*cos(phi) per harmonic (already aligned)
  float sub = synth_harmonics_at_time(0, sogi.fs, sogi);
  clean = x - sub;
}

// update SOGI with cleaned sample
sogi_update_sample(clean);

// Every hop samples, trigger window processing
if ((int)sampleIndex - (int)lastDFTProcessIndex >= hop) {
  lastDFTProcessIndex = sampleIndex;
  process_window_and_update_model();

  // After process, update sampling timer if sogi.fs changed
  // (we stored new sogi.fs earlier when adjusting sampling)
  static float last_fs_set = 0.0f;
  float desired_fs = sogi.fs;
  if (fabs(desired_fs - last_fs_set) > 1e-3f) {
    last_fs_set = desired_fs;
    setupTimerForRate(desired_fs);
  }
}

} }
